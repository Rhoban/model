#include "Model/ForwardSimulation.hpp"

namespace Leph
{
ForwardSimulation::ForwardSimulation(Model& model)
  : _model(&model)
  , _jointModels()
  , _isJointActuated()
  , _positions(Eigen::VectorXd::Zero(_model->sizeDOF()))
  , _velocities(Eigen::VectorXd::Zero(_model->sizeDOF()))
  , _goals(Eigen::VectorXd::Zero(_model->sizeDOF()))
  , _accelerations(Eigen::VectorXd::Zero(_model->sizeDOF()))
  , _jointTorques(Eigen::VectorXd::Zero(_model->sizeDOF()))
  , _frictionTorques(Eigen::VectorXd::Zero(_model->sizeDOF()))
  , _controlTorques(Eigen::VectorXd::Zero(_model->sizeDOF()))
  , _inertiaOffsets(Eigen::VectorXd::Zero(_model->sizeDOF()))
{
  // Init joint models
  for (size_t i = 0; i < _model->sizeDOF(); i++)
  {
    std::string name = _model->getDOFName(i);
    // Base DOF are not actuated
    if (name.find("base_") != std::string::npos)
    {
      // Dummy JointModel (unused)
      _isJointActuated.push_back(false);
      _jointModels.push_back(JointModel());
    }
    else
    {
      _jointModels.push_back(JointModel(name));
      _isJointActuated.push_back(true);
    }
  }
  // Load state
  _positions = _model->getDOFVect();
  _goals = _model->getDOFVect();
}

const JointModel& ForwardSimulation::jointModel(size_t index) const
{
  if (index >= _jointModels.size())
  {
    throw std::logic_error("ForwardSimulation invalid index");
  }
  if (!_isJointActuated[index])
  {
    throw std::logic_error("ForwardSimulation not actuated joint model");
  }
  return _jointModels[index];
}
JointModel& ForwardSimulation::jointModel(size_t index)
{
  if (index >= _jointModels.size())
  {
    throw std::logic_error("ForwardSimulation invalid index");
  }
  if (!_isJointActuated[index])
  {
    throw std::logic_error("ForwardSimulation not actuated joint model");
  }
  return _jointModels[index];
}
const JointModel& ForwardSimulation::jointModel(const std::string& name) const
{
  return jointModel(_model->getDOFIndex(name));
}
JointModel& ForwardSimulation::jointModel(const std::string& name)
{
  return jointModel(_model->getDOFIndex(name));
}

void ForwardSimulation::setJointModelParameters(const Eigen::VectorXd& params)
{
  for (size_t i = 0; i < _jointModels.size(); i++)
  {
    if (_isJointActuated[i])
    {
      _jointModels[i].setParameters(params);
    }
  }
}

const Eigen::VectorXd& ForwardSimulation::positions() const
{
  return _positions;
}
Eigen::VectorXd& ForwardSimulation::positions()
{
  return _positions;
}
const Eigen::VectorXd& ForwardSimulation::velocities() const
{
  return _velocities;
}
Eigen::VectorXd& ForwardSimulation::velocities()
{
  return _velocities;
}
const Eigen::VectorXd& ForwardSimulation::goals() const
{
  return _goals;
}
Eigen::VectorXd& ForwardSimulation::goals()
{
  return _goals;
}
const Eigen::VectorXd& ForwardSimulation::jointTorques() const
{
  return _jointTorques;
}
const Eigen::VectorXd& ForwardSimulation::frictionTorques() const
{
  return _frictionTorques;
}
const Eigen::VectorXd& ForwardSimulation::controlTorques() const
{
  return _controlTorques;
}
const Eigen::VectorXd& ForwardSimulation::accelerations() const
{
  return _accelerations;
}

void ForwardSimulation::update(double dt, RBDL::ConstraintSet* constraints)
{
  size_t size = _model->sizeDOF();

  // Compute partial (with fixed DOF
  // for static friction) Forward Dynamics
  if (constraints == nullptr)
  {
    _accelerations = _model->forwardDynamicsCustom(_positions, _velocities, _jointTorques, _inertiaOffsets,
                                                   RBDLMath::LinearSolverFullPivHouseholderQR);
  }
  else
  {
    /* TODO
    _accelerations =
        _model->forwardDynamicsContactsCustom(
        *constraints,
        _positions,
        _velocities,
        _jointTorques,
        _inertiaOffsets,
        RBDLMath::LinearSolverFullPivHouseholderQR);
    */
  }

  // TODO XXX
  // std::cout << "TEST impultive dynamics" << std::endl;
  Eigen::VectorXd tmpNextVel;
  if (constraints != nullptr)
  {
    tmpNextVel =
        _model->forwardImpulseDynamicsContactsCustom(dt, *constraints, _positions, _velocities, _jointTorques,
                                                     _inertiaOffsets, RBDLMath::LinearSolverFullPivHouseholderQR);
  }

  // Compute next state with
  // Euler integration.
  Eigen::VectorXd nextVelocities = _velocities + dt * _accelerations;
  _velocities = 0.5 * _velocities + 0.5 * nextVelocities;
  _positions = _positions + dt * _velocities;

  // TODO XXX
  if (constraints != nullptr)
  {
    _velocities = tmpNextVel;
    _positions = _positions + dt * _velocities;
  }

  // Assign model position state
  _model->setDOFVect(_positions);

  // Check numerical validity
  if (!_positions.allFinite() || !_velocities.allFinite() || !_accelerations.allFinite() || !_goals.allFinite() ||
      !_frictionTorques.allFinite() || !_controlTorques.allFinite() || !_jointTorques.allFinite())
  {
    throw std::runtime_error("ForwardSimulation numerical instability");
  }

  // Bound joint state
  for (size_t i = 0; i < size; i++)
  {
    if (_isJointActuated[i])
    {
      _jointModels[i].boundState(_positions(i), _velocities(i));
    }
  }

  // Update all joint models with
  // new computed position, velocity
  // and given goal.
  //(Not updated at the begining but
  // not important due to the control lag)
  //(Updated at the end to output fresh state)
  for (size_t i = 0; i < size; i++)
  {
    if (_isJointActuated[i])
    {
      // Update joint model state
      _jointModels[i].updateState(dt, _goals(i), _positions(i), _velocities(i));
      // Retrieve inertia offset
      _inertiaOffsets(i) = _jointModels[i].getInertia();
      // Recompute all friction
      // and control torque
      _frictionTorques(i) = _jointModels[i].frictionTorque(_velocities(i));
      _controlTorques(i) = _jointModels[i].controlTorque(_positions(i), _velocities(i));
      _jointTorques(i) = _frictionTorques(i) + _controlTorques(i);
    }
  }
}

void ForwardSimulation::computeImpulses(RBDL::ConstraintSet& constraints)
{
  _velocities = _model->impulseContactsCustom(constraints, _positions, _velocities, _inertiaOffsets,
                                              RBDLMath::LinearSolverFullPivHouseholderQR);
}

void ForwardSimulation::computeContactLCP(RBDL::ConstraintSet& constraints,
                                          const Eigen::VectorXi& isBilateralConstraint)
{
  _model->resolveContactConstraintLCP(constraints, isBilateralConstraint, _positions, _velocities, _jointTorques,
                                      _inertiaOffsets);
}

}  // namespace Leph

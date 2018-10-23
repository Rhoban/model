#include <stdexcept>
#include <cmath>
#include "Model/Model.hpp"
#include "Model/RBDLClosedLoop.h"
#include "Model/RBDLContactLCP.h"

namespace Leph {

Model::Model() :
    _model(),
    _isAutoUpdate(true),
    _dofIndexToName(),
    _dofNameToIndex(),
    _dofs(),
    _vectorDOF(),
    _frameIndexToName(),
    _frameNameToIndex(),
    _frameIndexToId(),
    _inertiaData(),
    _inertiaName(),
    _geometryData(),
    _geometryName()
{
}
        
Model::Model(const std::string& filename) :
    _model(),
    _isAutoUpdate(true),
    _dofIndexToName(),
    _dofNameToIndex(),
    _dofs(),
    _vectorDOF(),
    _frameIndexToName(),
    _frameNameToIndex(),
    _frameIndexToId(),
    _inertiaData(),
    _inertiaName(),
    _geometryData(),
    _geometryName()
{
    //URDF loading and retrieve inertia 
    //and geometry data
    RBDL::Model model;
    if (!RBDL::Addons::URDFReadFromFile(
        filename.c_str(), &model, false, 
        &_inertiaData, &_inertiaName, false,
        &_geometryData, &_geometryName, false)
    ) {
        throw std::runtime_error(
            "Model unable to load URDF file: " + filename);
    }

    //Parse and load RBDL model
    initializeModel(model, 
        _inertiaData, _inertiaName,
        _geometryData, _geometryName);
}
        
Model::Model(const std::string& filename, 
    const Eigen::MatrixXd& inertiaData,
    const std::map<std::string, size_t>& inertiaName,
    const Eigen::MatrixXd& geometryData,
    const std::map<std::string, size_t>& geometryName) :
    _model(),
    _isAutoUpdate(true),
    _dofIndexToName(),
    _dofNameToIndex(),
    _dofs(),
    _vectorDOF(),
    _frameIndexToName(),
    _frameNameToIndex(),
    _frameIndexToId(),
    _inertiaData(inertiaData),
    _inertiaName(inertiaName),
    _geometryData(geometryData),
    _geometryName(geometryName)
{
    //URDF loading with override inertia 
    //and geometry data
    RBDL::Model model;
    if (!RBDL::Addons::URDFReadFromFile(
        filename.c_str(), &model, false, 
        &_inertiaData, &_inertiaName, true,
        &_geometryData, &_geometryName, true)
    ) {
        throw std::runtime_error(
            "Model unable to load URDF file: " + filename);
    }

    //Parse and load RBDL model
    initializeModel(model, 
        _inertiaData, _inertiaName,
        _geometryData, _geometryName);
}
        
Model::Model(RBDL::Model& model,
    const Eigen::MatrixXd& inertiaData,
    const std::map<std::string, size_t>& inertiaName,
    const Eigen::MatrixXd& geometryData,
    const std::map<std::string, size_t>& geometryName) :
    _model(),
    _isAutoUpdate(true),
    _dofIndexToName(),
    _dofNameToIndex(),
    _dofs(),
    _vectorDOF(),
    _frameIndexToName(),
    _frameNameToIndex(),
    _frameIndexToId(),
    _inertiaData(inertiaData),
    _inertiaName(inertiaName),
    _geometryData(geometryData),
    _geometryName(geometryName)
{
    //Parse and load RBDL model
    initializeModel(model, 
        _inertiaData, _inertiaName,
        _geometryData, _geometryName);
}
        
bool Model::isAutoUpdate() const
{
    return _isAutoUpdate;
}
        
void Model::setAutoUpdate(bool isEnabled)
{
    _isAutoUpdate = isEnabled;
}
        
void Model::updateDOFPosition()
{
    if (!_isAutoUpdate) {
        RBDL::UpdateKinematicsCustom(
            _model, &_dofs, nullptr, nullptr);
    }
}
        
size_t Model::sizeDOF() const
{
    return _model.dof_count;
}
        
const VectorLabel& Model::getDOF()
{
    loadEigenToLabel();
    return _vectorDOF;
}
double Model::getDOF(const std::string& name) const
{
    try {
        return _dofs(_dofNameToIndex.at(name));
    } catch (const std::out_of_range& e) {
        throw std::logic_error("Model unknown getDOF(): " + name);
    }
}
double Model::getDOF(size_t index) const
{
    if (index >= _dofIndexToName.size()) {
        throw std::logic_error("Model invalid DOF index");
    }
    return _dofs(index);
}
void Model::setDOF(const VectorLabel& vect, bool setBase)
{
    loadLabelToEigen(vect, _dofs, setBase);
}
void Model::setDOF(const std::string& name, double value)
{
    try {
        _dofs(_dofNameToIndex.at(name)) = value;
    } catch (const std::out_of_range& e) {
        throw std::logic_error("Model unknown setDOF(): " + name);
    }
}
void Model::setDOF(size_t index, double value)
{
    if (index >= _dofIndexToName.size()) {
        throw std::logic_error("Model invalid DOF index");
    }
    _dofs(index) = value;
}

void Model::setDOFZeros()
{
    _dofs.setZero();
}
        
const std::string& Model::getDOFName(size_t index) const
{
    if (index >= _dofIndexToName.size()) {
        throw std::logic_error("Model invalid DOF index");
    }
    return _dofIndexToName.at(index);
}
size_t Model::getDOFIndex(const std::string& name) const
{
    return _dofNameToIndex.at(name);
}

const Eigen::VectorXd& Model::getDOFVect() const
{
    return _dofs;
}
void Model::setDOFVect(const Eigen::VectorXd& vect)
{
    if (vect.size() != _dofs.size()) {
        throw std::logic_error(
            "Model invalid DOF vector size");
    }
    _dofs = vect;
}
        
void Model::importDOF(Model& model)
{
    for (size_t i=0;i<(size_t)_dofs.size();i++) {
        _dofs(i) = model._dofs(
            model._dofNameToIndex.at(_dofIndexToName.at(i)));
    }
}

size_t Model::sizeFrame() const
{
    return _frameIndexToName.size();
}

const std::string& Model::getFrameName(size_t index) const
{
    return _frameIndexToName.at(index);
}
        
size_t Model::getFrameIndex(const std::string& name) const
{
    return _frameNameToIndex.at(name);
}
        
Eigen::Vector3d Model::position(
    size_t srcFrameIndex, size_t dstFrameIndex,
    const Eigen::Vector3d& point)
{
    if (srcFrameIndex == dstFrameIndex) {
        return point;
    }

    //Convert to body id
    srcFrameIndex = _frameIndexToId.at(srcFrameIndex);
    dstFrameIndex = _frameIndexToId.at(dstFrameIndex);

    //Compute transformation from body1 to base and base to body2
    RBDLMath::Vector3d ptBase;
    if (srcFrameIndex != 0) {
        ptBase = RBDL::CalcBodyToBaseCoordinates(
            _model, _dofs, srcFrameIndex, point, _isAutoUpdate);
    } else {
        ptBase = point;
    }
    RBDLMath::Vector3d ptBody;
    if (dstFrameIndex != 0) {
        ptBody = RBDL::CalcBaseToBodyCoordinates(
            _model, _dofs, dstFrameIndex, ptBase, _isAutoUpdate);
    } else {
        ptBody = ptBase;
    }

    return ptBody;
}
Eigen::Vector3d Model::position(
    const std::string& srcFrame, const std::string& dstFrame,
    const Eigen::Vector3d& point)
{
    return position(
        getFrameIndex(srcFrame), 
        getFrameIndex(dstFrame),
        point);
}
        
Eigen::Matrix3d Model::orientation(
    size_t srcFrameIndex, size_t dstFrameIndex)
{
    //Convert to body id
    srcFrameIndex = _frameIndexToId.at(srcFrameIndex);
    dstFrameIndex = _frameIndexToId.at(dstFrameIndex);
    
    RBDLMath::Matrix3d transform1;
    transform1 = CalcBodyWorldOrientation(
        _model, _dofs, srcFrameIndex, _isAutoUpdate);
    RBDLMath::Matrix3d transform2;
    transform2 = CalcBodyWorldOrientation(
        _model, _dofs, dstFrameIndex, _isAutoUpdate);

    return transform1*transform2.transpose();
}
Eigen::Matrix3d Model::orientation(
    const std::string& srcFrame, const std::string& dstFrame)
{
    return orientation(
        getFrameIndex(srcFrame), 
        getFrameIndex(dstFrame));
}

Eigen::Affine3d Model::getTransform(size_t srcFrameIndex, size_t dstFrameIndex)
{
  Eigen::Affine3d rotation(orientation(srcFrameIndex, dstFrameIndex));
  Eigen::Translation3d translation(-position(srcFrameIndex, dstFrameIndex));
  return rotation * translation;
}

Eigen::Affine3d Model::getTransform(
    const std::string& srcFrame, const std::string& dstFrame)
{
    return getTransform(
        getFrameIndex(srcFrame), 
        getFrameIndex(dstFrame));
}
        
double Model::orientationYaw(
    size_t srcFrameIndex,
    size_t dstFrameIndex)
{
    Eigen::Matrix3d rotation = 
        orientation(srcFrameIndex, dstFrameIndex);
    rotation.transposeInPlace();

    return atan2(rotation(1, 0), rotation(0, 0));
}
double Model::orientationYaw(
    const std::string& srcFrame,
    const std::string& dstFrame)
{
    Eigen::Matrix3d rotation = orientation(srcFrame, dstFrame);
    rotation.transposeInPlace();
    
    return atan2(rotation(1, 0), rotation(0, 0));
}
        
Eigen::MatrixXd Model::pointJacobian(
    const std::string& srcFrame,
    const std::string& dstFrame,
    const Eigen::Vector3d& point)
{
    //Convert to body id
    size_t srcFrameIndex = getFrameIndex(srcFrame);
    srcFrameIndex = _frameIndexToId.at(srcFrameIndex);
    
    //Init matrix
    RBDLMath::MatrixNd G(6, _model.qdot_size);
    G.setZero();

    //Compute jacobian on given point in 
    //world origin frame
    CalcPointJacobian6D(_model, _dofs,
        srcFrameIndex, point, G, true);

    //Convertion to dst frame
    if (dstFrame != "origin") {
        Eigen::Matrix3d mat = orientation("origin", dstFrame);
        mat.transposeInPlace();
        for (size_t i=0;i<(size_t)G.cols();i++) {
            Eigen::Vector3d rot = G.block(0, i, 3, 1);
            Eigen::Vector3d trans = G.block(3, i, 3, 1);
            G.block(0, i, 3, 1) = mat * rot;
            G.block(3, i, 3, 1) = mat * trans;
        }
    }
        
    return G;
}

Eigen::VectorXd Model::pointVelocity(
    const std::string& pointFrame, 
    const std::string& dstFrame, 
    const Eigen::VectorXd& velocity,
    const Eigen::Vector3d& point)
{
    //Convert to body id
    size_t pointFrameIndex = getFrameIndex(pointFrame);
    pointFrameIndex = _frameIndexToId.at(pointFrameIndex);
    
    //Compute velocity
    Eigen::VectorXd vel = CalcPointVelocity6D(_model, _dofs, 
        velocity, pointFrameIndex, point, true);

    //Convertion to dst frame
    Eigen::Matrix3d mat = orientation("origin", dstFrame);
    mat.transposeInPlace();
    Eigen::Vector3d rot = vel.segment(0, 3);
    Eigen::Vector3d trans = vel.segment(3, 3);
    vel.segment(0, 3) = mat * rot;
    vel.segment(3, 3) = mat * trans;

    return vel;
}
Eigen::VectorXd Model::pointAcceleration(
    const std::string& pointFrame, 
    const std::string& dstFrame, 
    const Eigen::VectorXd& velocity,
    const Eigen::VectorXd& acceleration,
    const Eigen::Vector3d& point)
{
    //Convert to body id
    size_t pointFrameIndex = getFrameIndex(pointFrame);
    pointFrameIndex = _frameIndexToId.at(pointFrameIndex);
    
    //Compute acceleration
    Eigen::VectorXd acc =  CalcPointAcceleration6D(_model, _dofs, 
        velocity, acceleration, pointFrameIndex, point, true);
    
    //Convertion to dst frame
    Eigen::Matrix3d mat = orientation("origin", dstFrame);
    mat.transposeInPlace();
    Eigen::Vector3d rot = acc.segment(0, 3);
    Eigen::Vector3d trans = acc.segment(3, 3);
    acc.segment(0, 3) = mat * rot;
    acc.segment(3, 3) = mat * trans;

    return acc;
}
        
Eigen::Vector3d Model::centerOfMass(size_t frameIndex)
{
    double mass;
    RBDLMath::Vector3d com;
    RBDL::Utils::CalcCenterOfMass(
        _model, _dofs, _dofs, mass, com, 
        nullptr, nullptr, _isAutoUpdate);

    return position(getFrameIndex("origin"), frameIndex, com);
}
Eigen::Vector3d Model::centerOfMass(const std::string& frame)
{
    return centerOfMass(getFrameIndex(frame));
}

double Model::sumMass()
{
    RBDLMath::VectorNd Q(sizeDOF());
    double mass;
    RBDLMath::Vector3d com;
    RBDL::Utils::CalcCenterOfMass(_model, Q, Q, mass, com,
        nullptr, nullptr, _isAutoUpdate);

    return mass;
}
        
void Model::setGravity(const Eigen::Vector3d& vect)
{
    _model.gravity = vect;
}
        
Eigen::VectorXd Model::inverseDynamics(
    const Eigen::VectorXd& velocity,
    const Eigen::VectorXd& acceleration)
{
    RBDLMath::VectorNd QDot;
    RBDLMath::VectorNd QDDot;
    if (velocity.size() == 0) {
        QDot = RBDLMath::VectorNd::Zero(_model.dof_count);
    } else if (velocity.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid velocity vector size");
    } else {
        QDot = velocity;
    }
    if (acceleration.size() == 0) {
        QDDot = RBDLMath::VectorNd::Zero(_model.dof_count);
    } else if (acceleration.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid acceleration vector size");
    } else {
        QDDot = acceleration;
    }

    RBDLMath::VectorNd tau(_model.dof_count);
    tau.setZero();
    RBDL::InverseDynamics(
        _model, _dofs, QDot, QDDot,
        tau, NULL);

    return tau;
}
VectorLabel Model::inverseDynamics(
    const VectorLabel& velocity,
    const VectorLabel& acceleration)
{
    //Declare Eigen Vector
    Eigen::VectorXd vel = Eigen::VectorXd::Zero(_model.dof_count);
    Eigen::VectorXd acc = Eigen::VectorXd::Zero(_model.dof_count);
    //Convertion from VectorLabel
    loadLabelToEigen(velocity, vel, false);
    loadLabelToEigen(acceleration, acc, false);
    //Given velocity and acceleration of
    //base pitch and roll are also used
    vel(Model::getDOFIndex("base_pitch")) = velocity("base_pitch");
    vel(Model::getDOFIndex("base_roll")) = velocity("base_roll");
    acc(Model::getDOFIndex("base_pitch")) = acceleration("base_pitch");
    acc(Model::getDOFIndex("base_roll")) = acceleration("base_roll");

    //Call implementation
    Eigen::VectorXd torques = inverseDynamics(vel, acc);

    //Back conversion
    VectorLabel vect = _vectorDOF;
    for (size_t i=0;i<vect.size();i++) {
        const std::string& label = vect.getLabel(i);
        vect(i) = torques(_dofNameToIndex.at(label));
    }

    return vect;
}

Eigen::VectorXd Model::inverseDynamicsClosedLoop(
    size_t fixedFrameIndex,
    Eigen::VectorXd* contactForce,
    bool useInfinityNorm, 
    const Eigen::VectorXd& velocity,
    const Eigen::VectorXd& acceleration)
{
    RBDLMath::VectorNd QDot;
    RBDLMath::VectorNd QDDot;
    if (velocity.size() == 0) {
        QDot = RBDLMath::VectorNd::Zero(_model.dof_count);
    } else if (velocity.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid velocity vector size");
    } else {
        QDot = velocity;
    }
    if (acceleration.size() == 0) {
        QDDot = RBDLMath::VectorNd::Zero(_model.dof_count);
    } else if (acceleration.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid acceleration vector size");
    } else {
        QDDot = acceleration;
    }

    unsigned int fixedFrameId = _frameIndexToId.at(fixedFrameIndex);
    return RBDLClosedLoopInverseDynamics(
        _model, _dofs, QDot, QDDot,
        fixedFrameId, contactForce, useInfinityNorm);
}
Eigen::VectorXd Model::inverseDynamicsClosedLoop(
    const std::string& fixedFrameName,
    Eigen::VectorXd* contactForce,
    bool useInfinityNorm, 
    const Eigen::VectorXd& velocity,
    const Eigen::VectorXd& acceleration)
{
    return inverseDynamicsClosedLoop(
        getFrameIndex(fixedFrameName), 
        contactForce,
        useInfinityNorm, 
        velocity, acceleration);
}
        
Eigen::VectorXd Model::forwardDynamics(
    const Eigen::VectorXd& position,
    const Eigen::VectorXd& velocity,
    const Eigen::VectorXd& torque)
{
    //Sanity check
    if (position.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid position vector size");
    }
    if (velocity.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid velocity vector size");
    }
    if (torque.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid acceleration vector size");
    }
    RBDLMath::VectorNd QDDot(_model.dof_count);
    RBDL::ForwardDynamics(
        _model, position, velocity, torque, 
        QDDot, NULL);

    return QDDot;
}

Eigen::VectorXd Model::forwardDynamicsCustom(
    const Eigen::VectorXd& position,
    const Eigen::VectorXd& velocity,
    const Eigen::VectorXd& torque,
    const Eigen::VectorXd& inertiaOffset,
    RBDLMath::LinearSolver solver)
{
    //Sanity check
    if (position.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid position vector size");
    }
    if (velocity.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid velocity vector size");
    }
    if (torque.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid acceleration vector size");
    }
    if (inertiaOffset.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid inertia vector size");
    }

    //Retrieve size
    size_t sizeDOF = position.size();

    //Initialize returned acceleration
    Eigen::VectorXd acceleration = 
        Eigen::VectorXd::Zero(sizeDOF);

    //Compute full H anc C matrix
    RBDLMath::MatrixNd H = RBDLMath::MatrixNd::Zero(
        sizeDOF, sizeDOF);
    RBDLMath::VectorNd C = RBDLMath::VectorNd::Zero(
        sizeDOF);
    //Compute C with inverse dynamics
    acceleration.setZero();
    RBDL::InverseDynamics(_model, 
        position, velocity, acceleration, C, NULL);
    //Compute H
    RBDL::CompositeRigidBodyAlgorithm(
        _model, position, H, false);
    //Add inertial diagonal offsets
    for (size_t i=0;i<(size_t)inertiaOffset.size();i++) {
        H(i, i) += inertiaOffset(i);
    }

    //Solve the linear system
    switch (solver) {
        case RBDLMath::LinearSolverPartialPivLU:
            acceleration = H.partialPivLu().solve(-C + torque);
            break;
        case RBDLMath::LinearSolverColPivHouseholderQR:
            acceleration = H.colPivHouseholderQr().solve(-C + torque);
            break;
        case RBDLMath::LinearSolverHouseholderQR:
            acceleration = H.householderQr().solve(-C + torque);
            break;
        case RBDLMath::LinearSolverLLT:
            acceleration = H.llt().solve(-C + torque);
            break;
        case RBDLMath::LinearSolverFullPivLU:
            acceleration = H.fullPivLu().solve(-C + torque);
            break;
        case RBDLMath::LinearSolverFullPivHouseholderQR:
            acceleration = H.fullPivHouseholderQr().solve(-C + torque);
            break;
        default:
            assert(0);
            break;
    }
    
    return acceleration;
}

Eigen::VectorXd Model::forwardDynamicsContacts(
    RBDL::ConstraintSet& constraints,
    const Eigen::VectorXd& position,
    const Eigen::VectorXd& velocity,
    const Eigen::VectorXd& torque)
{
    //Sanity check
    if (position.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid position vector size");
    }
    if (velocity.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid velocity vector size");
    }
    if (torque.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid acceleration vector size");
    }
   
    RBDLMath::VectorNd QDDot(_model.dof_count);
    RBDL::ForwardDynamicsContactsDirect(
        _model, position, velocity, torque, constraints, QDDot);

    return QDDot;
}

Eigen::VectorXd Model::forwardDynamicsContactsCustom(
    RBDL::ConstraintSet& constraints,
    const Eigen::VectorXd& position,
    const Eigen::VectorXd& velocity,
    const Eigen::VectorXd& torque,
    const Eigen::VectorXd& inertiaOffset,
    RBDLMath::LinearSolver solver)
{
    //Sanity check
    if (position.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid position vector size");
    }
    if (velocity.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid velocity vector size");
    }
    if (torque.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid torque vector size");
    }
    if (inertiaOffset.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid inertia vector size");
    }
    
    //Retrieve sizes
    size_t sizeCst = constraints.size();
    size_t sizeDOF = position.size();
    
    //Compute full H, G matrix and C, gamma 
    //vectors into the constraint set
    //(actually, torque is not used by RBDL)
    RBDL::CalcContactSystemVariables(
        _model, position, velocity, torque, constraints);
    //Add inertial diagonal offsets
    for (size_t i=0;i<(size_t)inertiaOffset.size();i++) {
        constraints.H(i, i) += inertiaOffset(i);
    }

    //Build matrices Ax = b
    //|H Gt| |acc    | = |tau-C|
    //|G  0| |-lambda|   |gamma|
    constraints.A.setZero();
    constraints.b.setZero();
    constraints.A.block(0, 0, sizeDOF, sizeDOF) = 
        constraints.H;
    constraints.A.block(sizeDOF, 0, sizeCst, sizeDOF) = 
        constraints.G;
    constraints.A.block(0, sizeDOF, sizeDOF, sizeCst) = 
        constraints.G.transpose();
    constraints.b.segment(0, sizeDOF) = 
        torque - constraints.C;
    constraints.b.segment(sizeDOF, sizeCst) = 
        constraints.gamma;

    //Solve the linear system
    switch (solver) {
        case RBDLMath::LinearSolverPartialPivLU:
            constraints.x = constraints.
                A.partialPivLu().solve(constraints.b);
            break;
        case RBDLMath::LinearSolverColPivHouseholderQR:
            constraints.x = constraints.
                A.colPivHouseholderQr().solve(constraints.b);
            break;
        case RBDLMath::LinearSolverHouseholderQR:
            constraints.x = constraints.
                A.householderQr().solve(constraints.b);
            break;
        case RBDLMath::LinearSolverFullPivLU:
            constraints.x = constraints.
                A.fullPivLu().solve(constraints.b);
            break;
        case RBDLMath::LinearSolverFullPivHouseholderQR:
            constraints.x = constraints.
                A.fullPivHouseholderQr().solve(constraints.b);
            break;
        default:
            assert(0);
            break;
    }
	
    //Copy computed force
    constraints.force = -constraints.x.segment(sizeDOF, sizeCst);
    //Return computed acceleration
    return constraints.x.segment(0, sizeDOF);
}

Eigen::VectorXd Model::forwardImpulseDynamicsContactsCustom(
    double dt,
    RBDL::ConstraintSet& constraints,
    const Eigen::VectorXd& position,
    const Eigen::VectorXd& velocity,
    const Eigen::VectorXd& torque,
    const Eigen::VectorXd& inertiaOffset,
    RBDLMath::LinearSolver solver)
{
    //Sanity check
    if (position.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid position vector size");
    }
    if (velocity.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid velocity vector size");
    }
    if (torque.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid torque vector size");
    }
    if (inertiaOffset.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid inertia vector size");
    }
    
    //Retrieve sizes
    size_t sizeDOF = position.size();
    size_t sizeCst = constraints.size();

    //Compute full H, G matrix and C
    //vectors into the constraint set
    RBDL::CalcContactSystemVariables(
        _model, position /*+ dt*velocity TODO XXX ??? usefull*/, 
        velocity, torque, constraints);
    //Add inertial diagonal offsets
    for (size_t i=0;i<(size_t)inertiaOffset.size();i++) {
        constraints.H(i, i) += inertiaOffset(i);
    }
    
    //XXX TODO --->
    //Compute H inverse
    auto HQR = constraints.H.colPivHouseholderQr();
    if (!HQR.isInvertible()) {
        throw std::logic_error(
            "RBDLContactLCP non invertible H matrix");
    }
    RBDLMath::MatrixNd Hinv = HQR.inverse();
    //Compute M matrix and d vector
    Eigen::MatrixXd GG1 = constraints.G;
    Eigen::VectorXd gg1 = constraints.gamma;
    Eigen::MatrixXd M1 = GG1 * Hinv * GG1.transpose();
    Eigen::VectorXd d1 = gg1 - GG1 * Hinv * (torque - constraints.C);
    Eigen::VectorXd lll1 = M1.llt().solve(d1);
    /* XXX
    std::cout << "Solve 0 = M*l + d ---> Solve M*l = -d ###########" << std::endl;
    std::cout << "LAMBDA MANUAL: " << lll1.transpose() << std::endl;
    std::cout << "MANUAL gamma: " << gg1.transpose() << std::endl;
    Eigen::VectorXd Zeta = GG1 * velocity;
    std::cout << "ZETA: " << Zeta.transpose() << std::endl;
    */
    //XXX TODO <---

    //Build matrix system
    //|H -dt*Gt| |nextVel| = |dt*(tau - C) + H*oldVel|
    //|G   0   | |lambda |   |         0             |
    constraints.A.setZero();
    constraints.b.setZero();
    constraints.A.block(0, 0, sizeDOF, sizeDOF) = 
        constraints.H;
    constraints.A.block(sizeDOF, 0, sizeCst, sizeDOF) = 
        constraints.G;
    constraints.A.block(0, sizeDOF, sizeDOF, sizeCst) = 
        -dt*constraints.G.transpose();
    constraints.b.segment(0, sizeDOF) = 
        dt*(torque - constraints.C) + constraints.H*velocity;
    
    //Solve the linear system
    switch (solver) {
        case RBDLMath::LinearSolverPartialPivLU:
            constraints.x = constraints.
                A.partialPivLu().solve(constraints.b);
            break;
        case RBDLMath::LinearSolverColPivHouseholderQR:
            constraints.x = constraints.
                A.colPivHouseholderQr().solve(constraints.b);
            break;
        case RBDLMath::LinearSolverHouseholderQR:
            constraints.x = constraints.
                A.householderQr().solve(constraints.b);
            break;
        case RBDLMath::LinearSolverFullPivLU:
            constraints.x = constraints.
                A.fullPivLu().solve(constraints.b);
            break;
        case RBDLMath::LinearSolverFullPivHouseholderQR:
            constraints.x = constraints.
                A.fullPivHouseholderQr().solve(constraints.b);
            break;
        default:
            assert(0);
            break;
    }
            
    //Copy computed force
    constraints.force = constraints.x.segment(sizeDOF, sizeCst);
    //Return computed next velocity
    return constraints.x.segment(0, sizeDOF);
}

Eigen::VectorXd Model::inverseDynamicsContacts(
    RBDL::ConstraintSet& constraints,
    const Eigen::VectorXd& position,
    const Eigen::VectorXd& velocity,
    const Eigen::VectorXd& acceleration,
    const Eigen::VectorXd& inertiaOffset)
{
    //Sanity check
    if (position.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid position vector size");
    }
    if (velocity.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid velocity vector size");
    }
    if (acceleration.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid acceleration vector size");
    }
    if (inertiaOffset.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid inertia vector size");
    }
    
    //Retrieve sizes
    size_t sizeDOF = position.size();
    size_t sizeConstraints = constraints.size();
    
    //Compute full H, G matrix and C, gamma 
    //vectors into the constraint set
    //(actually, torque is not used by RBDL)
    RBDL::CalcContactSystemVariables(
        _model, position, velocity, 
        Eigen::VectorXd::Zero(sizeDOF), constraints);
    //Add inertial diagonal offsets
    for (size_t i=0;i<(size_t)inertiaOffset.size();i++) {
        constraints.H(i, i) += inertiaOffset(i);
    }

    //Build matrix system
    //|H Gt| |acc    | = |tau - C|
    //|G  0| |-lambda|   |gamma  |
    constraints.A.setZero();
    constraints.b.setZero();
    constraints.A.block(0, 0, sizeDOF, sizeDOF) = 
        constraints.H;
    constraints.A.block(sizeDOF, 0, sizeConstraints, sizeDOF) = 
        constraints.G;
    constraints.A.block(0, sizeDOF, sizeDOF, sizeConstraints) = 
        constraints.G.transpose();
    constraints.b.segment(0, sizeDOF) = 
        acceleration;
    constraints.b.segment(sizeDOF, sizeConstraints) = 
        -constraints.force;

    //Compute and retrieve DOF torques
    constraints.x = constraints.A*constraints.b;
    return constraints.x.segment(0, sizeDOF) + constraints.C;
}

Eigen::VectorXd Model::impulseContacts(
    RBDL::ConstraintSet& constraints,
    const Eigen::VectorXd& position,
    const Eigen::VectorXd& velocity)
{
    //Sanity check
    if (position.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid position vector size");
    }
    if (velocity.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid velocity vector size");
    }

    Eigen::VectorXd newVel = velocity;
    RBDL::ComputeContactImpulsesDirect(
        _model, position, velocity, constraints, newVel);

    return newVel;
}

Eigen::VectorXd Model::impulseContactsCustom(
    RBDL::ConstraintSet& constraints,
    const Eigen::VectorXd& position,
    const Eigen::VectorXd& velocity,
    const Eigen::VectorXd& inertiaOffset,
    RBDLMath::LinearSolver solver)
{
    //Sanity check
    if (position.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid position vector size");
    }
    if (velocity.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid velocity vector size");
    }
    if (inertiaOffset.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid inertia vector size");
    }
    
    //Retrieve sizes
    size_t sizeCst = constraints.size();
    size_t sizeDOF = position.size();
    
    //Compute full H, G matrix into the 
    //constraint set Compute H
    RBDL::UpdateKinematicsCustom(
        _model, &position, NULL, NULL);
    RBDL::CompositeRigidBodyAlgorithm(
        _model, position, constraints.H, false);
    //Add inertial diagonal offsets
    for (size_t i=0;i<(size_t)inertiaOffset.size();i++) {
        constraints.H(i, i) += inertiaOffset(i);
    }
    //Compute G
    RBDL::CalcContactJacobian(
        _model, position, constraints, 
        constraints.G, false);

    //Build matrices Ax = b
    //|H Gt| |newVel | = |H*oldVel|
    //|G  0| |impulse|   |    0   |
    constraints.A.setZero();
    constraints.b.setZero();
    constraints.A.block(0, 0, sizeDOF, sizeDOF) = 
        constraints.H;
    constraints.A.block(sizeDOF, 0, sizeCst, sizeDOF) = 
        constraints.G;
    constraints.A.block(0, sizeDOF, sizeDOF, sizeCst) = 
        constraints.G.transpose();
    constraints.b.segment(0, sizeDOF) = 
        constraints.H * velocity;
    constraints.b.segment(sizeDOF, sizeCst) = 
        constraints.v_plus;

    //Solve the linear system
    switch (solver) {
        case RBDLMath::LinearSolverPartialPivLU:
            constraints.x = constraints.
                A.partialPivLu().solve(constraints.b);
            break;
        case RBDLMath::LinearSolverColPivHouseholderQR:
            constraints.x = constraints.
                A.colPivHouseholderQr().solve(constraints.b);
            break;
        case RBDLMath::LinearSolverHouseholderQR:
            constraints.x = constraints.
                A.householderQr().solve(constraints.b);
            break;
        case RBDLMath::LinearSolverFullPivLU:
            constraints.x = constraints.
                A.fullPivLu().solve(constraints.b);
            break;
        case RBDLMath::LinearSolverFullPivHouseholderQR:
            constraints.x = constraints.
                A.fullPivHouseholderQr().solve(constraints.b);
            break;
        default:
            assert(0);
            break;
    }

    //Copy computed impulsed
    constraints.impulse = constraints.x.segment(sizeDOF, sizeCst);
    //Return computed new velocity
    return constraints.x.segment(0, sizeDOF);
}

void Model::resolveContactConstraintLCP(
    RBDL::ConstraintSet& constraints,
    const Eigen::VectorXi& isBilateralConstraint,
    const Eigen::VectorXd& position,
    const Eigen::VectorXd& velocity,
    const Eigen::VectorXd& torque,
    const Eigen::VectorXd& inertiaOffset)
{
    RBDLContactLCP(
        _model, 
        position, 
        velocity, 
        torque, 
        inertiaOffset, 
        constraints,
        isBilateralConstraint);
}
        
void Model::boundingBox(size_t frameIndex, 
    double& sizeX, double& sizeY, double& sizeZ,
    Eigen::Vector3d& center) const
{
    //Return null box for default behaviour
    (void)frameIndex;
    sizeX = 0.0;
    sizeY = 0.0;
    sizeZ = 0.0;
    center = Eigen::Vector3d::Zero();
}

const RBDL::Model& Model::getRBDLModel() const
{
    return _model;
}

size_t Model::bodyIdToFrameIndex(size_t index) const
{
    for (const auto& mapping : _frameIndexToId) {
        if (mapping.second == index) {
            return mapping.first;
        }
    }
    throw std::logic_error("Model invalid RBDL id");
}
size_t Model::frameIndexToBodyId(size_t index) const
{
    return _frameIndexToId.at(index);
}
        
const Eigen::MatrixXd& Model::getInertiaData() const
{
    return _inertiaData;
}
const std::map<std::string, size_t>& Model::getInertiaName() const
{
    return _inertiaName;
}

const Eigen::MatrixXd& Model::getGeometryData() const
{
    return _geometryData;
}
const std::map<std::string, size_t>& Model::getGeometryName() const
{
    return _geometryName;
}

std::string Model::filterJointName(const std::string& name) const
{
    std::string filtered = name;
    size_t pos = filtered.rfind("_link");
    if (pos != std::string::npos) {
        filtered = filtered.substr(0, pos);
    } 

    return filtered;
}
        
std::string Model::filterFrameName(const std::string& name) const
{
    std::string filtered = name;
    size_t pos = filtered.rfind("_link");
    if (pos != std::string::npos) {
        filtered = filtered.substr(0, pos);
    } 
    
    return filtered;
}
        
void Model::initializeModel(RBDL::Model& model, 
    const Eigen::MatrixXd& inertiaData,
    const std::map<std::string, size_t>& inertiaName,
    const Eigen::MatrixXd& geometryData,
    const std::map<std::string, size_t>& geometryName)
{
    //Assign RBDL model
    _model = model;
    _inertiaData = inertiaData;
    _inertiaName = inertiaName;
    _geometryData = geometryData;
    _geometryName = geometryName;
    //Build name-index joint mapping 
    //and VectorLabel structure
    for (size_t i=1;i<_model.mBodies.size();i++) {
        unsigned int virtualDepth = 0;
        std::string filteredName = filterJointName(
            getRBDLBodyName(i, virtualDepth));
        //Handle special case of 6 virtual bodies added by
        //the floating joint
        if (virtualDepth == 5) {
            addDOF(filteredName + "_x");
            addDOF(filteredName + "_y");
            addDOF(filteredName + "_z");
            addDOF(filteredName + "_yaw");
            addDOF(filteredName + "_pitch");
            addDOF(filteredName + "_roll");
            i += 5;
            continue;
        } else if (virtualDepth > 0) {
            std::string verbose1 = RBDL::Utils::GetModelHierarchy(_model);
            std::string verbose2 = RBDL::Utils::GetModelDOFOverview(_model);
            throw std::logic_error(
                "Model virtual body name not implemented: name=" 
                + filteredName + std::string(" depth=") 
                + std::to_string(virtualDepth)
                + std::string("\n")
                + std::string("ModelHierarchy:\n")
                + verbose1
                + std::string("\n")
                + std::string("ModelDOFOverview:\n")
                + verbose2
            );
        }
        addDOF(filteredName);
    }

    //Build name-index frame mapping
    size_t index = 0;
    for (const auto& name : _model.mBodyNameMap) {
        if (name.second == 0) continue;
        std::string filteredName = filterFrameName(name.first);
        _frameIndexToName[index] = filteredName;
        _frameNameToIndex[filteredName] = index;
        _frameIndexToId[index] = name.second;
        index++;
    }
    _frameIndexToName[index] = "origin";
    _frameNameToIndex["origin"] = index;
    _frameIndexToId[index] = 0;
}
        
std::string Model::getRBDLBodyName(size_t bodyId, 
    unsigned int& virtualDepth) const
{
    //If this is a virtual body that was added by a multi dof joint
    if (_model.mBodies[bodyId].mIsVirtual) {
        //If there is not a unique child we do not know what to do
        if (_model.mu[bodyId].size() != 1) {
            return "";
        }
        virtualDepth++;
        return getRBDLBodyName(_model.mu[bodyId][0], virtualDepth);
    }

    return _model.GetBodyName(bodyId);
}
        
void Model::addDOF(const std::string& name)
{
    _vectorDOF.append(name, 0.0);
    _dofNameToIndex[name] = _dofIndexToName.size();
    _dofIndexToName.push_back(name);
    _dofs = RBDLMath::VectorNd::Zero(_vectorDOF.size());
}
        
void Model::loadEigenToLabel()
{
    for (size_t i=0;i<(size_t)_dofs.size();i++) {
        _vectorDOF(_dofIndexToName.at(i)) = _dofs(i);
    }
}

void Model::loadLabelToEigen(const VectorLabel& vect, 
    Eigen::VectorXd& dst, bool setBase)
{
    for (size_t i=0;i<vect.size();i++) {
        const std::string& label = vect.getLabel(i);
        if (
            _dofNameToIndex.count(label) != 0 &&
            (setBase || 
            label.find("base_") == std::string::npos)
        ) {
            dst(_dofNameToIndex.at(label)) = vect(i);
        }
    }
}
 
}


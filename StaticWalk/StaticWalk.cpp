#include <stdexcept>
#include "StaticWalk/StaticWalk.hpp"
#include "Spline/SmoothSpline.hpp"

namespace Leph {

StaticWalk::StaticWalk() :
    _phase(0.0),
    _model(),
    _inverseModel(_model),
    _initCOMZOffset(0.0),
    _initFootYOffset(0.0)
{
    //Set foot model on zero z
    _model.putOnGround();
    //Initialize Inverse Kinematics config
    initIK(_inverseModel, true);
    //Compute initial COM and foot position
    _initCOMZOffset = _model.centerOfMass("origin").z();
    _initFootYOffset = _model.position("left foot tip", "right foot tip").y()/2.0;
}
        
VectorLabel StaticWalk::buildParams() const
{
    return VectorLabel(
        "param:comZOffset", 0.03,
        "param:footZOffset", 0.02,
        "param:step", 0.02,
        "param:lateral", 0.01,
        "param:timePeriod", 10.0,
        "param:singlePhaseRatio", 0.3);
}

double StaticWalk::getPhase() const
{
    return _phase;
}
void StaticWalk::setPhase(double phase)
{
    if (phase < 0.0 || phase > 1.0) {
        throw std::logic_error("StaticWalk invalid phase");
    }
    _phase = phase;
}
        
VectorLabel StaticWalk::initPose(const VectorLabel& params)
{
    //Reset model to zero position
    _model.setDOFZeros();
    _model.putOnGround();

    //Temporary inverse model with fixed
    //trunk orientation
    InverseKinematics tmpInv(_model);
    initIK(tmpInv, false);

    //Reset target position
    tmpInv.targetPosition("left foot") = 
        _model.position("left foot tip", "origin");
    tmpInv.targetPosition("right foot") = 
        _model.position("right foot tip", "origin");
    tmpInv.targetCOM() = _model.centerOfMass("origin");
    
    //Set COM target
    tmpInv.targetCOM().z() -= params("param:comZOffset");

    //Allow multiple iteration to converge to target position
    //since it is a big step
    unsigned int count = 0;
    bool isSuccess;
    do {
        tmpInv.run(0.01, 10);
        isSuccess = checkIKErrors(tmpInv, false);
        count++;
    } while (count < 10 && !isSuccess);
    checkIKErrors(tmpInv, true);
    
    return _model.getDOF();
}
        
VectorLabel StaticWalk::exec(double dt, const VectorLabel& params)
{
    //Single and double support phase length ratio
    double singlePhaseLength = params("param:singlePhaseRatio");
    double doublePhaseLength = 0.5-singlePhaseLength;

    //Build target splines
    //COM
    Leph::SmoothSpline splineCOM;
    splineCOM.addPoint(-doublePhaseLength/2.0, 0.0, 0.0);
    splineCOM.addPoint(doublePhaseLength/2.0, 1.0, 0.0);
    splineCOM.addPoint(doublePhaseLength/2.0+singlePhaseLength, 1.0, 0.0);
    splineCOM.addPoint(0.5+doublePhaseLength/2.0, 0.0, 0.0);
    splineCOM.addPoint(0.5+doublePhaseLength/2.0+singlePhaseLength, 0.0, 0.0);
    splineCOM.addPoint(1.0+doublePhaseLength/2.0, 1.0, 0.0);
    //Foot X
    Leph::SmoothSpline splineFootX;
    splineFootX.addPoint(0.0, -1.0, 0.0);
    splineFootX.addPoint(doublePhaseLength/2.0, -1.0, 0.0);
    splineFootX.addPoint(doublePhaseLength/2.0+singlePhaseLength, 1.0, 0.0);
    splineFootX.addPoint(0.5+doublePhaseLength/2.0, 1.0, 0.0);
    splineFootX.addPoint(0.5+doublePhaseLength/2.0+singlePhaseLength, -1.0, 0.0);
    splineFootX.addPoint(1.0, -1.0, 0.0);
    //Foot Z
    Leph::SmoothSpline splineFootZ;
    splineFootZ.addPoint(0, 0.0, 0.0);
    splineFootZ.addPoint(doublePhaseLength/2.0, 0.0, 0.0);
    splineFootZ.addPoint(doublePhaseLength/2.0+singlePhaseLength/2.0, 1.0, 0.0);
    splineFootZ.addPoint(doublePhaseLength/2.0+singlePhaseLength, 0.0, 0.0);
    splineFootZ.addPoint(1.0, 0.0, 0.0);
    
    //Set feet and COM target
    //Foot X 
    _inverseModel.targetPosition("left foot").x() = 
        params("param:step")*splineFootX.posMod(_phase+0.5);
    _inverseModel.targetPosition("right foot").x() = 
        params("param:step")*splineFootX.posMod(_phase);
    //Foot Y
    _inverseModel.targetPosition("left foot").y() = _initFootYOffset +
        params("param:lateral")*(splineFootX.posMod(_phase)*0.5+0.5);
    _inverseModel.targetPosition("right foot").y() = -_initFootYOffset -
        params("param:lateral")*(splineFootX.posMod(_phase)*0.5+0.5);
    //Foot Z
    _inverseModel.targetPosition("left foot").z() = 
        params("param:footZOffset")*splineFootZ.posMod(_phase+0.5);
    _inverseModel.targetPosition("right foot").z() = 
        params("param:footZOffset")*splineFootZ.posMod(_phase);
    //COM X
    double posCOM = splineCOM.posMod(_phase);
    _inverseModel.targetCOM().x() = 
        _inverseModel.targetPosition("left foot").x()*posCOM
        + _inverseModel.targetPosition("right foot").x()*(1.0-posCOM);
    //COM Y
    _inverseModel.targetCOM().y() = 
        _inverseModel.targetPosition("left foot").y()*posCOM
        + _inverseModel.targetPosition("right foot").y()*(1.0-posCOM);
    //COM Z
    _inverseModel.targetCOM().z() = 
        _initCOMZOffset - params("param:comZOffset");
    
    //Run Inverse Kinematics
    unsigned int count = 0;
    bool isSuccess;
    do {
        _inverseModel.run(0.001, 100);
        isSuccess = checkIKErrors(_inverseModel, false);
        count++;
    } while (count < 100 && !isSuccess);
    checkIKErrors(_inverseModel, true);

    //Update walk phase
    _phase += dt/params("param:timePeriod");
    if (_phase > 1.0) _phase -= 1.0;

    return _model.getDOF();
}
        
bool StaticWalk::checkIKErrors(const InverseKinematics& inv,
    bool throwError) const
{
    if (
        inv.errorPosition("left foot") > 0.001 ||
        inv.errorPosition("right foot") > 0.001 ||
        inv.errorCOM() > 0.001
    ) {
        if (throwError) {
            throw std::runtime_error(
                "StaticWalk IK failed to converge");
        } else {
            return false;
        }
    }

    return true;
}
        
void StaticWalk::initIK(InverseKinematics& inv,
    bool enableTrunkOrientation)
{
    //Add allowed degrees of freedom
    inv.addDOF("left foot roll");
    inv.addDOF("left foot pitch");
    inv.addDOF("left knee");
    inv.addDOF("left hip roll");
    inv.addDOF("left hip pitch");
    inv.addDOF("left hip yaw");
    inv.addDOF("right foot roll");
    inv.addDOF("right foot pitch");
    inv.addDOF("right knee");
    inv.addDOF("right hip roll");
    inv.addDOF("right hip pitch");
    inv.addDOF("right hip yaw");
    inv.addDOF("trunk Tx");
    inv.addDOF("trunk Tz");
    if (enableTrunkOrientation) {
        inv.addDOF("trunk Ty");
    }
    //Add target constraints
    inv.addTargetPosition("left foot", "left foot tip");
    inv.addTargetPosition("right foot", "right foot tip");
    inv.addTargetOrientation("left foot", "left foot tip");
    inv.addTargetOrientation("right foot", "right foot tip");
    inv.addTargetCOM();
    //Add degrees of freedom bounds 
    inv.setUpperBound("left knee", 0.0);
    inv.setUpperBound("right knee", 0.0);
}

}


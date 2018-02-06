#include "Model/RBDLRootUpdate.h"
#include "Model/HumanoidModel.hpp"

namespace Leph {

HumanoidModel::HumanoidModel(
    RobotType type,
    const std::string& frameRoot,
    bool isFloatingBase,
    const Eigen::MatrixXd& inertiaData,
    const std::map<std::string, size_t>& inertiaName,
    const Eigen::MatrixXd& geometryData,
    const std::map<std::string, size_t>& geometryName) :
    Model(),
    _type(type)
{
    //Select used URDF model file
    std::string urdfFile;
    if (_type == SigmabanModel) {
        urdfFile = "sigmaban.urdf";
    } else if (_type == SigmabanPlusModel) {
        urdfFile = "sigmaban_plus.urdf";
    } else if (_type == GrosbanModel) {
        urdfFile = "grosban.urdf";
    }

    //Check for overriden inertia
    bool isInertiaOverride = false;
    Eigen::MatrixXd tmpInertiaData;
    std::map<std::string, size_t> tmpInertiaName;
    if (
        inertiaData.rows() > 0 &&
        (size_t)inertiaData.rows() == inertiaName.size()
    ) {
        isInertiaOverride = true;
        tmpInertiaData = inertiaData;
        tmpInertiaName = inertiaName;
    }

    //Check for overriden geometry
    bool isGeometryOverride = false;
    Eigen::MatrixXd tmpGeometryData;
    std::map<std::string, size_t> tmpGeometryName;
    if (
        geometryData.rows() > 0 &&
        (size_t)geometryData.rows() == geometryName.size()
    ) {
        isGeometryOverride = true;
        tmpGeometryData = geometryData;
        tmpGeometryName = geometryName;
    }

    //Load model from URDF file
    RBDL::Model modelOld;
    if (!RBDL::Addons::URDFReadFromFile(
        urdfFile.c_str(), &modelOld, false,
        &tmpInertiaData, &tmpInertiaName, isInertiaOverride,
        &tmpGeometryData, &tmpGeometryName, isGeometryOverride)
    ) {
        std::runtime_error("Model unable to load URDF file");
    }

    //Select new RBDL body id root
    size_t frameRootId;
    if (frameRoot == "ROOT") {
        frameRootId = 0;
    } else {
        Leph::Model wrappedModelNew(modelOld);
        frameRootId = wrappedModelNew.frameIndexToBodyId(
            wrappedModelNew.getFrameIndex(frameRoot));
    }

    //Update old urdf model with new root frame
    RBDL::Model modelNew =
        Leph::RBDLRootUpdate(modelOld, frameRootId, isFloatingBase);
    //Initialize base model
    Model::initializeModel(modelNew,
        tmpInertiaData, tmpInertiaName,
        tmpGeometryData, tmpGeometryName);

    //Compute leg segments length
    Eigen::Vector3d hipPt = Model::position(
        "right_hip_roll", "origin");
    Eigen::Vector3d kneePt = Model::position(
        "right_knee", "origin");
    Eigen::Vector3d anklePt = Model::position(
        "right_ankle_pitch", "origin");
    Eigen::Vector3d footPt = Model::position(
        "right_foot_tip", "origin");
    _legHipToKnee = (hipPt-kneePt).norm();
    _legKneeToAnkle = (kneePt-anklePt).norm();
    _legAnkleToGround = (anklePt-footPt).norm();
    //Compute standart translation
    //from trunk in zero position
    _trunkToHipLeft = Model::position("left_hip_roll", "trunk");
    _trunkToHipRight = Model::position("right_hip_roll", "trunk");
    _trunkToFootTipLeft = Model::position("left_foot_tip", "trunk");
    _trunkToFootTipRight = Model::position("right_foot_tip", "trunk");

    //Compute neck segments length
    _headYawToPitch = Model::position("head_pitch", "head_yaw").z();
    _headPitchToCameraZ = Model::position("camera", "head_pitch").z();
    _headPitchToCameraX = Model::position("camera", "head_pitch").x();
}

HumanoidModel::~HumanoidModel()
{
}

void HumanoidModel::boundingBox(size_t frameIndex,
    double& sizeX, double& sizeY, double& sizeZ,
    Eigen::Vector3d& center) const
{
    if (_type == SigmabanModel) {
        if (Model::getFrameName(frameIndex) == "left_foot_tip") {
            sizeX = 0.063;
            sizeY = 0.040;
            sizeZ = 0.01;
            center = Eigen::Vector3d(0.000, 0.003, 0.01);
        } else if (Model::getFrameName(frameIndex) == "right_foot_tip") {
            sizeX = 0.063;
            sizeY = 0.040;
            sizeZ = 0.01;
            center = Eigen::Vector3d(0.000, -0.003, 0.01);
        } else {
            Model::boundingBox(frameIndex,
                sizeX, sizeY, sizeZ, center);
        }
    } else if (_type == SigmabanPlusModel) {
        if (Model::getFrameName(frameIndex) == "left_foot_tip") {
            sizeX = 0.098;
            sizeY = 0.051;
            sizeZ = 0.01;
            center = Eigen::Vector3d(0.0244, 0.0135, 0.01);
        } else if (Model::getFrameName(frameIndex) == "right_foot_tip") {
            sizeX = 0.098;
            sizeY = 0.051;
            sizeZ = 0.01;
            center = Eigen::Vector3d(0.0244, -0.0135, 0.01);
        } else {
            Model::boundingBox(frameIndex,
                sizeX, sizeY, sizeZ, center);
        }
    } else if (_type == GrosbanModel) {
        if (Model::getFrameName(frameIndex) == "left_foot_tip") {
            sizeX = 0.1225;
            sizeY = 0.065;
            sizeZ = 0.01;
            center = Eigen::Vector3d(0.0034, 0.036, 0.01);
        } else if (Model::getFrameName(frameIndex) == "right_foot_tip") {
            sizeX = 0.1225;
            sizeY = 0.065;
            sizeZ = 0.01;
            center = Eigen::Vector3d(0.0034, -0.036, 0.01);
        } else {
            Model::boundingBox(frameIndex,
                sizeX, sizeY, sizeZ, center);
        }
    }
}

bool HumanoidModel::legIkLeft(const std::string& frame,
    const Eigen::Vector3d& footPos,
    const Eigen::Matrix3d& rotation,
    double* boundIKDistance)
{
    //LegIK initialization
    LegIK::IK ik(_legHipToKnee,
        _legKneeToAnkle, _legAnkleToGround);
    //Convert foot position from given
    //target to LegIK base
    LegIK::Vector3D legIKTarget = buildTargetPos(
        frame, footPos, true);
    //Convert orientation from given frame
    //to LegIK base
    LegIK::Frame3D legIKMatrix = buildTargetOrientation(
        frame, rotation);

    //Run inverse kinematics
    LegIK::Position result;
    bool isSucess = ik.compute(
        legIKTarget, legIKMatrix, result, boundIKDistance);

    //Update degrees of freedom on success
    if (isSucess) {
        checkNaN(result, legIKTarget, legIKMatrix);
        setIKResult(result, true);
    }

    return isSucess;
}
bool HumanoidModel::legIkRight(const std::string& frame,
    const Eigen::Vector3d& footPos,
    const Eigen::Matrix3d& rotation,
    double* boundIKDistance)
{
    //LegIK initialization
    LegIK::IK ik(_legHipToKnee,
        _legKneeToAnkle, _legAnkleToGround);
    //Convert foot position from given
    //target to LegIK base
    LegIK::Vector3D legIKTarget = buildTargetPos(
        frame, footPos, false);
    //Convert orientation from given frame
    //to LegIK base
    LegIK::Frame3D legIKMatrix = buildTargetOrientation(
        frame, rotation);

    //Run inverse kinematics
    LegIK::Position result;
    bool isSucess = ik.compute(
        legIKTarget, legIKMatrix, result, boundIKDistance);

    //Update degrees of freedom on success
    if (isSucess) {
        checkNaN(result, legIKTarget, legIKMatrix);
        setIKResult(result, false);
    }

    return isSucess;
}

double HumanoidModel::legsLength() const
{
    return -_trunkToFootTipLeft.z();
}

double HumanoidModel::feetDistance() const
{
    return _trunkToHipLeft.y() - _trunkToHipRight.y();
}

Eigen::Vector3d HumanoidModel::getPose()
{
    Eigen::Vector3d pos = selfInFrame("origin");
    return Eigen::Vector3d(
        pos.x(),
        pos.y(),
        orientationYaw("trunk", "origin")
    );
}

Eigen::Vector3d HumanoidModel::trunkSelfOrientation()
{
    Eigen::Matrix3d mat = selfFrameOrientation("trunk");

    //Retrieve YawPitchRoll euler angles from rotation matrix
    //(Manual computing without singular check seems better than
    //Eigen euler angles and with better range)
    Eigen::Vector3d angles;
    //Roll
    angles(0) = atan2(mat(1, 2), mat(2, 2));
    //Pitch
    angles(1) = atan2(-mat(0, 2),
        sqrt(mat(0, 0)*mat(0, 0)
            + mat(0, 1)*mat(0, 1)));
    //Yaw
    angles(2) = Model::orientationYaw("trunk", "origin");

    return angles;
}

Eigen::Matrix3d HumanoidModel::selfFrameOrientation(
    const std::string& frame)
{
    //Compute self frame to trunk pitch/roll rotation
    Eigen::Matrix3d originToTrunk = Model::orientation("trunk", "origin");
    double roll = atan2(originToTrunk(1, 2), originToTrunk(2, 2));
    double pitch = atan2(-originToTrunk(0, 2),
        sqrt(originToTrunk(0, 0)*originToTrunk(0, 0)
            + originToTrunk(0, 1)*originToTrunk(0, 1)));
    //double yaw = atan2(originToTrunk(0, 1), originToTrunk(0, 0));

    //Compute trunk to target frame rotation
    Eigen::Matrix3d trunkToFrame = Model::orientation(frame, "trunk");

    //Build rotation matrix from self base to target frame
    //by using pitch/roll trunk orientation
    Eigen::AngleAxisd pitchRot(-pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rollRot(-roll, Eigen::Vector3d::UnitX());
    Eigen::Matrix3d baseToFrame =
        rollRot.toRotationMatrix() * pitchRot.toRotationMatrix();
    //Then adding trunk to frame orientation
    baseToFrame = trunkToFrame * baseToFrame;

    return baseToFrame;
}
Eigen::Vector3d HumanoidModel::selfFramePosition(
    const std::string& frame)
{
    //Compute self frame rotation state in origin
    double yaw = Model::orientationYaw("trunk", "origin");

    //Compute trunk and frame position in origin
    Eigen::Vector3d trunkPos = Model::position("trunk", "origin");
    Eigen::Vector3d framePos = Model::position(frame, "origin");
    //Project the trunk position on ground
    trunkPos.z() = 0.0;
    //Compute translation vector in origin
    Eigen::Vector3d translationInOrigin = framePos - trunkPos;
    //Rotate the translation into self frame
    Eigen::Vector3d translationInBase =
        Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ())
        .toRotationMatrix() * translationInOrigin;

    return translationInBase;
}

Eigen::Vector3d HumanoidModel::selfInFrame(
    const std::string& name, const Eigen::Vector3d& pos)
{
    //In: self to target in self
    //Out: frame to target in frame

    //Self to frame orientation
    Eigen::Matrix3d mat = selfFrameOrientation(name);
    //Self to frame in frame
    Eigen::Vector3d selfToFrameVect =
        mat*selfFramePosition(name);

    //Self to target in frame
    Eigen::Vector3d selfToTarget = mat*pos;
    //Frame to target in frame
    Eigen::Vector3d frameToTarget = -selfToFrameVect + selfToTarget;

    return frameToTarget;
}
Eigen::Vector3d HumanoidModel::frameInSelf(
    const std::string& name, const Eigen::Vector3d& pos)
{
    //In: frame to target in frame
    //Out: self to target in self

    //Self to frame orientation
    Eigen::Matrix3d mat = selfFrameOrientation(name);
    //Self to frame in self
    Eigen::Vector3d selfToFrameVect = selfFramePosition(name);

    //Frame to target in self
    Eigen::Vector3d frameToTarget = mat.transpose()*pos;
    //Self to target in self
    Eigen::Vector3d selfToTarget = selfToFrameVect + frameToTarget;

    return selfToTarget;
}

Eigen::Vector3d HumanoidModel::cameraPixelToViewVector(
    const CameraParameters& params,
    const Eigen::Vector2d& pixel)
{
    double focalLength = 0.01;
    //Optical center
    Eigen::Vector3d center = Model::position("camera", "origin");
    //Camera orientation
    Eigen::Matrix3d orientation = Model::orientation("camera", "origin");
    orientation.transposeInPlace();

    //Half width and height aperture distance on focal plane
    double widthLen = focalLength*tan(params.widthAperture/2.0);
    double heightLen = focalLength*tan(params.heightAperture/2.0);
    //Pixel width and height distance from optical center
    double pixelWidthPos = pixel.x()*widthLen;
    double pixelHeightPos = pixel.y()*heightLen;

    //Pixel position in world frame
    Eigen::Vector3d pixelPos =
        center
        + focalLength*orientation.col(0)
        - pixelWidthPos*orientation.col(1)
        - pixelHeightPos*orientation.col(2);

    //Unnormalize forward pixel vector
    Eigen::Vector3d forward = pixelPos - center;

    return forward;
}
Eigen::Vector3d HumanoidModel::cameraPanTiltToViewVector(
    const Eigen::Vector2d& anglesPanTilt)
{
    //World to Self yaw
    double pitch = anglesPanTilt.y();
    double yaw = anglesPanTilt.x();
    yaw += Model::orientationYaw("trunk", "origin");

    //Rebuilt transformation matrix
    Eigen::Matrix3d rot =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix()
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();

    return rot.col(0);
}

bool HumanoidModel::cameraViewVectorToWorld(
    const Eigen::Vector3d& viewVector,
    Eigen::Vector3d& pos, double groundHeight)
{
    //Optical center
    Eigen::Vector3d center = Model::position("camera", "origin");
    //Camera orientation
    Eigen::Matrix3d orientation = Model::orientation("camera", "origin");
    orientation.transposeInPlace();

    //Unnormalize forward pixel vector
    Eigen::Vector3d forward = viewVector;

    //Check if the asked point is above the horizon
    bool isBelowHorizon = true;
    if (forward.z() >= -0.0001) {
        forward.z() = -0.0001;
        isBelowHorizon = false;
    }

    //Line abscisse intersection on the plance parallel to the ground of height groundHeight
    double t = (groundHeight - center.z())/forward.z();

    if (t < 0) {
      // Actually, the target is above us
      isBelowHorizon = true;
    }
    //Intersection point
    Eigen::Vector3d pt = center + t*forward;

    pos = pt;
    return isBelowHorizon;
}

bool HumanoidModel::cameraViewVectorToBallWorld(
    const CameraParameters& params,
    const Eigen::Vector3d& viewVector,
    double radius,
    Eigen::Vector3d& ballCenter,
    Eigen::Vector2d* ballCenterPixel,
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>* bordersPixel,
    std::vector<Eigen::Vector3d>* borders)
{
    //Unnormalize forward pixel vector
    Eigen::Vector3d forward = viewVector;

    //Check if the asked point is above the horizon
    bool isBelowHorizon = true;
    if (forward.z() >= -0.0001) {
        forward.z() = -0.0001;
        isBelowHorizon = false;
    }

    //Optical center
    Eigen::Vector3d center = Model::position("camera", "origin");

    //Line abscisse intersection in the ground
    double t = -center.z()/forward.z();

    //Compute ball center using Thales
    //Optical center height in world frame
    double height = fabs(center.z());
    double opticalCenterToPointDist = (t*forward).norm();
    double ballCenterToPointDist;
    if (height < 0.0001) {
        ballCenterToPointDist = radius;
    } else {
        ballCenterToPointDist = opticalCenterToPointDist*radius/height;
    }
    ballCenter =
        center +
        forward.normalized()
        *(opticalCenterToPointDist-ballCenterToPointDist);

    //Compute radial vector from ball center and parallel
    //to projection frame
    Eigen::Vector3d radialVect1 =
        forward.cross(Eigen::Vector3d(0.0, 0.0, 1.0));
    Eigen::Vector3d radialVect2 =
        forward.cross(radialVect1);
    //Normalize them
    radialVect1.normalize();
    radialVect2.normalize();

    //Compute point on the ball view border from camera
    Eigen::Vector3d border1 = ballCenter + radius*radialVect1;
    Eigen::Vector3d border2 = ballCenter - radius*radialVect1;
    Eigen::Vector3d border3 = ballCenter + radius*radialVect2;
    Eigen::Vector3d border4 = ballCenter - radius*radialVect2;

    //Compute in the previous position in pixel spaces
    if (ballCenterPixel != nullptr) {
        cameraWorldToPixel(params, ballCenter, *ballCenterPixel);
    }
    if (bordersPixel != nullptr) {
        Eigen::Vector2d pix;
        cameraWorldToPixel(params, border1, pix);
        bordersPixel->push_back(pix);
        cameraWorldToPixel(params, border2, pix);
        bordersPixel->push_back(pix);
        cameraWorldToPixel(params, border3, pix);
        bordersPixel->push_back(pix);
        cameraWorldToPixel(params, border4, pix);
        bordersPixel->push_back(pix);
    }
    if (borders != nullptr) {
        borders->push_back(border1);
        borders->push_back(border2);
        borders->push_back(border3);
        borders->push_back(border4);
    }

    return isBelowHorizon;
}

Eigen::Vector2d HumanoidModel::cameraViewVectorToPanTilt(
    const Eigen::Vector3d& viewVector)
{
    //Retrieve the orientation self
    Eigen::Matrix3d mat = selfFrameOrientation("origin");
    //And convert view vector from world to self frame
    Eigen::Vector3d viewInSelf = mat.transpose() * viewVector;
    viewInSelf.normalize();

    //Conversion to yaw/pitch extrinsic euler angles
    double yaw = atan2(viewInSelf.y(), viewInSelf.x());
    double pitch = atan2(
        -viewInSelf.z(),
        sqrt(viewInSelf.x()*viewInSelf.x() + viewInSelf.y()*viewInSelf.y()));

    return Eigen::Vector2d(yaw, pitch);
}

Eigen::Vector2d HumanoidModel::cameraPixelToPanTilt(
    const CameraParameters& params,
    const Eigen::Vector2d& pixel,
    Eigen::Vector3d* viewVector)
{
    double focalLength = 0.01;
    //Optical center
    Eigen::Vector3d center = Model::position("camera", "origin");
    //Camera orientation
    Eigen::Matrix3d orientation = Model::orientation("camera", "origin");
    orientation.transposeInPlace();

    //Half width and height aperture distance on focal plane
    double widthLen = focalLength*tan(params.widthAperture/2.0);
    double heightLen = focalLength*tan(params.heightAperture/2.0);
    //Pixel width and height distance from optical center
    double pixelWidthPos = pixel.x()*widthLen;
    double pixelHeightPos = pixel.y()*heightLen;

    //Pixel position in world frame
    Eigen::Vector3d pixelPos =
        center
        + focalLength*orientation.col(0)
        - pixelWidthPos*orientation.col(1)
        - pixelHeightPos*orientation.col(2);

    //Compute pixel position and optical center in
    //robot self frame
    Eigen::Vector3d centerInSelf = frameInSelf("origin", center);
    Eigen::Vector3d pixelInSelf = frameInSelf("origin", pixelPos);

    Eigen::Vector3d viewInSelf = pixelInSelf - centerInSelf;
    viewInSelf.normalize();

    //Conversion to yaw/pitch extrinsic euler angles
    double yaw = atan2(viewInSelf.y(), viewInSelf.x());
    double pitch = atan2(
        -viewInSelf.z(),
        sqrt(viewInSelf.x()*viewInSelf.x() + viewInSelf.y()*viewInSelf.y()));

    //Assigning view vector
    if (viewVector != nullptr) {
        *viewVector = viewInSelf;
    }

    return Eigen::Vector2d(yaw, pitch);
}

bool HumanoidModel::cameraWorldToPixel(
    const CameraParameters& params,
    const Eigen::Vector3d& pos,
    Eigen::Vector2d& pixel)
{
    double focalLength = 0.01;
    //Optical center
    Eigen::Vector3d center = Model::position("camera", "origin");
    //Camera orientation
    Eigen::Matrix3d orientation = Model::orientation("camera", "origin");
    orientation.transposeInPlace();

    //Half width and height aperture distance on focal plane
    double widthLen = focalLength*tan(params.widthAperture/2.0);
    double heightLen = focalLength*tan(params.heightAperture/2.0);

    //Compute the view line projection on camera plane
    Eigen::Vector3d planeCenter = center + focalLength*orientation.col(0);
    Eigen::Vector3d viewVector = pos - center;
    Eigen::Vector3d widthVector = orientation.col(1);
    Eigen::Vector3d heightVector = orientation.col(2);
    //Build left side matrix to be solved
    Eigen::Matrix3d mat;
    mat.col(0) = viewVector;
    mat.col(1) = widthVector;
    mat.col(2) = heightVector;
    auto decomposition = mat.colPivHouseholderQr();

    //Check if the projection is not inversible
    if (!decomposition.isInvertible()) {
        pixel.setZero();
        return false;
    } else {
        //Solve the linear equation
        Eigen::Vector3d solution = decomposition.solve(planeCenter-center);
        //Assign pixel coordinates normalized by frame size
        pixel.x() = solution(1)/widthLen;
        pixel.y() = solution(2)/heightLen;
        //Check if the projection comes from backside
        if (solution(0) < 0.0 || fabs(pixel.x())>1.0 || fabs(pixel.y())>1.0) { //We also return false if the result is outside the image
          return false;
        } else {
            return true;
        }
    }
}

bool HumanoidModel::cameraPanTiltToPixel(
    const CameraParameters& params,
    const Eigen::Vector2d& anglesPanTilt,
    Eigen::Vector2d& pixel)
{
    //Optical center
    Eigen::Vector3d centerInSelf = frameInSelf("camera");

    //Build rotation matrix from extrinsic euler angles Yaw-Pitch
    Eigen::Matrix3d rot =
        Eigen::AngleAxisd(anglesPanTilt(0), Eigen::Vector3d::UnitZ()).toRotationMatrix()
        * Eigen::AngleAxisd(anglesPanTilt(1), Eigen::Vector3d::UnitY()).toRotationMatrix();
    //Build view vector to pixel in self
    Eigen::Vector3d vectInSelf = rot * Eigen::Vector3d(1.0, 0.0, 0.0);
    //Build a target point on the view vector in world
    Eigen::Vector3d pointInWorld = selfInFrame("origin", vectInSelf + centerInSelf);

    //Call WorlToPixel implementation
    return cameraWorldToPixel(params, pointInWorld, pixel);
}

bool HumanoidModel::cameraLookAt(
    const CameraParameters& params,
    const Eigen::Vector3d& posTarget,
    double offsetPixelTilt)
{
    double panDOF;
    double tiltDOF;
    bool isSucess = cameraLookAtNoUpdate(panDOF, tiltDOF,
        params, posTarget, offsetPixelTilt);
    if (isSucess) {
        Model::setDOF("head_yaw", panDOF);
        Model::setDOF("head_pitch", tiltDOF);
        //Update the model when optimization is enabled
        Model::updateDOFPosition();
    }

    return isSucess;
}
bool HumanoidModel::cameraLookAtNoUpdate(
    double& panDOF,
    double& tiltDOF,
    const CameraParameters& params,
    const Eigen::Vector3d& posTarget,
    double offsetPixelTilt)
{
    //Compute view vector in head yaw frame
    Eigen::Vector3d baseCenter = Model::position("head_yaw", "origin");
    Eigen::Matrix3d orientation = Model::orientation("trunk", "origin");
    Eigen::Vector3d viewVector = posTarget - baseCenter;
    Eigen::Vector3d viewVectorInBase = orientation*viewVector;

    //Compute yaw rotation arround Z aligned with the target
    double yaw = atan2(viewVectorInBase.y(), viewVectorInBase.x());
    //Assign head yaw DOF
    panDOF = yaw;

    //Compute target in head_pitch frame fixed
    //to head_yaw frame orientation
    Eigen::Vector3d targetInBase =
        Model::position("origin", "head_yaw", posTarget);
    //Here, the head_yaw (no update) used is not
    //aligned to the target point.
    //The missing yaw orientation is manually
    //computed to not update the model
    double deltaYaw = yaw - Model::getDOF("head_yaw");
    targetInBase = Eigen::AngleAxisd(-deltaYaw, Eigen::Vector3d::UnitZ())
        .toRotationMatrix() * targetInBase;
    targetInBase.z() -= _headYawToPitch;

    //Conversion of target point to polar representation
    double R = targetInBase.norm();
    double gamma = atan2(targetInBase.z(), targetInBase.x());

    //Compute polar distance for camera from pitch joint
    double r = sqrt(pow(_headPitchToCameraZ, 2) + pow(_headPitchToCameraX, 2));
    //Compute angular correction to handle a non null
    //camera X translation offset
    double epsilon = atan(_headPitchToCameraX/_headPitchToCameraZ);

    //Compute height angular offset from pixel space offset
    //offset from optical line)
    double focalLength = 0.01;
    double heightLen = focalLength*tan(params.heightAperture/2.0);
    double pixelHeightPos = offsetPixelTilt*heightLen;
    double beta = -atan(pixelHeightPos/focalLength);
    //Apply angular correction on view angle
    beta += epsilon;

    //Do the math. Geometric method is used (Alkashi, Trigo, Thales).
    //Use Maxima for system of 3 equations, 3 unknown solving
    double B = sin(beta);
    double cosAngle = r/R;
    if (R*R+r*r*B*B-r*r <= 0.0) {
        return false;
    }
    cosAngle = (B*sqrt(R*R+r*r*B*B-r*r)-r*B*B+r)/R;
    if (cosAngle >= 1.0 || cosAngle <= -1.0) {
        return false;
    }
    double alpha = M_PI/2.0 - acos(cosAngle) - gamma;
    //Apply (inverse) angular correction to pitch
    alpha += -epsilon;

    //Assignement head pitch DOF
    tiltDOF = alpha;

    return true;
}

double HumanoidModel::cameraScreenHorizon(
    const CameraParameters& params,
    double screenPosWidth)
{
    double focalLength = 0.01;
    //Optical center
    Eigen::Vector3d center = Model::position("camera", "origin");
    //Camera orientation
    Eigen::Matrix3d orientation = Model::orientation("camera", "origin");
    orientation.transposeInPlace();

    //Half width and height aperture distance on focal plane
    double widthLen = focalLength*tan(params.widthAperture/2.0);
    double heightLen = focalLength*tan(params.heightAperture/2.0);
    //Pixel width distance from optical center
    double pixelWidthPos = screenPosWidth*widthLen;

    //Position in world frame of asked width pixel line
    //at zero height
    Eigen::Vector3d pixelPos = center
        + focalLength*orientation.col(0)
        - pixelWidthPos*orientation.col(1);

    //Get position on vertical line where the vector
    //between the line's point and optical center is
    //horizontal
    double t = (center.z()-pixelPos.z())/orientation.col(2).z();
    //Compute this point in world frame
    Eigen::Vector3d horizonPos = pixelPos + t*orientation.col(2);

    //Conversion to optical plane vertical coordinate
    double horizonScreenHeight =
        (horizonPos-center).dot(orientation.col(2));

    //Convertion to screen normalized height coordinate
    return -horizonScreenHeight/heightLen;
}

LegIK::Vector3D HumanoidModel::buildTargetPos(
    const std::string& frame,
    const Eigen::Vector3d& footPos,
    bool isLeftLeg)
{
    Eigen::Vector3d target;
    if (frame == "foot_tip_init") {
        //Special frame where foot tip in zero position
        target = footPos;
        if (isLeftLeg) {
            target += _trunkToFootTipLeft;
            target -= _trunkToHipLeft;
        } else {
            target += _trunkToFootTipRight;
            target -= _trunkToHipRight;
        }
    } else if (frame == "LegIK") {
        target = footPos;
        //Raw LegIK frame
        //No transformation
    } else {
        target = Model::position(
            frame, "trunk", footPos);
        if (isLeftLeg) {
            target -= _trunkToHipLeft;
        } else {
            target -= _trunkToHipRight;
        }
    }

    //Building LegIK input target position
    //data structure
    LegIK::Vector3D legIKTarget;
    legIKTarget[0] = target(0);
    legIKTarget[1] = target(1);
    legIKTarget[2] = target(2);
    return legIKTarget;
}
LegIK::Frame3D HumanoidModel::buildTargetOrientation(
    const std::string& frame,
    const Eigen::Matrix3d& rotation)
{
    Eigen::Matrix3d rotMatrixTarget = rotation;
    if (frame == "foot_tip_init") {
        //Special frame where foot tip in zero position
        //No conversion
    } else if (frame == "LegIK") {
        //Raw LegIK frame
        //No conversion
    } else {
        rotMatrixTarget *= Model::orientation(frame, "trunk");
    }

    //Building LegIK input target
    //orientation data structure
    LegIK::Frame3D legIKMatrix;
    legIKMatrix[0][0] = rotMatrixTarget(0, 0);
    legIKMatrix[0][1] = rotMatrixTarget(0, 1);
    legIKMatrix[0][2] = rotMatrixTarget(0, 2);
    legIKMatrix[1][0] = rotMatrixTarget(1, 0);
    legIKMatrix[1][1] = rotMatrixTarget(1, 1);
    legIKMatrix[1][2] = rotMatrixTarget(1, 2);
    legIKMatrix[2][0] = rotMatrixTarget(2, 0);
    legIKMatrix[2][1] = rotMatrixTarget(2, 1);
    legIKMatrix[2][2] = rotMatrixTarget(2, 2);
    return legIKMatrix;
}

void HumanoidModel::setIKResult(
    const LegIK::Position& result, bool isLeftLeg)
{
    std::string prefix;
    if (isLeftLeg) {
        prefix = "left_";
    } else {
        prefix = "right_";
    }

    Model::setDOF(prefix+"hip_yaw", result.theta[0]);
    Model::setDOF(prefix+"hip_roll", result.theta[1]);
    Model::setDOF(prefix+"hip_pitch", -result.theta[2]);
    Model::setDOF(prefix+"knee", result.theta[3]);
    Model::setDOF(prefix+"ankle_pitch", -result.theta[4]);
    Model::setDOF(prefix+"ankle_roll", result.theta[5]);
    //Update the model when optimization is enabled
    Model::updateDOFPosition();
}

void HumanoidModel::checkNaN(
    const LegIK::Position& result,
    const LegIK::Vector3D& pos,
    const LegIK::Frame3D& orientation) const
{
    //Check if Nan is returned
    if (
        std::isnan(result.theta[0]) ||
        std::isnan(result.theta[1]) ||
        std::isnan(result.theta[2]) ||
        std::isnan(result.theta[3]) ||
        std::isnan(result.theta[4]) ||
        std::isnan(result.theta[5])
    ) {
        throw std::logic_error("LegIK NaN invalid result. "
            + std::string("theta0=")
            + std::to_string(result.theta[0])
            + std::string(" ")
            + std::string("theta1=")
            + std::to_string(result.theta[1])
                               + std::string(" ")
                               + std::string("theta2=")
                               + std::to_string(result.theta[2])
                               + std::string(" ")
                               + std::string("theta3=")
                               + std::to_string(result.theta[3])
                               + std::string(" ")
                               + std::string("theta4=")
                               + std::to_string(result.theta[4])
                               + std::string(" ")
                               + std::string("theta5=")
                               + std::to_string(result.theta[5])
                               + std::string(" pos=")
                               + pos.pp()
                               + std::string(" orientation=")
                               + orientation.pp()
        );
    }
}

}

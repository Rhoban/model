#ifndef LEPH_HUMANOIDMODEL_HPP
#define LEPH_HUMANOIDMODEL_HPP

#include "Model/Model.hpp"
#include "Model/CameraModel.hpp"
#include "LegIK/LegIK.hpp"

#include <Eigen/StdVector>

namespace Leph {

/**
 * Enum for humanoid robot
 * model type 
 * (Sigmaban or Grosban)
 */
enum RobotType {
    SigmabanModel,
    SigmabanPlusModel,
    GrosbanModel,
};


/**
 * HumanoidModel
 *
 * Inherit Model and implement
 * Sigmaban and Grosban 
 * feet bounding box and inverse
 * kinematics interface
 */
class HumanoidModel : public Model
{
    public:

        /**
         * urdfFile is determined automatically from robotType
         */
        HumanoidModel(
            RobotType type,
            const std::string& frameRoot,
            bool isFloatingBase = true,
            const Eigen::MatrixXd& inertiaData = Eigen::MatrixXd(),
            const std::map<std::string, size_t>& inertiaName = {},
            const Eigen::MatrixXd& geometryData = Eigen::MatrixXd(),
            const std::map<std::string, size_t>& geometryName = {});

        /**
         * Forward initialization to initialize function
         */
        HumanoidModel(
            const std::string& urdfFile,
            RobotType type,
            const std::string& frameRoot,
            bool isFloatingBase = true,
            const Eigen::MatrixXd& inertiaData = Eigen::MatrixXd(),
            const std::map<std::string, size_t>& inertiaName = {},
            const Eigen::MatrixXd& geometryData = Eigen::MatrixXd(),
            const std::map<std::string, size_t>& geometryName = {});
        
        /**
         * Virtual destructor
         */
        virtual ~HumanoidModel();


        /**
         * Initialize the model with given Robot type and root updater and
         * enable floating base 6 DOF if isFloatingBase is true.  If inertia
         * data and name are not empty, given inertia override default model
         * data.  If geometry data and name are not empty, given geometry
         * override default model data.
         */
        void initialize(
            const std::string& urdfFile,
            RobotType type,
            const std::string& frameRoot,
            bool isFloatingBase = true,
            const Eigen::MatrixXd& inertiaData = Eigen::MatrixXd(),
            const std::map<std::string, size_t>& inertiaName = {},
            const Eigen::MatrixXd& geometryData = Eigen::MatrixXd(),
            const std::map<std::string, size_t>& geometryName = {});
        
        /**
         * @Inherit
         * Draw feet bounding box
         */
        void boundingBox(size_t frameIndex, 
            double& sizeX, double& sizeY, double& sizeZ,
            Eigen::Vector3d& center) const override;

        /**
         * Run analytical inverse kinematics LegIK and update
         * Left ot Right legs angles to place the Left or Right
         * foot tip at given position and rotation matrix for orientation
         * with respect to given frame name.
         * "foot tip init" is a special frame name representing a frame 
         * bound to trunk frame with initial (zero angles) 
         * foot tip translation.
         * "LegIK" is the raw reference frame of LegIK 
         * implementation.
         * True is returned if angles are updated and inverse
         * kinematics is sucessful, else false is returned.
         * If not null, boundIKDistance is a signed "distance" 
         * from kinematics bound. If positive, the IK is valid.
         * If negative, the IK is out of bounds.
         */
        bool legIkLeft(const std::string& frame,
            const Eigen::Vector3d& footPos, 
            const Eigen::Matrix3d& rotation = Eigen::Matrix3d::Identity(),
            double* boundIKDistance = nullptr);
        bool legIkRight(const std::string& frame,
            const Eigen::Vector3d& footPos, 
            const Eigen::Matrix3d& rotation = Eigen::Matrix3d::Identity(),
            double* boundIKDistance = nullptr);

        /**
         * Return the initial vertical distance
         * from trunk frame to foot tip frame (Z)
         */
        double legsLength() const;

        /**
         * Return the initial lateral distance
         * between each feet
         */
        double feetDistance() const;

        /**
         * Return planar [x,y,theta] pose of robot self
         * frame expressed in world (origin) frame
         */
        Eigen::Vector3d getPose();

        /**
         * Return extrinsic [roll,pitch,yaw] trunk orientation
         * with respect to self frame (equivalent imu inputs)
         */
        Eigen::Vector3d trunkSelfOrientation();

        /**
         * Return the rotation matrix and translation vector
         * expressing given frame into the robot self frame
         * (flat (pitch/roll) on ground at the vertical of trunk).
         */
        Eigen::Matrix3d selfFrameOrientation(const std::string& frame);
        Eigen::Vector3d selfFramePosition(const std::string& frame);
        Eigen::Affine3d selfFrameTransform(const std::string& frame);

        /**
         * selfInFrame: return the position of the point expressed in self
         * robot frame (zero as default) into given frame name.
         * frameInSelf: return the position of the point expressed in given
         * frame name (zero as default) into robot self frame.
         */
        Eigen::Vector3d selfInFrame(
            const std::string& name, 
            const Eigen::Vector3d& pos = Eigen::Vector3d::Zero());
        Eigen::Vector3d frameInSelf(
            const std::string& name, 
            const Eigen::Vector3d& pos = Eigen::Vector3d::Zero());
        
        /**
         * Convert given pixel in image space [0,width-1]*[0,height-1]
         * or given extrinsic Pan/Tilt euler angles
         * with respect to robot self frame to unnormalized
         * view vector in world frame.
         */
        Eigen::Vector3d cameraPixelToViewVector(
            const CameraModel& cameraModel,
            const Eigen::Vector2d& pixel);
        Eigen::Vector3d cameraPanTiltToViewVector(
            const Eigen::Vector2d& anglesPanTilt);

        /**
         * Compute 3d position in world frame (origin) 
         * of given unnormalized view vector in world frame
         * projected on the ground.
         * params is used camera parameters.
         * pixel is normalized between -1 and 1 relatively
         * to image width and height in screen frame (X, Y).
         * pos is updated position on the ground in
         * world frame.
         * False is returned if asked point is above
         * the horizon and pos is shrink to the horizon line.
         */
        bool cameraViewVectorToWorld(
            const Eigen::Vector3d& viewVector,
            Eigen::Vector3d& pos, double groundHeight = 0.0);

        /**
         * Compute cartesian position in world
         * frame of a ball of given radius view at
         * given view vector in world frame.
         * World cartesian ball center, it coordinate
         * in pixel space and the viewed ball borders in pixel
         * space and borders in cartesian world 
         * are optionnaly computed.
         * False is returned if asked point is above
         * the horizon and pos is shrink to the horizon line.
         */
        bool cameraViewVectorToBallWorld(
            const CameraModel& cameraModel,
            const Eigen::Vector3d& viewVector,
            double radius,
            Eigen::Vector3d& ballCenter,
            Eigen::Vector2d* ballCenterPixel = nullptr,
            std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>* bordersPixel = nullptr,
            std::vector<Eigen::Vector3d>* borders = nullptr);

        /**
         * Compute from given view vector in world
         * frame the pan/tilt angles in radian with
         * respect to self robot frame.
         */
        Eigen::Vector2d cameraViewVectorToPanTilt(
            const Eigen::Vector3d& viewVector);

        /**
         * Compute the view vector in robot self
         * frame from given pixel in normalized 
         * pixel space and return its (pan,tilt) angles
         * in radian with respect to self robot frame.
         * If viewVector is not null, the unit normalized
         * view vector is assigned.
         */
        Eigen::Vector2d cameraPixelToPanTilt(
            const CameraModel& cameraModel,
            const Eigen::Vector2d& pixel,
            Eigen::Vector3d* viewVector = nullptr);

        /**
         * Compute the image position in camera space projected from given point
         * in world frame (origin).
         * False is returned if:
         * - Projected point comes from camera's backside (then pixel is set to (0,0))
         * - Point is outside of image (then pixel is set to a value out of image limits)
         */
        bool cameraWorldToPixel(
            const CameraModel& cameraModel,
            const Eigen::Vector3d& pos,
            Eigen::Vector2d& pixel);
        
        /**
         * Compute 2d pixel position in camera space from given Pan/Tilt
         * extrinsic angles of target view vector in robot self frame.
         *
         * False is returned if:
         * - projected point comes from camera's backside.
         * - projected point is outside of image sensor
         */
        bool cameraPanTiltToPixel(
            const CameraModel& cameraModel,
            const Eigen::Vector2d& anglesPanTilt,
            Eigen::Vector2d& pixel);

        /**
         * Set head yaw and pitch degrees of freedom to center camera on the
         * given target position in world (origin) frame. 
         *
         * In NoUpdate version, the underlying model is not updated and given
         * reference dof are assigned.  If the computation fails, the model is
         * not updated and false is returned.
         */
        bool cameraLookAt(const Eigen::Vector3d& posTarget);
        bool cameraLookAtNoUpdate(
            double& panDOF,
            double& tiltDOF,
            const Eigen::Vector3d& posTarget);

        /**
         * Compute and return the height in pixel normalized coordinate of the
         * horizon line at given width pixel normalized coordinate.  params is
         * used camera parameters.  screenWidth is width (X) pixel coordinate
         * between -1 and 1.
         */
        double cameraScreenHorizon(
            const CameraModel& cameraModel,
            double screenPosWidth);

    private:

        /**
         * Robot type (Sigmaban or Grosban)
         */
        RobotType _type;

        /**
         * Leg segments lengths used by
         * inverse kinematics
         */
        double _legHipToKnee;
        double _legKneeToAnkle;
        double _legAnkleToGround;

        /**
         * Translation from trunk frame
         * to hip frame in Zero position
         * (intersection of hip yaw/pitch/roll axes)
         * and to foot tip in Zero position.
         */
        Eigen::Vector3d _trunkToHipLeft;
        Eigen::Vector3d _trunkToHipRight;
        Eigen::Vector3d _trunkToFootTipLeft;
        Eigen::Vector3d _trunkToFootTipRight;

        /**
         * Neck segment lengts used by
         * camera inverse kinematics
         */
        double _headYawToPitch;
        double _headPitchToCameraZ;
        double _headPitchToCameraX;

        /**
         * Compute and return the IK position reference
         * vector and orientation reference matrix
         * in LegIK specifics structures
         */
        LegIK::Vector3D buildTargetPos(
            const std::string& frame,
            const Eigen::Vector3d& footPos, 
            bool isLeftLeg);
        LegIK::Frame3D buildTargetOrientation(
            const std::string& frame,
            const Eigen::Matrix3d& rotation);

        /**
         * Assign model leg DOF to given IK results
         */
        void setIKResult(
            const LegIK::Position& result, bool isLeftLeg);

        /**
         * Check inverse kinematics computed value
         * and throw an error in case of NaN
         */
        void checkNaN(
            const LegIK::Position& result, 
            const LegIK::Vector3D& pos,
            const LegIK::Frame3D& orientation) const;
};

}

#endif


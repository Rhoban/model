#ifndef LEPH_ODOMETRYDISPLACEMENTMODEL_HPP
#define LEPH_ODOMETRYDISPLACEMENTMODEL_HPP

#include <vector>
#include <Eigen/Dense>

namespace Leph {

/**
 * OdometryDisplacementModel
 *
 * Implement several odometry models
 * for pose displacement correction.
 */
class OdometryDisplacementModel
{
    public:

        /**
         * Different displacement
         * correction models types.
         */
        enum Type {
            DisplacementIdentity = 1,
            DisplacementProportionalXY = 2,
            DisplacementProportionalXYA = 3,
            DisplacementLinearSimpleXY = 4,
            DisplacementLinearSimpleXYA = 5,
            DisplacementLinearFullXY = 6,
            DisplacementLinearFullXYA = 7,
        };

        /**
         * Initialization with
         * displacement model type.
         * Parameters default values 
         * and bounds configuration.
         */
        OdometryDisplacementModel(Type type);

        /**
         * Return current model type
         */
        Type getType() const;

        /**
         * Return current or default
         * parameters for current model type.
         */
        const Eigen::VectorXd& getParameters() const;

        /**
         * Assign given parameters. 
         * If given parameters does not
         * comply with min or max bounds, 
         * a positive distance (for fitness scoring)
         * from given parameters is given.
         * The internal parameters are not updated.
         * Else, the parameters are assigned
         * and zero is returned.
         */
        double setParameters(
            const Eigen::VectorXd& params);

        std::vector<std::string> getParametersNames() const;

        /**
         * Return parameter normalization 
         * coefficients
         */
        const Eigen::VectorXd& getNormalization() const;

        /**
         * Correct and return given relative 
         * displacement [dX,dY,dTheta] using 
         * current model parameters.
         */
        Eigen::Vector3d displacementCorrection(
            const Eigen::Vector3d& diff) const;

        /**
         * Print current parameters on
         * standart output
         */
        void printParameters() const;

    private:

        /**
         * Current model type
         */
        Type _type;

        /**
         * Displacement model parameters
         */
        Eigen::VectorXd _params;

        /**
         * Min and max parameter bounds
         */
        Eigen::VectorXd _minBounds;
        Eigen::VectorXd _maxBounds;
};

}

#endif


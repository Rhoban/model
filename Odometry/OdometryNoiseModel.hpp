#ifndef LEPH_ODOMETRYNOISEMODEL_HPP
#define LEPH_ODOMETRYNOISEMODEL_HPP

#include <random>
#include <Eigen/Dense>

namespace Leph {

/**
 * OdometryNoiseModel
 *
 * Implement several odometry models
 * for gaussian displacement noise.
 */
class OdometryNoiseModel
{
    public:
        
        /**
         * Different noise model types.
         */
        enum Type {
            NoiseDisable = 1,
            NoiseConstant = 2,
            NoiseProportional = 3,
            NoiseLinearSimple = 4,
            NoiseLinearFull = 5,
        };
        
        /**
         * Initialization with
         * noise model type.
         * Parameters default values 
         * and bounds configuration.
         */
        OdometryNoiseModel(Type type);
        
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
         * Generate a gaussian noise over 
         * [dX,dY,dTheta] given displacement 
         * from given random engine and 
         * using current model parameters.
         */
        Eigen::Vector3d noiseGeneration(
            const Eigen::Vector3d& diff,
            std::default_random_engine& engine) const;
        
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
         * Maximum parameter bounds
         */
        Eigen::VectorXd _maxBounds;
};

}

#endif


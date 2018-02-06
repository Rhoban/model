#ifndef LEPH_LOGLIKELIHOODMAXIMIZATION_HPP
#define LEPH_LOGLIKELIHOODMAXIMIZATION_HPP

#include <vector>
#include <functional>
#include <random>
#include <Eigen/Dense>
#include <stdexcept>
#include <libcmaes/cmaes.h>
#include "Model/HumanoidFixedModel.hpp"
#include "Utils/GaussianDistribution.hpp"
#include "Plot/Plot.hpp"

namespace Leph {

/**
 * LogLikelihoodMaximization
 *
 * Model parameters calibration
 * through log likelihood maximization
 * and CMAES.
 * The first template TypeData is the custom 
 * data type associated to observations
 * uses by the scoring function.
 * The second template TypeModel is the custom
 * data type shared by evaluation functions
 * and initialized at scoring beginning.
 */
template <typename TypeData, typename TypeModel>
class LogLikelihoodMaximization
{
    public:

        /**
         * Typedef for model evaluation 
         * function under given parameters.
         * Parameters, custom data and random
         * engine is given.
         * If noRandom is true, the user is expected
         * to not generate any random number
         * and assume no noise.
         */
        typedef std::function<Eigen::VectorXd(
            const Eigen::VectorXd& params,
            const TypeData&,
            bool noRandom,
            TypeModel& model,
            std::default_random_engine& engine)> 
            EvalFunc;

        /**
         * Typedef for bounding user function.
         * The function takes as input a parameters
         * vector and returns an non zero cost if the
         * parameters does not comply bounds.
         */
        typedef std::function<double(
            const Eigen::VectorXd& params)>
            BoundFunc;

        /**
         * Typedef for model initialization
         * function. Takes parameters as input
         * and return a TypeModel shared by evaluation
         * functions.
         */
        typedef std::function<TypeModel(
            const Eigen::VectorXd& params)>
            InitFunc;

        /**
         * Optional transformation function
         * to directly compare final observation 
         * and estimation.
         */
        typedef std::function<Eigen::VectorXd(
            const Eigen::VectorXd& obs)>
            TransformFunc;

        /**
         * Default initialization
         */
        LogLikelihoodMaximization() :
            _params(),
            _normCoef(),
            _observations(),
            _dataContainer(),
            _evalFunc(),
            _boundFunc(),
            _initFunc(),
            _transformFunc(),
            _cyclicObservationDims(),
            _samplingNumber(2),
            _indexesLearning(),
            _indexesTesting()
        {
        }

        /**
         * Assign the initial parameters andù
         * the normalization coefficients
         */
        void setInitialParameters(
            const Eigen::VectorXd& initParams,
            const Eigen::VectorXd& normCoef)
        {
            _params = initParams;
            _normCoef = normCoef;
            if (_params.size() != _normCoef.size()) {
                throw std::logic_error(
                    "LogLikelihoodMaximization invalid params size");
            }
            for (size_t i=0;i<(size_t)_normCoef.size();i++) {
                if (_normCoef(i) <= 0.0) {
                    throw std::logic_error(
                        "LogLikelihoodMaximization negative or null normCoef");
                }
            }
        }

        /**
         * Assign the user evaluation function,
         * the user bounding patameters function and 
         * the model initialization function.
         */
        void setUserFunctions(
            EvalFunc func1, 
            BoundFunc func2, 
            InitFunc func3,
            TransformFunc func4 = TransformFunc())
        {
            _evalFunc = func1;
            _boundFunc = func2;
            _initFunc = func3;
            if (func4) {
                _transformFunc = func4;
            } 
        }

        /**
         * Assign the cyclic dimensions for observations
         */
        void setCyclicObsDims(const Eigen::VectorXi & cyclicDims)
        {
            _cyclicObservationDims = cyclicDims;          
        }

        /**
         * Add an observation vector and
         * associated custom data
         */
        void addObservation(
            const Eigen::VectorXd& obs,
            const TypeData& data)
        {
            //Check size
            if (
                _observations.size() > 0 && 
                _observations.front().size() != obs.size()
            ) {
                throw std::logic_error(
                    "LogLikelihoodMaximization invalid size");
            }
            //Append the observation
            _observations.push_back(obs);
            _dataContainer.push_back(data);
        }

        void runOptimization(
            unsigned int samplingNumber,
            double learningDataRatio,
            unsigned int maxIterations,
            unsigned int restart,
            unsigned int populationSize = 10,
            double sigma = -1.0,
            unsigned int elitismLevel = 0,
            Leph::Plot* plot = nullptr)
        {
            //Check ratio
            if (learningDataRatio <= 0.0 || learningDataRatio >= 1.0) {
                throw std::logic_error(
                    "LogLikelihoodMaximization invalid learning ratio");
            }

            //Shuffle learning and testing set
            unsigned int learningSize = 
                std::floor(learningDataRatio*_observations.size());
            runOptimization(samplingNumber, learningSize,
                            maxIterations, restart, populationSize,
                            sigma, elitismLevel, plot);
        }


        /**
         * Start and run CMA-ES parameters
         * optimization with given configuration
         */
        void runOptimization(
            unsigned int samplingNumber,
            unsigned int learningSize,
            unsigned int maxIterations,
            unsigned int restart,
            unsigned int populationSize = 10,
            double sigma = -1.0,
            unsigned int elitismLevel = 0,
            Leph::Plot* plot = nullptr)
        {
            //Check size
            if (_observations.size() < 4) {
                throw std::logic_error(
                    "LogLikelihoodMaximization not enough observations");
            }
            if (learningSize <= 0) {
                throw std::logic_error(
                    "LogLikelihoodMaximization invalid learning size: "
                    + std::to_string(learningSize));
            }
            unsigned int validationSize = _observations.size() - learningSize;
            if (validationSize < 2) {
              throw std::logic_error(
                "RunOptimization require at least 2 validation observations");
            }
            std::vector<size_t> learningIndices, testIndices;
            dataDispatch(learningSize, _observations.size(),
                         learningIndices, testIndices);
            runOptimization(samplingNumber, learningIndices, testIndices,
                            maxIterations, restart, populationSize,
                            sigma, elitismLevel, plot);
        }

        void runOptimization(
            unsigned int samplingNumber,
            const std::vector<size_t> & learningIndices,
            const std::vector<size_t> & testIndices,
            unsigned int maxIterations,
            unsigned int restart,
            unsigned int populationSize = 10,
            double sigma = -1.0,
            unsigned int elitismLevel = 0,
            Leph::Plot* plot = nullptr)
        {
            // Assign test and learn sets
            _indexesLearning = learningIndices;
            _indexesTesting = testIndices;
            
            //Assign sampling number
            _samplingNumber = samplingNumber;

            //Iteration counter
            unsigned long iterations = 1;
            
            //Fitness function
            libcmaes::FitFuncEigen fitness = 
                [this]
                (const Eigen::VectorXd& params) 
                {
                    return this->scoreFitness(
                        params.array() * this->_normCoef.array(), 
                        false);
                };
            
            //Progress function
            libcmaes::ProgressFunc<
                libcmaes::CMAParameters<>, libcmaes::CMASolutions> progress = 
                [this, &iterations, plot]
                (const libcmaes::CMAParameters<>& cmaparams, 
                 const libcmaes::CMASolutions& cmasols)
                {
                    //Empty case
                    if (cmasols.get_best_seen_candidate()
                        .get_x_dvec().size() == 0
                    ) {
                        //Call default CMA-ES default progress function
                        return libcmaes::CMAStrategy<libcmaes::CovarianceUpdate>
                            ::_defaultPFunc(cmaparams, cmasols);
                    }
                    //Retrieve current best parameters
                    Eigen::VectorXd params = this->_normCoef.array() * 
                        cmasols.get_best_seen_candidate().get_x_dvec().array();
                    if (iterations%10 == 0) {
                        double meanLearn;
                        double varLearn;
                        double meanTest;
                        double varTest;
                        double minLearn;
                        double maxLearn;
                        double minTest;
                        double maxTest;
                        this->testParameters(params, 
                            meanLearn, varLearn, 
                            minLearn, maxLearn, 
                            meanTest, varTest, 
                            minTest, maxTest);
                        double scoreLearn = this->scoreFitness(params, false);
                        double scoreTest = this->scoreFitness(params, true);
                        std::cout << "============" << std::endl;
                        std::cout 
                            << "Iteration=" << iterations
                            << " dim=" << params.size() 
                            << " LearnSize=" 
                            << this->_indexesLearning.size() 
                            << " TestSize=" 
                            << this->_indexesTesting.size() 
                            << std::endl;
                        std::cout << "Learn: score=" << scoreLearn 
                            << " mean=" << meanLearn 
                            << " stdDev=" << std::sqrt(varLearn) 
                            << " min=" << minLearn
                            << " max=" << maxLearn
                            << std::endl;
                        std::cout << "Test : score=" << scoreTest 
                            << " mean=" << meanTest 
                            << " stdDev=" << std::sqrt(varTest) 
                            << " min=" << minTest
                            << " max=" << maxTest
                            << std::endl;
                        std::cout << "============" << std::endl;
                        if (plot != nullptr) {
                            plot->add({
                                "iteration", (double)iterations,
                                "score_learn", scoreLearn,
                                "score_test", scoreTest,
                                "mean_learn", meanLearn,
                                "mean_test", meanTest,
                                "min_learn", minLearn,
                                "min_test", minTest,
                                "max_learn", maxLearn,
                                "max_test", maxTest,
                            });
                        }
                    }
                    iterations++;
                    //Retrieve best Trajectories and score
                    //Call default CMA-ES default progress function
                    return libcmaes::CMAStrategy<libcmaes::CovarianceUpdate>
                        ::_defaultPFunc(cmaparams, cmasols);
                };
            
            //CMAES initialization
            libcmaes::CMAParameters<> cmaparams(
                _params.array() / _normCoef.array(),
                sigma, populationSize);
            cmaparams.set_quiet(false);
            cmaparams.set_mt_feval(true);
            cmaparams.set_str_algo("abipop");
            cmaparams.set_elitism(elitismLevel);
            cmaparams.set_restarts(restart);
            cmaparams.set_max_iter(maxIterations);

            //Run optimization
            libcmaes::CMASolutions cmasols = 
                libcmaes::cmaes<>(fitness, cmaparams, progress);

            //Retrieve best Trajectories and score
            _params = cmasols.get_best_seen_candidate().get_x_dvec()
                .array() * _normCoef.array();
            std::cout << "Iterations: " << iterations << std::endl;
            std::cout << "Dimensions: " << _params.size() << std::endl;
            std::cout 
                << "LearnSize: " 
                << _indexesLearning.size() 
                << " TestSize: " 
                << _indexesTesting.size() 
                << std::endl;
            std::cout << "BestParams: " << _params.transpose() << std::endl;
            std::cout << "BestScoreLearning: " 
                << scoreFitness(_params, false) << std::endl;
            std::cout << "BestScoreTesting:  " 
                << scoreFitness(_params, true) << std::endl;
            double meanLearn;
            double varLearn;
            double meanTest;
            double varTest;
            double minLearn;
            double maxLearn;
            double minTest;
            double maxTest;
            testParameters(_params, 
                meanLearn, varLearn, 
                minLearn, maxLearn, 
                meanTest, varTest, 
                minTest, maxTest);
            std::cout << "IterationsNumber: " << iterations << std::endl;
            std::cout 
                << "MeanLearn: " << meanLearn 
                << " stdDevLearn: " << std::sqrt(varLearn) 
                << " min: " << minLearn
                << " max: " << maxLearn
                << std::endl;
            std::cout 
                << "MeanTest:  " << meanTest 
                << " stdDevTest:  " << std::sqrt(varTest) 
                << " min: " << minTest
                << " max: " << maxTest
                << std::endl;
        }
        
        /**
         * Retrieve optimized parameters
         */
        const Eigen::VectorXd& getParameters() const
        {
            return _params;
        }

        /**
         * Compute raw scalar error between
         * estimation and observation over learning
         * and testing set. Return learning and testing 
         * error mean and variance and min/max error 
         * into given references.
         */
        void testParameters(const Eigen::VectorXd& params, 
            double& meanLearn, double& varLearn,
            double& minLearn, double& maxLearn,
            double& meanTest, double& varTest,
            double& minTest, double& maxTest) const
        {
            // If the parameters are out of bound, it is forbidden to test
            if (_boundFunc(params) > 0) {
              //TODO: set all values to nan?
              return;
            }

            //Random device initialization
            std::random_device rd;
            std::default_random_engine engine(rd());
            //Model initialization
            TypeModel model = _initFunc(params);

            //Compute mean and variance over
            //raw learning and testing error
            //(between unnoisy estimation and observation)
            double sumErrorLearn = 0.0;
            double sum2ErrorLearn = 0.0;
            double sumErrorTest = 0.0;
            double sum2ErrorTest = 0.0;
            minLearn = -1.0;
            maxLearn = -1.0;
            minTest = -1.0;
            maxTest = -1.0;
            for (size_t i=0;i<_indexesLearning.size();i++) {
                size_t indexSet = _indexesLearning[i];
                Eigen::VectorXd estimate = _evalFunc(
                    params, _dataContainer[indexSet], true, model, engine);
                //Compute distance between 
                //observation and estimation
                double cmpError = 0.0;
                if (_transformFunc) {
                    //If available, apply user transformation
                    //before comparison
                    Eigen::VectorXd error = 
                        _transformFunc(estimate) 
                        - _transformFunc(_observations[indexSet]);
                    cmpError = error.norm();
                } else {
                    Eigen::VectorXd error = 
                        estimate - _observations[indexSet];
                    cmpError = error.norm();
                }
                sumErrorLearn += cmpError;
                sum2ErrorLearn += pow(cmpError, 2);
                if (minLearn < 0.0 || minLearn > cmpError) {
                    minLearn = cmpError;
                }
                if (maxLearn < 0.0 || maxLearn < cmpError) {
                    maxLearn = cmpError;
                }
            }
            for (size_t i=0;i<_indexesTesting.size();i++) {
                size_t indexSet = _indexesTesting[i];
                Eigen::VectorXd estimate = _evalFunc(
                    params, _dataContainer[indexSet], true, model, engine);
                //Compute distance between 
                //observation and estimation
                double cmpError = 0.0;
                if (_transformFunc) {
                    //If available, apply user transformation
                    //before comparison
                    Eigen::VectorXd error = 
                        _transformFunc(estimate) 
                        - _transformFunc(_observations[indexSet]);
                    cmpError = error.norm();
                } else {
                    Eigen::VectorXd error = 
                        estimate - _observations[indexSet];
                    cmpError = error.norm();
                }
                sumErrorTest += cmpError;
                sum2ErrorTest += pow(cmpError, 2);
                if (minTest < 0.0 || minTest > cmpError) {
                    minTest = cmpError;
                }
                if (maxTest < 0.0 || maxTest < cmpError) {
                    maxTest = cmpError;
                }
            }
            meanLearn = 
                sumErrorLearn/(double)_indexesLearning.size();
            meanTest = 
                sumErrorTest/(double)_indexesTesting.size();
            varLearn = 
                sum2ErrorLearn/(double)_indexesLearning.size() 
                - pow(meanLearn, 2);
            varTest = 
                sum2ErrorTest/(double)_indexesTesting.size() 
                - pow(meanTest, 2);
        }

        /**
         * Score given parameters using 
         * the user observations.
         * If useTestData is true, the testing set
         * is used instead of learning set.
         */
        double scoreFitness(
            const Eigen::VectorXd& params, bool useTestData) const
        {
            const std::vector<size_t>& usedSet = 
                useTestData ? _indexesTesting : _indexesLearning;

            //Check parameters
            double costBound = _boundFunc(params);
            if (costBound > 0) {
                return costBound;
            }
            
            //Random device initialization
            std::random_device rd;
            std::default_random_engine engine(rd());
            
            //Model initialization
            TypeModel model = _initFunc(params);
    
            //Iterate over all observations
            double logLikelihood = 0.0;
            for (size_t i=0;i<usedSet.size();i++) {
                size_t indexSet = usedSet[i];
                double tmpScore = scoreParametersLogLikelihood(
                    params, _observations[indexSet], 
                    _dataContainer[indexSet], model, engine);
                logLikelihood += tmpScore;
            }

            //Return normalized inversed score
            //(minimization)
            return -logLikelihood/(double)usedSet.size();
        }

    private:

        /**
         * Currently best found parameters
         */
        Eigen::VectorXd _params;

        /**
         * Parameters normalization 
         * coefficients
         */
        Eigen::VectorXd _normCoef;

        /**
         * The set of observations and 
         * custom user data associated 
         * with each observations
         */
        std::vector<Eigen::VectorXd> _observations;
        std::vector<TypeData> _dataContainer;

        /**
         * User model evaluation function,
         * user bounding parameter function
         * and model initialization function.
         */
        EvalFunc _evalFunc;
        BoundFunc _boundFunc;
        InitFunc _initFunc;

        /**
         * Tranformation function for
         * direct comparaison between 
         * observation and estimation
         */
        TransformFunc _transformFunc;

        /**
         * Cyclicity of observations
         * val[i] != 0 -> dim 'i' is cyclic
         */
        Eigen::VectorXi _cyclicObservationDims;

        /**
         * The number of time the user function
         * is called for each observations
         */
        unsigned int _samplingNumber;

        /**
         * Container of Learning and Testing 
         * observations indexes observations
         */
        std::vector<size_t> _indexesLearning;
        std::vector<size_t> _indexesTesting;

        /**
         * Sample the user evaluation function 
         * samplingNumber times using given parameters.
         * Estimate the resulting multivariate gaussian
         * distribution and compute the log maginal 
         * likelihood with respect to the given observation.
         */
        double scoreParametersLogLikelihood(
            const Eigen::VectorXd& params,
            const Eigen::VectorXd& obs,
            const TypeData& data,
            TypeModel& model,
            std::default_random_engine& engine) const
        {
            //Sample user function and store
            //the computed estimations
            std::vector<Eigen::VectorXd> estimations;
            for (unsigned int k=0;k<_samplingNumber;k++) {
                //Call user function
                Eigen::VectorXd estimate = _evalFunc(
                    params, data, false, model, engine);
                //Check size
                if (estimate.size() != obs.size()) {
                    throw std::logic_error(
                        "LogLikelihoodMaximization " +
                        std::string("user function invalid size"));
                }
                //Save it
                estimations.push_back(estimate);
            }
            
            //Estimate the gaussian distribution
            Leph::GaussianDistribution dist;
            dist.fit(estimations, _cyclicObservationDims);

            //Compute the log likelihood
            return dist.logProbability(obs);
        }

        /**
         * Randomly separate set {0, 1, ..., nbObservations-1} in two sets
         * The learning set of size 'learningSize'
         * the testing set of size 'nbObservations - learningSize'
         * Resulting indices are placed in the provided vectors
         */
        void dataDispatch(size_t learningSize,
                          size_t nbObservations,
                          std::vector<size_t> & learningIndexes,
                          std::vector<size_t> & testingIndexes)
        {
            learningIndexes.clear();
            testingIndexes.clear();
            if (learningSize > _observations.size()-1) {
                throw std::logic_error(
                    "LogLikelihoodMaximization invalid learning size");
            }
            //Random device initialization
            std::random_device rd;
            std::default_random_engine engine(rd());
            //Create the vector of sorted indexes
            std::vector<size_t> tmpIndexes;
            for (size_t i=0;i<nbObservations;i++) {
                tmpIndexes.push_back(i);
            }
            //Shuffle the vector
            std::shuffle(tmpIndexes.begin(), tmpIndexes.end(), engine);
            //Split under learning and testing set
            for (size_t i=0;i<learningSize;i++) {
                learningIndexes.push_back(tmpIndexes[i]);
            }
            for (size_t i=learningSize;i<nbObservations;i++) {
                testingIndexes.push_back(tmpIndexes[i]);
            }
        }
};

}

#endif


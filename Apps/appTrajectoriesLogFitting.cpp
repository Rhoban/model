#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <stdexcept>
#include <libcmaes/cmaes.h>
#include "Model/HumanoidFixedModel.hpp"
#include "Model/HumanoidSimulation.hpp"
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "TrajectoryGeneration/TrajectoryGeneration.hpp"
#include "TrajectoryDefinition/CommonTrajs.h"
#include "TrajectoryDefinition/TrajKickSingle.hpp"
#include "TrajectoryDefinition/TrajKickDouble.hpp"
#include "Utils/FileModelParameters.h"
#include "Utils/AxisAngle.h"
#include "Types/MapSeries.hpp"
#include "Model/NamesModel.h"

#ifdef LEPH_VIEWER_ENABLED
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Plot/Plot.hpp"
#endif

/**
 * Global CMA-ES configuration
 */
static const int cmaesElitismLevel = 0;
static const unsigned int cmaesMaxIterations = 2000;
static const unsigned int cmaesRestarts = 5;
static const unsigned int cmaesLambda = 100;
static const double cmaesSigma = -1.0;

/**
 * Score and return a distance error
 * between generated trajectory from given
 * parameters and given logs goal trajectory
 */
double scoreTrajectoryFitting(
    const Eigen::VectorXd& params,
    const Leph::TrajectoryParameters& trajParams,
    const Leph::TrajectoryGeneration& generator,
    const Leph::MapSeries& logs,
    double timeMin,
    double timeMax,
    bool isSimulation,
    int verboseLevel)
{
    //Check parameters validity
    double cost = generator.checkParameters(params);
    if (cost > 0.0) {
        return cost;
    }

    //Load model parameters
    Eigen::MatrixXd jointData;
    std::map<std::string, size_t> jointName;
    Eigen::MatrixXd inertiaData;
    std::map<std::string, size_t> inertiaName;
    Eigen::MatrixXd geometryData;
    std::map<std::string, size_t> geometryName;
    if (generator.modelParametersPath() != "") {
        Leph::ReadModelParameters(
            generator.modelParametersPath(),
            jointData, jointName,
            inertiaData, inertiaName,
            geometryData, geometryName);
    }

    //Sigmaban fixed model
    Leph::HumanoidFixedModel modelGoal(
        Leph::SigmabanModel, 
        inertiaData, inertiaName, 
        geometryData, geometryName);
    //Initialize full humanoid model
    //simulation with overrided 
    //inertia and geometry data
    Leph::HumanoidSimulation sim(
        Leph::SigmabanModel, 
        inertiaData,
        inertiaName,
        geometryData,
        geometryName);
    if (isSimulation) {
        //Assign joint parameters
        for (const std::string& name : Leph::NamesDOF) {
            if (jointName.count(name) > 0) {
                sim.jointModel(name).setParameters(
                    jointData.row(jointName.at(name)).transpose());
            }
        }
        //State initialization
        for (const std::string& name : Leph::NamesDOF) {
            //Initialization pos, vel and goal
            sim.setPos(name, logs.get("read:" + name, timeMin));
            sim.setGoal(name, logs.get("read:" + name, timeMin));
            sim.setVel(name, 0.0);
            //Reset backlash state
            sim.jointModel(name).resetBacklashState();
        }
        for (const std::string& name : Leph::NamesBase) {
            //Init base vel
            sim.setVel(name, 0.0); 
        }
        //Init model state
        sim.putOnGround(
            Leph::HumanoidFixedModel::LeftSupportFoot);
        sim.putFootAt(0.0, 0.0, 
            Leph::HumanoidFixedModel::LeftSupportFoot);
        //Run small time 0.5s for 
        //waiting stabilization (backlash)
        for (int k=0;k<500;k++) {
            sim.update(0.001);
        }
    }
    
    //Compute the trajectory
    Leph::Trajectories traj = generator.generateTrajectory(params);

#ifdef LEPH_VIEWER_ENABLED
    Leph::ModelViewer* viewer = nullptr;
    if (verboseLevel >= 1) {
        viewer = new Leph::ModelViewer(1200, 900);
    }
    Leph::Plot plot;
#endif
    
    //Loop over actual goal time length
    for (double t=timeMin;t<=timeMax;t+=0.01) {
#ifdef LEPH_VIEWER_ENABLED
        if (verboseLevel >= 1) {
            if (!viewer->update()) {
                break;
            }
        }
#endif
        //Compute DOF targets
        Eigen::Vector3d trunkPos;
        Eigen::Vector3d trunkAxis;
        Eigen::Vector3d footPos;
        Eigen::Vector3d footAxis;
        bool isDoubleSupport;
        Leph::HumanoidFixedModel::SupportFoot supportFoot;
        Leph::TrajectoriesTrunkFootPos(
            t-timeMin, traj,
            trunkPos, trunkAxis,
            footPos, footAxis);
        Leph::TrajectoriesSupportFootState(
            t-timeMin, traj, isDoubleSupport, supportFoot);
        //Check Cartesian State
        if (!isSimulation) {
            double costState = generator.checkState(
                params, t-timeMin, 
                trunkPos, trunkAxis, 
                footPos, footAxis);
            if (costState > 0.0) {
                cost += 1000.0 + costState;
                continue;
            }
        }
        //Compute kinematics
        double boundIKDistance = 0.0;
        bool isIKSuccess = modelGoal.trunkFootIK(
            supportFoot,
            trunkPos,
            Leph::AxisToMatrix(trunkAxis),
            footPos,
            Leph::AxisToMatrix(footAxis),
            &boundIKDistance);
        //Cost near IK bound
        double boundIKThreshold = 1e-2;
        if (boundIKDistance < boundIKThreshold) {
            cost += 1000.0 
                + 1000.0*(boundIKThreshold - boundIKDistance);
        }
        if (!isIKSuccess) {
            cost += 1000.0;
            continue;
        }
        //Check Joint DOF
        if (!isSimulation) {
            double costDOF = generator.checkDOF(
                params, t, modelGoal.get());
            if (costDOF > 0.0) {
                cost += 1000.0 + costDOF;
                continue;
            }
        }
        //Run simulator
        if (isSimulation) {
            //Assign motor goal
            for (const std::string& name : Leph::NamesDOF) {
                sim.setGoal(name, modelGoal.get().getDOF(name));
            }
            //Run simulation
            for (int k=0;k<10;k++) {
                sim.update(0.001);
            }
            //Score the error from simulation
            for (const std::string& name : Leph::NamesDOFLeg) {
                double error = 
                    sim.model().getDOF(name) 
                    - logs.get("read:"+name, t);
                cost += pow(error, 2);
            }
        } else {
            //Score the error from log goal
            for (const std::string& name : Leph::NamesDOFLeg) {
                double error = 
                    modelGoal.get().getDOF(name) 
                    - logs.get("goal:"+name, t);
                cost += pow(error, 2);
            }
        }
        //Verbose
#ifdef LEPH_VIEWER_ENABLED
        if (verboseLevel >= 1) {
            for (const std::string& name : Leph::NamesDOFLeg) {
                plot.add({
                    "t", t-timeMin,
                    "goal:"+name, logs.get("goal:"+name, t),
                    "fitted:"+name, modelGoal.get().getDOF(name),
                });
            }
            Leph::ModelDraw(modelGoal.get(), *viewer, 1.0);
            if (isSimulation) {
                Leph::ModelDraw(sim.model(), *viewer, 1.0);
            }
        }
#endif
    }
#ifdef LEPH_VIEWER_ENABLED
    if (verboseLevel >= 1) {
        delete viewer;
    }
    for (const std::string& name : Leph::NamesDOFLegLeft) {
        plot.plot("t", "goal:"+name);
        plot.plot("t", "fitted:"+name);
    }
    plot.render();
    for (const std::string& name : Leph::NamesDOFLegRight) {
        plot.plot("t", "goal:"+name);
        plot.plot("t", "fitted:"+name);
    }
    plot.render();
#endif

    return cost;
}

/**
 * Optimize a trajectory parameters to fit
 * given log goal or given log read though
 * simulation
 */
int main(int argc, char** argv)
{
    //Parse user inputs
    if (argc < 5) {
        std::cout << "./app GOAL trajectoryName " << 
            "inLog.mapseries outPrefix " << 
            "[SEED] [seed.params] [MODEL] [inPath.moelparams]" << std::endl;
        std::cout << "./app SIM  trajectoryName " << 
            "inLog.mapseries outPrefix " <<
            "[SEED] [seed.params] [MODEL] [inPath.modelparams]" << std::endl;
        std::cout << "Available trajectories:" << std::endl;
        std::cout << "-- kicksingle" << std::endl;
        std::cout << "-- kickdouble" << std::endl;
        return 1;
    }
    std::string mode = argv[1];
    std::string trajName = argv[2];
    std::string logPath = argv[3];
    std::string outPath = argv[4];
    std::string modelParamsPath = "";
    std::string seedParamsPath = "";
    bool isSimulation = false;
    if (mode != "GOAL" && mode != "SIM") {
        std::cout << "Invalid mode: " << mode << std::endl;
        return 1;
    } else if (mode == "GOAL") {
        std::cout << "Fitting GOAL goal trajectory" << std::endl;
        isSimulation = false;
    } else {
        std::cout << "Fitting SIM read and simulated trajectory" << std::endl;
        isSimulation = true;
    }
    int argIndex = 5;
    if (argc >= argIndex+2 && std::string(argv[argIndex]) == "SEED") {
        seedParamsPath = argv[argIndex+1];
        std::cout << "Using seed parameters: " 
            << seedParamsPath << std::endl;
        argIndex += 2;
    }
    if (argc >= argIndex+2 && std::string(argv[argIndex]) == "MODEL") {
        modelParamsPath = argv[argIndex+1];
        std::cout << "Using model parameters: " 
            << modelParamsPath << std::endl;
    }
    
    //Load data into MapSeries
    Leph::MapSeries logs;
    logs.importData(logPath);
    std::cout << "Loading log " 
        << logPath << ": "
        << logs.dimension() << " series from " 
        << logs.timeMin() << "s to " 
        << logs.timeMax() << "s with length "
        << logs.timeMax()-logs.timeMin() << "s" << std::endl;
    double timeMin = logs.timeMin();
    double timeMax = logs.timeMax();

    //Find actual goal trajectory length
    double goalTimeMin = timeMax;
    double goalTimeMax = timeMin;
    for (const std::string& name : Leph::NamesDOFLeg) {
        double initVal = logs.get("goal:"+name, timeMin);
        double finalVal = logs.get("goal:"+name, timeMax);
        for (double t=timeMin;t<timeMax;t+=0.01) {
            double val = logs.get("goal:"+name, t);
            if (fabs(initVal - val) > 1e-5) {
                if (t < goalTimeMin) {
                    goalTimeMin = t;
                }
                break;
            }
        }
        for (double t=timeMax;t>timeMin;t-=0.01) {
            double val = logs.get("goal:"+name, t);
            if (fabs(finalVal - val) > 1e-5) {
                if (t > goalTimeMax) {
                    goalTimeMax = t;
                }
                break;
            }
        }
    }
    double goalTimeLength = goalTimeMax - goalTimeMin;
    std::cout << "Actual log goal begin=" << goalTimeMin 
        << " end=" << goalTimeMax 
        << " length=" << goalTimeLength << std::endl;
    
    //Initialize trajectory parameters
    Leph::TrajectoryParameters trajParams = Leph::DefaultTrajParameters();
    
    //Initialize the generator
    Leph::TrajectoryGeneration generator(
        Leph::SigmabanModel, modelParamsPath);
    //Load trajectory template and
    //parameter trajectories initialization.
    //Enable forward (complete) parameter optimization.
    if (trajName == "kicksingle") {
        Leph::TrajKickSingle::initializeParameters(trajParams, true);
        generator.setTrajectoryGenerationFunc(Leph::TrajKickSingle::funcGeneration(trajParams));
        generator.setCheckParametersFunc(Leph::TrajKickSingle::funcCheckParams(trajParams));
        generator.setCheckStateFunc(Leph::TrajKickSingle::funcCheckState(trajParams));
        generator.setCheckDOFFunc(Leph::TrajKickSingle::funcCheckDOF(trajParams));
        generator.setSaveFunc(Leph::TrajKickSingle::funcSave(trajParams));
    } else if (trajName == "kickdouble") {
        Leph::TrajKickDouble::initializeParameters(trajParams, true);
        generator.setTrajectoryGenerationFunc(Leph::TrajKickDouble::funcGeneration(trajParams));
        generator.setCheckParametersFunc(Leph::TrajKickDouble::funcCheckParams(trajParams));
        generator.setCheckStateFunc(Leph::TrajKickDouble::funcCheckState(trajParams));
        generator.setCheckDOFFunc(Leph::TrajKickDouble::funcCheckDOF(trajParams));
        generator.setSaveFunc(Leph::TrajKickDouble::funcSave(trajParams));
    } else {
        std::cout << "Invalid trajectory name: " << trajName << std::endl;
        return 1;
    }

    //Load seed trajectory parameters
    if (seedParamsPath != "") {
        trajParams.importData(seedParamsPath);
    }

    //Assign trajectory length
    trajParams.set("time_length") = goalTimeLength;

    //Build initial parameters
    Eigen::VectorXd initParams = trajParams.buildVector();
    //Build normalization coefficents
    Eigen::VectorXd normCoefs = trajParams.buildNormalizationCoefs();
    
    //Normalization of initial parameters
    Eigen::VectorXd bestParams = initParams;
    double bestScore = -1.0;
    int iteration = 1;

    //Display initial score
    std::cout << "Initial Score: " << scoreTrajectoryFitting(
        initParams, trajParams, generator, 
        logs, goalTimeMin, goalTimeMax, isSimulation, 1) << std::endl;
    
    //Fitness function
    libcmaes::FitFuncEigen fitness = 
        [&logs, &generator, &normCoefs, 
        &trajParams, &goalTimeMin, &goalTimeMax,
        &isSimulation]
        (const Eigen::VectorXd& params) 
    {
        try {
            return scoreTrajectoryFitting(
                params.array() * normCoefs.array(), 
                trajParams, generator, logs, 
                goalTimeMin, goalTimeMax, isSimulation, 0);
        } catch (const std::runtime_error& e) {
            return 2000.0;
        }
    };
    
    //Progress function
    libcmaes::ProgressFunc<
        libcmaes::CMAParameters<>, libcmaes::CMASolutions> progress = 
        [&logs, &generator, &normCoefs, &outPath,
        &trajParams, &goalTimeMin, &goalTimeMax,
        &bestParams, &bestScore, &iteration, &isSimulation]
        (const libcmaes::CMAParameters<>& cmaparams, 
        const libcmaes::CMASolutions& cmasols)
    {
        //Empty case
        if (cmasols.get_best_seen_candidate().get_x_dvec().size() == 0) {
            //Call default CMA-ES default progress function
            return libcmaes::CMAStrategy<libcmaes::CovarianceUpdate>
                ::_defaultPFunc(cmaparams, cmasols);
        }
        //Retrieve best Trajectories and score
        Eigen::VectorXd params = normCoefs.array() * 
            cmasols.get_best_seen_candidate().get_x_dvec().array();
        double score = 
            cmasols.get_best_seen_candidate().get_fvalue();
        if (bestScore < 0.0 || bestScore > score) {
            bestParams = params;
            bestScore = score;
        }
        //Save current best found
        size_t periodIterations;
        if (isSimulation) {
            periodIterations = 10;
        } else {
            periodIterations = 100;
        }
        if (iteration%periodIterations == 0) {
            Leph::Trajectories bestTraj = 
                generator.generateTrajectory(bestParams);
            std::cout << "============" << std::endl;
            //Save best trajectories and parameters
            generator.save(outPath, bestTraj, bestParams);
            std::cout << "****** Dimension: " << bestParams.size() << std::endl;
            std::cout << "****** BestScore: " << bestScore << std::endl;
            std::cout << "****** CurrentScore: " << score << std::endl;
            std::cout << "============" << std::endl;
        }
        iteration++;
        
        //Call default CMA-ES default progress function
	return libcmaes::CMAStrategy<libcmaes::CovarianceUpdate>
            ::_defaultPFunc(cmaparams, cmasols);
    };
    
    //CMAES initialization
    libcmaes::CMAParameters<> cmaparams(
        initParams.array() / normCoefs.array(), 
        cmaesSigma, cmaesLambda);
    cmaparams.set_quiet(false);
    cmaparams.set_mt_feval(true);
    cmaparams.set_str_algo("abipop");
    cmaparams.set_elitism(cmaesElitismLevel);
    cmaparams.set_restarts(cmaesRestarts);
    cmaparams.set_max_iter(cmaesMaxIterations);
    cmaparams.set_ftolerance(1e-9);
    
    //Run optimization
    libcmaes::CMASolutions cmasols = 
        libcmaes::cmaes<>(fitness, cmaparams, progress);

    return 0;
}


###
### Quentin Rouxel
### Thesis (2014-201?)
###

cmake_minimum_required(VERSION 2.8)
project(model)

#Option for compiling the tests and apps
option(BUILD_LEPH_CODE_TESTS "Build Leph Code Tests" OFF)
option(BUILD_LEPH_CODE_APPS "Build Leph Code Apps" OFF)
option(BUILD_LEPH_CODE_VIEWER "Build Leph Code Viewer (SFML)" OFF)
option(BUILD_ROBOCUP_VIEWER "Force build of the Robocup Viewer (SFML required)" OFF)

set(CATKIN_DEPENDENCIES
  eigen
  lwpr
  libgp
  libcmaes
  rbdl_urdfreader
  )

if (BUILD_LEPH_CODE_VIEWER OR BUILD_ROBOCUP_VIEWER)
  set(CATKIN_DEPENDENCIES ${CATKIN_DEPENDENCIES} "sfml")
endif()

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

find_package(RhIOServer)
find_package(RhIOClient)
find_package(RhAL)

catkin_package(
  INCLUDE_DIRS .
  LIBRARIES model
  CATKIN_DEPENDS eigen lwpr libgp libcmaes rbdl_urdfreader
)

#Includes directories lists
set(LEPHCODE_INCLUDE_DIRECTORIES "")

#Include sources directory
set(LEPHCODE_INCLUDE_DIRECTORIES ${LEPHCODE_INCLUDE_DIRECTORIES} ${CMAKE_CURRENT_SOURCE_DIR})

#OpenMP flags for LibCMAES
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp" )

#Include RhAL (Rhoban Abstraction Layer Library) if found
if (RhAL_FOUND)
    add_definitions(-DBUILD_RHAL_RHIO_BINDING)
    include_directories(${RhAL_INCLUDE_DIRS})
endif (RhAL_FOUND)

#Include RhIOClient (Rhoban Input Output Library) if found
if (RhIOClient_FOUND)
    include_directories(${RhIOClient_INCLUDE_DIRS})
endif (RhIOClient_FOUND)

#Include RhIOServer (Rhoban Input Output Library) if found
if (RhIOServer_FOUND)
    include_directories(${RhIOServer_INCLUDE_DIRS})
endif (RhIOServer_FOUND)

#Enable C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#Enable Warning
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

#Sources files
set(SOURCES_FILES
    CartWalk/CartWalk.cpp  
    CartWalk/linear_algebra.cpp  
    CartWalk/math_basics.cpp  
    CartWalk/math_log.cpp  
    CartWalk/SigmabanLeg.cpp
    CartWalk/CartWalkProxy.cpp  
    CartWalk/CartWalkGradient.cpp  
    Concepts/HumanoidModelConcept.cpp
    Concepts/HumanoidSensorsModelConcept.cpp
    Concepts/FootStepDifferentiatorConcept.cpp
    Concepts/FootStepIntegratorConcept.cpp
    Concepts/FallDetectorConcept.cpp
    LinearRegression/SimpleLinearRegression.cpp
    LinearRegression/MedianLinearRegression.cpp
    Gradient/FiniteDifferenceGradient.cpp
    Utils/GeometricMedian.cpp
    Utils/Combination.cpp
    Utils/CircularBuffer.cpp
    Utils/Chrono.cpp
    Utils/Scheduling.cpp
    Utils/Differentiation.cpp
    Utils/NewtonBinomial.cpp
    Utils/LWPRUtils.cpp
    Utils/RandomWalk.cpp
    Utils/RandomVelocitySpline.cpp
    Utils/IterativeLearningControl.cpp
    Utils/LWPRInputsOptimization.cpp
    Utils/EncoderFilter.cpp
    Utils/ComputeModelData.cpp
    Utils/FileEigen.cpp
    Utils/FileModelParameters.cpp
    Utils/GaussianDistribution.cpp
    Ncurses/InterfaceCLI.cpp
    Model/Model.cpp
    Model/HumanoidModel.cpp
    Model/HumanoidFloatingModel.cpp
    Model/HumanoidFixedModel.cpp
    Model/HumanoidFixedPressureModel.cpp
    Model/InverseKinematics.cpp
    Model/RBDLRootUpdate.cpp
    Model/RBDLClosedLoop.cpp
    Model/RBDLContactLCP.cpp
    Model/NullSpace.cpp
    Model/ForwardSimulation.cpp
    Model/HumanoidSimulation.cpp
    Model/JointModel.cpp
    Odometry/Odometry.cpp
    Odometry/OdometryDisplacementModel.cpp
    Odometry/OdometryNoiseModel.cpp
    Odometry/OdometrySequence.cpp
    StaticWalk/StaticWalk.cpp
    Spline/Polynom.cpp
    Spline/Spline.cpp
    Spline/SmoothSpline.cpp
    Spline/LinearSpline.cpp
    Spline/CubicSpline.cpp
    Spline/FittedSpline.cpp
    Spline/PolyFit.cpp
    TrajectoryGeneration/TrajectoryGeneration.cpp
    TrajectoryGeneration/TrajectoryUtils.cpp
    LegIK/LegIK.cpp
    IKWalk/IKWalk.cpp
    QuinticWalk/Footstep.cpp
    QuinticWalk/QuinticExperiment.cpp
    QuinticWalk/QuinticWalk.cpp
    TimeSeries/SeriesUtils.cpp
    DMP/DMP.cpp
    DMP/DMPSpline.cpp
    LCPMobyDrake/LCPSolver.cpp
    TrajectoryDefinition/CommonTrajs.cpp
    TrajectoryDefinition/TrajKickSingle.cpp
    TrajectoryDefinition/TrajKickSingleContact.cpp
    TrajectoryDefinition/TrajKickDouble.cpp
    TrajectoryDefinition/TrajStaticPose.cpp
    TrajectoryDefinition/TrajLegLift.cpp
    TrajectoryDefinition/TrajWalk.cpp
)

#Tests main files
set(TESTS_FILES
    testVectorLabel
    testCartWalkProxy
    testPlot
    testSimpleLinearRegression
    testCartWalkGradient
    testFiniteDifferenceGradient
    testReweightedLinearRegression
    testGeometricMedian
    testMedianLinearRegression
    testCombination
    testInterfaceCLI
    testCircularBuffer
    benchVectorLabel
    testMatrixLabel
    testModel
    testLevenbergMarquardt
    testChrono
    testCubicSpline
    testSmoothSpline
    testScheduling
    testFittedSpline
    testDifferentiation
    testGaussianProcess
    testMetaSeries
    testTimeSeries
    testMetaParameter
    testConcept
    testRegressionLWPR
    testLWPR
    testLWPRAdvanced
    testIterativeLearningControl
    testRandomWalk
    testRandomVelocitySpline
    testLWPRInputsOptimization
    testEuler
    testAxisAngle
    testEncoderFilter
    testMapSeries
    testOdometry
    benchModel
    testDMP
    testDMPSpline
    testJointModel
    testTrajectoryParameters
    testForwardSimulationCalibration
)

#Applications main files
if (BUILD_LEPH_APPS)
    set(APPS_FILES
        appSplineGenerationCircle
        appTestLearningFromLogs
        appExperimentOdometry
        appOdometryOffline
        appTrajectoriesFitting
        appStatsDynaban
        appOdometryModelCalibration
        appTrajectoriesGeneration
        appTrajectoriesCorrection
        appTrajectoriesSimulation
        appFeedForwardLogsComparison
        appHumanoidSimulationIdentification
        appTrajectoriesLogFitting
        appMapSeriesLogsCut
        appModelParametersViewer
        #appHumanoidSimulationLearning
        appCameraModelLearning
        appOdometryModelLearning
        appOdometryModelLearning2
        appOdometrySimulator
        appOdometryPreparePredictiveModelData
        appOdometryLogsTK
        )
endif()

#Build App RhIO Viewer 
if (BUILD_LEPH_CODE_VIEWER OR BUILD_ROBOCUP_VIEWER)
    if (RhIOClient_FOUND)
        if (DEFINED APPS_FILES)
            set(APPS_FILES ${APPS_FILES}
                appRhIOViewerModel
                appRhIOViewerRobocup
                )
        else(DEFINED APPS_FILES)
            set(APPS_FILES
                appRhIOViewerModel
                appRhIOViewerRobocup
                )
        endif(DEFINED APPS_FILES)
    else (RhIOClient_FOUND)
        message(FATAL_ERROR "Failed to find RhIOClient while asking for viewer")
    endif (RhIOClient_FOUND)
endif (BUILD_LEPH_CODE_VIEWER OR BUILD_ROBOCUP_VIEWER)

#LephCode linked libraries
set(CODE_LIBRARIES
    ncurses
    ${catkin_LIBRARIES}
)

if (RhIOClient_FOUND)
  set(CODE_LIBRARIES
    ${CODE_LIBRARIES}
    ${RhIOClient_LIBRARIES}
    )
endif(RhIOClient_FOUND)

#Add URDF model symbolic link
execute_process(COMMAND "/bin/bash" "-c" 
    "cd ${CMAKE_BINARY_DIR}; ln -f -s ${CMAKE_CURRENT_SOURCE_DIR}/Data/sigmaban_metabot_cleats.urdf sigmaban.urdf")
execute_process(COMMAND "/bin/bash" "-c" 
    "cd ${CMAKE_BINARY_DIR}; ln -f -s ${CMAKE_CURRENT_SOURCE_DIR}/Data/grosban_metabot.urdf grosban.urdf")
#Add RhAL config symbolic link
execute_process(COMMAND "/bin/bash" "-c" 
    "cd ${CMAKE_BINARY_DIR}; ln -f -s ${CMAKE_CURRENT_SOURCE_DIR}/Data/django_rhal.json django_rhal.json")
execute_process(COMMAND "/bin/bash" "-c" 
    "cd ${CMAKE_BINARY_DIR}; ln -f -s ${CMAKE_CURRENT_SOURCE_DIR}/Data/chewbacca_rhal.json chewbacca_rhal.json")

#Viewer (SFML) build option and files required
if (BUILD_LEPH_CODE_VIEWER OR BUILD_ROBOCUP_VIEWER)
    add_definitions(-DLEPH_VIEWER_ENABLED)
    set(SOURCES_FILES ${SOURCES_FILES} 
        Viewer/ModelViewer.cpp
        Viewer/ModelDraw.cpp
        TrajectoryGeneration/TrajectoryDisplay.cpp
    )
endif (BUILD_LEPH_CODE_VIEWER OR BUILD_ROBOCUP_VIEWER)

#Viewer (SFML) tests and apps
if (BUILD_LEPH_CODE_VIEWER)
    set(TESTS_FILES ${TESTS_FILES} 
        testModelViewer
        testInverseKinematics
        testStaticWalk
        testRBDLRootUpdate
        testHumanoidFixedModel
        testNullSpace
        testLegIK
        testModelTorques
        testIKWalk
        testZeroMomentPoint
        testHumanoidPressureModel
        testTrunkFootIK
        testForwardSimulation
        testQuinticExperiment
        testQuinticWalk
        testCameraModel
        testHumanoidSimulation
        testFeedForward
    )
    set(APPS_FILES ${APPS_FILES}
        appViewerModelSplines
        appViewerModelLogsOld
        appViewerModelLogs
        appViewerModelZMP
        appDynamicTrajectoryOptimizationTorques
        appTrajectoriesViewer
        appTestLegTorques
        appExperimentKick
        appExperimentKick2
        appTestZMPControl
        appMapSeriesLogsViewer
        appDMPTrajectoryOptimization
        appTrajectoriesKickGeneration
        appHumanoidSimulationPlayer
        appFeedForwardSimulation
    )
endif (BUILD_LEPH_CODE_VIEWER)

#Include need directories
set(LEPHCODE_INCLUDE_DIRECTORIES ${LEPHCODE_INCLUDE_DIRECTORIES} CACHE STRING "LephCode include directories")
include_directories(${LEPHCODE_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})

#Build Code as shared library
add_library(model SHARED ${SOURCES_FILES})
target_link_libraries(model ${CODE_LIBRARIES})

#Build Tests
if (BUILD_LEPH_CODE_TESTS)
    foreach (TEST ${TESTS_FILES})
        add_executable(${TEST} Tests/${TEST}.cpp)
        target_link_libraries(${TEST} model)
    endforeach ()
endif (BUILD_LEPH_CODE_TESTS)

#Build Apps
if (BUILD_LEPH_CODE_APPS)
    foreach (APP ${APPS_FILES})
        add_executable(${APP} Apps/${APP}.cpp)
        target_link_libraries(${APP} model)
    endforeach ()
endif (BUILD_LEPH_CODE_APPS)

#Build App RhAL Motion 
if (RhAL_FOUND)
    set(APPS_RHAL_LIBRARIES ${RhAL_LIBRARIES} ${RhIO_SERVER_LIBRARIES})
    set(APPS_RHAL_SOURCES_FILES
        RhAL/RhALUtils
    )
    set(APPS_RHAL_FILES
        appRhALTrajectory
    )
    foreach (APP ${APPS_RHAL_FILES})
        add_executable(${APP} Apps/${APP}.cpp ${APPS_RHAL_SOURCES_FILES})
        target_link_libraries(${APP} model ${APPS_RHAL_LIBRARIES})
    endforeach ()
endif (RhAL_FOUND)


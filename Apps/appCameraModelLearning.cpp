#include "Calibration/LogLikelihoodMaximization.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Model/HumanoidModel.hpp"
#include "Model/JointModel.hpp"
#include "Model/NamesModel.h"
#include "Types/MatrixLabel.hpp"
#include "Utils/FileEigen.h"
#include "Utils/FileMap.h"
#include "Utils/FileModelParameters.h"
#include "Viewer/ModelDraw.hpp"
#include "Viewer/ModelViewer.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>
#include <map>

// TODO actually remove the imu yaw and probably the 3 camera DoFs.

/**
 * Number of sampling per parameters evaluation
 * for log likelyhood maximisation
 */
static const unsigned int samplingNumber = 100;

/**
 * Learning/Testing data ratio between 0 and 1
 */
static const double learningDataRatio = 0.75;

/**
 * CMA-ES optimization configuration
 */
static const int cmaesElitismLevel = 0;
static const unsigned int cmaesMaxIterations = 40;
static const unsigned int cmaesRestarts = 1;
// Choses the number of points created when exploring (pop size)
static const unsigned int cmaesLambda = 100;
// Choses the distance of the exploration. -1=auto.
static const double cmaesSigma = -1.0;

/**
 * Global default geometry data
 */
static Eigen::MatrixXd defaultGeometryData;
static std::map<std::string, size_t> defaultGeometryName;

// The angles can be in [-margin, margin]
static float margin = 8.0;
static float maxPixelNoise =
    0.2; // 66% of measures are supposed to be in the ~~[-5°, 5°] margin
// These parameters are not optimized anymore (in radians)
Leph::CameraParameters camParams;

enum PARAMS_ID {
  PIXEL_NOISE,
  CAMERA_ROLL,
  CAMERA_PITCH,
  CAMERA_YAW,
  IMU_ROLL,
  IMU_PITCH,
  IMU_YAW,
  NECK_ROLL,
  NECK_PITCH,
  NECK_YAW
};
// Adding NB_IDS at the end of the enum was a decent idea but Eigen doesn't like
// it.
static const int NB_IDS = 10;

std::string getTime() {
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer,sizeof(buffer),"%Y-%m-%d_%I-%M-%S",timeinfo);
  std::string str(buffer);
  return str;
}

int createDir(const char *path) {
  // read/write/search permissions for owner and group, and with read/search
  // permissions for others. Why not
  return mkdir(path, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
}

void saveHumanParams(std::string & path, const Eigen::VectorXd & bestParams) {
  std::ofstream file;
  std::string name = path + "/human.results";
  file.open(name, std::ios_base::trunc);
  file << "(Angles in degrees, noisePixel in 640 pixel image)"
            << "\n";
  file << "noisePixel:     " << bestParams(PIXEL_NOISE) * 320 << "\n";
  file << "CamOffsetRoll:  " << bestParams(CAMERA_ROLL) * 180.0 / M_PI
            << "\n";
  file << "CamOffsetPitch: " << bestParams(CAMERA_PITCH) * 180.0 / M_PI
            << "\n";
  file << "CamOffsetYaw:   " << bestParams(CAMERA_YAW) * 180.0 / M_PI
            << "\n";
  file << "IMUOffsetRoll:  " << bestParams(IMU_ROLL) * 180.0 / M_PI
            << "\n";
  file << "IMUOffsetPitch: " << bestParams(IMU_PITCH) * 180.0 / M_PI
            << "\n";
  file << "IMUOffsetYaw:   " << 0.0
            << "\n";
  file << "NeckOffsetRoll:  " << bestParams(NECK_ROLL) * 180.0 / M_PI
            << "\n";
  file << "NeckOffsetPitch: " << 0.0
            << "\n";
  file << "NeckOffsetYaw:   " << bestParams(NECK_YAW) * 180.0 / M_PI
            << "\n";
  file.close();
}


/**
 * Build and return the geometryData matrix
 * from default data and given parameters vector
 */
Eigen::MatrixXd getGeometryDataFromParameters(const Eigen::VectorXd &params) {
  // Retrieve camera angular offset
  // 3: camera angle offset roll (radian)
  double camOffsetRoll = params(CAMERA_ROLL);
  // 4: camera angle offset pitch (radian)
  double camOffsetPitch = params(CAMERA_PITCH);
  // 5: camera angle offset yaw (radian)
  double camOffsetYaw = params(CAMERA_YAW);
  // 9: neck angle offset roll (radian)
  double neckOffsetRoll = params(NECK_ROLL);
  // 10: neck angle offset pitch (radian)
  double neckOffsetPitch = 0.0;//params(NECK_PITCH);
  // 11: neck angle offset yaw (radian)
  double neckOffsetYaw = params(NECK_YAW);

  // Copy default geometry model
  Eigen::MatrixXd geometryData = defaultGeometryData;
  // Assign model angular offset
  // on camera transformation
  geometryData(defaultGeometryName.at("camera"), 0) += camOffsetRoll;
  geometryData(defaultGeometryName.at("camera"), 1) += camOffsetPitch;
  geometryData(defaultGeometryName.at("camera"), 2) += camOffsetYaw;
  // Tried replacing head_yaw by head_pitch, worked poorer on the 1 test I did
  geometryData(defaultGeometryName.at("head_yaw"), 0) += neckOffsetRoll;
  geometryData(defaultGeometryName.at("head_yaw"), 1) += neckOffsetPitch;
  geometryData(defaultGeometryName.at("head_yaw"), 2) += neckOffsetYaw;

  return geometryData;
}

/**
 * Build and return the initial
 * parameters vector
 */
Eigen::VectorXd buildInitialParameters() {
  // Parameters initialization
  Eigen::VectorXd initParams(NB_IDS);
  // All of the offsets are angular in radians and start at 0 except the pixel
  // noise (it's a stadard deviation on the famous [-1, 1] pixel space)
  for (unsigned int i = 0; i < NB_IDS; i++) {
    initParams(i) = 0.0;
  }
  initParams(PIXEL_NOISE) = 0.05;

  return initParams;
}

/**
 * Build and return the
 * normalization coefficients
 */
Eigen::VectorXd buildNormalizationCoef() {
  // Parameters initialization
  Eigen::VectorXd normCoef(NB_IDS);

  for (unsigned int i = 0; i < NB_IDS; i++) {
    normCoef(i) = 2 * margin * M_PI / 180.0;
  }
  // noise aiming pixel (pixel space)
  normCoef(PIXEL_NOISE) = 0.05;

  return normCoef;
}

/**
 * Check and bound parameters to
 * acceptable range.
 * Return non zero cost if
 * parameters are unbounded.
 */
double boundParameters(const Eigen::VectorXd &params) {
  double cost = 0.0;

  for (unsigned int i = 0; i < NB_IDS; i++) {
    if (fabs(params(i)) > margin * M_PI / 180.0) {
      cost += 1000.0 + 1000.0 * (fabs(params(i)) - margin * M_PI / 180.0);
    }
  }

  // noise aiming pixel (pixel space)
  if (params(PIXEL_NOISE) <= 0.0) {
    cost += 1000.0 - 1000.0 * (params(0));
  }
  if (params(PIXEL_NOISE) >= maxPixelNoise) {
    cost += 1000.0 + 1000.0 * (params(0) - maxPixelNoise);
  }

  return cost;
}

/**
 * Model initialization function
 */
Leph::HumanoidFixedModel initModel(const Eigen::VectorXd &params) {
  // Build the geometry from parameters
  Eigen::MatrixXd geometryData = getGeometryDataFromParameters(params);

  // Load the model with
  // the geometry updated
  Leph::HumanoidFixedModel model(Leph::SigmabanModel, Eigen::MatrixXd(), {},
                                 geometryData, defaultGeometryName);

  return model;
}

/**
 * Compute and return the estimation
 * (try to predict observation) from
 * given data and using given parameters
 */
Eigen::VectorXd evaluateParameters(const Eigen::VectorXd &params,
                                   const Leph::VectorLabel &data, bool noRandom,
                                   Leph::HumanoidFixedModel &model,
                                   std::default_random_engine &engine) {
  // Parse parameters
  // The other parameters are handled in the getGeometryData (called in
  // initModel)
  // noise aiming pixel (pixel space)
  double noiseAimingPixel = params(PIXEL_NOISE);
  // 6: imu angle offset roll (radian)
  double imuOffsetRoll = params(IMU_ROLL);
  // 7: imu angle offset pitch (radian)
  double imuOffsetPitch = params(IMU_PITCH);
  // 8: imu angle offset yaw (radian)
  double imuOffsetYaw = 0.0;// TODO doesn't change anything to put pi/2 here...//params(IMU_YAW);

  // Disable caching optimization
  model.get().setAutoUpdate(true);
  // Assign DOF state
  model.setSupportFoot(Leph::HumanoidFixedModel::LeftSupportFoot);
  for (const std::string &name : Leph::NamesDOF) {
    model.get().setDOF(name, data(name));
  }
  // Assign trunk orientation from IMU
  double imuPitch = data("imu_pitch");
  double imuRoll = data("imu_roll");
  Eigen::Matrix3d imuMatrix =
      Eigen::AngleAxisd(imuOffsetYaw, Eigen::Vector3d::UnitZ())
          .toRotationMatrix() *
      Eigen::AngleAxisd(imuPitch + imuOffsetPitch, Eigen::Vector3d::UnitY())
          .toRotationMatrix() *
      Eigen::AngleAxisd(imuRoll + imuOffsetRoll, Eigen::Vector3d::UnitX())
          .toRotationMatrix();
  model.setOrientation(imuMatrix);
  // Assign the left foot to origin
  model.get().setDOF("base_x", 0.0);
  model.get().setDOF("base_y", 0.0);
  model.get().setDOF("base_z", 0.0);
  model.get().setDOF("base_yaw", 0.0);
  // Enable model caching
  model.get().setAutoUpdate(false);
  model.get().updateDOFPosition();

  // Retrieve camera aiming point
  // in pixel space
  Eigen::Vector2d pixelPoint;
  pixelPoint.x() = data("pixel_x");
  pixelPoint.y() = data("pixel_y");

  // Add random gaussian on recorded pixel
  if (!noRandom) {
    std::normal_distribution<double> distPixel(0.0, noiseAimingPixel);
    pixelPoint.x() += distPixel(engine);
    pixelPoint.y() += distPixel(engine);
  }

  // Compute head view vector in world
  Eigen::Vector3d viewVectorInWorld =
      model.get().cameraPixelToViewVector(camParams, pixelPoint);
  // Compute the cartesian position estimation.
  //(the computed point is in world frame which
  // is coincident with the left foot).
  Eigen::Vector3d groundEstimation;
  // Astuce ! We're giving the z of the object here, the function calculates
  // the intersection
  // with a plan parallel to ground of height ground_z
  bool isSuccess = model.get().cameraViewVectorToWorld(
      viewVectorInWorld, groundEstimation, data("ground_z"));

  // Return cartesian estimation
  Eigen::VectorXd estimate(2);
  if (!isSuccess) {
    estimate << 1000.0, 1000.0;
  } else {
    estimate << groundEstimation.x(), groundEstimation.y();
  }
  return estimate;
}

void viewLog(const Leph::MatrixLabel &log, const Eigen::VectorXd &params) {
  Leph::ModelViewer viewer(1200, 900);

  // Applying some of the params and creating the model
  Leph::HumanoidFixedModel model = initModel(params);

  // Parse parameters
  // 6: imu angle offset roll (radian)
  double imuOffsetRoll = params(IMU_ROLL);
  // 7: imu angle offset pitch (radian)
  double imuOffsetPitch = params(IMU_PITCH);
  // 8: imu angle offset yaw (radian)
  double imuOffsetYaw = 0.0;//TODO I put PI/2 and it didn't change anything...//params(IMU_YAW);

  // Leph::VectorLabel data = log
  for (unsigned int i = 0; i < log.size(); i++) {
    const Leph::VectorLabel &data = log[i];
    while (viewer.update()) {
      if (viewer.isKeyPressed(sf::Keyboard::L)) {
        // Leaving
        i=log.size();
        break;
      }
      // Assign DOF state
      model.setSupportFoot(Leph::HumanoidFixedModel::LeftSupportFoot);
      for (const std::string &name : Leph::NamesDOF) {
        model.get().setDOF(name, data(name));
      }
      // Assign trunk orientation from IMU
      double imuPitch = data("imu_pitch");
      double imuRoll = data("imu_roll");
      Eigen::Matrix3d imuMatrix =
          Eigen::AngleAxisd(imuOffsetYaw, Eigen::Vector3d::UnitZ())
              .toRotationMatrix() *
          Eigen::AngleAxisd(imuPitch + imuOffsetPitch, Eigen::Vector3d::UnitY())
              .toRotationMatrix() *
          Eigen::AngleAxisd(imuRoll + imuOffsetRoll, Eigen::Vector3d::UnitX())
              .toRotationMatrix();
      model.setOrientation(imuMatrix);
      // Assign the left foot to origin
      model.get().setDOF("base_x", 0.0);
      model.get().setDOF("base_y", 0.0);
      model.get().setDOF("base_z", 0.0);
      model.get().setDOF("base_yaw", 0.0);

      Eigen::Vector3d ground(data("ground_x"), data("ground_y"),
                             data("ground_z"));
      // viewer.drawFrame(ground, Eigen::Matrix3d::Identity());
      viewer.drawSphere(ground, 0.01);

      Eigen::Vector2d pixelPoint;
      pixelPoint.x() = data("pixel_x");
      pixelPoint.y() = data("pixel_y");

      // Compute head view vector in world
      Eigen::Vector3d viewVectorInWorld =
          model.get().cameraPixelToViewVector(camParams, pixelPoint);

      // Compute the cartesian position estimation.
      //(the computed point is in world frame which
      // is coincident with the left foot).
      Eigen::Vector3d groundEstimation;
      // Astuce ! We're giving the z of the object here, the function calculates
      // the intersection
      // with a plan parallel to ground of height ground_z
      bool isSuccess = model.get().cameraViewVectorToWorld(
          viewVectorInWorld, groundEstimation, data("ground_z"));

      if (!isSuccess) {
        std::cout << "Failed to cameraViewVectorToWorld" << std::endl;
      }
      viewer.drawFrame(groundEstimation, Eigen::Matrix3d::Identity());
      // Display model and view box
      Leph::ModelDraw(model.get(), viewer);
    }
  }
}

/**
 * Use log likellihood maximisation
 * and CMA-ES to find camera model parameters
 * matching given observations
 */
int main(int argc, char **argv) {
  // Parse user inputs
  if (argc != 3 && argc != 5) {
    std::cout << "Usage: ./app "
              << "calibrationLogs.matrixlabel"
              << " outputPrefix"
              << " [MODEL model.modelparams]"
              << " [-v]" << std::endl;
    std::cout << "Usage: ./app "
              << "calibrationLogs.matrixlabel"
              << " outputPrefix"
              << " [SEED  seed.camparams]"
              << " [-v]" << std::endl;
    return 1;
  }
  std::string logPath = argv[1];
  std::string outPath = argv[2];
  std::string modelPath = "";
  std::string seedPath = "";
  if (argc == 5 && std::string(argv[3]) == "MODEL") {
    modelPath = argv[4];
  }
  if (argc == 5 && std::string(argv[3]) == "SEED") {
    seedPath = argv[4];
  }

  std::string path = outPath + "_" + getTime();
  std::cout << "The outputs will be saved in the folder " << path << std::endl;
  if (createDir(path.c_str())) {
    std::cout << "Failed to create directory at " << path << std::endl;
    return -1;
  }
  std::string symLink = "last";
  if (remove(symLink.c_str())) {
    std::cout << "Couldn't delete " << symLink << std::endl;
  }
  if (symlink(path.c_str(), "last")) {
    std::cout << "Couldn't create the symlink to '" << symLink << "', moving on."  << std::endl;
  }

  // TODO read a value instead of having a constant value
  camParams.widthAperture = 67 * M_PI / 180.0;
  camParams.heightAperture = 52.47 * M_PI / 180.0;
  std::cout << "Using a camera width of " << camParams.widthAperture*180.0/M_PI << "°, and a camera height of " << camParams.heightAperture*180.0/M_PI << std::endl;


  // Loading data
  Leph::MatrixLabel logs;
  logs.load(logPath);
  std::cout << "Loading data from " << logPath << ": " << logs.size()
            << " points" << std::endl;

  std::cout << "Viewing log" << std::endl;
  // Load default model parameters
  Eigen::MatrixXd jointData;
  std::map<std::string, size_t> jointName;
  Eigen::MatrixXd inertiaData;
  std::map<std::string, size_t> inertiaName;
  if (modelPath == "") {
    std::cout << "Loading default Sigmaban model parameters" << std::endl;
    // Load default geometry and inertia
    // data from Sigmaban model
    Leph::HumanoidModel tmpModel(Leph::SigmabanModel, "left_foot_tip", false);
    defaultGeometryData = tmpModel.getGeometryData();
    defaultGeometryName = tmpModel.getGeometryName();
    inertiaData = tmpModel.getInertiaData();
    inertiaName = tmpModel.getInertiaName();
    // Build default joint parameters matrix
    Leph::JointModel tmpJoint;
    Eigen::VectorXd jointParams = tmpJoint.getParameters();
    jointData = Eigen::MatrixXd(Leph::NamesDOF.size(), jointParams.size());
    size_t tmpIndex = 0;
    for (const std::string &name : Leph::NamesDOF) {
      jointName[name] = tmpIndex;
      jointData.block(tmpIndex, 0, 1, jointParams.size()) =
          jointParams.transpose();
      tmpIndex++;
    }
  } else {
    std::cout << "Loading model parameters from: " << modelPath << std::endl;
    Leph::ReadModelParameters(modelPath, jointData, jointName, inertiaData,
                              inertiaName, defaultGeometryData,
                              defaultGeometryName);
  }
  std::cout << "Loaded." << std::endl;
  // Get the initial parameters
  Eigen::VectorXd initParams = buildInitialParameters();

  // If given, seed initial parameters
  if (seedPath != "") {
    std::cout << "Loading seed parameters from: " << seedPath << std::endl;
    // Open file
    std::ifstream file(seedPath);
    if (!file.is_open()) {
      throw std::runtime_error("Unable to open file: " + seedPath);
    }
    // Read data from file
    Eigen::VectorXd camData = Leph::ReadEigenVectorFromStream(file);
    Eigen::VectorXd imuData = Leph::ReadEigenVectorFromStream(file);
    Eigen::MatrixXd tmpGeometryData = Leph::ReadEigenMatrixFromStream(file);
    std::map<std::string, size_t> tmpGeometryName =
        Leph::ReadMapFromStream<std::string, size_t>(file);
    file.close();
    // IMU roll, pitch, yaw offset
    initParams(IMU_ROLL) = imuData(0);
    initParams(IMU_PITCH) = imuData(1);
    initParams(IMU_YAW) = imuData(2);
    // Geometry camera offsets
    initParams(CAMERA_ROLL) = tmpGeometryData(tmpGeometryName.at("camera"), 0);
    initParams(CAMERA_PITCH) = tmpGeometryData(tmpGeometryName.at("camera"), 1);
    initParams(CAMERA_YAW) = tmpGeometryData(tmpGeometryName.at("camera"), 2);
    // Geometry neck offsets
    initParams(NECK_ROLL) = tmpGeometryData(tmpGeometryName.at("head_yaw"), 0);
    initParams(NECK_PITCH) = tmpGeometryData(tmpGeometryName.at("head_yaw"), 1);
    initParams(NECK_YAW) = tmpGeometryData(tmpGeometryName.at("head_yaw"), 2);
  }

  viewLog(logs, initParams);
  // Initialize the logLikelihood
  // maximisation process
  Leph::LogLikelihoodMaximization<Leph::VectorLabel, Leph::HumanoidFixedModel>
      calibration;
  calibration.setInitialParameters(initParams, buildNormalizationCoef());
  calibration.setUserFunctions(evaluateParameters, boundParameters, initModel);
  // Add observations data
  for (size_t i = 0; i < logs.size(); i++) {
    Eigen::VectorXd obs(2);
    obs << logs[i]("ground_x"), logs[i]("ground_y");
    calibration.addObservation(obs, logs[i]);
  }

  // Start the CMA-ES optimization
  Leph::Plot plot;
  calibration.runOptimization(samplingNumber, learningDataRatio,
                              cmaesMaxIterations, cmaesRestarts, cmaesLambda,
                              cmaesSigma, cmaesElitismLevel, &plot);

  // Display and save the best found parameters
  Eigen::VectorXd bestParams = calibration.getParameters();
  saveHumanParams(path, bestParams);

  // Export the optimized model parameters
  // Imu roll, pitch, yaw in radian
  Eigen::VectorXd imuOffsets(3);
  imuOffsets << bestParams(IMU_ROLL), bestParams(IMU_PITCH),
    0.0;//bestParams(IMU_YAW);
  // Camera angular aperture width, height in radian
  Eigen::VectorXd camApertures(2);
  camApertures << camParams.widthAperture, camParams.heightAperture;

  // Write to file
  std::string camParamsPath = path + "/cameraModel.params";
  std::cout << "Writing geometry and camera data to: " << camParamsPath
            << std::endl;
  std::ofstream file(camParamsPath);
  if (!file.is_open()) {
    throw std::runtime_error("Unable to open file: " + camParamsPath);
  }

  Leph::WriteEigenVectorToStream(file, camApertures);
  Leph::WriteEigenVectorToStream(file, imuOffsets);
  Leph::WriteEigenMatrixToStream(file,
                                 getGeometryDataFromParameters(bestParams));
  Leph::WriteMapToStream(file, defaultGeometryName);
  file.close();

  // Writing to file a second time, for good measure
  camParamsPath = "./cameraModel.params";
  std::cout << "Writing geometry and camera data to: " << camParamsPath
            << std::endl;
  std::ofstream camfile(camParamsPath);
  if (!camfile.is_open()) {
    throw std::runtime_error("Unable to open file: " + camParamsPath);
  }

  Leph::WriteEigenVectorToStream(camfile, camApertures);
  Leph::WriteEigenVectorToStream(camfile, imuOffsets);
  Leph::WriteEigenMatrixToStream(camfile,
                                 getGeometryDataFromParameters(bestParams));
  Leph::WriteMapToStream(camfile, defaultGeometryName);
  camfile.close();

  // Plot the convergence graphic
  plot.plot("iteration", "all").render();
  viewLog(logs, bestParams);

  return 0;
}

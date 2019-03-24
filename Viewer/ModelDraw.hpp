#ifndef LEPH_MODELDRAW_HPP
#define LEPH_MODELDRAW_HPP

#include "Model/Model.hpp"
#include "Model/HumanoidModel.hpp"
#include "Model/HumanoidSimulation.hpp"
#include "Viewer/ModelViewer.hpp"

namespace Leph
{
/**
 * ModelDraw
 *
 * Draw the given Model with given ModelViewer
 * and with optional color coeficient
 */
void ModelDraw(Model& model, ModelViewer& viewer, double color = 1.0);

/**
 * CameraDraw
 *
 * Draw with given HumanoidModel the camera
 * projected field of view with given ModelViewer
 */
void CameraDraw(const CameraModel& cameraModel, HumanoidModel& model, ModelViewer& viewer);

/**
 * FieldDraw
 *
 * Draw a Robocup field at given center and given
 * yaw orientation
 */
void FieldDraw(const Eigen::Vector3d& center, double yaw, ModelViewer& viewer);

/**
 * CleatsDraw
 *
 * Display foot cleat of given
 * HumanoidModel given given force
 */
void CleatsDraw(HumanoidSimulation& simulation, ModelViewer& viewer);

}  // namespace Leph

#endif

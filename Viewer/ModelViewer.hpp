#ifndef LEPH_MODELVIEWER_HPP
#define LEPH_MODELVIEWER_HPP

#include <list>
#include <SFML/Window.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/Graphics.hpp>
#include <Eigen/Dense>
#include <string>

namespace Leph
{
/**
 * ModelViewer
 *
 * Simple OpenGL viewer to
 * display in SFML windows
 */
class ModelViewer
{
public:
  /**
   * Enum for point tracker color
   */
  enum Color
  {
    Red,
    Green,
    Blue,
    Yellow,
    Purple,
    Cyan,
  };

  /**
   * Drawing config
   */
  double frameLength = 0.02;
  double frameThickness = 3.0;
  double groundThickness = 1.0;
  size_t maxTrajectory = 500;

  /**
   * Camera control config
   */
  double camPosVel = 0.005;
  double camViewVel = 0.01;

  /**
   * Initialize SFML and OpenGL rendering
   * with optional windows size
   */
  ModelViewer(unsigned int width = 800, unsigned int height = 600);

  /**
   * Poll event and update screen drawing
   * return false on exit asked
   * If freeFly is false, center the camera
   * on zero and use rotation view
   */
  bool update(bool freeFly = false);

  /**
   * Return true if given SFML key is pressed
   */
  bool isKeyPressed(sf::Keyboard::Key);

  /**
   * Draw frame
   * Orientation is the transform rotation matrix
   * from local coordinates to base
   */
  void drawFrame(const Eigen::Vector3d& center, const Eigen::Matrix3d& orientation);

  /**
   * Draw mass
   * Orientation is the transform rotation matrix
   * from local coordinates to base.
   * Color is a fading color coeficient.
   */
  void drawMass(const Eigen::Vector3d& center, const Eigen::Matrix3d& orientation, double color = 1.0);

  /**
   * Draw joint
   * Orientation is the transform rotation matrix
   * from local coordinates to base.
   * Color is a fading color coeficient.
   */
  void drawJoint(const Eigen::Vector3d& center, const Eigen::Matrix3d& orientation, double color = 1.0);

  /**
   * Draw a link between given two points
   * Color is a fading color coeficient.
   */
  void drawLink(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2, double color = 1.0);

  /**
   * Draw a wireframe box with given x,y,z half size at
   * given position and orientation
   * Optional box color can be given
   */
  void drawBox(double sizeX, double sizeY, double sizeZ, const Eigen::Vector3d& center,
               const Eigen::Matrix3d& orientation, double r = 0.3, double g = 0.6, double b = 0.6);

  /**
   * Draw a sphere at given position and
   * of given radius
   */
  void drawSphere(const Eigen::Vector3d& center, double radius, double r = 0.0, double g = 0.0, double b = 0.5);

  /**
   * Draw a cylinder at given base position and of given
   * radiuas and height
   */
  void drawCylinder(const Eigen::Vector3d& base, double radius, double height, double r = 0.0, double g = 0.0,
                    double b = 0.5);

  /**
   * Draw an arrow from given point with given
   * direction and length
   */
  void drawArrow(const Eigen::Vector3d& center, const Eigen::Vector3d& vect, double length, double r = 0.0,
                 double g = 0.0, double b = 1.0);

  /**
   * Draw a simple line between given points with
   * line thick and color
   */
  void drawLine(const Eigen::Vector3d& start, const Eigen::Vector3d& end, double thick, double r = 1.0, double g = 1.0,
                double b = 1.0);

  /**
   * Draw given text at a 3d position
   * with given color and size in pixel
   */
  void drawText(const Eigen::Vector3d& position, unsigned int size, const std::string& str, double r = 0.0,
                double g = 0.0, double b = 1.0);

  /**
   * Add a position for drawing trajectory
   */
  void addTrackedPoint(const Eigen::Vector3d& point, Color color = Red);

  /**
   * Draw rolling point trajectory
   */
  void drawTrajectory();

  /**
   * Override camera position
   */
  void setCamPosition(double x, double y);

private:
  /**
   * Windows size
   */
  unsigned int _width;
  unsigned int _height;

  /**
   * SFML Windows instance
   */
  sf::RenderWindow _window;

  /**
   * Text font
   */
  sf::Font _font;

  /**
   * Camera position and view direction
   */
  Eigen::Vector3d _camPos;
  Eigen::Vector3d _camView;

  /**
   * Used angle when not freefly
   */
  double _camRadius;
  double _camAngle1;
  double _camAngle2;
  double _camOffsetX;
  double _camOffsetY;

  /**
   * Rolling history of tracked point trajectory
   */
  std::list<Eigen::Vector3d> _trajectoryRed;
  std::list<Eigen::Vector3d> _trajectoryGreen;
  std::list<Eigen::Vector3d> _trajectoryBlue;
  std::list<Eigen::Vector3d> _trajectoryYellow;
  std::list<Eigen::Vector3d> _trajectoryPurple;
  std::list<Eigen::Vector3d> _trajectoryCyan;

  /**
   * Last recorded mouse position
   */
  int _lastMousePosX;
  int _lastMousePosY;

  /**
   * Update the opengl camera
   */
  void updateCamera(bool useViewVector);

  /**
   * Convert given rotation matrix to
   * OpenGL and multiply current matrix with
   */
  void applyRotation(const Eigen::Matrix3d& orientation);

  /**
   * Draw ground centered on origin with
   * given size
   */
  void drawGround(double size);

  /**
   * Draw a unit cube with given color
   * and optionaly in wireframe mode
   */
  void drawCube(float r, float g, float b, bool isWireFrame = false);

  /**
   * Draw given trajectory with given color
   */
  void drawColorTrajectory(const std::list<Eigen::Vector3d>& traj, double r, double g, double b);

  /**
   * Return pixel position of given 3d
   * point in world frame
   */
  Eigen::Vector2d getPointProjection(const Eigen::Vector3d& pos) const;
};

}  // namespace Leph

#endif

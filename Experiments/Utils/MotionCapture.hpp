#ifndef RHOBAN_MOTIONCAPTURE_HPP
#define RHOBAN_MOTIONCAPTURE_HPP

#include <string>

namespace Rhoban
{
/**
 * MotionCapture
 *
 * Connect to given motion capture streaming server
 * and keep track of object position and velocity
 */
class MotionCapture
{
public:
  /**
   * Motion capture result
   */
  struct CapturePosition
  {
    bool isValid;
    double x;
    double y;
    double z;
    double azimuth;
    double pitch;
    double roll;
  };

  /**
   * Output
   */
  // Position and computed velocity
  CapturePosition pos;
  CapturePosition vel;

  /**
   * Parameter
   */
  // Mobile state average coeficient
  // for position and velocity
  double averageCoefPos;
  double averageCoefVel;
  // Maximum number of tolerated invalid packet
  unsigned int maxInvalidTick;

  /**
   * Initialization
   */
  MotionCapture();

  /**
   * Close open connection
   */
  ~MotionCapture();

  /**
   * Set and reset motion capture
   * server connection
   */
  void setCaptureStream(std::string captureStream);

  /**
   * Receive motion capture packets and
   * update output
   */
  void tick(double elapsed);

private:
  /**
   * ZMQ motion capture server url
   */
  std::string _captureStream;

  /**
   * ZMQ context and socket
   */
  void* _context;
  void* _socket;

  /**
   * Last valid capture position
   * and velocity
   */
  unsigned int _countInvalidPos;
  unsigned int _countInvalidVel;

  /**
   * Last valid position captured
   */
  CapturePosition _lastUpdated;

  /**
   * Connection to motion capture service
   */
  void connection();

  /**
   * Read given string packet and return
   * converted Capture Position
   */
  CapturePosition parsePacket(unsigned char* buffer);

  /**
   * Read and return a (possibly non valid)
   * Capture Position
   */
  CapturePosition readPacket();

  /**
   * Read all available incomming packet
   * and return the last one
   */
  CapturePosition readLastPacket();

  /**
   * Deconnection from motion capture service
   */
  void deconnection();

  /**
   * Quaternion orientation to euler angles
   * in degrees
   */
  void quat2Euler(double qx, double qy, double qz, double qw, double& azimuth, double& pitch, double& roll) const;

  /**
   * Compute and update state with mobile average
   * and new values x, y, z, azimuth, pitch, roll
   * and use given last valid state count and
   * average coefficient alpha
   */
  void mobileAverage(CapturePosition& state, double x, double y, double z, double azimuth, double pitch, double roll,
                     unsigned int countInvalid, double alpha);
};

}  // namespace Rhoban

#endif

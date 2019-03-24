#include <iostream>
#include <stdexcept>
#include <zmq.h>
#include <cmath>
#include <cstdlib>
#include "MotionCapture.hpp"

namespace Rhoban
{
MotionCapture::MotionCapture()
  : _captureStream("tcp://192.168.16.100:3232")
  , _context(NULL)
  , _socket(NULL)
  , _countInvalidPos(0)
  , _countInvalidVel(0)
  , _lastUpdated()
{
  connection();

  // Parameter
  averageCoefPos = 0.7;
  averageCoefVel = 0.95;
  maxInvalidTick = 5;

  // Output
  pos.x = 0.0;
  pos.y = 0.0;
  pos.z = 0.0;
  pos.azimuth = 0.0;
  pos.pitch = 0.0;
  pos.roll = 0.0;
  pos.isValid = false;
  vel.x = 0.0;
  vel.y = 0.0;
  vel.z = 0.0;
  vel.azimuth = 0.0;
  vel.pitch = 0.0;
  vel.roll = 0.0;
  vel.isValid = false;
}

MotionCapture::~MotionCapture()
{
  deconnection();
}

void MotionCapture::setCaptureStream(std::string captureStream)
{
  deconnection();
  _captureStream = captureStream;
  connection();
}

/**
 * Compute the oriented distance between the two angle
 * in the range 180:180 degrees from angle1 to angle2
 */
static double distanceAngle(double angle1, double angle2)
{
  if (angle1 > 180.0)
    angle1 -= 360.0;
  if (angle1 < -180.0)
    angle1 += 360.0;
  if (angle2 > 180.0)
    angle2 -= 360.0;
  if (angle2 < -180.0)
    angle2 += 360.0;

  double max, min;
  if (angle1 > angle2)
  {
    max = angle1;
    min = angle2;
  }
  else
  {
    max = angle2;
    min = angle1;
  }

  double dist1 = max - min;
  double dist2 = 360.0 - max + min;

  if (dist1 < dist2)
  {
    if (angle1 > angle2)
    {
      return -dist1;
    }
    else
    {
      return dist1;
    }
  }
  else
  {
    if (angle1 > angle2)
    {
      return dist2;
    }
    else
    {
      return -dist2;
    }
  }
}

void MotionCapture::tick(double elapsed)
{
  CapturePosition current = readLastPacket();

  if (!pos.isValid && current.isValid)
  {
    pos = current;
    pos.isValid = true;
    _countInvalidPos = 0;
    _lastUpdated = current;

    vel.x = 0.0;
    vel.y = 0.0;
    vel.z = 0.0;
    vel.azimuth = 0.0;
    vel.pitch = 0.0;
    vel.roll = 0.0;
    vel.isValid = false;
    _countInvalidVel++;
  }
  if (pos.isValid && current.isValid)
  {
    mobileAverage(pos, current.x, current.y, current.z, current.azimuth, current.pitch, current.roll, _countInvalidPos,
                  averageCoefPos);
    pos.isValid = true;

    double time = elapsed * (_countInvalidVel + 1.0);
    mobileAverage(vel, (current.x - _lastUpdated.x) / time, (current.y - _lastUpdated.y) / time,
                  (current.z - _lastUpdated.z) / time, distanceAngle(_lastUpdated.azimuth, current.azimuth) / time,
                  distanceAngle(_lastUpdated.pitch, current.pitch) / time,
                  distanceAngle(_lastUpdated.roll, current.roll) / time, _countInvalidVel, averageCoefVel);
    vel.isValid = true;

    _countInvalidPos = 0;
    _countInvalidVel = 0;
    _lastUpdated = current;
  }
  if (!current.isValid)
  {
    _countInvalidPos++;
    _countInvalidVel++;
  }
  if (_countInvalidPos >= maxInvalidTick)
  {
    pos.isValid = false;
    vel.isValid = false;
  }
}

void MotionCapture::connection()
{
  _context = zmq_ctx_new();
  if (_context == NULL)
  {
    throw std::runtime_error("MotionCapture zmq context fail");
  }

  _socket = zmq_socket(_context, ZMQ_SUB);
  if (_socket == NULL)
  {
    throw std::runtime_error("MotionCapture zmq socket fail");
  }

  int error;
  error = zmq_connect(_socket, _captureStream.c_str());
  if (error != 0)
  {
    throw std::runtime_error("MotionCapture zmq connect fail");
  }

  error = zmq_setsockopt(_socket, ZMQ_SUBSCRIBE, NULL, 0);
  if (error != 0)
  {
    throw std::runtime_error("MotionCapture zmq sockopt fail");
  }
}

MotionCapture::CapturePosition MotionCapture::parsePacket(unsigned char* buffer)
{
  char number[1024];
  int index = 0;
  int numberIndex;

  CapturePosition current;
  current.isValid = false;
  int count = 0;

  double qx, qy, qz, qw;

  double nbMarkers, minError;

  while (true)
  {
    numberIndex = 0;
    while (buffer[index] != ' ' && buffer[index] != '\0')
    {
      number[numberIndex] = buffer[index];
      numberIndex++;
      index++;
    }
    number[numberIndex] = '\0';
    count++;

    if (count == 1)
      current.x = atof(number);
    if (count == 2)
      current.y = atof(number);
    if (count == 3)
      current.z = atof(number);
    if (count == 4)
      qx = atof(number);
    if (count == 5)
      qy = atof(number);
    if (count == 6)
      qz = atof(number);
    if (count == 7)
      qw = atof(number);
    if (count == 8)
      nbMarkers = atof(number);
    if (count == 9)
      minError = atof(number);

    if (buffer[index] == '\0')
    {
      if (count >= 9)
      {
        quat2Euler(qx, qy, qz, qw, current.azimuth, current.pitch, current.roll);
        if (fabs(current.x) > 0.01 || fabs(current.y) > 0.01 || fabs(current.z) > 0.01 || fabs(qx) > 0.01 ||
            fabs(qy) > 0.01 || fabs(qz) > 0.01 || fabs(qw) > 0.01)
        {
          current.isValid = true;
        }
      }

      return current;
    }
    index++;
  }
}

MotionCapture::CapturePosition MotionCapture::readPacket()
{
  CapturePosition current;
  current.isValid = false;

  unsigned char buffer[1024];
  int len = zmq_recv(_socket, buffer, 1024 - 1, ZMQ_DONTWAIT);

  if (len != -1)
  {
    len = len > 1024 - 1 ? 1024 - 1 : len;
    buffer[len] = '\0';
    current = parsePacket(buffer);
  }

  return current;
}

MotionCapture::CapturePosition MotionCapture::readLastPacket()
{
  CapturePosition current = readPacket();

  if (current.isValid)
  {
    CapturePosition tmp = readPacket();
    while (tmp.isValid)
    {
      current = tmp;
      tmp = readPacket();
    }
  }

  return current;
}

void MotionCapture::deconnection()
{
  zmq_close(_socket);
  zmq_ctx_destroy(_context);
}

void MotionCapture::quat2Euler(double qx, double qy, double qz, double qw, double& azimuth, double& pitch,
                               double& roll) const
{
  double test = qx * qy + qz * qw;
  if (test > 0.499)
  {
    azimuth = 2.0 * atan2(qx, qw);
    pitch = M_PI / 2.0;
    roll = 0.0;
  }
  else if (test < -0.499)
  {
    azimuth = -2.0 * atan2(qx, qw);
    pitch = -M_PI / 2;
    roll = 0.0;
  }
  else
  {
    double sqx = qx * qx;
    double sqy = qy * qy;
    double sqz = qz * qz;

    azimuth = atan2(2.0 * qy * qw - 2.0 * qx * qz, 1.0 - 2.0 * sqy - 2.0 * sqz);
    pitch = asin(2.0 * test);
    roll = atan2(2.0 * qx * qw - 2.0 * qy * qz, 1.0 - 2.0 * sqx - 2.0 * sqz);
  }

  azimuth = azimuth * 180.0 / M_PI;
  pitch = pitch * 180.0 / M_PI;
  roll = roll * 180.0 / M_PI;
}

/**
 * Do a weighted average between the two given
 * angle in degree
 */
static double angleWeightedMean(double weight1, double angle1, double weight2, double angle2)
{
  double x1 = cos(angle1 * M_PI / 180.0);
  double y1 = sin(angle1 * M_PI / 180.0);
  double x2 = cos(angle2 * M_PI / 180.0);
  double y2 = sin(angle2 * M_PI / 180.0);

  double meanX = weight1 * x1 + weight2 * x2;
  double meanY = weight1 * y1 + weight2 * y2;

  return atan2(meanY, meanX) * 180.0 / M_PI;
}

void MotionCapture::mobileAverage(CapturePosition& state, double x, double y, double z, double azimuth, double pitch,
                                  double roll, unsigned int countInvalid, double alpha)
{
  double coefPow = pow(alpha, countInvalid + 1);
  double coef = 1.0;
  for (unsigned int i = 1; i <= countInvalid; i++)
  {
    coef += pow(alpha, i);
  }

  state.x = coefPow * state.x + (1.0 - alpha) * coef * x;
  state.y = coefPow * state.y + (1.0 - alpha) * coef * y;
  state.z = coefPow * state.z + (1.0 - alpha) * coef * z;
  state.azimuth = angleWeightedMean(coefPow, state.azimuth, (1.0 - alpha) * coef, azimuth);
  state.pitch = angleWeightedMean(coefPow, state.pitch, (1.0 - alpha) * coef, pitch);
  state.roll = angleWeightedMean(coefPow, state.roll, (1.0 - alpha) * coef, roll);
}

}  // namespace Rhoban

#include <stdexcept>
#include <chrono>
#include <thread>
#include "SDKConnection.hpp"

namespace Leph
{
SDKConnection::SDKConnection() : _robot(new CommandsStore, "local")
{
  try
  {
    _robot.connect("127.0.0.1", 7777);
    if (!_robot.isConnected())
    {
      throw std::runtime_error("SDKConnection no connection");
    }

    _robot.loadEnvironment("/home/rhoban/Environments/RhobanServer/Mowgly");

    Rhoban::Motors* motors = _robot.getMotors();
    motors->start(100);
    Rhoban::Sensors* sensors = _robot.getSensors();
    sensors->start(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  catch (const std::string exception)
  {
    throw std::runtime_error("SDKConnection exception: " + exception);
  }
}

SDKConnection::~SDKConnection()
{
  Rhoban::Motors* motors = _robot.getMotors();
  motors->stop();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void SDKConnection::setMotorAngles(const VectorLabel& outputs)
{
  Rhoban::Motors* motors = _robot.getMotors();
  motors->get("Pied G")->setAngle(outputs("output:left foot roll"));
  motors->get("Cheville G")->setAngle(outputs("output:left foot pitch"));
  motors->get("Genou G")->setAngle(outputs("output:left knee"));
  motors->get("Cuisse G")->setAngle(outputs("output:left hip pitch"));
  motors->get("Hanche G Lat")->setAngle(outputs("output:left hip roll"));
  motors->get("Hanche G Rot")->setAngle(outputs("output:left hip yaw"));
  motors->get("Pied D")->setAngle(outputs("output:right foot roll"));
  motors->get("Cheville D")->setAngle(outputs("output:right foot pitch"));
  motors->get("Genou D")->setAngle(outputs("output:right knee"));
  motors->get("Cuisse D")->setAngle(outputs("output:right hip pitch"));
  motors->get("Hanche D Lat")->setAngle(outputs("output:right hip roll"));
  motors->get("Hanche D Rot")->setAngle(outputs("output:right hip yaw"));
}

void SDKConnection::getMotorAngles(VectorLabel& vect)
{
  Rhoban::Motors* motors = _robot.getMotors();
  vect.setOrAppend("motor:left foot roll", motors->get("Pied G")->getRelAngle());
  vect.setOrAppend("motor:left foot pitch", motors->get("Cheville G")->getRelAngle());
  vect.setOrAppend("motor:left knee", motors->get("Genou G")->getRelAngle());
  vect.setOrAppend("motor:left hip pitch", motors->get("Cuisse G")->getRelAngle());
  vect.setOrAppend("motor:left hip roll", motors->get("Hanche G Lat")->getRelAngle());
  vect.setOrAppend("motor:left hip yaw", motors->get("Hanche G Rot")->getRelAngle());
  vect.setOrAppend("motor:right foot roll", motors->get("Pied D")->getRelAngle());
  vect.setOrAppend("motor:right foot pitch", motors->get("Cheville D")->getRelAngle());
  vect.setOrAppend("motor:right knee", motors->get("Genou D")->getRelAngle());
  vect.setOrAppend("motor:right hip pitch", motors->get("Cuisse D")->getRelAngle());
  vect.setOrAppend("motor:right hip roll", motors->get("Hanche D Lat")->getRelAngle());
  vect.setOrAppend("motor:right hip yaw", motors->get("Hanche D Rot")->getRelAngle());
}

void SDKConnection::getSensorValues(VectorLabel& vect)
{
  Rhoban::Sensors* sensors = _robot.getSensors();
  for (const auto& sensor : sensors->getSensors())
  {
    vect.setOrAppend("sensor:" + sensor.second->getName(), sensor.second->getValue());
  }
}

void SDKConnection::init()
{
  _robot.getMotors()->goToInit(5, true);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}
void SDKConnection::compliant()
{
  _robot.emergency();
}

}  // namespace Leph

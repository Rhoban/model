#ifndef LEPH_TRAJECTORYPARAMETERS_HPP
#define LEPH_TRAJECTORYPARAMETERS_HPP

#include <string>
#include <map>
#include <iomanip>
#include <stdexcept>
#include <iostream>
#include <Eigen/Dense>
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "Utils/FileMap.h"

namespace Leph
{
/**
 * TrajectoryParameters
 *
 * Simple header container for mapping
 * name to parameters for CMA-ES optimization
 * and especially trajectory generation
 */
class TrajectoryParameters
{
public:
  /**
   * Initialization
   */
  TrajectoryParameters() : _countOptimized(0), _container()
  {
  }

  /**
   * Define the new given name as parameter
   * with given value and flag isOptimized
   */
  inline void add(const std::string& name, double value = 0.0, bool isOptimized = false)
  {
    // Check parameter exists
    if (isDefined(name))
    {
      throw std::logic_error("TrajectoryParameters exist parameter: " + name);
    }
    // Create parameters
    _container[name] = Parameter();
    _container.at(name).name = name;
    _container.at(name).isOptimized = isOptimized;
    _container.at(name).defaultValue = value;
    _container.at(name).alias = "";
    computeIndex();
  }

  /**
   * Define the new given name as parameter
   * alias for given others parameter name
   * (not optimized)
   */
  inline void cpy(const std::string& name, const std::string& alias)
  {
    // Check parameter exists
    if (isDefined(name))
    {
      throw std::logic_error("TrajectoryParameters exist parameter: " + name);
    }
    if (!isDefined(alias))
    {
      throw std::logic_error("TrajectoryParameters alias not exist: " + alias);
    }
    // Create parameters
    _container[name] = Parameter();
    _container.at(name).name = name;
    _container.at(name).isOptimized = false;
    _container.at(name).defaultValue = 0.0;
    _container.at(name).alias = alias;
    computeIndex();
  }

  /**
   * Return true if given parameter
   * name is already contained
   */
  inline bool isDefined(const std::string& name) const
  {
    return (_container.count(name) != 0);
  }

  /**
   * Return true if given parameter
   * name is an alias
   */
  inline bool isAlias(const std::string& name) const
  {
    checkName(name);
    return (_container.at(name).alias != "");
  }

  /**
   * Enable or disable a parameter optimization
   */
  inline void optimize(const std::string& name, bool isOptimized)
  {
    const std::string& alias = resolveAlias(name);
    _container.at(alias).isOptimized = isOptimized;
    computeIndex();
  }

  /**
   * Return value access to
   * given parameter name.
   * Unknown parameter are not created.
   */
  inline double& set(const std::string& name)
  {
    const std::string& alias = resolveAlias(name);
    return _container.at(alias).defaultValue;
  }

  /**
   * Return value access to
   * given parameter name.
   * Unknown parameter are created
   * with given optimized flag.
   */
  inline double& set(const std::string& name, bool isOptimized)
  {
    if (!isDefined(name))
    {
      add(name, 0.0, isOptimized);
      return _container.at(name).defaultValue;
    }
    else
    {
      const std::string& alias = resolveAlias(name);
      _container.at(alias).isOptimized = isOptimized;
      computeIndex();
      return _container.at(alias).defaultValue;
    }
  }

  /**
   * Build and return the vector of
   * optimized parameters
   */
  inline Eigen::VectorXd buildVector() const
  {
    // Build and assign the vector
    Eigen::VectorXd params = Eigen::VectorXd::Zero(_countOptimized);
    for (const auto& it : _container)
    {
      if (it.second.isOptimized)
      {
        params(it.second.index) = it.second.defaultValue;
      }
    }

    return params;
  }

  /**
   * Build and return the vector of
   * normalization coefficients.
   * Independent of current parameter values.
   * All coefficients are non zero.
   */
  inline Eigen::VectorXd buildNormalizationCoefs() const
  {
    // Build and assign the vector
    Eigen::VectorXd coefs = Eigen::VectorXd::Ones(_countOptimized);
    for (const auto& it : _container)
    {
      if (it.second.isOptimized)
      {
        if (it.first.find("time_ratio") != std::string::npos)
        {
          coefs(it.second.index) = 1.0;
        }
        else if (it.first.find("time_length") != std::string::npos)
        {
          coefs(it.second.index) = 3.0;
        }
        else if (it.first.find("pos_trunk_pos_z") != std::string::npos)
        {
          coefs(it.second.index) = 0.28;
        }
        else if (it.first.find("pos_trunk_pos_x") != std::string::npos ||
                 it.first.find("pos_trunk_pos_y") != std::string::npos)
        {
          coefs(it.second.index) = 0.01;
        }
        else if (it.first.find("vel_trunk_pos_x") != std::string::npos ||
                 it.first.find("vel_trunk_pos_y") != std::string::npos ||
                 it.first.find("vel_trunk_pos_z") != std::string::npos)
        {
          coefs(it.second.index) = 0.05;
        }
        else if (it.first.find("acc_trunk_pos_x") != std::string::npos ||
                 it.first.find("acc_trunk_pos_y") != std::string::npos ||
                 it.first.find("acc_trunk_pos_z") != std::string::npos)
        {
          coefs(it.second.index) = 2.0;
        }
        else if (it.first.find("pos_trunk_axis_x") != std::string::npos ||
                 it.first.find("pos_trunk_axis_y") != std::string::npos ||
                 it.first.find("pos_trunk_axis_z") != std::string::npos ||
                 it.first.find("pos_foot_axis_x") != std::string::npos ||
                 it.first.find("pos_foot_axis_y") != std::string::npos ||
                 it.first.find("pos_foot_axis_z") != std::string::npos)
        {
          coefs(it.second.index) = 1.0;
        }
        else if (it.first.find("vel_trunk_axis_x") != std::string::npos ||
                 it.first.find("vel_trunk_axis_y") != std::string::npos ||
                 it.first.find("vel_trunk_axis_z") != std::string::npos ||
                 it.first.find("vel_foot_axis_x") != std::string::npos ||
                 it.first.find("vel_foot_axis_y") != std::string::npos ||
                 it.first.find("vel_foot_axis_z") != std::string::npos)
        {
          coefs(it.second.index) = 3.0;
        }
        else if (it.first.find("acc_trunk_axis_x") != std::string::npos ||
                 it.first.find("acc_trunk_axis_y") != std::string::npos ||
                 it.first.find("acc_trunk_axis_z") != std::string::npos ||
                 it.first.find("acc_foot_axis_x") != std::string::npos ||
                 it.first.find("acc_foot_axis_y") != std::string::npos ||
                 it.first.find("acc_foot_axis_z") != std::string::npos)
        {
          coefs(it.second.index) = 30.0;
        }
        else if (it.first.find("pos_foot_pos_x") != std::string::npos ||
                 it.first.find("pos_foot_pos_y") != std::string::npos ||
                 it.first.find("pos_foot_pos_z") != std::string::npos)
        {
          coefs(it.second.index) = 0.1;
        }
        else if (it.first.find("vel_foot_pos_x") != std::string::npos ||
                 it.first.find("vel_foot_pos_y") != std::string::npos ||
                 it.first.find("vel_foot_pos_z") != std::string::npos)
        {
          coefs(it.second.index) = 2.0;
        }
        else if (it.first.find("acc_foot_pos_x") != std::string::npos ||
                 it.first.find("acc_foot_pos_y") != std::string::npos ||
                 it.first.find("acc_foot_pos_z") != std::string::npos)
        {
          coefs(it.second.index) = 10.0;
        }
        else
        {
          // Default coeficient
          coefs(it.second.index) = 1.0;
        }
      }
    }

    return coefs;
  }

  /**
   * Retrieve the given parameter name either
   * from unoptimized value of from given
   * parameters vector
   */
  inline double get(const std::string& name, const Eigen::VectorXd& parameters) const
  {
    const std::string& alias = resolveAlias(name);
    // Get from container or vector
    const Parameter& p = _container.at(alias);
    if (p.isOptimized)
    {
      return parameters(p.index);
    }
    else
    {
      return p.defaultValue;
    }
  }

  /**
   * Retrieve the default value from
   * given parameter name
   */
  inline double get(const std::string& name) const
  {
    const std::string& alias = resolveAlias(name);
    // Get from container or vector
    const Parameter& p = _container.at(alias);
    return p.defaultValue;
  }

  /**
   * Retrieve the given vector name if exists
   * either from unoptimized value of from given
   * parameters vector.
   * Convention is _x, _y, _z.
   * If asked data does not exist,
   * zero is filled.
   */
  inline Eigen::Vector3d getVect(const std::string& name, const Eigen::VectorXd& parameters) const
  {
    // Build name
    std::string nameX = name + "_x";
    std::string nameY = name + "_y";
    std::string nameZ = name + "_z";

    // Build vector
    Eigen::Vector3d vect(0.0, 0.0, 0.0);

    // Check if elements exist
    if (isDefined(nameX))
    {
      // Retrieve default value or parameter
      const std::string& aliasX = resolveAlias(nameX);
      const Parameter& pX = _container.at(aliasX);
      if (pX.isOptimized)
      {
        vect.x() = parameters(pX.index);
      }
      else
      {
        vect.x() = pX.defaultValue;
      }
    }
    else
    {
      // Zero if not exist
      vect.x() = 0.0;
    }

    // Check if elements exist
    if (isDefined(nameY))
    {
      // Retrieve default value or parameter
      const std::string& aliasY = resolveAlias(nameY);
      const Parameter& pY = _container.at(aliasY);
      if (pY.isOptimized)
      {
        vect.y() = parameters(pY.index);
      }
      else
      {
        vect.y() = pY.defaultValue;
      }
    }
    else
    {
      // Zero if not exist
      vect.y() = 0.0;
    }

    // Check if elements exist
    if (isDefined(nameZ))
    {
      // Retrieve default value or parameter
      const std::string& aliasZ = resolveAlias(nameZ);
      const Parameter& pZ = _container.at(aliasZ);
      if (pZ.isOptimized)
      {
        vect.z() = parameters(pZ.index);
      }
      else
      {
        vect.z() = pZ.defaultValue;
      }
    }
    else
    {
      // Zero if not exist
      vect.z() = 0.0;
    }

    return vect;
  }

  /**
   * Print with given output stream the
   * current container state
   */
  inline void print(std::ostream& os = std::cout) const
  {
    for (const auto& it : _container)
    {
      os << "[" << std::setw(40) << it.first << ":";
      if (it.second.isOptimized)
      {
        os << std::setw(3) << it.second.index << "] ";
      }
      else
      {
        os << std::setw(5) << "] ";
      }
      if (it.second.alias != "")
      {
        os << it.second.alias;
      }
      else
      {
        os << it.second.defaultValue;
      }
      os << std::endl;
    }
  }

  /**
   * Assign to given Trajectories container
   * at given time the parameters for
   * trunk_pos, trunk_axis, foot_pos, foot_axis,
   * using following name format:
   * prefix_[pos|vel|acc]_[trunk|foot]_[pos|axis]_[x|y|z]
   * (Zero is set to missing data)
   * If doInverse is true, the input cartesian state
   * is supposed to be expressed in the other support foot
   * and inversion is aplied to meet same state as expected but
   * with current supporting foot.
   * If doMirror is true, the input cartesian state
   * is mirrored with respect to X-Z plane (Y translation
   * and X rotation are inversed).
   */
  void trajectoriesAssign(Trajectories& traj, double time, const std::string& prefix, const Eigen::VectorXd& vect,
                          bool doInverse = false, bool doMirror = false) const
  {
    Eigen::Vector3d posTrunkPos = getVect(prefix + "_pos_trunk_pos", vect);
    Eigen::Vector3d velTrunkPos = getVect(prefix + "_vel_trunk_pos", vect);
    Eigen::Vector3d accTrunkPos = getVect(prefix + "_acc_trunk_pos", vect);
    Eigen::Vector3d posTrunkAxis = getVect(prefix + "_pos_trunk_axis", vect);
    Eigen::Vector3d velTrunkAxis = getVect(prefix + "_vel_trunk_axis", vect);
    Eigen::Vector3d accTrunkAxis = getVect(prefix + "_acc_trunk_axis", vect);
    Eigen::Vector3d posFootPos = getVect(prefix + "_pos_foot_pos", vect);
    Eigen::Vector3d velFootPos = getVect(prefix + "_vel_foot_pos", vect);
    Eigen::Vector3d accFootPos = getVect(prefix + "_acc_foot_pos", vect);
    Eigen::Vector3d posFootAxis = getVect(prefix + "_pos_foot_axis", vect);
    Eigen::Vector3d velFootAxis = getVect(prefix + "_vel_foot_axis", vect);
    Eigen::Vector3d accFootAxis = getVect(prefix + "_acc_foot_axis", vect);
    double coef1 = (doInverse ? -1.0 : 1.0);
    double coef2 = (doInverse ? 1.0 : 0.0);
    double coef3 = (doMirror ? -1.0 : 1.0);
    traj.get("trunk_pos_x")
        .addPoint(time, posTrunkPos.x() - coef2 * posFootPos.x(), coef1 * velTrunkPos.x(), coef1 * accTrunkPos.x());
    traj.get("trunk_pos_y")
        .addPoint(time, coef3 * (posTrunkPos.y() - coef2 * posFootPos.y()), coef3 * coef1 * velTrunkPos.y(),
                  coef3 * coef1 * accTrunkPos.y());
    traj.get("trunk_pos_z")
        .addPoint(time, posTrunkPos.z() - coef2 * posFootPos.z(), coef1 * velTrunkPos.z(), coef1 * accTrunkPos.z());
    traj.get("trunk_axis_x")
        .addPoint(time, coef3 * posTrunkAxis.x(), coef3 * velTrunkAxis.x(), coef3 * accTrunkAxis.x());
    traj.get("trunk_axis_y").addPoint(time, posTrunkAxis.y(), velTrunkAxis.y(), accTrunkAxis.y());
    traj.get("trunk_axis_z")
        .addPoint(time, coef3 * posTrunkAxis.z(), coef3 * velTrunkAxis.z(), coef3 * accTrunkAxis.z());
    traj.get("foot_pos_x").addPoint(time, coef1 * posFootPos.x(), coef1 * velFootPos.x(), coef1 * accFootPos.x());
    traj.get("foot_pos_y")
        .addPoint(time, coef3 * coef1 * posFootPos.y(), coef3 * coef1 * velFootPos.y(), coef3 * coef1 * accFootPos.y());
    traj.get("foot_pos_z").addPoint(time, coef1 * posFootPos.z(), coef1 * velFootPos.z(), coef1 * accFootPos.z());
    traj.get("foot_axis_x").addPoint(time, coef3 * posFootAxis.x(), coef3 * velFootAxis.x(), coef3 * accFootAxis.x());
    traj.get("foot_axis_y").addPoint(time, posFootAxis.y(), velFootAxis.y(), accFootAxis.y());
    traj.get("foot_axis_z").addPoint(time, coef3 * posFootAxis.z(), coef3 * velFootAxis.z(), coef3 * accFootAxis.z());
  }

  /**
   * Dump all contained parameters values
   * from unoptimized value of from given
   * parameters vector to given filename.
   */
  inline void exportData(const std::string& filename, const Eigen::VectorXd& parameters) const
  {
    std::map<std::string, double> map;
    for (const auto& it : _container)
    {
      map[it.first] = get(it.first, parameters);
    }
    WriteMap(filename, map);
  }

  /**
   * Load and assign (no parameter creation)
   * parameters value from given filename.
   */
  inline void importData(const std::string& filename)
  {
    std::map<std::string, double> map = ReadMap<std::string, double>(filename);
    for (const auto& it : map)
    {
      set(it.first) = it.second;
    }
  }

private:
  /**
   * Parameter structure
   */
  struct Parameter
  {
    std::string name;
    bool isOptimized;
    double defaultValue;
    size_t index;
    std::string alias;
  };

  /**
   * Number of contained
   * optimized parameters
   */
  size_t _countOptimized;

  /**
   * Mapping name to parameters
   */
  std::map<std::string, Parameter> _container;

  /**
   * Throw exception if given parameter name
   * is not defined
   */
  void checkName(const std::string& name) const
  {
    if (!isDefined(name))
    {
      throw std::logic_error("TrajectoryParameters unknown parameter: " + name);
    }
  }

  /**
   * Return the parameter name (not alias)
   * referenced by given name
   */
  const std::string& resolveAlias(const std::string& name) const
  {
    checkName(name);
    const std::string& alias = _container.at(name).alias;
    if (alias == "")
    {
      return name;
    }
    else
    {
      return resolveAlias(alias);
    }
  }

  /**
   * Recompute all optimized parameter index
   */
  void computeIndex()
  {
    // Count optimized parameters
    // and reset index
    _countOptimized = 0;
    for (auto& it : _container)
    {
      if (it.second.isOptimized)
      {
        it.second.index = _countOptimized;
        _countOptimized++;
      }
      else
      {
        it.second.index = (size_t)-1;
      }
    }
  }
};

}  // namespace Leph

#endif

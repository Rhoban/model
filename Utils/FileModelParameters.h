#ifndef LEPH_FILEMODELPARAMETERS_HPP
#define LEPH_FILEMODELPARAMETERS_HPP

#include <map>
#include <string>
#include <Eigen/Dense>

namespace Leph
{
/**
 * Write to given filename the given
 * model parameters joint, inertia and geometry.
 */
void WriteModelParameters(const std::string& filename, const Eigen::MatrixXd& jointData,
                          const std::map<std::string, size_t>& jointName, const Eigen::MatrixXd& inertiaData,
                          const std::map<std::string, size_t>& inertiaName, const Eigen::MatrixXd& geometryData,
                          const std::map<std::string, size_t>& geometryName);

/**
 * Read from given filename and assign the given
 * model parameters joint, inertia and geometry.
 */
void ReadModelParameters(const std::string& filename, Eigen::MatrixXd& jointData,
                         std::map<std::string, size_t>& jointName, Eigen::MatrixXd& inertiaData,
                         std::map<std::string, size_t>& inertiaName, Eigen::MatrixXd& geometryData,
                         std::map<std::string, size_t>& geometryName);

}  // namespace Leph

#endif

#ifndef LEPH_COMPUTEMODELDATA_HPP
#define LEPH_COMPUTEMODELDATA_HPP

#include "Types/MatrixLabel.hpp"
#include "Model/HumanoidModel.hpp"

namespace Leph
{
/**
 * Append data to given log in MatrixLabel
 * and compute Model data
 */
void ComputeModelData(MatrixLabel& logs, RobotType type = SigmabanModel);

}  // namespace Leph

#endif

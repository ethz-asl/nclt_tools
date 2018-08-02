#ifndef INCLUDE_NCLT_TOOLS_TRANSFORMATION_HELPERS_H_
#define INCLUDE_NCLT_TOOLS_TRANSFORMATION_HELPERS_H_

#include <Eigen/Dense>

namespace nclt {

Eigen::Matrix3d eulerAnglesToRotationMatrix(
    const double roll_rad, const double pitch_rad, const double yaw_rad);

}  // namespace nclt

#endif  // INCLUDE_NCLT_TOOLS_TRANSFORMATION_HELPERS_H_

#include <nclt-tools/transformation-helpers.h>

namespace nclt {

Eigen::Matrix3d eulerAnglesToRotationMatrix(
    const double roll_rad, const double pitch_rad, const double yaw_rad) {
  Eigen::Matrix3d C_yaw;
  C_yaw << std::cos(yaw_rad), std::sin(yaw_rad), 0.0, -std::sin(yaw_rad),
      std::cos(yaw_rad), 0.0, 0.0, 0.0, 1.0;

  Eigen::Matrix3d C_pitch;
  C_pitch << std::cos(pitch_rad), 0.0, -std::sin(pitch_rad), 0.0, 1.0, 0.0,
      std::sin(pitch_rad), 0.0, std::cos(pitch_rad);

  Eigen::Matrix3d C_roll;
  C_roll << 1.0, 0.0, 0.0, 0.0, std::cos(roll_rad), std::sin(roll_rad), 0.0,
      -std::sin(roll_rad), std::cos(roll_rad);

  return C_yaw.transpose() * C_pitch.transpose() * C_roll.transpose();
}

}  // namespace nclt

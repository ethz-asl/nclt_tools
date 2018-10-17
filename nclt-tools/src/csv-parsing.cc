#include "nclt-tools/csv-parsing.h"

#include "nclt-tools/transformation-helpers.h"

namespace nclt {

void getTimeAndTransformationFromDataRecord(
    const std::vector<std::string>& data_record, int64_t* timestamp_nanoseconds,
    aslam::Transformation* T_fi_fo) {
  CHECK_NOTNULL(timestamp_nanoseconds);
  CHECK_NOTNULL(T_fi_fo);

  CHECK_EQ(data_record.size(), 7u);

  constexpr int64_t kMicrosecondsToNanoseconds = 1000;
  *timestamp_nanoseconds = static_cast<int64_t>(
      std::atoll(data_record[0].c_str()) * kMicrosecondsToNanoseconds);

  aslam::Position3D t_fi_fo(
      std::atof(data_record[1].c_str()), std::atof(data_record[2].c_str()),
      std::atof(data_record[3].c_str()));

  const double euler_roll_rad = std::atof(data_record[4].c_str());
  const double euler_pitch_rad = std::atof(data_record[5].c_str());
  const double euler_yaw_rad = std::atof(data_record[6].c_str());

  const kindr::minimal::RotationQuaternion::RotationMatrix C_fi_fo =
      eulerAnglesToRotationMatrix(
          euler_roll_rad, euler_pitch_rad, euler_yaw_rad);

  aslam::Quaternion q_fi_fo(C_fi_fo);
  *T_fi_fo = aslam::Transformation(q_fi_fo, t_fi_fo);
}

}  // namespace nclt

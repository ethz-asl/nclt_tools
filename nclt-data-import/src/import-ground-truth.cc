#include "nclt-data-import/import-ground-truth.h"

#include <Eigen/Core>
#include <glog/logging.h>
#include <maplab-common/csv-file-reader.h>
#include <maplab-common/progress-bar.h>
#include <nclt-tools/csv-parsing.h>
#include <sensors/ground-truth.h>
#include <vi-map/vi-map.h>
#include <vi-map/sensor-manager.h>

namespace nclt {

aslam::Transformation get_T_Bzup_Bzdown() {
  aslam::Position3D p_Bzup_Bzdown = aslam::Position3D::Zero();
  aslam::Quaternion::RotationMatrix R_Bzup_Bzdown;
  R_Bzup_Bzdown << 1.0,  0.0,  0.0,
                   0.0, -1.0,  0.0,
                   0.0,  0.0, -1.0;
  return aslam::Transformation(aslam::Quaternion(R_Bzup_Bzdown), p_Bzup_Bzdown);
}

void importGroundTruthDataFromCsv(
    const std::string& csv_filepath, const vi_map::SensorId& sensor_id,
    const vi_map::MissionId& mission_id, vi_map::VIMap* map) {
  CHECK_NOTNULL(map)->hasMission(mission_id);
  CHECK(common::fileExists(csv_filepath));

  vi_map::SensorManager* sensor_manager =
      CHECK_NOTNULL(map->getSensorManagerMutable());

  CHECK(sensor_manager->hasSensor(sensor_id));

  const aslam::Transformation T_Bzup_Bzdown = get_T_Bzup_Bzdown();

  constexpr char kDelimiter = ',';
  common::CsvFileReader csv_file_reader(csv_filepath, kDelimiter);

  const size_t num_lines = csv_file_reader.getNumLines();
  common::ProgressBar progress_bar(num_lines);

  constexpr size_t kNumFields = 7u;
  std::vector<std::string> line_with_fields;
  while (csv_file_reader.getNextLine(&line_with_fields)) {
    CHECK_EQ(line_with_fields.size(), kNumFields);

    int64_t timestamp_ns = aslam::time::getInvalidTime();
    aslam::Transformation gt_T_B0_B_zdown;
    getTimeAndTransformationFromDataRecord(
        line_with_fields, &timestamp_ns, &gt_T_B0_B_zdown);
    CHECK(aslam::time::isValidTime(timestamp_ns));

    const aslam::Transformation T_GT_B =
        T_Bzup_Bzdown * gt_T_B0_B_zdown * T_Bzup_Bzdown.inverse();

    if (!sensor_manager->isSensorAssociatedWithMission(sensor_id, mission_id)) {
      sensor_manager->associateExistingSensorWithMission(sensor_id, mission_id);
    }

    const vi_map::GroundTruthMeasurement ground_truth_measurement(
        sensor_id, timestamp_ns, T_GT_B);
    map->addOptionalSensorMeasurement(ground_truth_measurement, mission_id);

    progress_bar.increment();
  }

  const vi_map::OptionalSensorData& optional_sensor_data =
      map->getOptionalSensorData(mission_id);
  vi_map::SensorIdSet sensor_ids;
  optional_sensor_data.getAllSensorIds(&sensor_ids);
  LOG(INFO) << "num sensors: " << sensor_ids.size();
  CHECK(sensor_ids.count(sensor_id));
}

}  // namespace nclt

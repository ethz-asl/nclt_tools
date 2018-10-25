#ifndef INCLUDE_NCLT_DATA_IMPORT_IMPORT_GROUND_TRUTH_H_
#define INCLUDE_NCLT_DATA_IMPORT_IMPORT_GROUND_TRUTH_H_

#include <string>

#include <sensors/sensor.h>
#include <vi-map/unique-id.h>

namespace vi_map {
class VIMap;
}  // namespace vi_map

namespace nclt {

void importGroundTruthDataFromCsv(
    const std::string& csv_filepath, const vi_map::SensorId& sensor_id,
    const vi_map::MissionId& mission_id, vi_map::VIMap* map);

}  // namespace nclt

#endif /* INCLUDE_NCLT_DATA_IMPORT_IMPORT_GROUND_TRUTH_H_ */

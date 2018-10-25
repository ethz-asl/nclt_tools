#include "nclt-plugin/nclt-plugin.h"

#include <console-common/console.h>
#include <map-manager/map-manager.h>
#include <vi-map-helpers/import-sensor.h>
#include <nclt-data-import/import-ground-truth.h>
#include <sensors/ground-truth.h>
#include <vi-map/vi-map.h>

DECLARE_string(map_mission);
DECLARE_string(csv_file);

namespace nclt {

NcltPlugin::NcltPlugin(common::Console* console)
    : common::ConsolePluginBase(console) {
  addCommand(
      {"import_nclt_ground_truth_data_from_csv"},
      [this]() -> int { return importGroundTruthDataFromCsv(); }, "....",
      common::Processing::Sync);
}

int NcltPlugin::importGroundTruthDataFromCsv() const {
  const std::string& csv_file = FLAGS_csv_file;
  if (csv_file.empty()) {
    LOG(ERROR) << "The specified csv file parameter is empty. "
               << "Please specify a valid bag file with --csv_file.";
    return common::kStupidUserError;
  }

  if (!common::fileExists(csv_file)) {
    LOG(ERROR) << "The specified csv file does not "
               << "exist on the file-system. Please point to an existing csv "
               << "file with --csv_file.";
    return common::kStupidUserError;
  }

  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);


  vi_map::MissionId mission_id;
  map->ensureMissionIdValid(FLAGS_map_mission, &mission_id);
  if (!mission_id.isValid()) {
    LOG(ERROR) << "Mission ID invalid. Specify a valid mission id with "
                  "--map_mission.";
    return common::kUnknownError;
  }

  vi_map::SensorId sensor_id;
  if (!vi_map_helpers::importSensor<vi_map::GroundTruth>(map.get(), &sensor_id)) {
    return common::kStupidUserError;
  }

  CHECK(sensor_id.isValid());

  nclt::importGroundTruthDataFromCsv(csv_file, sensor_id, mission_id, map.get());

  return common::kSuccess;
}

}  // namespace nclt

MAPLAB_CREATE_CONSOLE_PLUGIN(nclt::NcltPlugin);

#include <base/PolygonMaze.h>
#include <utils/Log.h>
#include "base/PlannerSettings.h"

#include <fstream>
#include <iomanip>
#include <iostream>

namespace og = ompl::geometric;

/**
 * Generate a JSON file with all the polygon maze environments.
 */
int main(int argc, char **argv) {
  nlohmann::json info;

  global::settings.env.type = "grid";
  global::settings.env.grid.width = 100;
  global::settings.env.grid.height = 100;
  global::settings.env.grid.generator = "corridor";
  global::settings.env.grid.corridor.branches = 50;
  global::settings.env.grid.corridor.radius = 5;
  global::settings.benchmark.runs = 5;
  global::settings.benchmark.steer_functions = {
      Steering::STEER_TYPE_REEDS_SHEPP};
  Log::instantiateRun();
  for (int radius : {2, 3, 4, 5, 6}) {
    global::settings.steer.car_turning_radius = static_cast<double>(radius);
    // save benchmark configuradiusn
    global::settings.benchmark.log_file =
        "turning_radius" + std::to_string(radius);
    std::ofstream o("../benchmarks/turning_radius_" + std::to_string(radius) +
                    ".json");
    o << std::setw(2) << nlohmann::json(global::settings);
    o.close();
  }

  return EXIT_SUCCESS;
}

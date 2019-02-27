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
  global::settings.benchmark.runs = 5;
  Log::instantiateRun();
  for (int seed = 1; seed <= 5; ++seed) {
    for (int radius = 3; radius <= 8; ++radius) {
      global::settings.env.grid.corridor.radius = radius;
      global::settings.env.grid.seed = seed;
      global::settings.env.createEnvironment();

      global::settings.environment->to_json(info["environment"]);
      info["settings"] = nlohmann::json(global::settings)["settings"];
      Log::log(info);

      if (seed == 1) {
        // save benchmark configuration
        global::settings.benchmark.log_file =
            "corridor_radius_" + std::to_string(radius);
        std::ofstream o("../benchmarks/corridor_radius_" +
                        std::to_string(radius) + ".json");
        o << std::setw(2) << nlohmann::json(global::settings);
        o.close();
      }
    }
  }

  Log::save("corridor_radii.json");

  return EXIT_SUCCESS;
}

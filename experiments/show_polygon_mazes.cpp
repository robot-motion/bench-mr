#include <base/PolygonMaze.h>
#include <utils/Log.h>
#include "base/PlannerSettings.h"

namespace og = ompl::geometric;

/**
 * Generate a JSON file with all the polygon maze environments.
 */
int main(int argc, char **argv) {
  nlohmann::json info;

  std::vector<std::string> mazes{"parking1.svg", "parking2.svg", "parking3.svg",
                                 "warehouse.svg"};
  for (const auto &maze_filename : mazes) {
    auto maze = PolygonMaze::loadFromSvg("polygon_mazes/" + maze_filename);
    maze.to_json(info[maze_filename]);
  }

  Log::instantiateRun();
  Log::log(info);

  Log::save("polygon_mazes.json");

  return EXIT_SUCCESS;
}

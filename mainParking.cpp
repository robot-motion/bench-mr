#include <base/PolygonMaze.h>
#include "base/PlannerSettings.h"

#include "metrics/PathLengthMetric.h"

#include "planners/OMPLPlanner.hpp"
#include "planners/chomp/Chomp.h"
#include "planners/sbpl/SbplPlanner.h"
#include "planners/thetastar/ThetaStar.h"

#include "utils/ScenarioLoader.h"

#include "utils/PathEvaluation.hpp"

#include "PostSmoothing.h"

namespace og = ompl::geometric;

int main(int argc, char **argv) {
  if (argc < 3) {
    OMPL_ERROR(
        "Missing Parameter(s): SVG file names for the maze and the "
        "robot.");
    return EXIT_FAILURE;
  }

  const std::string maze_filename = argv[1];
  const std::string robot_filename = argv[2];

  auto maze = PolygonMaze::loadFromSvg(maze_filename);
  maze.setStart({0, -50});
  std::cout << "Start collides? " << std::boolalpha
            << maze.collides(maze.start().x, maze.start().y) << std::endl;
  std::cout << "Goal collides?  " << std::boolalpha
            << maze.collides(maze.goal().x, maze.goal().y) << std::endl;
  maze.setGoal({200, -150});
  global::settings.environment = &maze;
  global::settings.environment->setThetas(M_PI / 2, M_PI);

  global::settings.steer.steering_type = Steering::STEER_TYPE_REEDS_SHEPP;
  global::settings.steer.car_turning_radius = 100;
  global::settings.steer.initializeSteering();

  global::settings.collision_model = robot::ROBOT_POLYGON;
  global::settings.robot_shape = SvgPolygonLoader::load(robot_filename)[0];
  global::settings.robot_shape.value().center();
  for (auto &p : global::settings.robot_shape.value().points)
    std::cout << p << std::endl;

  std::vector<Point> gripsPath;
  std::vector<GNode> gripsTrajectory;

  auto info = nlohmann::json({{"plans", {}}, {"environment", maze}});

  Log::instantiateRun();
  //  PathEvaluation::evaluate<ChompPlanner>(info);
  //  PathEvaluation::evaluate<ThetaStar>(info);
  //  PathEvaluation::evaluate<RRTPlanner>(info);
  PathEvaluation::evaluate<RRTstarPlanner>(info);
  //  PathEvaluation::evaluate<RRTsharpPlanner>(info);
  //  PathEvaluation::evaluate<InformedRRTstarPlanner>(info);
  //  PathEvaluation::evaluate<SORRTstarPlanner>(info);
  //  PathEvaluation::evaluate<CForestPlanner>(info);
  //  PathEvaluation::evaluate<SbplPlanner>(info);

  Log::log(info);

  Log::save("parking.json");

  return EXIT_SUCCESS;
}

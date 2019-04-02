#include <base/PolygonMaze.h>
#include "base/PlannerSettings.h"

#include "metrics/PathLengthMetric.h"

#include "planners/OMPLPlanner.hpp"
#include "planners/sbpl/SbplPlanner.h"
#include "planners/thetastar/ThetaStar.h"

#include "utils/ScenarioLoader.h"

#include "utils/PathEvaluation.hpp"

#include "smoothers/grips/GRIPS.h"

namespace og = ompl::geometric;

/**
 * Parking scenario where the environment consists of polygons as obstacles and
 * the robot is represented by a convex shape.
 */
int main(int argc, char **argv) {
  if (argc < 3) {
    OMPL_ERROR(
        "Missing Parameter(s): SVG file names for the maze and the "
        "robot.");
    return EXIT_FAILURE;
  }

  const std::string maze_filename = argv[1];
  const std::string robot_filename = argv[2];

  auto *maze = PolygonMaze::loadFromSvg(maze_filename);
  maze->setStart({0, -50});
  std::cout << "Start collides? " << std::boolalpha
            << maze->collides(maze->start().x, maze->start().y) << std::endl;
  std::cout << "Goal collides?  " << std::boolalpha
            << maze->collides(maze->goal().x, maze->goal().y) << std::endl;
  maze->setGoal({200, -150});
  global::settings.estimate_theta = true;
  global::settings.environment = maze;
  global::settings.environment->setThetas(M_PI / 2, M_PI);

  global::settings.steer.steering_type = Steering::STEER_TYPE_REEDS_SHEPP;
  global::settings.steer.car_turning_radius = 100;
  global::settings.steer.initializeSteering();

  global::settings.env.collision.collision_model = robot::ROBOT_POLYGON;
  global::settings.env.collision.robot_shape_source = robot_filename;
  global::settings.env.collision.robot_shape = SvgPolygonLoader::load(robot_filename)[0];
  global::settings.env.collision.robot_shape.value().center();

  auto info = nlohmann::json({{"plans", {}}});
  global::settings.environment->to_json(info["environment"]);

  Log::instantiateRun();
  //  PathEvaluation::evaluate<ChompPlanner>(info);
  //  PathEvaluation::evaluate<ThetaStar>(info);
  //  PathEvaluation::evaluate<RRTPlanner>(info);
//  PathEvaluation::evaluate<RRTstarPlanner>(info);
  //  PathEvaluation::evaluate<RRTsharpPlanner>(info);
    PathEvaluation::evaluate<InformedRRTstarPlanner>(info);
  //  PathEvaluation::evaluate<SORRTstarPlanner>(info);
  //  PathEvaluation::evaluate<CForestPlanner>(info);
  //  PathEvaluation::evaluate<SbplPlanner>(info);

  Log::log(info);

  Log::save("parking.json");

  return EXIT_SUCCESS;
}

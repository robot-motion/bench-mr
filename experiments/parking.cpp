#include "base/PlannerSettings.h"
#include "base/environments/PolygonMaze.h"
#include "planners/OMPLPlanner.hpp"
#include "utils/PathEvaluation.hpp"
#include "utils/ScenarioLoader.h"

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
  maze->setStart({0.0, -2.27});
  maze->setGoal({7.72, -7.72});
  maze->setThetas(0, -M_PI_2);
  global::settings.environment = maze;

  global::settings.steer.steering_type = Steering::STEER_TYPE_REEDS_SHEPP;
  global::settings.steer.car_turning_radius = 4;
  global::settings.steer.initializeSteering();

  global::settings.env.collision.collision_model = robot::ROBOT_POLYGON;
  global::settings.env.collision.robot_shape_source = robot_filename;
  global::settings.env.collision.initializeCollisionModel();

  std::cout << "Start valid? " << std::boolalpha
            << maze->checkValidity(maze->start().toState(maze->startTheta()))
            << std::endl;
  std::cout << "Goal valid?  " << std::boolalpha
            << maze->checkValidity(maze->goal().toState(maze->goalTheta()))
            << std::endl;

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

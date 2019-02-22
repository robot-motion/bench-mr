#include <base/PolygonMaze.h>
#include "base/PlannerSettings.h"

#include "metrics/PathLengthMetric.h"

#include "planners/OMPLPlanner.hpp"
#include "planners/chomp/Chomp.h"
#include "planners/sbpl/SbplPlanner.h"
#include "planners/thetastar/ThetaStar.h"

#include "utils/ScenarioLoader.h"

#include "gui/PathEvaluation.h"
#if QT_SUPPORT
#include "gui/QtVisualizer.h"
#endif

#include "PostSmoothing.h"

namespace og = ompl::geometric;

void printStats(const PathStatistics &stats) {
  std::cout << stats.planner << std::endl;
  std::cout << "\tPath length:   \t" << stats.path_length << std::endl;
  std::cout << "\tMax curvature: \t" << stats.curvature << std::endl;
}

template <class PLANNER>
void evaluate(nlohmann::json &info) {
  PLANNER planner;
  PathStatistics stats(planner.name());
  auto &j = info["plans"][planner.name()];
  OMPL_INFORM(("Running " + planner.name() + "...").c_str());
  if (planner.run()) {
    PathEvaluation::evaluate(stats, planner.solution(), &planner);
    stats.path_found = true;
    j["path"] = Log::serializePath(planner.solutionPath());
  } else {
    j["path"] = Log::serializePath({});
  }
  std::cout << stats << std::endl;
  j["trajectory"] = Log::serializeTrajectory(planner.solution());
  j["stats"] = nlohmann::json(stats)["stats"];
}

int main(int argc, char **argv) {
  PathEvaluation::initialize();

#if QT_SUPPORT
  QtVisualizer::initialize();
#endif

  if (argc < 3) {
    OMPL_ERROR(
        "Missing Parameter(s). Provide SVG files for the maze and the robot.");
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
  global::settings.environment->setThetas(M_PI/2, M_PI);

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
  //  evaluate<ChompPlanner>(info);
//    evaluate<ThetaStar>(info);
//  evaluate<RRTPlanner>(info);
  evaluate<RRTstarPlanner>(info);
//  evaluate<RRTsharpPlanner>(info);
//  evaluate<InformedRRTstarPlanner>(info);
//  evaluate<SORRTstarPlanner>(info);
  //  evaluate<CForestPlanner>(info);
  //  evaluate<SbplPlanner>(info);

  Log::log(info);

  Log::save("parking.json");

#if DEBUG
  QtVisualizer::show();
  return QtVisualizer::exec();
#else
  return EXIT_SUCCESS;
#endif
}

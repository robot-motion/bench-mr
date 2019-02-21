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
  std::cout << "\tPath length:   \t" << stats.pathLength << std::endl;
  std::cout << "\tMax curvature: \t" << stats.curvature << std::endl;
}

template <class PLANNER>
void evaluate(nlohmann::json &info) {
  PLANNER planner;
  PathStatistics stats;
  stats.planner = planner.name();
  auto &j = info["plans"][planner.name()];
  OMPL_INFORM(("Running " + planner.name() + "...").c_str());
  if (planner.run()) {
    stats = PathEvaluation::evaluate(planner.solution(), planner.name());
    j["path"] = Log::serializePath(planner.solutionPath());
    j["smoothness"] = planner.solution().smoothness();
  } else {
    j["path"] = Log::serializePath({});
    j["smoothness"] = std::numeric_limits<double>::quiet_NaN();
  }
  printStats(stats);
  j["curvature"] = stats.curvature;
  j["pathLength"] = stats.pathLength;
  j["steps"] = std::nan("N/A");
  j["time"] = planner.planningTime();
  j["meanClearingDistance"] = stats.meanClearingDistance;
  j["medianClearingDistance"] = stats.medianClearingDistance;
  j["minClearingDistance"] = stats.minClearingDistance;
  j["maxClearingDistance"] = stats.maxClearingDistance;
  j["trajectory"] = Log::serializeTrajectory(planner.solution());
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
  settings.environment = &maze;
  settings.environment->setThetas(M_PI/2, 0);

  settings.steer.steering_type = Steering::STEER_TYPE_REEDS_SHEPP;
  settings.steer.car_turning_radius = 50;
  settings.steer.initializeSteering();

  settings.collision_model = robot::ROBOT_POLYGON;
  settings.robot_shape = SvgPolygonLoader::load(robot_filename)[0];
  settings.robot_shape.value().center();
  for (auto &p : settings.robot_shape.value().points)
    std::cout << p << std::endl;

  std::vector<Point> gripsPath;
  std::vector<GNode> gripsTrajectory;

  auto info = nlohmann::json({{"plans", {}}, {"environment", maze}});

  Log::instantiateRun();
  //  evaluate<ChompPlanner>(info);
//    evaluate<ThetaStar>(info);
//  evaluate<RRTPlanner>(info);
  evaluate<RRTstarPlanner>(info);
  evaluate<RRTsharpPlanner>(info);
  evaluate<InformedRRTstarPlanner>(info);
  evaluate<SORRTstarPlanner>(info);
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

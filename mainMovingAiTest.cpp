//#define DEBUG 1 // TODO activate DEBUG in Plannerglobal::settings.h

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
  global::settings.steer.steering_type = Steering::STEER_TYPE_POSQ;
  PathEvaluation::initialize();

#if QT_SUPPORT
  QtVisualizer::initialize();
#endif

  Log::instantiateRun();

  if (argc < 2) {
    OMPL_ERROR(
        "Missing Parameter. Enter .scen file name after execute command.");
    return 1;
  }

  const std::string scene_name = argv[1];

  ScenarioLoader scenarioLoader;
  scenarioLoader.load(scene_name);

  unsigned int counter = 0;
  std::cout << "Loaded " << scenarioLoader.scenarios().size() << " scenarios."
            << std::endl;
  for (auto &scenario : scenarioLoader.scenarios()) {
    if (counter++ < 925) continue;

    std::cout << "##############################################" << std::endl;
    std::cout << "# Scenario " << counter << std::endl;
    std::cout << "##############################################" << std::endl;

    // create environment
    global::settings.environment =
        GridMaze::createFromMovingAiScenario(scenario);
    global::settings.steer.initializeSteering();
#if QT_SUPPORT
    QtVisualizer::visualize(global::settings.environment, 0);
#endif
    PathStatistics thetaStarStats, rrtStarStats, gripsStats,
        smoothThetaStarStats, sbplStats, chompStats, rrtStats;

    std::vector<Point> gripsPath;
    std::vector<GNode> gripsTrajectory;

    auto info = nlohmann::json({{"plans", {}},
                                {"environment", *global::settings.environment},
                                {"optimalDistance", scenario.optimal_length}});

    evaluate<ChompPlanner>(info);
    evaluate<ThetaStar>(info);
    evaluate<RRTPlanner>(info);
    evaluate<RRTstarPlanner>(info);
    evaluate<RRTsharpPlanner>(info);
    evaluate<InformedRRTstarPlanner>(info);
    evaluate<SORRTstarPlanner>(info);
    evaluate<CForestPlanner>(info);
    evaluate<SbplPlanner>(info);

    Log::log(info);

    //    if (++counter > 10)
    //        break;
  }

  Log::save();

#if DEBUG
  QtVisualizer::show();
  return QtVisualizer::exec();
#else
  return EXIT_SUCCESS;
#endif
}

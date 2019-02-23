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
  global::settings.steer.steering_type = Steering::STEER_TYPE_POSQ;

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
    PathStatistics thetaStarStats, rrtStarStats, gripsStats,
        smoothThetaStarStats, sbplStats, chompStats, rrtStats;

    std::vector<Point> gripsPath;
    std::vector<GNode> gripsTrajectory;

    auto info = nlohmann::json({{"plans", {}},
                                {"optimalDistance", scenario.optimal_length}});
    global::settings.environment->to_json(info["environment"]);

    PathEvaluation::evaluate<ChompPlanner>(info);
    PathEvaluation::evaluate<ThetaStar>(info);
    PathEvaluation::evaluate<RRTPlanner>(info);
    PathEvaluation::evaluate<RRTstarPlanner>(info);
    PathEvaluation::evaluate<RRTsharpPlanner>(info);
    PathEvaluation::evaluate<InformedRRTstarPlanner>(info);
    PathEvaluation::evaluate<SORRTstarPlanner>(info);
    PathEvaluation::evaluate<CForestPlanner>(info);
    PathEvaluation::evaluate<SbplPlanner>(info);

    Log::log(info);

    //    if (++counter > 10)
    //        break;
  }

  Log::save();

  return EXIT_SUCCESS;
}

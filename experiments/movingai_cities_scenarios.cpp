#include "base/PlannerSettings.h"
#include "base/environments/GridMaze.h"
#include "metrics/PathLengthMetric.h"
#include "planners/OMPLPlanner.hpp"
#include "planners/sbpl/SbplPlanner.h"
#include "planners/thetastar/ThetaStar.h"
#include "smoothers/grips/GRIPS.h"
#include "utils/PathEvaluation.hpp"
#include "utils/ScenarioLoader.h"

namespace og = ompl::geometric;

int main(int argc, char **argv) {
  global::settings.steer.steering_type = Steering::STEER_TYPE_REEDS_SHEPP;

  Log::instantiateRun();

  if (argc < 2) {
    OMPL_ERROR(
        ("USAGE: " + std::string(argv[0]) + " scene_file number_of_scenarios")
            .c_str());
    return 1;
  }

  const std::string scene_name = argv[1];
  ScenarioLoader scenarioLoader;
  scenarioLoader.load(scene_name);

  int scenarioLimit = 5;
  if (argc > 2) {
    const std::string scenarioLimitStr = argv[2];
    scenarioLimit = std::stoi(scenarioLimitStr);
  }

  int counter = 0;
  std::cout << "Loaded " << scenarioLoader.scenarios().size() << " scenarios."
            << std::endl;
  for (auto &scenario : scenarioLoader.scenarios()) {
    if (counter++ < scenarioLoader.scenarios().size() - scenarioLimit) continue;

    std::cout << "##############################################" << std::endl;
    std::cout << "# Scenario " << counter << std::endl;
    std::cout << "##############################################" << std::endl;

    // create environment
    delete global::settings.environment;
    global::settings.environment =
        GridMaze::createFromMovingAiScenario(scenario);
    global::settings.steer.initializeSteering();

    auto info = nlohmann::json(
        {{"plans", {}}, {"optimalDistance", scenario.optimal_length}});
    global::settings.environment->to_json(info["environment"]);

    //    PathEvaluation::evaluate<ThetaStar>(info);
    PathEvaluation::evaluate<RRTPlanner>(info);
    //    PathEvaluation::evaluate<RRTstarPlanner>(info);
    //    PathEvaluation::evaluate<RRTsharpPlanner>(info);
    //    PathEvaluation::evaluate<InformedRRTstarPlanner>(info);
    //    PathEvaluation::evaluate<SORRTstarPlanner>(info);
    //    PathEvaluation::evaluate<CForestPlanner>(info);
    //    PathEvaluation::evaluate<SbplPlanner>(info);

    Log::log(info);
  }

  Log::save();

  return EXIT_SUCCESS;
}

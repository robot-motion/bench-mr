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

  global::settings.auto_choose_distance_computation_method = false;

  global::settings.distance_computation_method =
      distance_computation::BRUTE_FORCE;
  // create environment
  auto *grid1 =
      GridMaze::createFromMovingAiScenario(scenarioLoader.scenarios().front());
  grid1->computeDistances();
  global::settings.environment = grid1;
  auto info1 = nlohmann::json({{"plans", {}}});
  global::settings.environment->to_json(info1["environment"]);
  Log::log(info1);

  global::settings.distance_computation_method =
      distance_computation::DEAD_RECKONING;
  // create environment
  auto *grid2 =
      GridMaze::createFromMovingAiScenario(scenarioLoader.scenarios().front());
  grid2->computeDistances();
  global::settings.environment = grid2;
  auto info2 = nlohmann::json({{"plans", {}}});
  global::settings.environment->to_json(info2["environment"]);
  Log::log(info2);

  Log::save();

  return EXIT_SUCCESS;
}

#include "base/PlannerSettings.h"

#include "planners/OMPLPlanner.hpp"
#include "planners/sbpl/SbplPlanner.h"
#include "planners/thetastar/ThetaStar.h"

#include "utils/PathEvaluation.hpp"

namespace og = ompl::geometric;

void evaluatePlanners(nlohmann::json &info) {
  if (global::settings.benchmark.planning.bfmt)
    PathEvaluation::evaluateSmoothers<BFMTPlanner>(info);
  if (global::settings.benchmark.planning.cforest)
    PathEvaluation::evaluateSmoothers<CForestPlanner>(info);
  if (global::settings.benchmark.planning.est)
    PathEvaluation::evaluateSmoothers<ESTPlanner>(info);
  if (global::settings.benchmark.planning.fmt)
    PathEvaluation::evaluateSmoothers<FMTPlanner>(info);
  if (global::settings.benchmark.planning.informed_rrt_star)
    PathEvaluation::evaluateSmoothers<InformedRRTstarPlanner>(info);
  if (global::settings.benchmark.planning.kpiece1)
    PathEvaluation::evaluateSmoothers<KPIECEPlanner>(info);
  if (global::settings.benchmark.planning.prm_star)
    PathEvaluation::evaluateSmoothers<PRMstarPlanner>(info);
  if (global::settings.benchmark.planning.prm_star)
    PathEvaluation::evaluateSmoothers<PRMstarPlanner>(info);
  if (global::settings.benchmark.planning.rrt)
    PathEvaluation::evaluateSmoothers<RRTPlanner>(info);
  if (global::settings.benchmark.planning.rrt_sharp)
    PathEvaluation::evaluateSmoothers<RRTsharpPlanner>(info);
  if (global::settings.benchmark.planning.rrt_star)
    PathEvaluation::evaluateSmoothers<RRTstarPlanner>(info);
  if (global::settings.benchmark.planning.sbl)
    PathEvaluation::evaluateSmoothers<SBLPlanner>(info);
  if (global::settings.benchmark.planning.sbpl)
    PathEvaluation::evaluateSmoothers<SbplPlanner>(info);
  if (global::settings.benchmark.planning.sorrt_star)
    PathEvaluation::evaluateSmoothers<SORRTstarPlanner>(info);
  if (global::settings.benchmark.planning.sst)
    PathEvaluation::evaluateSmoothers<SSTPlanner>(info);
  if (global::settings.benchmark.planning.stride)
    PathEvaluation::evaluateSmoothers<STRIDEPlanner>(info);
  if (global::settings.benchmark.planning.theta_star)
    PathEvaluation::evaluateSmoothers<ThetaStar>(info);
}

int main(int argc, char **argv) {
  //  std::cout << std::setw(2) <<
  //  nlohmann::json::parse(nlohmann::json(global::settings).dump()) <<
  //  std::endl; std::ofstream o("benchmark.json"); o << std::setw(2) <<
  //  nlohmann::json(global::settings); o.close();

  if (argc <= 1) {
    std::cout << "Usage: " << argv[0] << " configuration.json" << std::endl;
    return EXIT_FAILURE;
  }

  std::ifstream stream(argv[1]);
  const nlohmann::json settings = nlohmann::json::parse(stream);
  global::settings.load(settings);
  std::cout << "Loaded the following settings:" << std::endl
            << global::settings << std::endl;

  Log::instantiateRun();
  if (global::settings.benchmark.moving_ai.active) {
    ScenarioLoader scenarioLoader;
    scenarioLoader.load(global::settings.benchmark.moving_ai.scenario);
    const auto n = scenarioLoader.scenarios().size();
    const int start_id = (global::settings.benchmark.moving_ai.start + n) % n;
    const int end_id = (global::settings.benchmark.moving_ai.end + n) % n;
    for (int i = start_id; i <= end_id; ++i) {
      std::cout << "##############################################"
                << std::endl;
      std::cout << "# Moving AI Scenario " << i << std::endl;
      std::cout << "##############################################"
                << std::endl;

      auto &scenario = scenarioLoader.scenarios()[i];
      delete global::settings.environment;
      global::settings.environment =
          GridMaze::createFromMovingAiScenario(scenario);
      global::settings.steer.initializeSteering();

      if (i == start_id) {
        if (global::settings.benchmark.log_file.value().empty())
          global::settings.benchmark.log_file = Log::filename() + ".json";
      }

      auto info = nlohmann::json(
          {{"plans", {}}, {"optimalDistance", scenario.optimal_length}});
      global::settings.environment->to_json(info["environment"]);

      evaluatePlanners(info);
      info["settings"] = nlohmann::json(global::settings)["settings"];
      Log::log(info);
    }
  } else {
    for (unsigned int i = 0; i < global::settings.benchmark.runs; ++i) {
      std::cout << "##############################################"
                << std::endl;
      std::cout << "# Benchmark Run " << i << " / "
                << global::settings.benchmark.runs.value() << std::endl;
      std::cout << "##############################################"
                << std::endl;
      global::settings.env.grid.seed = i + 1;
      global::settings.env.createEnvironment();

      global::settings.steer.initializeSteering();

      if (i == 0) {
        if (global::settings.benchmark.log_file.value().empty())
          global::settings.benchmark.log_file = Log::filename() + ".json";
      }

      auto info = nlohmann::json({{"plans", {}}});
      global::settings.environment->to_json(info["environment"]);

      evaluatePlanners(info);
      info["settings"] = nlohmann::json(global::settings)["settings"];
      Log::log(info);
    }
  }

  Log::save(global::settings.benchmark.log_file);

  return EXIT_SUCCESS;
}

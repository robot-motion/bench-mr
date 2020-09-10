#include <chrono>
#include <ctime>

#include "base/GridMaze.h"
#include "base/PlannerSettings.h"

#include "planners/OMPLControlPlanner.hpp"
#include "planners/OMPLPlanner.hpp"

#include "planners/sbpl/SbplPlanner.h"
#include "planners/thetastar/ThetaStar.h"

#include "utils/PathEvaluation.hpp"
#include "utils/ScenarioLoader.h"

namespace og = ompl::geometric;

void evaluatePlanners(nlohmann::json &info) {
  info["plans"] = {};
  if (global::settings.benchmark.planning.bfmt)
    PathEvaluation::evaluateSmoothers<BFMTPlanner>(info);
  if (global::settings.benchmark.planning.bit_star)
    PathEvaluation::evaluateSmoothers<BITstarPlanner>(info);
  if (global::settings.benchmark.planning.cforest)
    PathEvaluation::evaluateSmoothers<CForestPlanner>(info);
  if (global::settings.benchmark.planning.est)
    PathEvaluation::evaluateSmoothers<ESTPlanner>(info);
  if (global::settings.benchmark.planning.fmt)
    PathEvaluation::evaluateSmoothers<FMTPlanner>(info);
  if (global::settings.benchmark.planning.informed_rrt_star)
    PathEvaluation::evaluateSmoothers<InformedRRTstarPlanner>(info);
  if (global::settings.benchmark.planning.kpiece)
    PathEvaluation::evaluateSmoothers<KPIECEPlanner>(info);
  if (global::settings.benchmark.planning.prm)
    PathEvaluation::evaluateSmoothers<PRMPlanner>(info);
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

  if (global::settings.benchmark.control_planners_on) {
    if (global::settings.benchmark.planning.fprrt)
      PathEvaluation::evaluate<FPRRTPlanner>(info);
    if (global::settings.benchmark.planning.fpest)
      PathEvaluation::evaluate<FPESTPlanner>(info);
    if (global::settings.benchmark.planning.fpsst)
      PathEvaluation::evaluate<FPSSTPlanner>(info);
    if (global::settings.benchmark.planning.fppdst)
      PathEvaluation::evaluate<FPPDSTPlanner>(info);
    if (global::settings.benchmark.planning.fpkpiece)
      PathEvaluation::evaluate<FPKPIECEPlanner>(info);
  }

  if (global::settings.env.type.value() == "grid") {
    if (global::settings.benchmark.planning.sbpl_arastar)
      PathEvaluation::evaluateSmoothers<SbplPlanner<sbpl::SBPL_ARASTAR>>(info);
    if (global::settings.benchmark.planning.sbpl_anastar)
      PathEvaluation::evaluateSmoothers<SbplPlanner<sbpl::SBPL_ANASTAR>>(info);
    if (global::settings.benchmark.planning.sbpl_adstar)
      PathEvaluation::evaluateSmoothers<SbplPlanner<sbpl::SBPL_ADSTAR>>(info);
    if (global::settings.benchmark.planning.sbpl_lazy_ara)
      PathEvaluation::evaluateSmoothers<SbplPlanner<sbpl::SBPL_LAZY_ARA>>(info);
    if (global::settings.benchmark.planning.sbpl_mha)
      PathEvaluation::evaluateSmoothers<SbplPlanner<sbpl::SBPL_MHA>>(info);
  } else {
    std::cerr << "SBPL planners are only supported for grid environments!\n";
  }

  if (global::settings.benchmark.planning.sorrt_star)
    PathEvaluation::evaluateSmoothers<SORRTstarPlanner>(info);
  if (global::settings.benchmark.planning.sst)
    PathEvaluation::evaluateSmoothers<SSTPlanner>(info);
  if (global::settings.benchmark.planning.stride)
    PathEvaluation::evaluateSmoothers<STRIDEPlanner>(info);
  if (global::settings.benchmark.planning.spars)
    PathEvaluation::evaluateSmoothers<SPARSPlanner>(info);
  if (global::settings.benchmark.planning.spars2)
    PathEvaluation::evaluateSmoothers<SPARS2Planner>(info);
  if (global::settings.benchmark.planning.pdst)
    PathEvaluation::evaluateSmoothers<PDSTPlanner>(info);
  if (global::settings.benchmark.planning.theta_star)
    PathEvaluation::evaluateSmoothers<ThetaStar>(info);
}

void run(nlohmann::json &info) {
  global::settings.environment->to_json(info["environment"]);
  evaluatePlanners(info);
  info["settings"] = nlohmann::json(global::settings)["settings"];
  Log::log(info);
}

/**
 * Run benchmark for every selected steer function.
 */
void config_steering_and_run(std::size_t run_id, std::size_t start_id,
                             const nlohmann::json &base) {
  nlohmann::json info(base);
  if (run_id == start_id) {
    if (global::settings.benchmark.log_file.value().empty())
      global::settings.benchmark.log_file = Log::filename() + ".json";
  }
  if (global::settings.benchmark.control_planners_on) {
    for (const auto forward_propagation_type :
         global::settings.benchmark.forward_propagations.value()) {
      global::settings.forwardpropagation.forward_propagation_type =
          forward_propagation_type;
      global::settings.forwardpropagation.initializeForwardPropagation();
      run(info);
    }
  } else if (global::settings.benchmark.steer_functions.value().empty()) {
    global::settings.steer.initializeSteering();
    run(info);
  } else {
    for (const auto steer_type :
         global::settings.benchmark.steer_functions.value()) {
      global::settings.steer.steering_type = steer_type;

      global::settings.steer.initializeSteering();
      run(info);
    }
  }
  if (run_id == start_id) {
    if (global::settings.benchmark.log_file.value().empty())
      global::settings.benchmark.log_file = Log::filename() + ".json";
  }
}

int main(int argc, char **argv) {
  std::string template_filename = "benchmark_template.json";
  std::ofstream o(template_filename);
  o << std::setw(2) << nlohmann::json(global::settings);
  o.close();
  std::cout << "Saved " << template_filename << "." << std::endl;

  if (argc <= 1) {
    std::cout << "Usage: " << argv[0] << " configuration.json" << std::endl;
    return EXIT_FAILURE;
  }

  auto now = chrono::system_clock::to_time_t(chrono::system_clock::now());
  std::cout << "Time stamp: " << ctime(&now) << std::endl;

  std::ifstream stream(argv[1]);
  const nlohmann::json settings = nlohmann::json::parse(stream);
  global::settings.load(settings);
  std::cout << "Loaded the following settings from " << argv[1] << ":"
            << std::endl
            << global::settings << std::endl;

  Log::instantiateRun();
  if (global::settings.benchmark.moving_ai.active) {
    ScenarioLoader scenarioLoader;
    scenarioLoader.load(global::settings.benchmark.moving_ai.scenario);
    const auto n = scenarioLoader.scenarios().size();
    std::size_t start_id = (global::settings.benchmark.moving_ai.start + n) % n;
    std::size_t end_id = (global::settings.benchmark.moving_ai.end + n) % n;
    for (int i = global::settings.benchmark.moving_ai.start;
         i < global::settings.benchmark.moving_ai.end; ++i) {
      std::size_t id = (i + n) % n;
      std::cout << "##############################################"
                << std::endl;
      std::cout << "# Moving AI Scenario " << id << "  (" << (i - start_id + 1)
                << "/" << (end_id - start_id) << ")" << std::endl;
      std::cout << "##############################################"
                << std::endl;

      auto &scenario = scenarioLoader.scenarios()[id];
      delete global::settings.environment;
      global::settings.environment =
          GridMaze::createFromMovingAiScenario(scenario);
      global::settings.env.collision.initializeCollisionModel();

      auto info =
          nlohmann::json({{"optimalDistance", scenario.optimal_length}});
      config_steering_and_run(id, start_id, info);
      Log::save(global::settings.benchmark.log_file);
    }
  } else {
    for (std::size_t i = 0; i < global::settings.benchmark.runs; ++i) {
      std::cout << "##############################################"
                << std::endl;
      std::cout << "# Benchmark Run " << i + 1 << " / "
                << global::settings.benchmark.runs.value() << std::endl;
      std::cout << "##############################################"
                << std::endl;
      global::settings.env.createEnvironment();
      global::settings.env.grid.seed += 1;

      // global::settings.steer.initializeSteering();

      nlohmann::json info;
      config_steering_and_run(i, 0u, info);
      Log::save(global::settings.benchmark.log_file);
    }
  }

  return EXIT_SUCCESS;
}

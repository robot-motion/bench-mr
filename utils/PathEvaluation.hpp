#pragma once

#include <metrics/ClearingMetric.h>
#include <metrics/CurvatureMetric.h>
#include <metrics/PathLengthMetric.h>

#include "base/PathStatistics.hpp"
#include "planners/AbstractPlanner.hpp"
#include "utils/Log.h"

struct PathEvaluation {
  static void evaluate(PathStatistics &stats,
                       const ompl::geometric::PathGeometric &path,
                       const AbstractPlanner *planner) {
    stats.planning_time = planner->planningTime();
    if (path.getStateCount() < 2) {
      stats.path_found = false;
      stats.exact_goal_path = false;
    } else {
      stats.path_found = true;
      auto solution = PlannerUtils::interpolated(path);
      stats.exact_goal_path =
          Point(solution.getStates().back())
              .distance(global::settings.environment->goal()) <=
          global::settings.exact_goal_radius;
      stats.path_collides = planner->isValid(solution);
      stats.path_length = PathLengthMetric::evaluate(solution);
      stats.curvature = CurvatureMetric::evaluate(solution);
      stats.smoothness = solution.smoothness();

      if (global::settings.evaluate_clearing) {
        OMPL_INFORM("Evaluating clearing distance metric.");
        auto clearings = ClearingMetric::clearingDistances(solution);
        stats.mean_clearing_distance = stat::mean(clearings);
        stats.median_clearing_distance = stat::median(clearings);
        stats.min_clearing_distance = stat::min(clearings);
        stats.max_clearing_distance = stat::max(clearings);
      }
    }
  }

  template <class PLANNER>
  static void evaluate(nlohmann::json &info) {
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

    // add intermediary solutions
    std::vector<nlohmann::json> intermediaries;
    for (const auto &is : planner.intermediarySolutions) {
      PathStatistics is_stats;
      evaluate(is_stats, is.solution, &planner);
      nlohmann::json s{{"time", is.time},
                       {"cost", is.cost},
                       {"trajectory", Log::serializeTrajectory(is.solution)},
                       {"stats", nlohmann::json(is_stats)["stats"]}};
      intermediaries.emplace_back(s);
    }
    j["intermediary_solutions"] = intermediaries;
  }

  PathEvaluation() = delete;
};

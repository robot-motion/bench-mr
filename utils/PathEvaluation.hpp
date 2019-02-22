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
    stats.path_length = PathLengthMetric::evaluate(path);
    stats.curvature = CurvatureMetric::evaluate(path);
    stats.smoothness = path.smoothness();

    if (global::settings.evaluate_clearing) {
      auto clearings = ClearingMetric::clearingDistances(path);
      stats.mean_clearing_distance = stat::mean(clearings);
      stats.median_clearing_distance = stat::median(clearings);
      stats.min_clearing_distance = stat::min(clearings);
      stats.max_clearing_distance = stat::max(clearings);
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
  }

  PathEvaluation() = delete;
};

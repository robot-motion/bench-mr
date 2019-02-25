#pragma once

#include <metrics/ClearingMetric.h>
#include <metrics/CurvatureMetric.h>
#include <metrics/PathLengthMetric.h>
#include <smoothers/chomp/CHOMP.h>
#include <smoothers/ompl/OmplSmoother.hpp>

#include "base/PathStatistics.hpp"
#include "planners/AbstractPlanner.hpp"
#include "utils/Log.h"

#include "smoothers/grips/GRIPS.h"

struct PathEvaluation {
  static bool evaluate(PathStatistics &stats,
                       const ompl::geometric::PathGeometric &path,
                       const AbstractPlanner *planner) {
    stats.planning_time = planner->planningTime();
    stats.planner = planner->name();
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
    return stats.path_found && stats.exact_goal_path;
  }

  template <class PLANNER>
  static bool evaluate(PLANNER &planner, nlohmann::json &info) {
    PathStatistics stats(planner.name());
    auto &j = info["plans"][planner.name()];
    OMPL_INFORM(("Running " + planner.name() + "...").c_str());
    bool success = false;
    if (planner.run()) {
      success = PathEvaluation::evaluate(stats, planner.solution(), &planner);
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
    return success;
  }

  template <class PLANNER>
  static bool evaluate(nlohmann::json &info) {
    PLANNER planner;
    evaluate(planner, info);
  }

  template <class PLANNER>
  static bool evaluateSmoothers(nlohmann::json &info) {
    PLANNER planner;
    if (!evaluate<PLANNER>(planner, info)) {
      OMPL_WARN("Cannot evaluate smoothers since no solution could be found.");
      return false;
    }
    auto &j = info["plans"][planner.name()]["smoothing"];

    {
      // GRIPS
      og::PathGeometric grips(planner.solution());
      GRIPS::smooth(grips);
      PathStatistics grips_stats;
      evaluate(grips_stats, grips, &planner);
      j["grips"] = {{"time", GRIPS::smoothingTime},
                    {"inserted_nodes", GRIPS::insertedNodes},
                    {"pruning_rounds", GRIPS::pruningRounds},
                    {"cost", grips.length()},
                    {"trajectory", Log::serializeTrajectory(grips)},
                    {"stats", nlohmann::json(grips_stats)["stats"]},
                    {"round_stats", GRIPS::statsPerRound}};
    }
    {
      // CHOMP
      CHOMP chomp;
      chomp.run(planner.solution());
      PathStatistics chomp_stats;
      evaluate(chomp_stats, chomp.solution(), &planner);
      j["chomp"] = {{"time", chomp.planningTime()},
                    {"cost", chomp.solution().length()},
                    {"trajectory", chomp.solutionPath()},
                    {"stats", nlohmann::json(chomp_stats)["stats"]}};
    }
    {
      // OMPL Smoothers
      OmplSmoother smoother(planner.simpleSetup(), planner.solution());
      {
        // Shortcut
        PathStatistics stats;
        TimedResult tr = smoother.shortcutPath();
        evaluate(stats, tr.trajectory, &planner);
        j["ompl_shortcut"] = {
            {"time", tr.elapsed()},
            {"cost", tr.trajectory.length()},
            {"trajectory", Log::serializeTrajectory(tr.trajectory)},
            {"stats", nlohmann::json(stats)["stats"]}};
      }
      {
        // B-Spline
        PathStatistics stats;
        TimedResult tr = smoother.smoothBSpline();
        evaluate(stats, tr.trajectory, &planner);
        j["ompl_bspline"] = {
            {"time", tr.elapsed()},
            {"cost", tr.trajectory.length()},
            {"trajectory", Log::serializeTrajectory(tr.trajectory)},
            {"stats", nlohmann::json(stats)["stats"]}};
      }
      {
        // Simplify Max
        PathStatistics stats;
        TimedResult tr = smoother.simplifyMax();
        evaluate(stats, tr.trajectory, &planner);
        j["ompl_simplify_max"] = {
            {"time", tr.elapsed()},
            {"cost", tr.trajectory.length()},
            {"trajectory", Log::serializeTrajectory(tr.trajectory)},
            {"stats", nlohmann::json(stats)["stats"]}};
      }

      // OMPL Anytime Path Shortening
      if (planner.omplPlanner() == nullptr) {
        OMPL_WARN(
            "OMPL smoothing method Anytime Path Shortening is only available "
            "for OMPL planners.");
      } else {
        // Anytime Path Shortening
        PathStatistics stats;
        TimedResult tr = smoother.anytimePathShortening(planner.omplPlanner());
        evaluate(stats, tr.trajectory, &planner);
        j["ompl_anytime_ps"] = {
            {"time", tr.elapsed()},
            {"cost", tr.trajectory.length()},
            {"trajectory", Log::serializeTrajectory(tr.trajectory)},
            {"stats", nlohmann::json(stats)["stats"]}};
      }
    }

    return true;
  }

  PathEvaluation() = delete;
};

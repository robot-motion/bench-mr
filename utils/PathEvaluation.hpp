#pragma once

#include <metrics/ClearingMetric.h>
#include <metrics/CurvatureMetric.h>
#include <metrics/PathLengthMetric.h>
#include <smoothers/chomp/CHOMP.h>
#include <smoothers/ompl/OmplSmoother.hpp>

#include "base/PathStatistics.hpp"
#include "base/PlannerConfigurator.hpp"
#include "planners/AbstractPlanner.hpp"
#include "utils/Log.h"

#include "smoothers/grips/GRIPS.h"

struct PathEvaluation {
  /**
   * Identifies cusps in a solution path by comparing the yaw angles between
   * every second state.
   */
  static void computeCusps(PathStatistics &stats,
                           const ompl::geometric::PathGeometric &p) {
    std::vector<Point> &cusps = stats.cusps.value();
    const auto path = Point::fromPath(p);
    for (unsigned int i = 1; i < path.size() - 1; ++i) {
      const double yaw_prev =
          std::fmod(PlannerUtils::slope(path[i - 1], path[i]), 2. * M_PI);
      const double yaw_next =
          std::fmod(PlannerUtils::slope(path[i], path[i + 1]), 2. * M_PI);

      if (std::fmod(std::abs(yaw_next - yaw_prev), 2. * M_PI) >
          global::settings.cusp_angle_threshold)
        cusps.emplace_back(path[i]);
    }
  }

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
      stats.path_collides = !planner->isValid(solution);
      stats.path_length = solution.length();
      stats.curvature = CurvatureMetric::evaluate(solution);
      stats.smoothness = solution.smoothness();

      if (global::settings.evaluate_clearing &&
          global::settings.environment->distance(0., 0.) >= 0.) {
        const auto clearings = ClearingMetric::clearingDistances(solution);
        stats.mean_clearing_distance = stat::mean(clearings);
        stats.median_clearing_distance = stat::median(clearings);
        stats.min_clearing_distance = stat::min(clearings);
        stats.max_clearing_distance = stat::max(clearings);
      }

      computeCusps(stats, path);
    }
    return stats.path_found;
  }

  template <class PLANNER>
  static bool evaluate(PLANNER &planner, nlohmann::json &info) {
    PathStatistics stats(planner.name());
    auto &j = info["plans"][planner.name()];
    OMPL_INFORM(("Running " + planner.name() + "...").c_str());
    bool success = false;
    if (planner.run()) {
      success = PathEvaluation::evaluate(stats, planner.solution(), &planner);
      j["path"] = Log::serializeTrajectory(planner.solution(), false);
    } else {
      j["path"] = {};
    }
    std::cout << stats << std::endl;
    std::cout << "Steer function: "
              << Steering::to_string(global::settings.steer.steering_type)
              << std::endl;
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
                       {"path", Log::serializeTrajectory(is.solution, false)},
                       {"stats", nlohmann::json(is_stats)["stats"]}};
      intermediaries.emplace_back(s);
    }
    j["intermediary_solutions"] = intermediaries;
    return success;
  }

  template <class PLANNER>
  static bool evaluate(nlohmann::json &info) {
    PLANNER planner;
    return evaluate(planner, info);
  }

  template <class PLANNER>
  static bool evaluateSmoothers(nlohmann::json &info) {
    PLANNER planner;
    PlannerConfigurator::configure(planner);
    if (!evaluate<PLANNER>(planner, info)) {
      OMPL_WARN("Cannot evaluate smoothers since no solution could be found.");
      return false;
    }
    auto &j = info["plans"][planner.name()]["smoothing"];

    if (global::settings.benchmark.smoothing.grips) {
      // GRIPS
      og::PathGeometric grips(planner.solution());
      GRIPS::smooth(grips);
      PathStatistics grips_stats;
      evaluate(grips_stats, grips, &planner);
      j["grips"] = {{"time", GRIPS::smoothingTime},
                    {"name", "GRIPS"},
                    {"inserted_nodes", GRIPS::insertedNodes},
                    {"pruning_rounds", GRIPS::pruningRounds},
                    {"cost", grips.length()},
                    {"trajectory", Log::serializeTrajectory(grips)},
                    {"path", Log::serializeTrajectory(grips, false)},
                    {"stats", nlohmann::json(grips_stats)["stats"]},
                    {"round_stats", GRIPS::statsPerRound}};
    }
    if (global::settings.benchmark.smoothing.chomp) {
      // CHOMP
      CHOMP chomp;
      chomp.run(planner.solution());
      PathStatistics chomp_stats;
      evaluate(chomp_stats, chomp.solution(), &planner);
      j["chomp"] = {{"time", chomp.planningTime()},
                    {"name", "CHOMP"},
                    {"cost", chomp.solution().length()},
                    {"path", Log::serializeTrajectory(chomp.solution(), false)},
                    {"trajectory", chomp.solutionPath()},
                    {"stats", nlohmann::json(chomp_stats)["stats"]}};
    }

    // OMPL Smoothers
    OmplSmoother smoother(planner.simpleSetup(), planner.solution());
    if (global::settings.benchmark.smoothing.ompl_shortcut) {
      // Shortcut
      PathStatistics stats;
      TimedResult tr = smoother.shortcutPath();
      evaluate(stats, tr.trajectory, &planner);
      j["ompl_shortcut"] = {
          {"time", tr.elapsed()},
          {"name", "Shortcut"},
          {"cost", tr.trajectory.length()},
          {"path", Log::serializeTrajectory(tr.trajectory, false)},
          {"trajectory", Log::serializeTrajectory(tr.trajectory)},
          {"stats", nlohmann::json(stats)["stats"]}};
    }
    if (global::settings.benchmark.smoothing.ompl_bspline) {
      // B-Spline
      PathStatistics stats;
      TimedResult tr = smoother.smoothBSpline();
      evaluate(stats, tr.trajectory, &planner);
      j["ompl_bspline"] = {
          {"time", tr.elapsed()},
          {"name", "B-Spline"},
          {"cost", tr.trajectory.length()},
          {"path", Log::serializeTrajectory(tr.trajectory, false)},
          {"trajectory", Log::serializeTrajectory(tr.trajectory)},
          {"stats", nlohmann::json(stats)["stats"]}};
    }
    if (global::settings.benchmark.smoothing.ompl_simplify_max) {
      // Simplify Max
      PathStatistics stats;
      TimedResult tr = smoother.simplifyMax();
      evaluate(stats, tr.trajectory, &planner);
      j["ompl_simplify_max"] = {
          {"time", tr.elapsed()},
          {"name", "SimplifyMax"},
          {"cost", tr.trajectory.length()},
          {"path", Log::serializeTrajectory(tr.trajectory, false)},
          {"trajectory", Log::serializeTrajectory(tr.trajectory)},
          {"stats", nlohmann::json(stats)["stats"]}};
    }

    return true;
  }

  PathEvaluation() = delete;
};

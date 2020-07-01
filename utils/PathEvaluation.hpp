#pragma once

#include <metrics/ClearingMetric.h>
#include <metrics/CurvatureMetric.h>
#include <metrics/PathLengthMetric.h>
#include <smoothers/chomp/CHOMP.h>
#include <smoothers/ompl/OmplSmoother.hpp>

#include "base/PathStatistics.hpp"
#include "base/PlannerConfigurator.hpp"
#include "planners/AbstractPlanner.h"
#include "utils/Log.h"

#include "smoothers/grips/GRIPS.h"

struct PathEvaluation {
 private:
  /**
   * Creates an empty entry for the given planner in the JSON info object. Used
   * in cases where the planner failed to find a solution or an error was
   * encountered.
   */
  static void createEmptyEntry(const std::string &planner_name,
                               nlohmann::json &info) {
    static PathStatistics empty_stats;
    auto &j = info["plans"][planner_name];
    j["path"] = {};
    j["stats"] = nlohmann::json(empty_stats)["stats"];
    j["trajectory"] = {};
    j["intermediary_solutions"] = {};
  }

 public:
  /**
   * Identifies cusps in a solution path by comparing the yaw angles between
   * every second state.
   */
  static void computeCusps(PathStatistics &stats,
                           const ompl::geometric::PathGeometric &p) {
    std::vector<Point> &cusps = stats.cusps.value();
    const auto path = Point::fromPath(p);
    for (std::size_t i = 1; i < path.size() - 1; ++i) {
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
      auto solution = path;

      // assume if SBPL has found a solution, it does not collide and is exact
      if (planner->name().rfind("SBPL", 0) == 0) {
        // do not interpolate the path returned by SBPL (it uses its own steer
        // function)
        stats.path_collides = false;
        stats.exact_goal_path = true;
      } else {
        solution = PlannerUtils::interpolated(path);
        stats.path_collides = !planner->isValid(solution, stats.collisions);
        stats.exact_goal_path =
            Point(solution.getStates().back())
                .distance(global::settings.environment->goal()) <=
            global::settings.exact_goal_radius;
      }
      stats.path_length = PathLengthMetric::evaluate(solution);
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
    bool success;
    try {
      if (planner.run()) {
        success = PathEvaluation::evaluate(stats, planner.solution(), &planner);
        j["path"] = Log::serializeTrajectory(planner.solution(), false);
      } else {
        createEmptyEntry(planner.name(), info);
        std::cout << "<stats> No solution was found. </stats>\n";
        return false;
      }
    } catch (std::bad_alloc &ba) {
      // we ran out of memory
      OMPL_ERROR("<stats> Error </stats>\nPlanner %s ran out of memory: %s.",
                 planner.name().c_str(), ba.what());
      createEmptyEntry(planner.name(), info);
      return false;
    } catch (ompl::Exception &ex) {
      OMPL_ERROR("Unable to evaluate new planner %s.\n%s",
                 planner.name().c_str(), ex.what());
      createEmptyEntry(planner.name(), info);
      return false;
    } catch (...) {
      OMPL_ERROR(
          "<stats> Error </stats>\nAn unknown exception occurred while running "
          "planner %s.",
          planner.name().c_str());
      createEmptyEntry(planner.name(), info);
      return false;
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
    PLANNER *planner = nullptr;
    try {
      planner = new PLANNER;
      PlannerConfigurator::configure(*planner);
    } catch (std::bad_alloc &ba) {
      // we ran out of memory
      OMPL_ERROR(
          "<stats> Error </stats>\nRan out of memory while creating planner "
          "%s: %s.",
          AbstractPlanner::LastCreatedPlannerName.c_str(), ba.what());
      createEmptyEntry(AbstractPlanner::LastCreatedPlannerName, info);
      delete planner;
      return false;
    } catch (ompl::Exception &ex) {
      OMPL_ERROR("Unable to create new planner %s.\n%s",
                 AbstractPlanner::LastCreatedPlannerName.c_str(), ex.what());
      createEmptyEntry(AbstractPlanner::LastCreatedPlannerName, info);
      delete planner;
      return false;
    } catch (...) {
      OMPL_ERROR(
          "<stats> Error </stats>\nAn unknown exception occurred while "
          "creating planner %s.",
          AbstractPlanner::LastCreatedPlannerName.c_str());
      createEmptyEntry(AbstractPlanner::LastCreatedPlannerName, info);
      delete planner;
      return false;
    }
    auto result = evaluate(*planner, info);
    delete planner;
    return result;
  }

  template <class PLANNER>
  static bool evaluateSmoothers(nlohmann::json &info) {
    PLANNER *planner = nullptr;
    try {
      planner = new PLANNER;
      PlannerConfigurator::configure(*planner);
    } catch (std::bad_alloc &ba) {
      // we ran out of memory
      OMPL_ERROR(
          "<stats> Error </stats>\nRan out of memory while creating planner "
          "%s: %s.",
          AbstractPlanner::LastCreatedPlannerName.c_str(), ba.what());
      createEmptyEntry(AbstractPlanner::LastCreatedPlannerName, info);
      delete planner;
      return false;
    } catch (...) {
      OMPL_ERROR(
          "<stats> Error </stats>\nAn unknown exception occurred while "
          "creating planner %s.",
          AbstractPlanner::LastCreatedPlannerName.c_str());
      createEmptyEntry(AbstractPlanner::LastCreatedPlannerName, info);
      delete planner;
      return false;
    }
    if (!evaluate<PLANNER>(*planner, info)) {
      OMPL_WARN("Cannot evaluate smoothers since no solution could be found.");
      delete planner;
      return false;
    }
    auto &j = info["plans"][planner->name()]["smoothing"];

    if (global::settings.benchmark.smoothing.grips) {
      const double cached_min_node_dist =
          global::settings.smoothing.grips.min_node_distance;
      if (global::settings.steer.steering_type ==
          Steering::STEER_TYPE_CC_DUBINS) {
        // XXX increase min distance between vertices to ensure GRIPS can steer
        // using CC Dubins
        global::settings.smoothing.grips.min_node_distance = 40.;
      }
      // GRIPS
      og::PathGeometric grips(planner->solution());
      GRIPS::smooth(grips);
      PathStatistics grips_stats;
      evaluate(grips_stats, grips, planner);
      j["grips"] = {{"time", GRIPS::smoothingTime},
                    {"name", "GRIPS"},
                    {"inserted_nodes", GRIPS::insertedNodes},
                    {"pruning_rounds", GRIPS::pruningRounds},
                    {"cost", grips.length()},
                    {"trajectory", Log::serializeTrajectory(grips)},
                    {"path", Log::serializeTrajectory(grips, false)},
                    {"stats", nlohmann::json(grips_stats)["stats"]},
                    {"round_stats", GRIPS::statsPerRound}};
      global::settings.smoothing.grips.min_node_distance = cached_min_node_dist;
    }
    if (global::settings.benchmark.smoothing.chomp) {
      // CHOMP
      CHOMP chomp;
      chomp.run(planner->solution());
      PathStatistics chomp_stats;
      evaluate(chomp_stats, chomp.solution(), planner);
      j["chomp"] = {{"time", chomp.planningTime()},
                    {"name", "CHOMP"},
                    {"cost", chomp.solution().length()},
                    {"path", Log::serializeTrajectory(chomp.solution(), false)},
                    {"trajectory", chomp.solutionPath()},
                    {"stats", nlohmann::json(chomp_stats)["stats"]}};
    }

    // OMPL Smoothers
    OmplSmoother smoother(planner->simpleSetup(), planner->solution());
    if (global::settings.benchmark.smoothing.ompl_shortcut) {
      // Shortcut
      PathStatistics stats;
      TimedResult tr = smoother.shortcutPath();
      evaluate(stats, tr.trajectory, planner);
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
      evaluate(stats, tr.trajectory, planner);
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
      evaluate(stats, tr.trajectory, planner);
      j["ompl_simplify_max"] = {
          {"time", tr.elapsed()},
          {"name", "SimplifyMax"},
          {"cost", tr.trajectory.length()},
          {"path", Log::serializeTrajectory(tr.trajectory, false)},
          {"trajectory", Log::serializeTrajectory(tr.trajectory)},
          {"stats", nlohmann::json(stats)["stats"]}};
    }

    delete planner;
    return true;
  }

  /**
   * Evaluates an anytime path planner by running the planner for each of the
   * provided time intervals (in seconds). This method populates the
   * "intermediary_solutions" field of the JSON object for the given planner.
   */
  // TODO remove? Python front-end adds this functionality
  template <class PLANNER>
  static bool evaluateAnytime(nlohmann::json &info) {
    PLANNER planner;
    const std::vector<double> times =
        global::settings.benchmark.anytime_intervals;
    PathStatistics stats(planner.name());
    auto &j = info["plans"][planner.name()];
    std::vector<nlohmann::json> intermediaries;
    bool success = false;
    const double cached_time_limit = global::settings.max_planning_time;
    for (double time : times) {
      OMPL_INFORM(("Running " + planner.name() + " for " +
                   std::to_string(time) + "s...")
                      .c_str());
      global::settings.max_planning_time = time;
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

      // add intermediary solutions
      nlohmann::json s{
          {"time", planner.planningTime()},
          {"max_time", time},
          {"cost", stats.path_length},
          {"trajectory", Log::serializeTrajectory(planner.solution())},
          {"path", Log::serializeTrajectory(planner.solution(), false)},
          {"stats", nlohmann::json(stats)["stats"]}};
      intermediaries.emplace_back(s);
    }

    j["intermediary_solutions"] = intermediaries;
    j["trajectory"] = Log::serializeTrajectory(planner.solution());
    j["stats"] = nlohmann::json(stats)["stats"];
    // restore global time limit
    global::settings.max_planning_time = cached_time_limit;
    return success;
  }

  PathEvaluation() = delete;
};

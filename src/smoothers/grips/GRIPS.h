#pragma once
#include <chrono>
#include <fstream>
#include <params.hpp>

#include "base/PlannerSettings.h"
#include "base/TimedResult.hpp"
#include "metrics/MaxCurvatureMetric.h"
#include "metrics/PathLengthMetric.h"
#include "utils/PlannerUtils.hpp"

using namespace params;
class GRIPS {
 public:
  enum RoundType { ROUND_GD, ROUND_PRUNING, ROUND_ORIGINAL, ROUND_UNKOWN };

  struct RoundStats : public Group {
    Property<double> path_length{std::numeric_limits<double>::quiet_NaN(),
                                 "path_length", this};
    Property<double> max_curvature{std::numeric_limits<double>::quiet_NaN(),
                                   "max_curvature", this};
    Property<double> time{std::numeric_limits<double>::quiet_NaN(), "time",
                          this};
    Property<int> nodes{-1, "nodes", this};
    Property<double> median_node_obstacle_distance{
        std::numeric_limits<double>::quiet_NaN(),
        "median_node_obstacle_distance", this};
    Property<double> mean_node_obstacle_distance{
        std::numeric_limits<double>::quiet_NaN(), "mean_node_obstacle_distance",
        this};
    Property<double> min_node_obstacle_distance{
        std::numeric_limits<double>::quiet_NaN(), "min_node_obstacle_distance",
        this};
    Property<double> max_node_obstacle_distance{
        std::numeric_limits<double>::quiet_NaN(), "max_node_obstacle_distance",
        this};
    Property<double> std_node_obstacle_distance{
        std::numeric_limits<double>::quiet_NaN(), "std_node_obstacle_distance",
        this};
    Property<double> median_traj_obstacle_distance{
        std::numeric_limits<double>::quiet_NaN(),
        "median_traj_obstacle_distance", this};
    Property<double> mean_traj_obstacle_distance{
        std::numeric_limits<double>::quiet_NaN(), "mean_traj_obstacle_distance",
        this};
    Property<double> min_traj_obstacle_distance{
        std::numeric_limits<double>::quiet_NaN(), "min_traj_obstacle_distance",
        this};
    Property<double> max_traj_obstacle_distance{
        std::numeric_limits<double>::quiet_NaN(), "max_traj_obstacle_distance",
        this};
    Property<double> std_traj_obstacle_distance{
        std::numeric_limits<double>::quiet_NaN(), "std_traj_obstacle_distance",
        this};
    Property<RoundType> type{ROUND_UNKOWN, "type", this};
    TimedResult stopWatch;

    std::string typeName() const {
      switch (type.value()) {
        case ROUND_GD:
          return "gd";
        case ROUND_PRUNING:
          return "pruning";
        case ROUND_ORIGINAL:
          return "original";
        default:
          return "unknown";
      }
    }

    RoundStats() : Group("round") {}
  };

  static int insertedNodes;
  static int pruningRounds;
  static std::vector<int> nodesPerRound;
  static std::vector<RoundStats> statsPerRound;
  static double smoothingTime;

  static bool smooth(ompl::geometric::PathGeometric &path,
                     const std::vector<Point> &originalPathIntermediaries);

  static bool smooth(ompl::geometric::PathGeometric &path) {
    auto intermediary = ompl::geometric::PathGeometric(path);
    OMPL_DEBUG("GRIPS path has %d node(s).", path.getStateCount());
    return smooth(path, Point::fromPath(intermediary));
  }

 private:
  static RoundStats roundStats;

  static void beginRound(RoundType type = ROUND_UNKOWN);
  static void endRound(const ompl::geometric::PathGeometric &path);

  static Stopwatch stopWatch;
};

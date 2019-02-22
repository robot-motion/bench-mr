#pragma once
#include <chrono>
#include <fstream>

#include <params.hpp>

#include "base/PlannerSettings.h"
#include "base/PlannerUtils.hpp"
#include "base/TimedResult.hpp"

#include "metrics/CurvatureMetric.h"
#include "metrics/PathLengthMetric.h"

using namespace params;
class PostSmoothing {
 public:
  static const bool MINIMIZE_PATHLENGTH = true;  // otherwise minimize curvature
  static const bool FIX_COLLISIONS = false;

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
  static int collisionFixAttempts;
  static int roundsWithCollisionFixAttempts;
  static std::vector<int> nodesPerRound;
  static std::vector<RoundStats> statsPerRound;
  static double smoothingTime;

  static bool smooth(ompl::geometric::PathGeometric &path,
                     const std::vector<Point> &originalPathIntermediaries);

  static bool smooth(ompl::geometric::PathGeometric &path) {
    auto intermediary = ompl::geometric::PathGeometric(path);
    intermediary.interpolate();
    smooth(path, Point::fromPath(intermediary));
  }

 private:
  //    static void fixCollision(std::vector<GNode> &path,
  //                             const std::vector<Tpoint>
  //                             &originalPathIntermediaries, const Tpoint
  //                             &node, unsigned int i)
  //    {
  //        auto closest = PlannerUtils::closestPoint(node,
  //        originalPathIntermediaries); GNode repair; if
  //        (closest.euclidianDistance(path[i - 1]) < MIN_NODE_DISTANCE)
  //        {
  //            path[i - 1].x_r = closest.x_r;
  //            path[i - 1].y_r = closest.y_r;
  ////                    path[i-1].theta = closest.theta;
  //            repair = path[i - 1];
  //        } else if (closest.euclidianDistance(path[i]) < MIN_NODE_DISTANCE)
  //        {
  //            path[i].x_r = closest.x_r;
  //            path[i].y_r = closest.y_r;
  ////                    path[i].theta = closest.theta;
  //            repair = path[i];
  //        } else {
  //            repair = GNode(closest.x, closest.y, PlannerUtils::slope(path[i
  //            - 1], path[i])); path.insert(path.begin() + i, repair);
  //        }
  //#ifdef DEBUG
  //        QtVisualizer::drawNode(repair, Qt::cyan, 0.4);
  //#endif
  //    }

  static RoundStats roundStats;

  static void beginRound(RoundType type = ROUND_UNKOWN);

  static void endRound(const ompl::geometric::PathGeometric &path);

  static Stopwatch stopWatch;
};

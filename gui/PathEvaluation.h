#pragma once

#include <unordered_map>

#include <metrics/CurvatureMetric.h>

#include "PostSmoothing.h"
#include "base/PathStatistics.hpp"
#include "planners/OMPLAnytimePathShortening.hpp"
#include "planners/OMPLPlanner.hpp"
#include "utils/Log.h"

class PathStatisticsAggregator {
 public:
  inline void add(const PathStatistics &stats) {
    _map[stats.planner].emplace_back(stats);
    _planners.emplace(stats.planner);
    Log::log(stats);
  }

 private:
  std::unordered_map<std::string, std::vector<PathStatistics> > _map;
  std::unordered_set<std::string> _planners;
};

class PathEvaluation {
 public:
  static void initialize();

  //#if QT_SUPPORT
  //  static PathStatistics add(AbstractPlanner *planner, const std::string
  //  &label,
  //                            const QColor &color = Qt::black);
  //
  //  static PathStatistics evaluate(const ompl::geometric::PathGeometric &path,
  //                                 const std::string &label,
  //                                 const QColor &color = Qt::black);
  //#else
  //  static PathStatistics add(AbstractPlanner *planner, std::string label);

  static PathStatistics evaluate(PathStatistics &stats,
                                 const ompl::geometric::PathGeometric &path,
                                 const AbstractPlanner *planner);
  //#endif

  static std::vector<double> computeObstacleDistances(
      const std::vector<Point> &path);

 private:
  PathEvaluation() = default;

  static double _maxPathLength;
  static double _maxCurvature;
  static double _maxTime;

  static double findMedian(const std::vector<double> &distances, size_t l,
                           size_t r);

  static bool _smoothCollides;
};

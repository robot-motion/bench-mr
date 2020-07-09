#pragma once

#include <cmath>
#include <vector>

#include "TrajectoryMetric.h"

class PathLengthMetric : public TMetric<PathLengthMetric> {
 public:
  static double evaluateMetric(const ompl::geometric::PathGeometric &trajectory,
                               double) {
    const auto path = Point::fromPath(trajectory);
    return PlannerUtils::totalLength(path);
  }

  static double evaluateMetric(const ompl::control::PathControl &trajectory,
                               double) {
    const auto path = Point::fromPath(trajectory);
    return PlannerUtils::totalLength(path);
  }
  static const bool MoreIsBetter = false;
};
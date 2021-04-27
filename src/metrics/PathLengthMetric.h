#pragma once

#include <cmath>
#include <vector>

#include "TrajectoryMetric.h"
#include "utils/PlannerUtils.hpp"

class PathLengthMetric : public TMetric<PathLengthMetric> {
 public:
  static double evaluateMetric(const ompl::geometric::PathGeometric &trajectory,
                               double) {
    std::cout << "Evaluate Metric in geometric space " << std::endl;
    const auto path = Point::fromPath(trajectory);
    return PlannerUtils::totalLength(path);
  }

  static double evaluateMetric(const ompl::control::PathControl &trajectory,
                               double) {
    std::cout << "Evaluate Metric in control space " << std::endl;
    const auto path = Point::fromPath(trajectory);
    return PlannerUtils::totalLength(path);
  }
  static const bool MoreIsBetter = false;
};
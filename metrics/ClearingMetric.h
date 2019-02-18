#pragma once

#include <base/PathStatistics.hpp>
#include "TrajectoryMetric.h"

class ClearingMetric : public TMetric<ClearingMetric> {
 public:
  static std::vector<double> clearingDistances(
      const ompl::geometric::PathGeometric &trajectory) {
    std::vector<double> clearings;
    const auto path = Point::fromPath(trajectory);

    for (auto &p : path)
      clearings.push_back(
          settings.environment->bilinearDistance(p.x, p.y));

    return clearings;
  }

  static double evaluateMetric(const ompl::geometric::PathGeometric &trajectory,
                               double, bool visualize = false) {
    return stat::mean(clearingDistances(trajectory));
  }
};

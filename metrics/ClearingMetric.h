#pragma once

#include <base/PathStatistics.hpp>
#include "TrajectoryMetric.h"

class ClearingMetric : public TMetric<ClearingMetric> {
 public:
  static std::vector<double> clearingDistances(const Trajectory *trajectory) {
    std::vector<double> clearings;
    std::vector<Tpoint> path = trajectory->getPath();

    for (auto &p : path)
      clearings.push_back(
          PlannerSettings::environment->bilinearDistance(p.x, p.y));

    return clearings;
  }

  static double evaluateMetric(const Trajectory *trajectory, double,
                               bool visualize = false) {
    return stat::mean(clearingDistances(trajectory));
  }
};

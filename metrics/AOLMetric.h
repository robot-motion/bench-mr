#pragma once

#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include "base/PlannerSettings.h"
#include "base/Primitives.h" 
#include "metrics/TrajectoryMetric.h"
#include "utils/PlannerUtils.hpp"
#include "metrics/PathLengthMetric.h"

#include <ompl/geometric/PathGeometric.h>
#include <ompl/control/PathControl.h>

#if QT_SUPPORT
#include "gui/QtVisualizer.h"
#endif

class AOLMetric : public TMetric<AOLMetric> {
 public:
  /**
   * Computes total angle / path length metric.
   *
   * Total angle is \int |\dot{yaw}(t)|dt, where yaw is the heading angle
   * pointing forwards, i.e., it changes by pi for a cusp.
   * Total path length is \int 1 dt.
   *
   * This metric can be interpreted as a combination of curvature and cusp.
   *
   * @param trajectory The trajectory to evaluate.
   * @param visualize
   * @return AOL.
   */
  static inline double max_curvature = std::numeric_limits<double>::max();

  static double evMetric(const std::vector<Point>& path,
                         bool visualize = false) {
    double path_length = PlannerUtils::totalLength(path);
    double total_yaw_change = 0;

    auto prev = path.begin();
    auto current = prev;
    auto next = prev;
    while (next != path.end()) {
      // advance until current point != prev point, i.e., skip duplicates
      if (prev->distance(*current) <= 0) {
        ++current;
        ++next;
      }
      else if (current->distance(*next) <= 0) {
        ++next;
      }
      else {
        const double yaw_prev = PlannerUtils::slope(*prev, *current);
        const double yaw_next = PlannerUtils::slope(*current, *next);

        // compute angle difference in [0, pi)
        // close to pi -> cusp; 0 -> straight line; inbetween -> curve
        const double yaw_change =
            std::abs(PlannerUtils::normalizeAngle(yaw_next - yaw_prev));

        // both in [-pi, pi]
        /* std::cout << yaw_prev << " " << yaw_next << " " << yaw_change << std::endl; */

        total_yaw_change += yaw_change;

        prev = current;
        current = next;
        ++next;
      }
    }

    return total_yaw_change / path_length;
  }

  static double evaluateMetric(const ompl::geometric::PathGeometric& trajectory,
                               double, bool visualize = false) {
    const auto path = Point::fromPath(trajectory);
    return evMetric(path, visualize);
  }

  static double evaluateMetric(const ompl::control::PathControl& trajectory,
                               double, bool visualize = false) {
    const auto path = Point::fromPath(trajectory);
    return evMetric(path, visualize);
  }

  static const bool MoreIsBetter = false;

 private:
  static double distance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2.) + std::pow(y2 - y1, 2.));
  }
};

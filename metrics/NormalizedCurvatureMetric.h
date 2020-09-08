#pragma once

#include <cmath>
#include <vector>

#include "../base/PlannerSettings.h"
#include "TrajectoryMetric.h"

#if QT_SUPPORT
#include "gui/QtVisualizer.h"
#endif

class NormalizedCurvatureMetric : public TMetric<NormalizedCurvatureMetric> {
 public:
  /**
   * Computes a normalized curvature metric of the given trajectory.
   *
   * This is the sum over segment_curvature * segment_length.
   *
   * @param trajectory The trajectory to evaluate.
   * @param visualize
   * @return Normalized curvature.
   */
  static inline double max_curvature = std::numeric_limits<double>::max();

  static double evaluateMetric(const ompl::geometric::PathGeometric &trajectory,
                               double, bool visualize = false) {
    const auto path = Point::fromPath(trajectory);

    double x1, x2, x3, y1, y2, y3, v1x, v2x, v1y, v2y, v1, v2;
    double infinity = std::numeric_limits<double>::max();
    double normalized_k = 0;

    size_t traj_size = path.size();

    // Handles the empty input path, setting curvature to 0
    // TODO: why is this infinity for MaxCurvatureMetric.h (?)
    if (traj_size == 0) {
      return 0;
    }

    // Handles the input path of length 1 or 2, setting curvature to 0
    if (traj_size < 3) return 0;

    // We can compute the curvature in all the points of the path
    // except the first and the last one
    for (int i = 0; i < (traj_size - 2); i++) {
      // skip by 2 two steps in both directions
      // to better catch abrupt changes in position
      x1 = path[i].x;
      y1 = path[i].y;

      do {
        ++i;
        if (i >= path.size()) return normalized_k;
        x2 = path[i].x;
        y2 = path[i].y;
      } while (distance(x1, y1, x2, y2) < 0.3);

      do {
        ++i;
        if (i >= path.size()) return normalized_k;
        x3 = path[i].x;
        y3 = path[i].y;
      } while (distance(x2, y2, x3, y3) < 0.3);

      // if two points in a row repeat, we skip curvature computation
      if (x1 == x2 && y1 == y2 || x2 == x3 && y2 == y3) continue;

      // Infinite curvature in case the path goes a step backwards:
      // p1 - p2 - p1
      if (x1 == x3 && y1 == y3) {
        OMPL_WARN("Undefined curvature. Skipping three steps...");
        continue;
      }

      // Compute center of circle that goes through the 3 points
      double cx =
          (std::pow(x3, 2.) * (-y1 + y2) + std::pow(x2, 2.) * (y1 - y3) -
           (std::pow(x1, 2.) + (y1 - y2) * (y1 - y3)) * (y2 - y3)) /
          (2. * (x3 * (-y1 + y2) + x2 * (y1 - y3) + x1 * (-y2 + y3)));
      double cy =
          (-(std::pow(x2, 2.) * x3) + std::pow(x1, 2.) * (-x2 + x3) +
           x3 * (std::pow(y1, 2.) - std::pow(y2, 2.)) +
           x1 * (std::pow(x2, 2.) - std::pow(x3, 2.) + std::pow(y2, 2.) -
                 std::pow(y3, 2.)) +
           x2 * (std::pow(x3, 2.) - std::pow(y1, 2.) + std::pow(y3, 2.))) /
          (2. * (x3 * (y1 - y2) + x1 * (y2 - y3) + x2 * (-y1 + y3)));

      // Curvature = 1/Radius
      double radius = std::sqrt(std::pow(x1 - cx, 2.) + std::pow(y1 - cy, 2.));
      double ki = 1. / radius;

#ifdef DEBUG
#if QT_SUPPORT
      if (visualize && ki > 0.5) {
        QtVisualizer::drawNode(
            x1, y1, QColor(255, std::max(0, (int)(255 - ki * 10)), 0, 50), .8);
        QtVisualizer::drawNode(
            x2, y2, QColor(255, std::max(0, (int)(255 - ki * 10)), 0, 50), .8);
        QtVisualizer::drawNode(
            x3, y3, QColor(255, std::max(0, (int)(255 - ki * 10)), 0, 50), .8);
      }
#endif
#endif

      ki = std::min(ki, max_curvature);

      normalized_k +=
          ki * (distance(x1, y1, x2, y2) + distance(x2, y2, x3, y3));
    }

    return normalized_k;
  }

  static const bool MoreIsBetter = false;

 private:
  static double distance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2.) + std::pow(y2 - y1, 2.));
  }
};
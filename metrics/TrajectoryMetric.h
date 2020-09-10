#pragma once

#include "utils/PlannerUtils.hpp"

#include "steer_functions/Steering.h"

template <class METRIC, bool MoreIsBetter = false>
class TMetric {
 public:
  static constexpr double ComparisonTolerance = 1e-6;

  static double evaluate(const ompl::geometric::PathGeometric &trajectory,
                         double dt = 0.1) {
    return METRIC::evaluateMetric(trajectory, dt);
  }

  static double evaluate(const ompl::control::PathControl &trajectory,
                         double dt = 0.1) {
    return METRIC::evaluateMetric(trajectory, dt);
  }

  /**
   * Compares two trajectories by evaluating this metric on them.
   * @param a Trajectory.
   * @param b Trajectory.
   * @return 1 if a is better than b, 0 if a and b have the same value, -1 if b
   * is better than a
   */
  static int compare(const ompl::geometric::PathGeometric &a,
                     const ompl::geometric::PathGeometric &b, double dt = 0.1) {
    double va = evaluate(a, dt), vb = evaluate(b, dt);
    if (std::abs(va - vb) < ComparisonTolerance) return 0;
    if (MoreIsBetter && va > vb) return 1;
    return -1;
  }

  /**
   * Compares two trajectories by evaluating this metric on them.
   * @param a Trajectory.
   * @param b Trajectory.
   * @return 1 if a is better than b, 0 if a and b have the same value, -1 if b
   * is better than a
   */
  static int compare(const ompl::control::PathControl &a,
                     const ompl::control::PathControl &b, double dt = 0.1) {
    double va = evaluate(a, dt), vb = evaluate(b, dt);
    if (std::abs(va - vb) < ComparisonTolerance) return 0;
    if (MoreIsBetter && va > vb) return 1;
    return -1;
  }
};
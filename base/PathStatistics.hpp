#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <params.hpp>

#include "Primitives.h"

using namespace params;

struct PathStatistics : public Group {
  Property<double> planning_time{std::numeric_limits<double>::quiet_NaN(),
                                 "planning_time", this};
  Property<bool> path_found{false, "path_found", this};
  Property<bool> path_collides{true, "path_collides", this};
  Property<bool> exact_goal_path{true, "exact_goal_path", this};
  Property<double> path_length{std::numeric_limits<double>::quiet_NaN(),
                               "path_length", this};
  Property<double> curvature{std::numeric_limits<double>::quiet_NaN(),
                             "curvature", this};
  Property<double> smoothness{std::numeric_limits<double>::quiet_NaN(),
                              "smoothness", this};
  Property<double> mean_clearing_distance{
      std::numeric_limits<double>::quiet_NaN(), "mean_clearing_distance", this};
  Property<double> median_clearing_distance{
      std::numeric_limits<double>::quiet_NaN(), "median_clearing_distance",
      this};
  Property<double> min_clearing_distance{
      std::numeric_limits<double>::quiet_NaN(), "min_clearing_distance", this};
  Property<double> max_clearing_distance{
      std::numeric_limits<double>::quiet_NaN(), "max_clearing_distance", this};
  Property<std::string> planner{"UNKNOWN", "planner", this};
  Property<std::vector<Point>> cusps{{}, "cusps", this};

  explicit PathStatistics(const std::string &planner = "UNKNOWN")
      : Group("stats") {
    this->planner = planner;
  }
};

namespace stat {
inline double median(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  size_t count = values.size();
  if (count % 2 == 1) return values[count / 2];

  double right = values[count / 2];
  double left = values[count / 2 - 1];
  return (right + left) / 2.0;
}

inline double mean(const std::vector<double> &values) {
  double avg = 0;
  for (double v : values) avg += v;
  return avg / values.size();
}

inline double min(const std::vector<double> &values) {
  double m = std::numeric_limits<double>::max();
  for (double v : values) m = std::min(m, v);
  return m;
}

inline double max(const std::vector<double> &values) {
  double m = std::numeric_limits<double>::min();
  for (double v : values) m = std::max(m, v);
  return m;
}

inline double std(const std::vector<double> &values) {
  double s = 0;
  double m = mean(values);
  for (double v : values) s += std::pow(v - m, 2.);
  return std::sqrt(1. / values.size() * s);
}
}  // namespace stat

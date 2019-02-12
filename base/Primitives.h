#pragma once

#include <ompl/base/State.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <ompl/geometric/PathGeometric.h>

namespace ob = ompl::base;

typedef ob::SE2StateSpace::StateType State;

namespace base {
ob::State *StateFromXYT(double x, double y, double theta);

ob::State *StateFromXY(double x, double y);
}  // namespace base

struct Rectangle {
  double x1{0}, y1{0};
  double x2{0}, y2{0};

  Rectangle() = default;
  Rectangle(double x1, double y1, double x2, double y2)
      : x1(x1), y1(y1), x2(x2), y2(y2) {}

  inline double x() const { return std::min(x1, x2); }

  inline double y() const { return std::min(y1, y2); }

  inline double width() const { return std::abs(x1 - x2); }

  inline double height() const { return std::abs(y1 - y2); }
};

struct Point {
  double x{0}, y{0};

  Point() = default;
  Point(double x, double y) : x(x), y(y) {}
  Point(const ob::State *state) {
    x = state->as<State>()->getX();
    y = state->as<State>()->getY();
  }

  static std::vector<Point> fromPath(const ompl::geometric::PathGeometric &p,
                                     bool interpolate = true);

  static Point centroid(const std::vector<Point> &points) {
    double x = 0, y = 0;
    for (auto &p : points) {
      x += p.x;
      y += p.y;
    }
    return {x / points.size(), y / points.size()};
  }

  double distance(const Point &p) const {
    const double dx = p.x - x;
    const double dy = p.y - y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double distance(double x, double y) const {
    const double dx = this->x - x;
    const double dy = this->y - y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double distanceSquared(const Point &p) const {
    const double dx = p.x - x;
    const double dy = p.y - y;
    return dx * dx + dy * dy;
  }

  double distanceSquared(double x, double y) const {
    const double dx = this->x - x;
    const double dy = this->y - y;
    return dx * dx + dy * dy;
  }

  ompl::base::State *toState(double theta = 0) const;
};
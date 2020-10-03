#include "base/Primitives.h"

#include "base/PlannerSettings.h"
#include "utils/PlannerUtils.hpp"

ompl::base::State *base::StateFromXYT(double x, double y, double theta) {
  ompl::base::State *state = global::settings.ompl.state_space->allocState();
  state->as<State>()->setX(x);
  state->as<State>()->setY(y);
  state->as<State>()->setYaw(theta);
  return state;
}

ompl::base::State *base::StateFromXY(double x, double y) {
  ompl::base::State *state = global::settings.ompl.state_space->allocState();
  state->as<State>()->setX(x);
  state->as<State>()->setY(y);
  return state;
}

ompl::base::State *Point::toState(double theta) const {
  return base::StateFromXYT(x, y, theta);
}

std::vector<Point> Point::fromPath(const ompl::geometric::PathGeometric &p,
                                   bool interpolate) {
  ompl::geometric::PathGeometric path(p);
  if (interpolate) path = PlannerUtils::interpolated(path);
  std::vector<Point> result;
  for (const auto *state : path.getStates()) {
    double x, y;
    if (global::settings.forwardpropagation.forward_propagation_type ==
        ForwardPropagation::FORWARD_PROPAGATION_TYPE_KINEMATIC_SINGLE_TRACK) {
      const auto *compState = state->as<ob::CompoundStateSpace::StateType>();
      const auto *se2state = compState->as<ob::SE2StateSpace::StateType>(0);
      x = se2state->getX();
      y = se2state->getY();
    } else {
      x = state->as<State>()->getX();
      y = state->as<State>()->getY();
    }
    result.emplace_back(Point(x, y));
  };
  return result;
}

std::vector<Point> Point::fromPath(const ompl::control::PathControl &p,
                                   bool interpolate) {
  ompl::control::PathControl path(p);
  if (interpolate) path = PlannerUtils::interpolated(path);
  std::vector<Point> result;
  for (const auto *state : path.getStates()) {
    double x, y;
    if (global::settings.forwardpropagation.forward_propagation_type ==
        ForwardPropagation::FORWARD_PROPAGATION_TYPE_KINEMATIC_SINGLE_TRACK) {
      const auto *compState = state->as<ob::CompoundStateSpace::StateType>();
      const auto *se2state = compState->as<ob::SE2StateSpace::StateType>(0);
      x = se2state->getX();
      y = se2state->getY();
    } else {
      x = state->as<State>()->getX();
      y = state->as<State>()->getY();
    }
    result.emplace_back(Point(x, y));
  };
  return result;
}

bool Polygon::isConvex() const {
  // reference:
  // http://csharphelper.com/blog/2014/07/determine-whether-a-polygon-is-convex-in-c/
  bool got_negative = false;
  bool got_positive = false;
  int num_points = static_cast<int>(points.size());
  for (int i = 0; i < num_points; i++) {
    int j = (i + 1) % num_points;
    int k = (j + 1) % num_points;
    double jix = points[i].x - points[j].x;
    double jiy = points[i].y - points[j].y;
    double jkx = points[k].x - points[j].x;
    double jky = points[k].y - points[j].y;
    double cross_product = jix * jky - jiy * jkx;
    if (cross_product < 0.) {
      got_negative = true;
    } else if (cross_product > 0.) {
      got_positive = true;
    }
    if (got_negative && got_positive) {
      return false;
    }
  }
  return true;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(const Point &p, const Point &q, const Point &r) {
  double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
  if (std::abs(val) < 1e-8) return 0;  // colinear
  return (val > 0) ? 1 : 2;            // clock or counterclock wise
}

Polygon Polygon::convexHull() const {
  // Jarvis' algorithm
  // reference:
  // https://www.geeksforgeeks.org/convex-hull-set-1-jarviss-algorithm-or-wrapping/

  std::size_t n = points.size();

  // There must be at least 3 points
  if (n < 3) return *this;

  // Initialize Result
  Polygon hull;

  // Find the leftmost point
  std::size_t l = 0;
  for (std::size_t i = 1; i < n; i++) {
    if (points[i].x < points[l].x) {
      l = i;
    }
  }

  // Start from leftmost point, keep moving counterclockwise
  // until reach the start point again.  This loop runs O(h)
  // times where h is number of points in result or output.
  std::size_t p = l, q;
  do {
    // Add current point to result
    hull.points.push_back(points[p]);

    // Search for a point 'q' such that orientation(p, x,
    // q) is counterclockwise for all points 'x'. The idea
    // is to keep track of last visited most counterclock-
    // wise point in q. If any point 'i' is more counterclock-
    // wise than q, then update q.
    q = (p + 1) % n;
    for (int i = 0; i < n; i++) {
      // If i is more counterclockwise than current q, then
      // update q
      if (orientation(points[p], points[i], points[q]) == 2) {
        q = i;
      }
    }

    // Now q is the most counterclockwise with respect to p
    // Set p as q for next iteration, so that q is added to
    // result 'hull'
    p = q;

  } while (p != l);  // While we don't come to first point

  return hull;
}

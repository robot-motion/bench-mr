#include "Primitives.h"
#include "PlannerSettings.h"
#include "PlannerUtils.hpp"

ompl::base::State *base::StateFromXYT(double x, double y, double theta) {
  ompl::base::State *state = settings.ompl.state_space->allocState();
  state->as<State>()->setX(x);
  state->as<State>()->setY(y);
  state->as<State>()->setYaw(theta);
  return state;
}

ompl::base::State *base::StateFromXY(double x, double y) {
  ompl::base::State *state = settings.ompl.state_space->allocState();
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
  if (interpolate) path.interpolate();
  std::vector<Point> result;
  for (const auto *state : path.getStates()) result.emplace_back(Point(state));
  return result;
}

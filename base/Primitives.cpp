#include "Primitives.h"
#include "PlannerSettings.h"
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

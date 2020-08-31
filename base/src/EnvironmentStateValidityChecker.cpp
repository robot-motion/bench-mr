#include "base/EnvironmentStateValidityChecker.h"

// TODO: set specs_: clearance

bool EnvironmentStateValidityChecker::isValid(const ob::State *state) const {
  return env_->checkValidity(state);

  // TODO: check old code, compare with Environment::checkValidity
  // if (global::settings.env.collision.collision_model == robot::ROBOT_POINT) {
  //   ss->setStateValidityChecker([&](const ob::State *state) -> bool {
  //     const auto *s = state->as<ob::SE2StateSpace::StateType>();
  //     const double x = s->getX(), y = s->getY();
  //     //        if (global::settings.environment->collides(x, y))
  //     //            std::cout << "[" << x << " " << y << "] is occupied\n";
  //     return !global::settings.environment->collides(x, y);
  //   });
  // } else {
  //   if (global::settings.env.collision.robot_shape.value().points.size() < 3)
  //   {
  //     OMPL_ERROR(
  //         "Robot shape is empty or not convex. Cannot perform polygon-based "
  //         "collision detection.");
  //     return;
  //   }
  //   ss->setStateValidityChecker([&](const ob::State *state) -> bool {
  //     return !global::settings.environment->collides(
  //         global::settings.env.collision.robot_shape.value().transformed(
  //             state));
  //   });
  // }
}

double EnvironmentStateValidityChecker::clearance(
    const ob::State *state) const {
  return env_->distance(state);
}
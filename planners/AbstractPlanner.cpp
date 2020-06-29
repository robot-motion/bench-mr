#include "AbstractPlanner.h"

std::string AbstractPlanner::LastCreatedPlannerName = "";

AbstractPlanner::AbstractPlanner(const std::string &name) {
  LastCreatedPlannerName = name;
  if (ss) {
    ss->clear();
    ss->clearStartStates();
  }
  delete ss;

  if (ss_c) {
    ss_c->clear();
    ss_c->clearStartStates();
  }
  delete ss_c;

  if (!global::settings.benchmark.forward_propagations.value().empty()) {
    ss_c = new oc::SimpleSetup(global::settings.ompl.control_space);

    if (global::settings.env.collision.collision_model == robot::ROBOT_POINT) {
      ss_c->setStateValidityChecker([&](const ob::State *state) -> bool {
        const auto *s = state->as<ob::SE2StateSpace::StateType>();
        const double x = s->getX(), y = s->getY();
        return !global::settings.environment->collides(x, y);
      });
    } else {
      if (global::settings.env.collision.robot_shape.value().points.size() <
          3) {
        OMPL_ERROR(
            "Robot shape is empty or not convex. Cannot perform polygon-based "
            "collision detection.");
        return;
      }
      ss_c->setStateValidityChecker([&](const ob::State *state) -> bool {
        return !global::settings.environment->collides(
            global::settings.env.collision.robot_shape.value().transformed(
                state));
      });
    }
  } else {
    ss = new og::SimpleSetup(global::settings.ompl.state_space);

    if (global::settings.env.collision.collision_model == robot::ROBOT_POINT) {
      ss->setStateValidityChecker([&](const ob::State *state) -> bool {
        const auto *s = state->as<ob::SE2StateSpace::StateType>();
        const double x = s->getX(), y = s->getY();
        return !global::settings.environment->collides(x, y);
      });
    } else {
      if (global::settings.env.collision.robot_shape.value().points.size() <
          3) {
        OMPL_ERROR(
            "Robot shape is empty or not convex. Cannot perform polygon-based "
            "collision detection.");
        return;
      }
      ss->setStateValidityChecker([&](const ob::State *state) -> bool {
        return !global::settings.environment->collides(
            global::settings.env.collision.robot_shape.value().transformed(
                state));
      });
    }
  }

  auto &si = global::settings.ompl.space_info;

  if (global::settings.steer.steering_type == Steering::STEER_TYPE_POSQ) {
    ob::MotionValidatorPtr motionValidator(new POSQMotionValidator(si));
    si->setMotionValidator(motionValidator);
  }
#ifdef G1_AVAILABLE
  else if (global::settings.steer.steering_type ==
           Steering::STEER_TYPE_CLOTHOID) {
    ob::MotionValidatorPtr motionValidator(
        new G1ClothoidStateSpaceValidator(si));
    si->setMotionValidator(motionValidator);
    si->setStateValidityCheckingResolution(0.03);
    // lower granularity necessary to avoid too densely spaced nodes
    // which causes problems in Clothoid steering
  }
#endif

  si->setStateValidityCheckingResolution(
      global::settings.steer.sampling_resolution);

  const auto start = global::settings.environment->startScopedState();
  const auto goal = global::settings.environment->goalScopedState();
  std::cout << "Start: " << std::endl << start << std::endl;
  std::cout << "Goal: " << std::endl << goal << std::endl;
  if (!global::settings.benchmark.forward_propagations.value().empty()) {
    ss_c->setOptimizationObjective(global::settings.ompl.objective);
    ss_c->setStartAndGoalStates(start, goal,
                                global::settings.exact_goal_radius);
    ss_c->setup();
  } else {
    ss->setOptimizationObjective(global::settings.ompl.objective);
    ss->setStartAndGoalStates(start, goal, global::settings.exact_goal_radius);
    ss->setup();
  }
}

AbstractPlanner::~AbstractPlanner() {
  if (ss) delete ss;
  if (ss_c) delete ss_c;
}

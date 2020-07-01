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

  control_based_ = global::settings.benchmark.control_planners_on;

  if (control_based_) {
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
    std::cout << "Creating Steer " << std::endl;

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

  if (!control_based_) {
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
  } else {
    auto &si = global::settings.ompl.control_space_info;
    si->setStateValidityCheckingResolution(
        global::settings.steer.sampling_resolution);
  }

  const auto start = global::settings.environment->startScopedState();
  const auto goal = global::settings.environment->goalScopedState();

  if (control_based_) {
    ss_c->setOptimizationObjective(global::settings.ompl.objective);

    // KinematicCar is SO3
    if (global::settings.forwardpropagation.forward_propagation_type ==
        ForwardPropagation::FORWARD_PROPAGATION_TYPE_KINEMATIC_CAR) {
      ss_c->setStartAndGoalStates(start, goal,
                                  global::settings.exact_goal_radius);
    }

    // KinematicSingleTrack is SO3 + R^2
    if (global::settings.forwardpropagation.forward_propagation_type ==
        ForwardPropagation::FORWARD_PROPAGATION_TYPE_KINEMATIC_SINGLE_TRACK) {
      start->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0] = 0;
      start->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[1] = 0;
      goal->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0] = 0.0;
      goal->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[1] = 0.0;
      ss_c->setStartAndGoalStates(start, goal,
                                  global::settings.exact_goal_radius);
    }

  } else {
    ss->setOptimizationObjective(global::settings.ompl.objective);
    ss->setStartAndGoalStates(start, goal, global::settings.exact_goal_radius);
    ss->setup();
  }

  std::cout << "Start: " << std::endl << start << std::endl;
  std::cout << "Goal: " << std::endl << goal << std::endl;
}

AbstractPlanner::~AbstractPlanner() {
  if (ss) delete ss;
  if (ss_c) delete ss_c;
}

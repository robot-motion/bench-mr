#pragma once

#include <utility>

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>

#ifdef G1_AVAILABLE
#include "steer_functions/G1Clothoid/ClothoidSteering.hpp"
#include "steer_functions/G1Clothoid/G1ClothoidStateSpace.h"
#endif

#include "steer_functions/POSQ/POSQStateSpace.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class AbstractPlanner {
 public:
  virtual std::string name() const = 0;

  virtual ob::PlannerStatus run() = 0;

  virtual std::vector<Point> solutionPath() const {
    auto path = solution();
    return Point::fromPath(path);
  }

  struct IntermediarySolution {
    double time{std::numeric_limits<double>::quiet_NaN()};
    double cost{std::numeric_limits<double>::quiet_NaN()};
    og::PathGeometric solution;
    IntermediarySolution(double time, double cost,
                         const og::PathGeometric &solution)
        : time(time), cost(cost), solution(solution) {}
  };

  std::vector<IntermediarySolution> intermediarySolutions;

  /**
   * Returns the solution of the planner, which is a sparse PathGeometric.
   */
  virtual og::PathGeometric solution() const = 0;

  virtual bool hasReachedGoalExactly() const = 0;
  virtual double planningTime() const = 0;

 public:
  bool isValid(const ob::State *state) const {
    return ss->getStateValidityChecker()->isValid(state);
  }
  bool isValid(og::PathGeometric &path) const {
    for (const auto *state : path.getStates())
      if (!ss->getStateValidityChecker()->isValid(state)) return false;
    return true;
  }

  og::SimpleSetup *simpleSetup() const { return ss; }

 protected:
  og::SimpleSetup *ss{nullptr};

  AbstractPlanner() {
    ss = new og::SimpleSetup(global::settings.ompl.state_space);
    auto &si = global::settings.ompl.space_info;

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

    ss->setOptimizationObjective(global::settings.ompl.objective);

    const auto start = global::settings.environment->startScopedState();
    const auto goal = global::settings.environment->goalScopedState();
    std::cout << "Start: " << std::endl << start << std::endl;
    std::cout << "Goal: " << std::endl << goal << std::endl;
    ss->setStartAndGoalStates(start, goal, global::settings.exact_goal_radius);

    ss->setup();
  }

 public:
  virtual ob::Planner *omplPlanner() { return nullptr; }
};

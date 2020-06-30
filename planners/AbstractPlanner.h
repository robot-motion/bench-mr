#pragma once

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/Control.h>
#include <ompl/control/PathControl.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <utility>
#ifdef G1_AVAILABLE
#include "steer_functions/G1Clothoid/ClothoidSteering.hpp"
#include "steer_functions/G1Clothoid/G1ClothoidStateSpace.h"
#endif

#include "steer_functions/POSQ/POSQStateSpace.h"

#include <ompl/control/ODESolver.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

class AbstractPlanner {
 public:
  virtual std::string name() const = 0;
  virtual ~AbstractPlanner();

  /**
   * Stores the name of the planner created last. Used for tracing errors
   * during planner construction.
   */
  static std::string LastCreatedPlannerName;

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

  struct IntermediaryControlSolution {
    double time{std::numeric_limits<double>::quiet_NaN()};
    double cost{std::numeric_limits<double>::quiet_NaN()};
    oc::PathControl solution;
    IntermediaryControlSolution(double time, double cost,
                                const oc::PathControl &solution)
        : time(time), cost(cost), solution(solution) {}
  };

  std::vector<IntermediaryControlSolution> intermediaryControlSolutions;

  /**
   * Returns the solution of the planner, which is a sparse PathGeometric.
   */
  virtual og::PathGeometric solution() const = 0;

  virtual bool hasReachedGoalExactly() const = 0;
  virtual double planningTime() const = 0;

 public:
  bool isValid(const ob::State *state) const {
    return getCurrStateValidityCheckerPtr()->isValid(state);
  }
  bool isValid(og::PathGeometric &path) const {
    for (const auto *state : path.getStates())
      if (!getCurrStateValidityCheckerPtr()->isValid(state)) return false;
    return true;
  }
  bool isValid(og::PathGeometric &path, std::vector<Point> &collisions) const {
    collisions.clear();
    for (const auto *state : path.getStates())
      if (!getCurrStateValidityCheckerPtr()->isValid(state))
        collisions.emplace_back(state);
    return collisions.empty();
  }

  og::SimpleSetup *simpleSetup() const { return ss; }

  ob::StateValidityCheckerPtr getCurrStateValidityCheckerPtr() const {
    if (control_based_) {
      return ss_c->getStateValidityChecker();
    } else {
      return ss->getStateValidityChecker();
    }
  }

 protected:
  og::SimpleSetup *ss{nullptr};
  oc::SimpleSetup *ss_c{nullptr};
  bool control_based_{false};
  explicit AbstractPlanner(const std::string &name);

 public:
  virtual ob::Planner *omplPlanner() { return nullptr; }
};

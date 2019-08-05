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

  AbstractPlanner(const std::string &name);

 public:
  virtual ob::Planner *omplPlanner() { return nullptr; }
};

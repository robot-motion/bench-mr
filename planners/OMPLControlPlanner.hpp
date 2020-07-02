#pragma once

#include <memory>

#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>

#include "base/PlannerSettings.h"
#include "fp_models/kinematic_car/kinematic_car.hpp"
#include "fp_models/kinematic_single_track/kinematic_single_track.hpp"

#include "planners/AbstractPlanner.h"

#include "utils/PlannerUtils.hpp"
#include "utils/Stopwatch.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

template <class PLANNER>
class OMPLControlPlanner : public AbstractPlanner {
 public:
  OMPLControlPlanner() : AbstractPlanner("OMPL") {
    _omplPlanner = ob::PlannerPtr(new PLANNER(ss_c->getSpaceInformation()));
    AbstractPlanner::LastCreatedPlannerName = name();
  }

  ob::PlannerStatus run() override {
    intermediaryControlSolutions.clear();

    ss_c->setPlanner(_omplPlanner);
    this->setStatePropagator();
    ss_c->setup();

    Stopwatch watch;
    auto problem = _omplPlanner->getProblemDefinition();

    problem->setIntermediateSolutionCallback(
        [&](const ob::Planner *planner,
            const std::vector<const ob::State *> &states, const ob::Cost cost) {
          oc::PathControl solution(global::settings.ompl.control_space_info);
          // the OMPL intermediary solution doesn't include the start state
          solution.append(global::settings.environment->startState());
          for (const auto *state : states) solution.append(state);
          // the OMPL intermediary solution doesn't include the goal state
          solution.append(global::settings.environment->goalState());
          IntermediaryControlSolution is(watch.elapsed(), cost.value(),
                                         solution);
          intermediaryControlSolutions.emplace_back(is);
        });

    watch.start();
    auto solved = ss_c->solve(global::settings.max_planning_time);
    OMPL_INFORM("OMPL %s planning status: %s", _omplPlanner->getName().c_str(),
                solved.asString().c_str());

    if (solved) {
      // Output the length of the path found
      _solution = oc::PathControl(global::settings.ompl.control_space_info);
      // _solution.append(global::settings.environment->startState());
      const auto &f = ss_c->getSolutionPath();
      for (std::size_t i = 0; i < f.getStateCount(); ++i) {
        _solution.append(f.getState(i));
      }

      // In ompl::control there is no notion of cost
      // OMPL_INFORM(
      //     "%s found a solution of length %f with an optimization objective "
      //     "value of %f",
      //     _omplPlanner->getName().c_str(), _solution.length(),
      //     _solution.cost(
      //         ss_c->getProblemDefinition()->getOptimizationObjective()));
    } else
      OMPL_WARN("No solution found.");
    return solved;
  }

  /**
   * @brief Set the needed state propagator
   * @return void
   */
  void setStatePropagator() {
    if (global::settings.forwardpropagation.forward_propagation_type ==
        ForwardPropagation::FORWARD_PROPAGATION_TYPE_KINEMATIC_CAR) {
      std::cout << "Setting KinematicCar" << std::endl;
      auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(
          ss_c->getSpaceInformation(), &KinematicCar::kinematicCarODE));

      ss_c->setStatePropagator(oc::ODESolver::getStatePropagator(
          odeSolver, &KinematicCar::kinematicCarPostIntegration));
    }

    if (global::settings.forwardpropagation.forward_propagation_type ==
        ForwardPropagation::FORWARD_PROPAGATION_TYPE_KINEMATIC_SINGLE_TRACK) {
      std::cout << "Setting kinematicSingleTrackODE" << std::endl;

      auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(
          ss_c->getSpaceInformation(),
          &kinematicSingleTrack::kinematicSingleTrackODE));

      ss_c->setStatePropagator(oc::ODESolver::getStatePropagator(
          odeSolver,
          &kinematicSingleTrack::kinematicSingleTrackPostIntegration));
    }
  }

  std::string name() const override { return _omplPlanner->getName(); }

  og::PathGeometric solution() const override {
    // og::PathGeometric path(_solution);
    return _solution.asGeometric();
  }

  bool hasReachedGoalExactly() const override {
    return ss_c->haveExactSolutionPath();
  }

  double planningTime() const override {
    return ss_c->getLastPlanComputationTime();
  }

  ob::Planner *omplPlanner() override { return _omplPlanner.get(); }

 private:
  ob::PlannerPtr _omplPlanner;
  oc::PathControl _solution{global::settings.ompl.control_space_info};
};

// control space
typedef OMPLControlPlanner<oc::KPIECE1> FPKPIECEPlanner;
typedef OMPLControlPlanner<oc::EST> FPESTPlanner;
typedef OMPLControlPlanner<oc::SST> FPSSTPlanner;
typedef OMPLControlPlanner<oc::RRT> FPRRTPlanner;
typedef OMPLControlPlanner<oc::PDST> FPPDSTPlanner;

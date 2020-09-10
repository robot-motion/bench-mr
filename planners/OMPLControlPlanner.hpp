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

#include "ompl/util/Console.h"

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
    ompl::msg::setLogLevel(ompl::msg::LOG_DEV1);

    ss_c->setPlanner(_omplPlanner);
    this->setStatePropagator();
    ss_c->getSpaceInformation()->setPropagationStepSize(.1);
    ss_c->getSpaceInformation()->setMinMaxControlDuration(2, 10);
    ss_c->setup();

    ss_c->getSpaceInformation()->printProperties(std::cout);
    Stopwatch watch;
    auto problem = ss_c->getProblemDefinition();

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

    const ob::PlannerTerminationCondition &ptc_approx =
        approxSolnPlannerTerminationCondition(problem);

    // auto solved =
    //     ss_c->solve(ptc_approx);  This is actually an ever ending loop. The
    //     ptc is never true. The control-based planners do not add approximate
    //     solutions

    auto solved = ss_c->solve(global::settings.max_planning_time);  // This stops the planner after max_planning_time sec

    OMPL_INFORM("OMPL %s planning status: %s", _omplPlanner->getName().c_str(),
                solved.asString().c_str());

    if (solved) {
      // Output the length of the path found
      _solution = ss_c->getSolutionPath();
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
    std::cout << "Returning solution << " << std::endl;
    return og::PathGeometric(ss_c->getSolutionPath().asGeometric());
  }

  bool hasReachedGoalExactly() const override {
    return ss_c->haveExactSolutionPath();
  }

  double planningTime() const override {
    return ss_c->getLastPlanComputationTime();
  }

  ob::Planner *omplPlanner() override { return _omplPlanner.get(); }

  ob::PlannerTerminationCondition approxSolnPlannerTerminationCondition(
      const ompl::base::ProblemDefinitionPtr &pdef) {
    return ob::PlannerTerminationCondition(
        [pdef] { return pdef->hasApproximateSolution(); });
  }

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

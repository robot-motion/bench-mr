#pragma once

#include <base/PlannerSettings.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/AnytimePathShortening.h>

#include "base/TimedResult.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class OmplSmoother {
 public:
  explicit OmplSmoother(og::SimpleSetup *ss, const og::PathGeometric &solution)
      : ss(ss), _solution(solution) {}

  virtual TimedResult shortcutPath() const {
    OMPL_INFORM("Running OMPL Shortcutting...");
    TimedResult r;
    r.status = ss->getLastPlannerStatus();
    og::PathGeometric path(ss->getSpaceInformation());
    path.append(_solution);
//            ss->getSpaceInformation()->setStateValidityCheckingResolution(0.2);
    og::PathSimplifier ps(ss->getSpaceInformation(), ss->getGoal());

    r.start();
    ps.shortcutPath(path, global::settings.smoothing.ompl.shortcut_max_steps,
                    global::settings.smoothing.ompl.shortcut_max_empty_steps,
                    global::settings.smoothing.ompl.shortcut_range_ratio,
                    global::settings.smoothing.ompl.shortcut_snap_to_vertex);
    r.stop();

    // have to reduce resolution
    //    path.interpolate();
    //        ss->getSpaceInformation()->setStateValidityCheckingResolution(0.05);
    r.trajectory = path;
    return r;
  }

  virtual TimedResult smoothBSpline() const {
    OMPL_INFORM("Running OMPL B-spline smoothing...");
    TimedResult r;
    r.status = ss->getLastPlannerStatus();
      og::PathGeometric path(ss->getSpaceInformation());
      path.append(_solution);
    //        ss->getSpaceInformation()->setStateValidityCheckingResolution(0.2);
    og::PathSimplifier ps(ss->getSpaceInformation(), ss->getGoal());

    r.start();
    ps.smoothBSpline(path, global::settings.smoothing.ompl.bspline_max_steps,
                     global::settings.smoothing.ompl.bspline_epsilon);
    r.stop();

    // have to reduce resolution
    //    path.interpolate();
    //        ss->getSpaceInformation()->setStateValidityCheckingResolution(0.05);
    r.trajectory = path;
    return r;
  }

  virtual TimedResult simplifyMax() const {
    OMPL_INFORM("Running OMPL Simplify Max smoothing...");
    TimedResult r;
    r.status = ss->getLastPlannerStatus();
      og::PathGeometric path(ss->getSpaceInformation());
      path.append(_solution);
    og::PathSimplifier ps(ss->getSpaceInformation(), ss->getGoal());

    r.start();
    ps.simplifyMax(path);
    r.stop();

    //    path.interpolate();
    r.trajectory = path;
    return r;
  }

//  virtual TimedResult anytimePathShortening(ob::Planner *omplPlanner) {
//    OMPL_INFORM("Running OMPL Anytime Path Shortening...");
//    ob::PlannerPtr planner;
//    planner =
//        std::make_shared<og::AnytimePathShortening>(ss->getSpaceInformation());
//    planner->as<og::AnytimePathShortening>()->setHybridize(
//        global::settings.smoothing.ompl.anytime_hybridize);
//    planner->as<og::AnytimePathShortening>()->setShortcut(
//        global::settings.smoothing.ompl.anytime_shortcut);
//    ob::PlannerPtr optimizingPlanner(omplPlanner);
//    planner->as<og::AnytimePathShortening>()->addPlanner(optimizingPlanner);
//    this->ss->setPlanner(planner);
//    this->ss->setup();
//    auto solved = this->ss->solve(global::settings.max_planning_time);
//    std::cout << "OMPL anytime path shortening planning status: "
//              << solved.asString().c_str() << std::endl;
//
//    TimedResult r;
//    r.status = solved;
//    if (solved) {
//      //            ss->simplifySolution(); // TODO define time limit?
//
//      auto path = ss->getSolutionPath();
//      //      path.interpolate();
//      //        ss->getSpaceInformation()->setStateValidityCheckingResolution(0.05);
//      r.trajectory = path;
//    } else
//      std::cout << "No solution found." << std::endl;
//
//    return r;
//  }

 private:
  og::SimpleSetup *ss{nullptr};
  og::PathGeometric _solution;
};

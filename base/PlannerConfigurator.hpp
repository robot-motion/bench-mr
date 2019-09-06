#pragma once

#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "PlannerSettings.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class PlannerConfigurator {
 public:
  PlannerConfigurator() = delete;

  template <typename PLANNER>
  static void configure(PLANNER &planner) {}

  static void configure(og::RRTstar &planner) {
    planner.setGoalBias(global::settings.ompl.rrt_star.goal_bias);
    planner.setRange(global::settings.ompl.rrt_star.max_distance);
  }
};

#pragma once

#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "PlannerSettings.h"
#include "planners/OMPLControlPlanner.hpp"
#include "planners/OMPLPlanner.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class PlannerConfigurator {
 public:
  PlannerConfigurator() = delete;

  template <typename PLANNER>
  static void configure(PLANNER &planner) {}

  template <typename OMPL_PLANNER>
  static void configure(OMPLPlanner<OMPL_PLANNER> &planner) {
    configure(*planner.omplPlanner(),
              global::settings.ompl.geometric_planner_settings.value());
  }

  template <typename OMPL_PLANNER>
  static void configure(OMPLControlPlanner<OMPL_PLANNER> &planner) {
    configure(*planner.omplPlanner(),
              global::settings.ompl.control_planner_settings.value());
  }

  static void configure(og::RRTstar &planner) {
    planner.setGoalBias(global::settings.ompl.rrt_star.goal_bias);
    planner.setRange(global::settings.ompl.rrt_star.max_distance);
  }

 private:
  static void configure(ob::Planner &planner, const nlohmann::json &settings) {
    const auto name = planner.getName();
    auto params = planner.params();
    if (settings.contains(name)) {
      for (auto &[key, value] : settings[name].items()) {
        params.setParam(key, value);
      }
    }
  }
};

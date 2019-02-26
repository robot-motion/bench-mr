#include "PlannerSettings.h"

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <steering_functions/include/ompl_state_spaces/CurvatureStateSpace.hpp>
#include "GridMaze.h"
#include "PolygonMaze.h"
#include "steer_functions/POSQ/POSQStateSpace.h"

#ifdef G1_AVAILABLE
#include "steer_functions/G1Clothoid/ClothoidSteering.hpp"
#endif

PlannerSettings::GlobalSettings global::settings;

void PlannerSettings::GlobalSettings::SteerSettings::initializeSteering()
    const {
  // Construct the robot state space in which we're planning.
  if (steering_type == Steering::STEER_TYPE_REEDS_SHEPP)
    global::settings.ompl.state_space =
        ob::StateSpacePtr(new ob::ReedsSheppStateSpace(car_turning_radius));
  else if (steering_type == Steering::STEER_TYPE_POSQ)
    global::settings.ompl.state_space = ob::StateSpacePtr(new POSQStateSpace());
  else if (steering_type == Steering::STEER_TYPE_DUBINS)
    global::settings.ompl.state_space =
        ob::StateSpacePtr(new ob::DubinsStateSpace(car_turning_radius));
  else if (steering_type == Steering::STEER_TYPE_LINEAR)
    global::settings.ompl.state_space =
        ob::StateSpacePtr(new ob::SE2StateSpace);
  else if (steering_type == Steering::STEER_TYPE_CC_DUBINS)
    global::settings.ompl.state_space =
        ob::StateSpacePtr(new hc_cc_spaces::CCDubinsStateSpace(
            hc_cc.kappa, hc_cc.sigma, sampling_resolution));
  else if (steering_type == Steering::STEER_TYPE_CC_REEDS_SHEPP)
    global::settings.ompl.state_space =
        ob::StateSpacePtr(new hc_cc_spaces::CCReedsSheppStateSpace(
            hc_cc.kappa, hc_cc.sigma, sampling_resolution));
  else if (steering_type == Steering::STEER_TYPE_HC_REEDS_SHEPP)
    global::settings.ompl.state_space =
        ob::StateSpacePtr(new hc_cc_spaces::HCReedsSheppStateSpace(
            hc_cc.kappa, hc_cc.sigma, sampling_resolution));
#ifdef G1_AVAILABLE
  else if (steering_type == Steering::STEER_TYPE_CLOTHOID)
    global::settings.ompl.state_space =
        ob::StateSpacePtr(new G1ClothoidStateSpace());
#else
  else if (steering_type == Steering::STEER_TYPE_CLOTHOID) {
    OMPL_ERROR("G1 Clothoid steering is not available in this release!");
    OMPL_ERROR(
        "Select a steering type other than STEER_TYPE_CLOTHOID in the "
        "global::settings.");
  }
#endif
  else {
    OMPL_ERROR(
        "Unknown steer function has been defined. The state space is invalid.");
    return;
  }

  global::settings.ompl.state_space->as<ob::SE2StateSpace>()->setBounds(
      global::settings.environment->bounds());
  //  global::settings.ompl.state_space->setup();

  global::settings.ompl.space_info =
      std::make_shared<ompl::base::SpaceInformation>(
          global::settings.ompl.state_space);
  global::settings.ompl.objective = ompl::base::OptimizationObjectivePtr(
      new ob::PathLengthOptimizationObjective(
          global::settings.ompl.space_info));
  global::settings.ompl.objective->setCostThreshold(
      ob::Cost(global::settings.ompl.cost_threshold));

#ifdef DEBUG
  std::cout << "global::settings.ompl.state_space->hasDefaultProjection() ? "
            << std::boolalpha
            << global::settings.ompl.state_space->hasDefaultProjection()
            << std::endl;
#endif

  OMPL_INFORM("Initialized steer function %s",
              Steering::to_string(steering_type).c_str());
}

void PlannerSettings::GlobalSettings::EnvironmentSettings::createEnvironment() {
  delete global::settings.environment;
  if (type.value() == "grid") {
    if (grid.generator.value() == "corridor") {
      global::settings.environment = GridMaze::createRandomCorridor(
          grid.width, grid.height, grid.corridor.radius, grid.corridor.branches,
          grid.seed);
    } else if (grid.generator.value() == "random") {
      global::settings.environment = GridMaze::createRandom(
          grid.width, grid.height, grid.random.obstacle_ratio, grid.seed);
    } else {
      OMPL_ERROR("Unknown grid environment generator \"%s\".",
                 grid.generator.value().c_str());
    }
  } else if (type.value() == "polygon") {
    global::settings.environment = PolygonMaze::loadFromSvg(polygon.source);
  } else {
    OMPL_ERROR("Unknown environment type \"%s\".", type.value().c_str());
  }

  // Load polygon for polygon-based collision checker if necessary
  if (global::settings.collision_model == robot::ROBOT_POLYGON) {
    global::settings.robot_shape =
        SvgPolygonLoader::load(global::settings.robot_shape_source)[0];
    global::settings.robot_shape.value().center();
    OMPL_INFORM("Loaded polygon robot model from %s with %d vertices.",
                global::settings.robot_shape_source.value().c_str(),
                global::settings.robot_shape.value().points.size());
  }
}

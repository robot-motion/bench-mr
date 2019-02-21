#include "PlannerSettings.h"

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <steering_functions/include/ompl_state_spaces/CurvatureStateSpace.hpp>
#include "steer_functions/POSQ/POSQStateSpace.h"

#ifdef G1_AVAILABLE
#include "steer_functions/G1Clothoid/ClothoidSteering.hpp"
#endif

void GlobalSettings::SteerSettings::initializeSteering() const {
  // Construct the robot state space in which we're planning.
  if (steering_type == Steering::STEER_TYPE_REEDS_SHEPP)
    settings.ompl.state_space =
        ob::StateSpacePtr(new ob::ReedsSheppStateSpace(car_turning_radius));
  else if (steering_type == Steering::STEER_TYPE_POSQ)
    settings.ompl.state_space = ob::StateSpacePtr(new POSQStateSpace());
  else if (steering_type == Steering::STEER_TYPE_DUBINS)
    settings.ompl.state_space =
        ob::StateSpacePtr(new ob::DubinsStateSpace(car_turning_radius));
  else if (steering_type == Steering::STEER_TYPE_LINEAR)
    settings.ompl.state_space = ob::StateSpacePtr(new ob::SE2StateSpace);
  else if (steering_type == Steering::STEER_TYPE_CC_DUBINS)
    settings.ompl.state_space =
        ob::StateSpacePtr(new hc_cc_spaces::CCDubinsStateSpace(
            hc_cc.kappa, hc_cc.sigma, sampling_resolution));
  else if (steering_type == Steering::STEER_TYPE_CC_REEDS_SHEPP)
    settings.ompl.state_space =
        ob::StateSpacePtr(new hc_cc_spaces::CCReedsSheppStateSpace(
            hc_cc.kappa, hc_cc.sigma, sampling_resolution));
  else if (steering_type == Steering::STEER_TYPE_HC_REEDS_SHEPP)
    settings.ompl.state_space =
        ob::StateSpacePtr(new hc_cc_spaces::HCReedsSheppStateSpace(
            hc_cc.kappa, hc_cc.sigma, sampling_resolution));
#ifdef G1_AVAILABLE
  else if (steering_type == Steering::STEER_TYPE_CLOTHOID)
    settings.ompl.state_space = ob::StateSpacePtr(new G1ClothoidStateSpace());
#else
  else if (steering_type == Steering::STEER_TYPE_CLOTHOID) {
    OMPL_ERROR("G1 Clothoid steering is not available in this release!");
    OMPL_ERROR(
        "Select a steering type other than STEER_TYPE_CLOTHOID in the "
        "settings.");
  }
#endif

  settings.ompl.state_space->as<ob::SE2StateSpace>()->setBounds(
      settings.environment->bounds());

  settings.ompl.space_info =
      std::make_shared<ompl::base::SpaceInformation>(settings.ompl.state_space);
  settings.ompl.objective = ompl::base::OptimizationObjectivePtr(
      new ob::PathLengthOptimizationObjective(settings.ompl.space_info));

  OMPL_INFORM("Initialized steer function");
}

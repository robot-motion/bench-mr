#include "AbstractPlanner.h"

#include "base/EnvironmentStateValidityChecker.h"

std::string AbstractPlanner::LastCreatedPlannerName = "";

AbstractPlanner::AbstractPlanner(const std::string &name) {
  LastCreatedPlannerName = name;
  if (ss) {
    ss->clear();
    ss->clearStartStates();
  }
  delete ss;
  ss = new og::SimpleSetup(global::settings.ompl.space_info);
  auto &si = global::settings.ompl.space_info;

  ss->setStateValidityChecker(std::make_shared<EnvironmentStateValidityChecker>(
      global::settings.ompl.space_info, global::settings.environment));

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

AbstractPlanner::~AbstractPlanner() { delete ss; }

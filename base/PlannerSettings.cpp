#include <memory>

#include "PlannerSettings.h"

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include "steer_functions/POSQ/POSQStateSpace.h"

#ifdef G1_AVAILABLE
#include "steer_functions/G1Clothoid/ClothoidSteering.hpp"
#endif

Environment *PlannerSettings::environment = nullptr;

int PlannerSettings::numberEdges = 10;

double PlannerSettings::stateEqualityTolerance = 1e-4;

// steering function settings
Steering::SteeringType PlannerSettings::steeringType =
    Steering::STEER_TYPE_REEDS_SHEPP;
double PlannerSettings::CarTurningRadius = 4;
double PlannerSettings::LinearSteeringDelta = 3.0;

double PlannerSettings::samplingResolution = 0.2;

ompl::base::StateSpacePtr PlannerSettings::stateSpace(nullptr);
ompl::base::SpaceInformationPtr PlannerSettings::spaceInfo{nullptr};
ompl::base::OptimizationObjectivePtr PlannerSettings::objective{nullptr};

bool PlannerSettings::VisualizeSmoothing1 = true;
bool PlannerSettings::VisualizeSmoothing2 = true;
bool PlannerSettings::VisualizeSmoothing3 = true;
bool PlannerSettings::VisualizeSmoothing4 = true;

void PlannerSettings::initializeSteering() {
  ob::RealVectorBounds bounds(2);
  bounds.setLow(0, 0);
  bounds.setLow(1, 0);
  bounds.setHigh(0, PlannerSettings::environment->width());
  bounds.setHigh(1, PlannerSettings::environment->height());

  // Construct the robot state space in which we're planning.
  if (PlannerSettings::steeringType == Steering::STEER_TYPE_REEDS_SHEPP)
    PlannerSettings::stateSpace = ob::StateSpacePtr(
        new ob::ReedsSheppStateSpace(PlannerSettings::CarTurningRadius));
  else if (PlannerSettings::steeringType == Steering::STEER_TYPE_POSQ)
    PlannerSettings::stateSpace = ob::StateSpacePtr(new POSQStateSpace());
  else if (PlannerSettings::steeringType == Steering::STEER_TYPE_DUBINS)
    PlannerSettings::stateSpace = ob::StateSpacePtr(
        new ob::DubinsStateSpace(PlannerSettings::CarTurningRadius));
  else if (PlannerSettings::steeringType == Steering::STEER_TYPE_LINEAR)
    PlannerSettings::stateSpace = ob::StateSpacePtr(new ob::SE2StateSpace);
#ifdef G1_AVAILABLE
  else if (PlannerSettings::steeringType == Steering::STEER_TYPE_CLOTHOID)
    PlannerSettings::stateSpace = ob::StateSpacePtr(new G1ClothoidStateSpace());
#else
  else if (PlannerSettings::steeringType == Steering::STEER_TYPE_CLOTHOID) {
    OMPL_ERROR("G1 Clothoid steering is not available in this release!");
    OMPL_ERROR(
            "Select a steering type other than STEER_TYPE_CLOTHOID in "
            "PlannerSettings.");
  }
#endif

  PlannerSettings::stateSpace->as<ob::SE2StateSpace>()->setBounds(bounds);

  spaceInfo = std::make_shared<ompl::base::SpaceInformation>(PlannerSettings::stateSpace);
  objective = ompl::base::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(spaceInfo));

  OMPL_INFORM("Initialized steer function");

//  if (steeringType == Steering::STEER_TYPE_REEDS_SHEPP)
//    PlannerSettings::steering =
//        new ReedsSheppSteering(PlannerSettings::CarTurningRadius);
//  else if (steeringType == Steering::STEER_TYPE_POSQ)
//    PlannerSettings::steering = new POSQSteering;
//  else if (steeringType == Steering::STEER_TYPE_LINEAR)
//    PlannerSettings::steering = new LinearSteering;
//  else if (steeringType == Steering::STEER_TYPE_DUBINS)
//    PlannerSettings::steering =
//        new DubinsSteering(PlannerSettings::CarTurningRadius);
//#ifdef G1_AVAILABLE
//  else if (steeringType == Steering::STEER_TYPE_CLOTHOID)
//    PlannerSettings::steering = new ClothoidSteering;
//#else
//  else if (steeringType == Steering::STEER_TYPE_CLOTHOID) {
//    OMPL_ERROR("G1 Clothoid steering is not available in this release!");
//    OMPL_ERROR(
//        "Select a steering type other than STEER_TYPE_CLOTHOID in "
//        "PlannerSettings.");
//  }
//#endif
}

bool PlannerSettings::estimateTheta = true;

// GRIPS settings
double PlannerSettings::gripsMinNodeDistance = 3;
double PlannerSettings::gripsEta = 0.5;
double PlannerSettings::gripsEtaDiscount = 0.8;
unsigned int PlannerSettings::gripsGradientDescentRounds = 5;
unsigned int PlannerSettings::gripsMaxPruningRounds = 100;

// SmoothStar settings
bool PlannerSettings::gradientDescentOpenNodes = true;
bool PlannerSettings::annealedGradientDescentOpenNodes = true;
bool PlannerSettings::gradientDescentCurrent = false;
bool PlannerSettings::gradientDescentSuccessors = false;
double PlannerSettings::gradientDescentEta = 0.5;
double PlannerSettings::gradientDescentEtaDiscount = 0.8;
unsigned int PlannerSettings::gradientDescentRounds = 10u;
bool PlannerSettings::averageAngles = true;

// SBPL settings
bool PlannerSettings::sbplSearchUntilFirstSolution = false;
double PlannerSettings::sbplInitialSolutionEps = 3.0;
double PlannerSettings::sbplFordwardVelocity = 0.4;
double PlannerSettings::sbplTimeToTurn45DegsInPlace = 0.6;
char *PlannerSettings::sbplMotionPrimitiveFilename =
    const_cast<char *>("./sbpl_mprim/unicycle_0.125.mprim");
// char *PlannerSettings::sbplMotionPrimitiveFilename = const_cast<char
// *>("./sbpl_mprim/pr2.mprim");
double PlannerSettings::sbplResolution = 0.125;
double PlannerSettings::sbplScaling = 5.;
double PlannerSettings::sbplGoalToleranceX = 1;
double PlannerSettings::sbplGoalToleranceY = 1;
double PlannerSettings::sbplGoalToleranceTheta = 2. * M_PI;
unsigned int PlannerSettings::sbplNumThetaDirs = 16u;

// CHOMP settings
unsigned int PlannerSettings::chompNodes = 127;
double PlannerSettings::chompAlpha = 0.05;
float PlannerSettings::chompEpsilon = 2.f;
double PlannerSettings::chompGamma = 0.8;
double PlannerSettings::chompErrorTolerance = 1e-6;
unsigned int PlannerSettings::chompMaxIterations = 1500;
chomp::ChompObjectiveType PlannerSettings::chompObjectiveType =
    chomp::MINIMIZE_VELOCITY;
chomp::ChompInitialization PlannerSettings::chompInitialization =
    chomp::THETA_STAR;

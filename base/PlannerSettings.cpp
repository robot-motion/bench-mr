#include "PlannerSettings.h"
#include "steer_functions/ReedsShepp/ReedsSheppSteering.h"
#include "steer_functions/POSQ/POSQSteering.h"
#include "steer_functions/Linear/LinearSteering.h"
#include "steer_functions/Dubins/DubinsSteering.h"
#ifdef G1_AVAILABLE
    #include "steer_functions/G1Clothoid/ClothoidSteering.hpp"
#endif

Environment *PlannerSettings::environment = nullptr;

int PlannerSettings::numberEdges = 10;

// steering function settings
Steering::SteeringType PlannerSettings::steeringType = Steering::STEER_TYPE_REEDS_SHEPP;
double PlannerSettings::CarTurningRadius = 4;
double PlannerSettings::LinearSteeringDelta = 3.0;

SteerFunction *PlannerSettings::steering = nullptr;

bool PlannerSettings::VisualizeSmoothing1 = true;
bool PlannerSettings::VisualizeSmoothing2 = true;
bool PlannerSettings::VisualizeSmoothing3 = true;
bool PlannerSettings::VisualizeSmoothing4 = true;

void PlannerSettings::initializeSteering()
{
    if (steeringType == Steering::STEER_TYPE_REEDS_SHEPP)
        PlannerSettings::steering = new ReedsSheppSteering(PlannerSettings::CarTurningRadius);
    else if (steeringType == Steering::STEER_TYPE_POSQ)
        PlannerSettings::steering = new POSQSteering;
    else if (steeringType == Steering::STEER_TYPE_LINEAR)
        PlannerSettings::steering = new LinearSteering;
    else if (steeringType == Steering::STEER_TYPE_DUBINS)
        PlannerSettings::steering = new DubinsSteering(PlannerSettings::CarTurningRadius);
#ifdef G1_AVAILABLE
    else if (steeringType == Steering::STEER_TYPE_CLOTHOID)
        PlannerSettings::steering = new ClothoidSteering;
#else
    else if (steeringType == Steering::STEER_TYPE_CLOTHOID)
    {
        OMPL_ERROR("G1 Clothoid steering is not available in this release!");
        OMPL_ERROR("Select a steering type other than STEER_TYPE_CLOTHOID in PlannerSettings.");
    }
#endif
}

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
//char *PlannerSettings::sbplMotionPrimitiveFilename = const_cast<char *>("/home/eric/catkin_ws_planning/src/smooth-planning/sbpl_mprim/unicycle_0.0625.mprim");
char *PlannerSettings::sbplMotionPrimitiveFilename = const_cast<char *>("/home/eric/catkin_ws_planning/src/smooth-planning/sbpl_mprim/pr2.mprim");
double PlannerSettings::sbplResolution = 0.025;
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
chomp::ChompObjectiveType PlannerSettings::chompObjectiveType = chomp::MINIMIZE_VELOCITY;
chomp::ChompInitialization PlannerSettings::chompInitialization = chomp::THETA_STAR;

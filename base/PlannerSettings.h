#pragma once

#include "Environment.h"
#include "steer_functions/steer_base.h"


typedef Steer_base SteerFunction;

//#define DEBUG
#define STATS

struct PlannerSettings
{
public:
    static Environment *environment;

    static int numberEdges;

    // steering function settings
    static Steering::SteeringType steeringType;
    static SteerFunction *steering;

    static void initializeSteering();

    static double CarTurningRadius;

    static constexpr double PlanningTime{3.0};

    static bool VisualizeSmoothing1;
    static bool VisualizeSmoothing2;
    static bool VisualizeSmoothing3;
    static bool VisualizeSmoothing4;

    // GRIPS settings

    /*
     * Minimum distance to be maintained between two consecutive nodes.
     */
    static double gripsMinNodeDistance;
    /*
     * Gradient descent rate.
     */
    static double gripsEta;
    /*
     * Discount factor for gradient descent rate.
     */
    static double gripsEtaDiscount;

    /**
     * Number of gradient descent rounds.
     */
    static unsigned int gripsGradientDescentRounds;
    /**
     * Maximum number of pruning rounds after which the algorithm
     * should terminate.
     */
    static unsigned int gripsMaxPruningRounds;

    static bool gradientDescentOpenNodes;
    static bool annealedGradientDescentOpenNodes;
    static bool gradientDescentCurrent;
    static bool gradientDescentSuccessors;
    static double gradientDescentEta;
    static double gradientDescentEtaDiscount;
    static unsigned int gradientDescentRounds;
    static bool averageAngles;


    // SBPL settings
    static bool sbplSearchUntilFirstSolution; // search until it finds a solution? (even if allotted time is over)
    static double sbplInitialSolutionEps; // >1, ignored by planners that don't have notion of eps, 1 means optimal search
    static double sbplFordwardVelocity; // in meters/sec
    static double sbplTimeToTurn45DegsInPlace; // in sec
    static char *sbplMotionPrimitiveFilename;
    // Tolerances are most likely ignored by SPBL at the moment
    static double sbplGoalToleranceX;
    static double sbplGoalToleranceY;
    static double sbplGoalToleranceTheta;
    static double sbplResolution; // XXX Important: resolution must match resolution in motion primitive definition file!!!
    static unsigned int sbplNumThetaDirs; // XXX Important: number of theta directions must match resolution in motion primitive definition file!!!
};

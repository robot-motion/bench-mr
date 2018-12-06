#pragma once

#define SBPL_PRINTF OMPL_WARN
#define DEBUG 1

#include <sbpl/headers.h>

#include "AbstractPlanner.hpp"


class SbplPlanner : public AbstractPlanner {
public:
    enum SbplType {
        SBPL_ARASTAR,
        SBPL_ADSTAR,
        SBPL_RSTAR,
        SBPL_ANASTAR
    };

    const static bool ForwardSearch = true;

    SbplPlanner(SbplType type);
    virtual ~SbplPlanner();

    ob::PlannerStatus run() override;

    std::vector<GNode> solutionTrajectory() const override;
    std::vector<Tpoint> solutionPath() const override;
    og::PathGeometric geometricPath() const override;

    bool hasReachedGoalExactly() const override;
    double planningTime() const override;

private:
    SBPLPlanner *_sbPlanner;
    EnvironmentNAVXYTHETALAT *_env;
    EnvironmentNAV2D _env2d;
    std::vector<GNode> _solution;
    double _planningTime;
    const unsigned char *_mapData;
};

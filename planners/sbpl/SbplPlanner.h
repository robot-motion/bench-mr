#pragma once

#define SBPL_PRINTF OMPL_WARN
#define DEBUG 1

#include <sbpl/headers.h>

#include "../AbstractPlanner.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class SbplPlanner : public AbstractPlanner {
 public:
  const static bool ForwardSearch = true;

  SbplPlanner();
  virtual ~SbplPlanner();

  std::string name() const override { return "SBPL"; }

  ob::PlannerStatus run() override;

  og::PathGeometric solution() const override;

  bool hasReachedGoalExactly() const override;
  double planningTime() const override;

 private:
  SBPLPlanner *_sbPlanner;
  EnvironmentNAVXYTHETALAT *_env;
  og::PathGeometric _solution;
  double _planningTime;
};

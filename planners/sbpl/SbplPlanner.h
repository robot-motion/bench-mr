#pragma once

#include <sbpl/headers.h>

#include "planners/AbstractPlanner.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

template <sbpl::Planner PlannerT>
class SbplPlanner : public AbstractPlanner {
 public:
  const static bool ForwardSearch = true;

  SbplPlanner();
  virtual ~SbplPlanner();

  std::string name() const override {
    switch (PlannerT) {
      case sbpl::SBPL_ADSTAR:
        return "SBPL_ADstar";
      case sbpl::SBPL_ARASTAR:
        return "SBPL_ARstar";
      case sbpl::SBPL_RSTAR:
        return "SBPL_Rstar";
      case sbpl::SBPL_ANASTAR:
        return "SBPL_ANAstar";
      case sbpl::SBPL_MHA:
        return "SBPL_MHA";
      case sbpl::SBPL_LAZY_ARA:
        return "SBPL_Lazy_ARA";
    }
    return "SBPL";
  }

  ob::PlannerStatus run() override;

  og::PathGeometric solution() const override;

  bool hasReachedGoalExactly() const override;
  double planningTime() const override;

 private:
  SBPLPlanner *_sbPlanner{nullptr};
  EnvironmentNAVXYTHETALAT *_env{nullptr};
  og::PathGeometric _solution;
  double _planningTime{0};
  EmbeddedHeuristic *_heuristic{nullptr};
};

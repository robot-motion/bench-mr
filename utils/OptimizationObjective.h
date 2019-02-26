#pragma once

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/samplers/InformedStateSampler.h>
#include <ompl/base/samplers/informed/PathLengthDirectInfSampler.h>

namespace ob = ompl::base;

class CustomPathLengthDirectInfSampler : public ob::InformedSampler {
 public:
  CustomPathLengthDirectInfSampler(const ob::ProblemDefinitionPtr &probDefn,
                                   unsigned int maxNumberCalls);

  ~CustomPathLengthDirectInfSampler() override;

  bool sampleUniform(ob::State *statePtr, const ob::Cost &maxCost) override;

  bool sampleUniform(ob::State *statePtr, const ob::Cost &minCost,
                     const ob::Cost &maxCost) override;

  bool hasInformedMeasure() const override;

  double getInformedMeasure(const ob::Cost &currentCost) const override;

  ob::Cost heuristicSolnCost(const ob::State *statePtr) const override;

 private:
  typedef std::shared_ptr<const ompl::ProlateHyperspheroid>
      ProlateHyperspheroidCPtr;

  // Helper functions:
  // High level
  bool sampleUniform(ob::State *statePtr, const ob::Cost &maxCost,
                     unsigned int *iters);

  bool sampleBoundsRejectPhs(ob::State *statePtr, unsigned int *iters);

  bool samplePhsRejectBounds(ob::State *statePtr, unsigned int *iters);

  // Low level
  std::vector<double> getInformedSubstate(const ob::State *statePtr) const;

  void createFullState(ob::State *statePtr,
                       const std::vector<double> &informedVector);

  void updatePhsDefinitions(const ob::Cost &maxCost);

  ompl::ProlateHyperspheroidPtr randomPhsPtr();

  bool keepSample(const std::vector<double> &informedVector);

  bool isInAnyPhs(const std::vector<double> &informedVector) const;

  bool isInPhs(const ProlateHyperspheroidCPtr &phsCPtr,
               const std::vector<double> &informedVector) const;

  unsigned int numberOfPhsInclusions(
      const std::vector<double> &informedVector) const;

  // Variables
  std::list<ompl::ProlateHyperspheroidPtr> listPhsPtrs_;

  double summedMeasure_;

  unsigned int informedIdx_;

  ob::StateSpacePtr informedSubSpace_;

  unsigned int uninformedIdx_;

  ob::StateSpacePtr uninformedSubSpace_;

  ob::StateSamplerPtr baseSampler_;

  ob::StateSamplerPtr uninformedSubSampler_;

  ompl::RNG rng_;
};

class OptimizationObjective : public ob::PathLengthOptimizationObjective {
 public:
  OptimizationObjective(ob::SpaceInformationPtr &space_info)
      : ob::PathLengthOptimizationObjective(space_info) {}
  ob::InformedSamplerPtr allocInformedStateSampler(
      const ob::ProblemDefinitionPtr &probDefn,
      unsigned int maxNumberCalls) const override {
    OMPL_WARN("Using a custom informed state sampler.");
    return ob::InformedSamplerPtr(
        new CustomPathLengthDirectInfSampler(probDefn, maxNumberCalls));
  }
};

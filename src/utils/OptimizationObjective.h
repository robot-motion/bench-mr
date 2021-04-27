#pragma once

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/samplers/InformedStateSampler.h>
#include <ompl/base/samplers/informed/PathLengthDirectInfSampler.h>

#include <limits>

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

class CustomPathLengthOptimizationObjective
    : public ob::PathLengthOptimizationObjective {
 public:
  CustomPathLengthOptimizationObjective(ob::SpaceInformationPtr &space_info)
      : ob::PathLengthOptimizationObjective(space_info) {}
  ob::InformedSamplerPtr allocInformedStateSampler(
      const ob::ProblemDefinitionPtr &probDefn,
      unsigned int maxNumberCalls) const override {
    OMPL_DEBUG(
        "Using a custom informed state sampler to support CC/HC steer "
        "functions.");
    return ob::InformedSamplerPtr(
        new CustomPathLengthDirectInfSampler(probDefn, maxNumberCalls));
  }
};

/** \brief An optimization objective for minimizing OMPL's smoothness.
 *
 * Note, that this is only supported for geometric motion planning. Since OMPL
 * does not implement smoothness for PathControl.
 */
class SmoothnessOptimizationObjective : public ob::OptimizationObjective {
 public:
  SmoothnessOptimizationObjective(ob::SpaceInformationPtr &space_info)
      : ob::OptimizationObjective(space_info) {}

  /** \brief Returns identity cost. */
  ob::Cost stateCost(const ob::State *s) const override;

  /** \brief Motion cost for this objective is defined as
      the smoothness between the path between \p s1 and \p s2 . */
  ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const override;
};

/** \brief An optimization objective for minimizing curvature of a path.
 *
 * This optimization objective sums up the normalized curvature (see below)
 * along the path. Hence, there is some implicit optimization of path length as
 * well (i.e., there is no point in going a very long straight path).
 *
 * The curvature is normalized by multiplying by the length of the segments such
 * that this metric does not increase with higher path resolution.
 *
 * The curvature can be capped at a specified value to deal with infinite
 * curvature points in the path (i.e., cusps).
 */
class CurvatureOptimizationObjective : public ob::OptimizationObjective {
 public:
  CurvatureOptimizationObjective(
      ob::SpaceInformationPtr &space_info,
      double max_curvature = std::numeric_limits<double>::max())
      : ob::OptimizationObjective(space_info), max_curvature_(max_curvature) {}

  /** \brief Returns identity cost. */
  ob::Cost stateCost(const ob::State *s) const override;

  /** \brief Motion cost for this objective is defined as the sum of normalied
   * curvature along the path. */
  ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const override;

 protected:
  double max_curvature_;
};
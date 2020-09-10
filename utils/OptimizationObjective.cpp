#include "OptimizationObjective.h"

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/util/ProlateHyperspheroid.h>

#include <cmath>

namespace og = ompl::geometric;

CustomPathLengthDirectInfSampler::CustomPathLengthDirectInfSampler(
    const ob::ProblemDefinitionPtr &probDefn, unsigned int maxNumberCalls)
    : ob::InformedSampler(probDefn, maxNumberCalls),
      informedIdx_(0u),
      uninformedIdx_(0u) {
  // Variables
  // The number of start states
  unsigned int numStarts;
  // The number of goal states
  unsigned numGoals;
  // The foci of the PHSs as a std::vector of states. Goals must be nonconst, as
  // we need to allocate them (unfortunately):
  std::vector<const ob::State *> startStates;
  std::vector<ob::State *> goalStates;

  if (!probDefn_->getGoal()->hasType(ompl::base::GOAL_SAMPLEABLE_REGION)) {
    throw ompl::Exception(
        "CustomPathLengthDirectInfSampler: The direct path-length informed "
        "sampler "
        "currently only "
        "supports goals that can be cast to a sampleable goal region (i.e., "
        "are countable "
        "sets).");
  }

  // Store the number of starts and goals
  numStarts = probDefn_->getStartStateCount();
  numGoals = probDefn_->getGoal()
                 ->as<ompl::base::GoalSampleableRegion>()
                 ->maxSampleCount();

  // Sanity check that there is atleast one of each
  if (numStarts < 1u || numGoals < 1u) {
    throw ompl::Exception(
        "CustomPathLengthDirectInfSampler: There must be at least 1 start and "
        "and 1 "
        "goal state when "
        "the informed sampler is created.");
  }

  // Check that the provided statespace is compatible and extract the necessary
  // indices. The statespace must either be R^n or SE(2) or SE(3). If it is
  // UNKNOWN, warn and treat it as R^n
  //  if (!InformedSampler::space_->isCompound()) {
  //    if (InformedSampler::space_->getType() == ob::STATE_SPACE_REAL_VECTOR) {
  //      // R^n, this is easy
  //      informedIdx_ = 0u;
  //      uninformedIdx_ = 0u;
  //    } else if (InformedSampler::space_->getType() ==
  //    ob::STATE_SPACE_UNKNOWN) {
  //      // Unknown, this is annoying. I hope the user knows what they're doing
  //      OMPL_WARN(
  //          "CustomPathLengthDirectInfSampler: Treating the StateSpace of type
  //          "
  //          "\"ob::STATE_SPACE_UNKNOWN\" as type "
  //          "\"ob::STATE_SPACE_REAL_VECTOR\".");
  informedIdx_ = 0u;
  uninformedIdx_ = 0u;
  //    } else {
  //      throw ompl::Exception(
  //          "CustomPathLengthDirectInfSampler only supports Unknown,
  //          RealVector, " "SE2, " "and SE3 StateSpaces.");
  //    }
  //  } else if (InformedSampler::space_->isCompound()) {
  //    // Check that it is SE2 or SE3
  //    if (InformedSampler::space_->getType() == ob::STATE_SPACE_SE2 ||
  //        InformedSampler::space_->getType() == ob::STATE_SPACE_SE3) {
  //      // Variable:
  //      // An ease of use upcasted pointer to the space as a compound space
  //      const ob::CompoundStateSpace *compoundSpace =
  //          InformedSampler::space_->as<ob::CompoundStateSpace>();
  //
  //      // Sanity check
  //      //      if (compoundSpace->getSubspaceCount() != 2u) {
  //      //        // Pout
  //      //        throw ompl::Exception(
  //      //            "The provided compound StateSpace is SE(2) or SE(3) but
  //      does
  //      //            not " "have exactly " "2 subspaces.");
  //      //      }
  //
  //      // Iterate over the state spaces, finding the real vector and SO
  //      // components.
  //      // XXX we fixed the number of subspaces to consider to 2 here
  //      for (unsigned int idx = 0u; idx < 2u; ++idx) {
  //        OMPL_DEBUG("Subspace %i is of type %i.", idx,
  //                   compoundSpace->getSubspace(idx)->getType());
  //        // Check if the space is real-vectored, SO2 or SO3
  //        if (compoundSpace->getSubspace(idx)->getType() ==
  //            ob::STATE_SPACE_REAL_VECTOR) {
  //          informedIdx_ = idx;
  //        } else if (compoundSpace->getSubspace(idx)->getType() ==
  //                   ob::STATE_SPACE_SO2) {
  //          uninformedIdx_ = idx;
  //        } else if (compoundSpace->getSubspace(idx)->getType() ==
  //                   ob::STATE_SPACE_SO3) {
  //          uninformedIdx_ = idx;
  //        } else {
  //          // Pout
  //          throw ompl::Exception(
  //              "The provided compound StateSpace is SE(2) or SE(3) but
  //              contains " "a " "subspace that is not R^2, R^3, SO(2), or
  //              SO(3).");
  //        }
  //      }
  //    } else {
  //      throw ompl::Exception(
  //          "CustomPathLengthDirectInfSampler only supports RealVector, SE2
  //          and " "SE3 " "statespaces.");
  //    }
  //  }

  // Create a sampler for the whole space that we can use if we have no
  // information
  baseSampler_ = InformedSampler::space_->allocDefaultStateSampler();

  // Check if the space is compound
  if (!InformedSampler::space_->isCompound()) {
    // It is not.

    // The informed subspace is the full space
    informedSubSpace_ = InformedSampler::space_;

    // And the uniformed subspace and its associated sampler are null
    uninformedSubSpace_ = ob::StateSpacePtr();
    uninformedSubSampler_ = ob::StateSamplerPtr();
  } else {
    // It is

    // Get a pointer to the informed subspace...
    informedSubSpace_ =
        InformedSampler::space_->as<ob::CompoundStateSpace>()->getSubspace(
            informedIdx_);

    // And the uninformed subspace is the remainder.
    uninformedSubSpace_ =
        InformedSampler::space_->as<ob::CompoundStateSpace>()->getSubspace(
            uninformedIdx_);

    // Create a sampler for the uniformed subset:
    uninformedSubSampler_ = uninformedSubSpace_->allocDefaultStateSampler();
  }

  // Store the foci, first the starts:
  for (unsigned int i = 0u; i < numStarts; ++i) {
    startStates.push_back(probDefn_->getStartState(i));
  }

  // Extract the state of each goal one and place into the goal vector!
  for (unsigned int i = 0u; i < numGoals; ++i) {
    // Allocate a state onto the back of the vector:
    goalStates.push_back(InformedSampler::space_->allocState());

    // Now sample a goal into that state:
    probDefn_->getGoal()->as<ompl::base::GoalSampleableRegion>()->sampleGoal(
        goalStates.back());
  }

  // Now, iterate create a PHS for each start-goal pair
  // Each start
  for (unsigned int i = 0u; i < numStarts; ++i) {
    // Variable
    // The start as a vector
    std::vector<double> startFocusVector =
        getInformedSubstate(startStates.at(i));

    // Each goal
    for (unsigned int j = 0u; j < numGoals; ++j) {
      // Variable
      // The goal as a vector
      std::vector<double> goalFocusVector =
          getInformedSubstate(goalStates.at(j));

      // Create the definition of the PHS
      listPhsPtrs_.push_back(std::make_shared<ompl::ProlateHyperspheroid>(
          informedSubSpace_->getDimension(), &startFocusVector[0],
          &goalFocusVector[0]));
    }
  }

  // Finally deallocate the states in the goal state vector:
  for (unsigned int i = 0u; i < numGoals; ++i) {
    // Free the state in the vector:
    InformedSampler::space_->freeState(goalStates.at(i));
  }

  if (listPhsPtrs_.size() > 100u) {
    OMPL_WARN(
        "CustomPathLengthDirectInfSampler: Rejection sampling is used in order "
        "to "
        "maintain uniform density "
        "in the presence of overlapping informed subsets. At some number of "
        "independent subsets, "
        "this will become prohibitively expensive. Current number of "
        "independent subsets: %d",
        listPhsPtrs_.size());
  }
}

CustomPathLengthDirectInfSampler::~CustomPathLengthDirectInfSampler() = default;

bool CustomPathLengthDirectInfSampler::sampleUniform(ob::State *statePtr,
                                                     const ob::Cost &maxCost) {
  // Variable
  // The persistent iteration counter:
  unsigned int iter = 0u;

  // Call the sampleUniform helper function with my iteration counter:
  return sampleUniform(statePtr, maxCost, &iter);
}

bool CustomPathLengthDirectInfSampler::sampleUniform(ob::State *statePtr,
                                                     const ob::Cost &minCost,
                                                     const ob::Cost &maxCost) {
  // Sample from the larger PHS until the sample does not lie within the smaller
  // PHS. Since volume in a sphere/spheroid is proportionately concentrated near
  // the surface, this isn't horribly inefficient, though a direct method would
  // be better

  // Variable
  // Whether we were successful in creating an informed sample. Initially not:
  bool foundSample = false;

  // Spend numIters_ iterations trying to find an informed sample:
  for (unsigned int i = 0u; i < InformedSampler::numIters_ && !foundSample;
       ++i) {
    // Call the helper function for the larger PHS. It will move our iteration
    // counter:
    foundSample = sampleUniform(statePtr, maxCost, &i);

    // Did we find a sample?
    if (foundSample) {
      // We did, but it only satisfied the upper bound. Check that it meets the
      // lower bound.

      // Variables
      // The cost of the sample we found
      ob::Cost sampledCost = heuristicSolnCost(statePtr);

      // Check if the sample's cost is greater than or equal to the lower bound
      foundSample =
          InformedSampler::opt_->isCostEquivalentTo(minCost, sampledCost) ||
          InformedSampler::opt_->isCostBetterThan(minCost, sampledCost);
    }
    // No else, no sample was found.
  }

  // All done, one way or the other.
  return foundSample;
}

bool CustomPathLengthDirectInfSampler::hasInformedMeasure() const {
  return true;
}

double CustomPathLengthDirectInfSampler::getInformedMeasure(
    const ob::Cost &currentCost) const {
  // Variable
  // The measure of the informed set
  double informedMeasure = 0.0;

  // The informed measure is then the sum of the measure of the individual PHSs
  // for the given cost:
  for (const auto &phsPtr : listPhsPtrs_) {
    // It is nonsensical for a PHS to have a transverse diameter less than the
    // distance between its foci, so skip those that do
    if (currentCost.value() > phsPtr->getMinTransverseDiameter()) {
      informedMeasure =
          informedMeasure + phsPtr->getPhsMeasure(currentCost.value());
    }
    // No else, this value is better than this ellipse. It will get removed
    // later.
  }

  // And if the space is compound, further multiplied by the measure of the
  // uniformed subspace
  if (InformedSampler::space_->isCompound()) {
    informedMeasure = informedMeasure * uninformedSubSpace_->getMeasure();
  }

  // Return the smaller of the two measures
  return std::min(InformedSampler::space_->getMeasure(), informedMeasure);
}

ob::Cost CustomPathLengthDirectInfSampler::heuristicSolnCost(
    const ob::State *statePtr) const {
  // Variable
  // The raw data in the state
  std::vector<double> rawData = getInformedSubstate(statePtr);
  // The Cost, infinity to start
  ob::Cost minCost = InformedSampler::opt_->infiniteCost();

  // Iterate over the separate subsets and return the minimum
  for (const auto &phsPtr : listPhsPtrs_) {
    minCost = InformedSampler::opt_->betterCost(
        minCost, ob::Cost(phsPtr->getPathLength(&rawData[0])));
  }

  return minCost;
}

// Private functions:
bool CustomPathLengthDirectInfSampler::sampleUniform(ob::State *statePtr,
                                                     const ob::Cost &maxCost,
                                                     unsigned int *iters) {
  // Variable
  // Whether we were successful in creating an informed sample. Initially not:
  bool foundSample = false;

  // Whether we successfully returnes
  // Check if a solution path has been found
  if (!InformedSampler::opt_->isFinite(maxCost)) {
    // We don't have a solution yet, we sample from our basic sampler instead...
    baseSampler_->sampleUniform(statePtr);

    // Up our counter by one:
    ++(*iters);

    // Mark that we sampled:
    foundSample = true;
  } else  // We have a solution
  {
    // Update the definitions of the PHSs
    updatePhsDefinitions(maxCost);

    // Sample from the PHSs.

    // When the summed measure of the PHSes are suitably large, it makes more
    // sense to just sample from the entire planning space and keep the sample
    // if it lies in any PHS Check if the average measure is greater than half
    // the domain's measure. Half is an arbitrary number.
    if (informedSubSpace_->getMeasure() <
        summedMeasure_ / static_cast<double>(listPhsPtrs_.size())) {
      // The measure is large, sample from the entire world and keep if it's in
      // any PHS
      foundSample = sampleBoundsRejectPhs(statePtr, iters);
    } else {
      // The measure is sufficiently small that we will directly sample the
      // PHSes, with the weighting given by their relative measures
      foundSample = samplePhsRejectBounds(statePtr, iters);
    }
  }

  // Return:
  return foundSample;
}

bool CustomPathLengthDirectInfSampler::sampleBoundsRejectPhs(
    ob::State *statePtr, unsigned int *iters) {
  // Variable
  // Whether we've found a sample:
  bool foundSample = false;

  // Spend numIters_ iterations trying to find an informed sample:
  while (!foundSample && *iters < InformedSampler::numIters_) {
    // Generate a random sample
    baseSampler_->sampleUniform(statePtr);

    // The informed substate
    std::vector<double> informedVector = getInformedSubstate(statePtr);

    // Check if the informed state is in any PHS.
    foundSample = isInAnyPhs(informedVector);

    // Increment the provided counter
    ++(*iters);
  }

  // successful?
  return foundSample;
}

bool CustomPathLengthDirectInfSampler::samplePhsRejectBounds(
    ob::State *statePtr, unsigned int *iters) {
  // Variable
  // Whether we were successful in creating an informed sample. Initially not:
  bool foundSample = false;

  // Due to the possibility of overlap between multiple PHSs, we keep a sample
  // with a probability of 1/K, where K is the number of PHSs the sample is in.
  while (!foundSample && *iters < InformedSampler::numIters_) {
    // Variables
    // The informed subset of the sample as a vector
    std::vector<double> informedVector(informedSubSpace_->getDimension());
    // The random PHS in use for this sample.
    ProlateHyperspheroidCPtr phsCPtr = randomPhsPtr();

    // Use the PHS to get a sample in the informed subspace irrespective of
    // boundary
    rng_.uniformProlateHyperspheroid(phsCPtr, &informedVector[0]);

    // Keep with probability 1/K
    foundSample = keepSample(informedVector);

    // If we're keeping it, then check if the state is in the problem domain:
    if (foundSample) {
      // Turn into a state of our full space
      createFullState(statePtr, informedVector);

      // Return if the resulting state is in the problem:
      foundSample = InformedSampler::space_->satisfiesBounds(statePtr);
    }
    // No else
  }

  // Successful?
  return foundSample;
}

std::vector<double> CustomPathLengthDirectInfSampler::getInformedSubstate(
    const ob::State *statePtr) const {
  // Variable
  // The raw data in the state
  std::vector<double> rawData(informedSubSpace_->getDimension());

  // Get the raw data
  if (!InformedSampler::space_->isCompound()) {
    informedSubSpace_->copyToReals(rawData, statePtr);
  } else {
    informedSubSpace_->copyToReals(
        rawData, statePtr->as<ob::CompoundState>()->components[informedIdx_]);
  }

  return rawData;
}

void CustomPathLengthDirectInfSampler::createFullState(
    ob::State *statePtr, const std::vector<double> &informedVector) {
  // If there is an extra "uninformed" subspace, we need to add that to the
  // state before converting the raw vector representation into a state....
  if (!InformedSampler::space_->isCompound()) {
    // No, space_ == informedSubSpace_
    // Copy into the state pointer
    informedSubSpace_->copyFromReals(statePtr, informedVector);
  } else {
    // Yes, we need to also sample the uninformed subspace
    // Variables
    // A state for the uninformed subspace
    ob::State *uninformedState = uninformedSubSpace_->allocState();

    // Copy the informed subspace into the state pointer
    informedSubSpace_->copyFromReals(
        statePtr->as<ob::CompoundState>()->components[informedIdx_],
        informedVector);

    // Sample the uniformed subspace
    uninformedSubSampler_->sampleUniform(uninformedState);

    // Copy the informed subspace into the state pointer
    uninformedSubSpace_->copyState(
        statePtr->as<ob::CompoundState>()->components[uninformedIdx_],
        uninformedState);

    // Free the state
    uninformedSubSpace_->freeState(uninformedState);
  }
}

void CustomPathLengthDirectInfSampler::updatePhsDefinitions(
    const ob::Cost &maxCost) {
  // Variable
  // The iterator for the list:
  auto phsIter = listPhsPtrs_.begin();

  // Iterate over the list of PHSs, updating the summed measure
  // Reset the sum
  summedMeasure_ = 0.0;
  while (phsIter != listPhsPtrs_.end()) {
    // Check if the specific PHS can ever be better than the given maxCost,
    // i.e., if the distance between the foci is less than the current max cost
    if ((*phsIter)->getMinTransverseDiameter() < maxCost.value()) {
      // It can improve the solution, or it's the only PHS we have, update it

      // Update the transverse diameter
      (*phsIter)->setTransverseDiameter(maxCost.value());

      // Increment the summed measure of the ellipses.
      summedMeasure_ = summedMeasure_ + (*phsIter)->getPhsMeasure();

      // Increment the iterator
      ++phsIter;
    } else if (listPhsPtrs_.size() > 1u) {
      // It can't, and it is not the last PHS, remove it

      // Remove the iterator to delete from the list, this returns the next:
      phsIter = listPhsPtrs_.erase(phsIter);
    } else {
      // It can't, but it's the last PHS, so we can't remove it.

      // Make sure it's transverse diameter is set to something:
      (*phsIter)->setTransverseDiameter((*phsIter)->getMinTransverseDiameter());

      // Set the summed measure to 0.0 (as a degenerate PHS is a line):
      summedMeasure_ = 0.0;

      // Increment the iterator so we move past this to the end.
      ++phsIter;
    }
  }
}

ompl::ProlateHyperspheroidPtr CustomPathLengthDirectInfSampler::randomPhsPtr() {
  // Variable
  // The return value
  ompl::ProlateHyperspheroidPtr rval;

  // If we only have one PHS, this can be simplified:
  if (listPhsPtrs_.size() == 1u) {
    // One PHS, keep this simple.

    // Return it
    rval = listPhsPtrs_.front();
  } else {
    // We have more than one PHS to consider

    // Variables
    // A randomly generated number in the interval [0,1]
    double randDbl = rng_.uniform01();
    // The running measure
    double runningRelativeMeasure = 0.0;

    // The probability of using each PHS is weighted by it's measure. Therefore,
    // if we iterate up the list of PHSs, the first one who's relative measure
    // is greater than the PHS randomly selected
    for (std::list<ompl::ProlateHyperspheroidPtr>::const_iterator phsIter =
             listPhsPtrs_.begin();
         phsIter != listPhsPtrs_.end() && !static_cast<bool>(rval); ++phsIter) {
      // Update the running measure
      runningRelativeMeasure =
          runningRelativeMeasure + (*phsIter)->getPhsMeasure() / summedMeasure_;

      // Check if it's now greater than the proportion of the summed measure
      if (runningRelativeMeasure > randDbl) {
        // It is, return this PHS:
        rval = *phsIter;
      }
      // No else, continue
    }
  }

  // Return
  return rval;
}

bool CustomPathLengthDirectInfSampler::keepSample(
    const std::vector<double> &informedVector) {
  // Variable
  // The return value, do we keep this sample? Start true.
  bool keep = true;

  // Is there more than 1 goal?
  if (listPhsPtrs_.size() > 1u) {
    // There is, do work

    // Variable
    // The number of PHSs the sample is in
    unsigned int numIn = numberOfPhsInclusions(informedVector);
    // The random number between [0,1]
    double randDbl = rng_.uniform01();

    // Keep the sample if the random number is less than 1/K
    keep = (randDbl <= 1.0 / static_cast<double>(numIn));
  }
  // No else, keep is true by default.

  return keep;
}

bool CustomPathLengthDirectInfSampler::isInAnyPhs(
    const std::vector<double> &informedVector) const {
  // Variable
  // The return value, whether the given state is in any PHS
  bool inPhs = false;

  // Iterate over the list, stopping as soon as we get our first true
  for (auto phsIter = listPhsPtrs_.begin();
       phsIter != listPhsPtrs_.end() && !inPhs; ++phsIter) {
    inPhs = isInPhs(*phsIter, informedVector);
  }

  return inPhs;
}

bool CustomPathLengthDirectInfSampler::isInPhs(
    const ProlateHyperspheroidCPtr &phsCPtr,
    const std::vector<double> &informedVector) const {
  return phsCPtr->isInPhs(&informedVector[0]);
}

unsigned int CustomPathLengthDirectInfSampler::numberOfPhsInclusions(
    const std::vector<double> &informedVector) const {
  // Variable
  // The return value, the number of PHSs the vector is in
  unsigned int numInclusions = 0u;

  // Iterate over the list counting
  for (const auto &phsPtr : listPhsPtrs_) {
    // Conditionally increment
    if (phsPtr->isInPhs(&informedVector[0])) {
      ++numInclusions;
    }
    // No else
  }

  return numInclusions;
}

ob::Cost SmoothnessOptimizationObjective::stateCost(const ob::State *s) const {
  return identityCost();
}

ob::Cost SmoothnessOptimizationObjective::motionCost(
    const ob::State *s1, const ob::State *s2) const {
  auto segment = og::PathGeometric(si_, s1, s2);
  // need to interpolate as smoothness is not defined for paths made up of only
  // two states
  segment.interpolate();
  return ob::Cost(segment.smoothness());
}

ob::Cost CurvatureOptimizationObjective::stateCost(const ob::State *s) const {
  return identityCost();
}

ob::Cost CurvatureOptimizationObjective::motionCost(const ob::State *s1,
                                                    const ob::State *s2) const {
  auto segment = og::PathGeometric(si_, s1, s2);
  segment.interpolate();
  double curvature_sum = 0;
  const auto &states = segment.getStates();
  for (int i = 1; i < states.size() - 1; i += 2) {
    auto state_0 = states[i - 1]->as<ob::SE2StateSpace::StateType>();
    auto state_1 = states[i]->as<ob::SE2StateSpace::StateType>();
    auto state_2 = states[i + 1]->as<ob::SE2StateSpace::StateType>();
    double x_0 = state_0->getX(), y_0 = state_0->getY();
    double x_1 = state_1->getX(), y_1 = state_1->getY();
    double x_2 = state_2->getX(), y_2 = state_2->getY();

    double length =
        (std::sqrt(std::pow(x_0 - x_1, 2) + std::pow(y_0 - y_1, 2)) +
         std::sqrt(std::pow(x_1 - x_2, 2) + std::pow(y_1 - y_2, 2)));

    // if two points in a row repeat, we skip curvature computation
    if (x_0 == x_1 && y_0 == y_1 || x_1 == x_2 && y_1 == y_2) continue;

    // Infinite curvature in case the path goes a step backwards
    if (x_0 == x_2 && y_0 == y_2) {
      curvature_sum += max_curvature_ * length;
      continue;
    }

    // 0 curvature in case the path just goes forward
    if (x_2 * (-y_0 + y_1) + x_1 * (y_0 - y_2) + x_0 * (-y_1 + y_2) == 0) {
      continue;
    }

    // Compute center of circle that goes through the 3 points
    double cx =
        (std::pow(x_2, 2) * (-y_0 + y_1) + std::pow(x_1, 2) * (y_0 - y_2) -
         (std::pow(x_0, 2) + (y_0 - y_1) * (y_0 - y_2)) * (y_1 - y_2)) /
        (2 * (x_2 * (-y_0 + y_1) + x_1 * (y_0 - y_2) + x_0 * (-y_1 + y_2)));
    double cy =
        (-(std::pow(x_1, 2) * x_2) + std::pow(x_0, 2) * (-x_1 + x_2) +
         x_2 * (std::pow(y_0, 2) - std::pow(y_1, 2)) +
         x_0 * (std::pow(x_1, 2) - std::pow(x_2, 2) + std::pow(y_1, 2) -
                std::pow(y_2, 2)) +
         x_1 * (std::pow(x_2, 2) - std::pow(y_0, 2) + std::pow(y_2, 2))) /
        (2 * (x_2 * (y_0 - y_1) + x_0 * (y_1 - y_2) + x_1 * (-y_0 + y_2)));

    // Curvature = 1/Radius
    double radius = std::sqrt(std::pow(x_0 - cx, 2) + std::pow(y_0 - cy, 2));

    double ki = std::min(1. / radius, max_curvature_);

    curvature_sum += ki * length;
  }
  return ob::Cost(curvature_sum);
}
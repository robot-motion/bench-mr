#ifndef _FORWARD_PROPAGATION_
#define _FORWARD_PROPAGATION_

#pragma once
#include <string>

namespace ForwardPropagation {
enum ForwardPropagationType {
  FORWARD_PROPAGATION_TYPE_KINEMATIC_CAR = 0,
  FORWARD_PROPAGATION_TYPE_KINEMATIC_SINGLE_TRACK = 1
};

inline std::string to_string(ForwardPropagation::ForwardPropagationType t) {
  switch (t) {
    case FORWARD_PROPAGATION_TYPE_KINEMATIC_CAR:
      return "KinematicCar";
    case FORWARD_PROPAGATION_TYPE_KINEMATIC_SINGLE_TRACK:
      return "KinematicSingleTruck";
    default:
      return "Unknown";
  }
}

// Define how we project a state
class KinematicSingleTrackProjectionEvaluator : public ob::ProjectionEvaluator {
 public:
  KinematicSingleTrackProjectionEvaluator(const ob::StateSpace* space)
      : ob::ProjectionEvaluator(space) {}

  // returning only Cartesian positions
  unsigned int getDimension() const override { return 2; }

  void project(const ob::State* state,
               Eigen::Ref<Eigen::VectorXd> projection) const override {
    const auto* compState = state->as<ob::CompoundStateSpace::StateType>();
    const auto* se2state = compState->as<ob::SE2StateSpace::StateType>(0);
    projection[0] = se2state->getX();
    projection[1] = se2state->getY();
  }
};

}  // namespace ForwardPropagation
#endif /* _FORWARD_PROPAGATION_ */
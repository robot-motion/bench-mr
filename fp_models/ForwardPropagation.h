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

}  // namespace ForwardPropagation
#endif /* _FORWARD_PROPAGATION_ */
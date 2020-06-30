#ifndef _FORWARD_PROPAGATION_
#define _FORWARD_PROPAGATION_

#pragma once
#include <string>

namespace ForwardPropagation {
enum ForwardPropagationType { FORWARD_PROPAGATION_TYPE_KINEMATIC_CAR = 0 };

inline std::string to_string(ForwardPropagation::ForwardPropagationType t) {
  switch (t) {
    case FORWARD_PROPAGATION_TYPE_KINEMATIC_CAR:
      return "KinematicCar";
    default:
      return "Unknown";
  }
}

}  // namespace ForwardPropagation
#endif /* _FORWARD_PROPAGATION_ */
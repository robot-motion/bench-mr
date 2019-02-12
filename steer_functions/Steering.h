#pragma once
#include <string>

namespace Steering {
enum SteeringType {
  STEER_TYPE_REEDS_SHEPP,
  STEER_TYPE_DUBINS,
  STEER_TYPE_POSQ,
  STEER_TYPE_CLOTHOID,
  STEER_TYPE_LINEAR,
  STEER_TYPE_CC_DUBINS,
  STEER_TYPE_HC_REEDS_SHEPP,
  STEER_TYPE_CC_REEDS_SHEPP
};

inline std::string to_string(Steering::SteeringType t) {
  switch (t) {
    case STEER_TYPE_REEDS_SHEPP:
      return "Reeds-Shepp";
    case STEER_TYPE_DUBINS:
      return "Dubins";
    case STEER_TYPE_LINEAR:
      return "Linear";
    case STEER_TYPE_POSQ:
      return "POSQ";
    case STEER_TYPE_CLOTHOID:
      return "Clothoid";
    case STEER_TYPE_CC_DUBINS:
      return "CC Dubins";
    case STEER_TYPE_HC_REEDS_SHEPP:
      return "HC Reeds-Shepp";
    case STEER_TYPE_CC_REEDS_SHEPP:
      return "CC Reeds-Shepp";
  }
}
}  // namespace Steering

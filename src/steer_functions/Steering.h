#pragma once
#include <nlohmann/json.hpp>
#include <string>

namespace Steering {
enum SteeringType {
  STEER_TYPE_REEDS_SHEPP = 0,
  STEER_TYPE_DUBINS = 1,
  STEER_TYPE_POSQ = 2,
  STEER_TYPE_CLOTHOID = 3,
  STEER_TYPE_LINEAR = 4,
  STEER_TYPE_CC_DUBINS = 5,
  STEER_TYPE_HC_REEDS_SHEPP = 6,
  STEER_TYPE_CC_REEDS_SHEPP = 7
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
    default:
      return "Unknown";
  }
}

// clang-format off
//NLOHMANN_JSON_SERIALIZE_ENUM(SteeringType, {
//  {STEER_TYPE_REEDS_SHEPP, "Reeds-Shepp"},
//  {STEER_TYPE_DUBINS, "Dubins"},
//  {STEER_TYPE_LINEAR, "Linear"},
//  {STEER_TYPE_POSQ, "POSQ"},
//  {STEER_TYPE_CLOTHOID, "Clothoid"},
//  {STEER_TYPE_CC_DUBINS, "CC Dubins"},
//  {STEER_TYPE_HC_REEDS_SHEPP, "HC Reeds-Shepp"},
//  {STEER_TYPE_CC_REEDS_SHEPP, "CC Reeds-Shepp"}
//})
// clang-format on
}  // namespace Steering

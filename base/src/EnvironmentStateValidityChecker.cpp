#include "base/EnvironmentStateValidityChecker.h"

// TODO: set specs_: clearance

bool EnvironmentStateValidityChecker::isValid(const ob::State *state) const {
  return env_->checkValidity(state);
}

double EnvironmentStateValidityChecker::clearance(
    const ob::State *state) const {
  return env_->bilinearDistance(state);
}
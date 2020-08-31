// STL
#include <memory>

// MPB
#include "base/Environment.h"

// Other
#include <ompl/base/StateValidityChecker.h>

namespace ob = ompl::base;

class EnvironmentStateValidityChecker : public ob::StateValidityChecker {
 public:
  EnvironmentStateValidityChecker(ob::SpaceInformation *si, Environment *env)
      : ob::StateValidityChecker(si), env_(env) {}

  EnvironmentStateValidityChecker(const ob::SpaceInformationPtr &si,
                                  const std::shared_ptr<Environment> &env)
      : ob::StateValidityChecker(si.get()), env_(env) {}

  virtual ~EnvironmentStateValidityChecker() = default;

  virtual bool isValid(const ob::State *state) const override;

  virtual double clearance(const ob::State *state) const override;

 protected:
  std::shared_ptr<Environment> env_;
};
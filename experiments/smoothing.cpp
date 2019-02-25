#include "base/PlannerSettings.h"

#include "planners/OMPLPlanner.hpp"
#include "planners/sbpl/SbplPlanner.h"
#include "planners/thetastar/ThetaStar.h"

#include "utils/PathEvaluation.hpp"

namespace og = ompl::geometric;

int main(int argc, char **argv) {
  Log::instantiateRun();

  const unsigned int rounds = 8;
  for (unsigned int i = 0; i < rounds; ++i) {
    OMPL_INFORM("+++ Run %d / %d +++", i + 1, rounds);
    delete global::settings.environment;
    global::settings.environment = GridMaze::createRandomCorridor(
        50, 50, 3,
        30, i + 1);

    global::settings.steer.steering_type = Steering::STEER_TYPE_REEDS_SHEPP;
    global::settings.steer.car_turning_radius = 3;
    global::settings.steer.initializeSteering();

    auto info = nlohmann::json({{"plans", {}}});
    global::settings.environment->to_json(info["environment"]);

    PathEvaluation::evaluateSmoothers<RRTPlanner>(info);
    Log::log(info);
  }

  Log::save("smoothing.json");

  return EXIT_SUCCESS;
}

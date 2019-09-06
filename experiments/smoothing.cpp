#include "base/GridMaze.h"
#include "base/PlannerSettings.h"

#include "planners/OMPLPlanner.hpp"

#include "utils/PathEvaluation.hpp"

namespace og = ompl::geometric;

int main(int argc, char **argv) {
  const unsigned int rounds = 5;
  for (unsigned int i = 0; i < rounds; ++i) {
    OMPL_INFORM("+++ Run %d / %d +++", i + 1, rounds);
    delete global::settings.environment;
    global::settings.environment =
        GridMaze::createRandomCorridor(50, 50, 3, 30, i + 1);

    global::settings.steer.steering_type = Steering::STEER_TYPE_CC_DUBINS;
    global::settings.steer.car_turning_radius = 3;
    global::settings.steer.initializeSteering();
    global::settings.env.collision.initializeCollisionModel();
    if (i == 0) Log::instantiateRun();

    auto info = nlohmann::json({{"plans", {}}});
    global::settings.environment->to_json(info["environment"]);

    PathEvaluation::evaluateSmoothers<RRTPlanner>(info);
    Log::log(info);
  }

  Log::save("smoothing_cc_dubins.json");

  return EXIT_SUCCESS;
}

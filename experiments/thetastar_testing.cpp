#include "base/GridMaze.h"
#include "base/PlannerSettings.h"

#include "planners/thetastar/ThetaStar.h"

#include "utils/PathEvaluation.hpp"

namespace og = ompl::geometric;

int main(int argc, char **argv) {
  global::settings.max_planning_time = 30;
  int successes = 0;
  int total = 5;
  for (int i = 0; i < total; ++i) {
    delete global::settings.environment;
    global::settings.environment =
        GridMaze::createRandomCorridor(50, 50, 6, 30, i + 1);

    global::settings.steer.steering_type = Steering::STEER_TYPE_CC_REEDS_SHEPP;
    global::settings.steer.initializeSteering();

    global::settings.env.collision.collision_model = robot::ROBOT_POLYGON;
    global::settings.env.collision.initializeCollisionModel();
    if (i == 0) Log::instantiateRun();

    auto info = nlohmann::json({{"plans", {}}});
    global::settings.environment->to_json(info["environment"]);

    successes += PathEvaluation::evaluateSmoothers<ThetaStar>(info);
    Log::log(info);
  }

  std::cout << "Theta* found a solution " << successes << " out of " << total
            << " times.\n";

  Log::save("thetastar_testing.json");

  return EXIT_SUCCESS;
}

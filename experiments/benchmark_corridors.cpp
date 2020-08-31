#include "base/PlannerSettings.h"
#include "base/environments/GridMaze.h"
#include "planners/OMPLPlanner.hpp"
#include "planners/sbpl/SbplPlanner.h"
#include "planners/thetastar/ThetaStar.h"
#include "utils/PathEvaluation.hpp"

namespace og = ompl::geometric;

int main(int argc, char **argv) {
  Log::instantiateRun();

  for (unsigned int i = 0; i < 1; ++i) {
    global::settings.environment =
        GridMaze::createRandomCorridor(150, 150, 4, 130, i + 1);

    global::settings.max_planning_time = 3;
    global::settings.steer.steering_type = Steering::STEER_TYPE_REEDS_SHEPP;
    global::settings.steer.initializeSteering();
    global::settings.env.collision.initializeCollisionModel();

    auto info = nlohmann::json({{"plans", {}}});
    global::settings.environment->to_json(info["environment"]);

    //    PathEvaluation::evaluate<ThetaStar>(info);
    //    PathEvaluation::evaluate<BITstarPlanner>(info);
    PathEvaluation::evaluate<RRTstarPlanner>(info);
    PathEvaluation::evaluate<RRTsharpPlanner>(info);
    //    PathEvaluation::evaluate<InformedRRTstarPlanner>(info);
    //    PathEvaluation::evaluate<SORRTstarPlanner>(info);
    //    PathEvaluation::evaluate<CForestPlanner>(info);
    //    PathEvaluation::evaluate<SbplPlanner>(info);
    Log::log(info);
  }

  Log::save("benchmark_corridors.json");

  return EXIT_SUCCESS;
}

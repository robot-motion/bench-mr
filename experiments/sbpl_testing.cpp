#include "base/PlannerSettings.h"
#include "base/GridMaze.h"

#include "planners/OMPLPlanner.hpp"
#include "planners/sbpl/SbplPlanner.h"
#include "planners/thetastar/ThetaStar.h"

#include "utils/PathEvaluation.hpp"

namespace og = ompl::geometric;

int main(int argc, char **argv) {
  global::settings.max_planning_time = 15;
  for (unsigned int i = 0; i < 10; ++i) {
    delete global::settings.environment;
    global::settings.environment =
        GridMaze::createRandomCorridor(150, 150, 8, 160, i + 1);

    global::settings.steer.initializeSteering();
    global::settings.env.collision.initializeCollisionModel();
    if (i == 0) Log::instantiateRun();

    auto info = nlohmann::json({{"plans", {}}});
    global::settings.environment->to_json(info["environment"]);

    PathEvaluation::evaluate<SbplPlanner<sbpl::SBPL_ARASTAR>>(info);
    PathEvaluation::evaluate<SbplPlanner<sbpl::SBPL_ADSTAR>>(info);
    PathEvaluation::evaluate<SbplPlanner<sbpl::SBPL_ANASTAR>>(info);
    PathEvaluation::evaluate<SbplPlanner<sbpl::SBPL_MHA>>(info);
    PathEvaluation::evaluate<SbplPlanner<sbpl::SBPL_LAZY_ARA>>(info);
    Log::log(info);
  }

  Log::save("sbpl_testing.json");

  return EXIT_SUCCESS;
}

#include "base/PlannerSettings.h"

#include "planners/OMPLPlanner.hpp"
#include "planners/sbpl/SbplPlanner.h"
#include "planners/thetastar/ThetaStar.h"

#include "utils/PathEvaluation.hpp"

namespace og = ompl::geometric;

int main(int argc, char **argv) {
  Log::instantiateRun();

  for (unsigned int i = 0; i < 10; ++i) {
    OMPL_INFORM("+++ Run %d / %d +++", i + 1, 10);
    delete global::settings.environment;
    global::settings.environment = GridMaze::createRandomCorridor(
        50, 50, 3,
        30,  // 1540486476); //1540445576); //1502484532); //1502407983);
        // //1502323408); //1502316103); //1502231684); //1502227898);
        // //1501893283); //1501892155);//1501089540); //1501089410
        // );//1500660612);// 1500551721);// 1500550472);
        i + 1);

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

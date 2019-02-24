#include "base/PlannerSettings.h"

#include "metrics/PathLengthMetric.h"

#include "planners/OMPLPlanner.hpp"
#include "planners/chomp/Chomp.h"
#include "planners/sbpl/SbplPlanner.h"
#include "planners/thetastar/ThetaStar.h"

#include "utils/PathEvaluation.hpp"

#include "PostSmoothing.h"

namespace og = ompl::geometric;

int main(int argc, char **argv) {
  Log::instantiateRun();

  for (unsigned int i = 0; i < 10; ++i) {
    global::settings.environment = GridMaze::createRandomCorridor(
        50, 50, 3,
        30,  // 1540486476); //1540445576); //1502484532); //1502407983);
        // //1502323408); //1502316103); //1502231684); //1502227898);
        // //1501893283); //1501892155);//1501089540); //1501089410
        // );//1500660612);// 1500551721);// 1500550472);
        (unsigned int)(time(nullptr) + 123));

    global::settings.steer.steering_type = Steering::STEER_TYPE_POSQ;
      global::settings.steer.car_turning_radius = 5;
    global::settings.steer.initializeSteering();

    auto info = nlohmann::json({{"plans", {}}});
//    global::settings.environment->to_json(info["environment"]);

    //    PathEvaluation::evaluate<ChompPlanner>(info);
//    PathEvaluation::evaluate<ThetaStar>(info);
        PathEvaluation::evaluate<RRTPlanner>(info);
    //    PathEvaluation::evaluate<RRTstarPlanner>(info);
    //    PathEvaluation::evaluate<RRTsharpPlanner>(info);
    //    PathEvaluation::evaluate<InformedRRTstarPlanner>(info);
    //    PathEvaluation::evaluate<SORRTstarPlanner>(info);
    //    PathEvaluation::evaluate<CForestPlanner>(info);
    //      PathEvaluation::evaluate<SbplPlanner>(info);
    Log::log(info);
  }

  Log::save();

  return EXIT_SUCCESS;
}

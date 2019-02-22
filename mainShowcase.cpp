//#define DEBUG 1 // TODO activate DEBUG in Plannerglobal::settings.h

#include "base/PlannerSettings.h"

#include "metrics/PathLengthMetric.h"

#include "planners/OMPLPlanner.hpp"
#include "planners/chomp/Chomp.h"
#include "planners/sbpl/SbplPlanner.h"
#include "planners/thetastar/ThetaStar.h"

#include "utils/PathEvaluation.hpp"
#if QT_SUPPORT
#include "gui/QtVisualizer.h"
#endif

#include "PostSmoothing.h"

namespace og = ompl::geometric;

int main(int argc, char **argv) {
#if QT_SUPPORT
  QtVisualizer::initialize();
#endif

  //    std::vector<Rectangle> obstacles;
  //    obstacles.emplace_back(Rectangle(10, 0, 15, 14));
  //    obstacles.emplace_back(Rectangle(26, 10, 31, 25));
  //    global::settings.environment =
  //    Environment::createFromObstacles(obstacles, 40, 25);
  //    global::settings.environment->setStart(Tpoint(5, 3));
  //    global::settings.environment->setGoal(Tpoint(36, 22));
  //    global::settings.environment = Environment::createRandom(50, 50, 0.1,
  //    1542671305);

  Log::instantiateRun();

  for (unsigned int i = 0; i < 10; ++i) {
    global::settings.environment = GridMaze::createRandomCorridor(
        50, 50, 3,
        30,  // 1540486476); //1540445576); //1502484532); //1502407983);
        // //1502323408); //1502316103); //1502231684); //1502227898);
        // //1501893283); //1501892155);//1501089540); //1501089410
        // );//1500660612);// 1500551721);// 1500550472);
        (unsigned int)(time(nullptr) + 123));

    global::settings.steer.steering_type = Steering::STEER_TYPE_REEDS_SHEPP;
    //    global::settings.CarTurningRadius = 1.5;
    global::settings.steer.initializeSteering();

#if QT_SUPPORT
    QtVisualizer::visualize(global::settings.environment, 0);
#endif
    PathStatistics thetaStarStats, rrtStarStats, gripsStats,
        smoothThetaStarStats, sbplStats, chompStats;

    std::vector<Point> gripsPath;
    std::vector<GNode> gripsTrajectory;

    auto info = nlohmann::json(
        {{"plans", {}}, {"environment", *global::settings.environment}});

    //        ChompPlanner chompPlanner;
    //        if (chompPlanner.run()) {
    //            std::vector<Tpoint> path = chompPlanner.solutionPath();
    //            chompStats = PathEvaluation::evaluate(path, "CHOMP",
    //            Qt::darkCyan);
    //        }
    //        info["plans"]["chomp"] = {
    //                {"curvature", chompStats.curvature},
    //                {"path_length", chompStats.path_length},
    //                {"steps", std::nan("N/A")},
    //                {"time", chompPlanner.planningTime()},
    //                {"mean_clearing_distance",
    //                chompStats.mean_clearing_distance},
    //                {"median_clearing_distance",
    //                chompStats.median_clearing_distance},
    //                {"min_clearing_distance",
    //                chompStats.min_clearing_distance},
    //                {"max_clearing_distance",
    //                chompStats.max_clearing_distance},
    //                {"path", chompPlanner.solutionTrajectory().empty() ?
    //                Log::serializePath({}) :
    //                Log::serializePath(chompPlanner.solutionPath())},
    //                {"trajectory",
    //                Log::serializeTrajectory(chompPlanner.solutionTrajectory())}
    //        };

    auto *thetaStar = new ThetaStar;
    if (thetaStar->run()) {
      //                std::vector<Point> path = thetaStar->solutionPath();
      PathEvaluation::evaluate(thetaStarStats, thetaStar->solution(),
                               thetaStar);

      //                std::vector<GNode> trajectory =
      //                thetaStar->solutionTrajectory(); gripsTrajectory =
      //                trajectory; PostSmoothing::smooth(gripsTrajectory,
      //                path); gripsPath =
      //                PlannerUtils::toSteeredTrajectoryPoints(gripsTrajectory);
      //                gripsStats = PathEvaluation::evaluate(gripsPath,
      //                "GRIPS", Qt::red);
    } else {
      OMPL_ERROR("Theta* couldn't find a solution.");
    }
    info["plans"]["thetaStar"] = {
        {"curvature", thetaStarStats.curvature},
        {"path_length", thetaStarStats.path_length},
        {"steps", thetaStar->steps()},
        {"time", thetaStar->planningTime()},
        {"mean_clearing_distance", thetaStarStats.mean_clearing_distance},
        {"median_clearing_distance", thetaStarStats.median_clearing_distance},
        {"min_clearing_distance", thetaStarStats.min_clearing_distance},
        {"max_clearing_distance", thetaStarStats.max_clearing_distance},
        {"path", Log::serializePath(thetaStar->solutionPath())},
        {"trajectory", Log::serializeTrajectory(thetaStar->solution())}};
    //        info["plans"]["grips"] = {
    //                {"curvature", gripsStats.curvature},
    //                {"path_length", gripsStats.path_length},
    //                {"steps", thetaStar->steps()},
    //                {"time", PostSmoothing::smoothingTime +
    //                thetaStar->planningTime()},
    //                {"mean_clearing_distance",
    //                gripsStats.mean_clearing_distance},
    //                {"median_clearing_distance",
    //                gripsStats.median_clearing_distance},
    //                {"min_clearing_distance",
    //                gripsStats.min_clearing_distance},
    //                {"max_clearing_distance",
    //                gripsStats.max_clearing_distance},
    //                {"path", Log::serializePath(gripsPath)},
    //                {"trajectory", Log::serializeTrajectory(gripsTrajectory)}
    //        };
    //        delete thetaStar;
    //
    //        auto *rrtStar = new RRTstarPlanner;
    //        if (rrtStar->run()) {
    //            std::vector<Tpoint> path = rrtStar->solutionPath();
    //            rrtStarStats = PathEvaluation::evaluate(path, "RRT*",
    //            Qt::black);
    //        } else {
    //            OMPL_ERROR("RRT* couldn't find a solution.");
    //        }
    //        info["plans"]["rrtStar"] = {
    //                {"curvature", rrtStarStats.curvature},
    //                {"path_length", rrtStarStats.path_length},
    //                {"steps", std::nan("N/A")},
    //                {"time", rrtStar->planningTime()},
    //                {"mean_clearing_distance",
    //                rrtStarStats.mean_clearing_distance},
    //                {"median_clearing_distance",
    //                rrtStarStats.median_clearing_distance},
    //                {"min_clearing_distance",
    //                rrtStarStats.min_clearing_distance},
    //                {"max_clearing_distance",
    //                rrtStarStats.max_clearing_distance},
    //                {"path", Log::serializePath(rrtStar->solutionPath())},
    //                {"trajectory",
    //                Log::serializeTrajectory(rrtStar->solutionTrajectory())}
    //        };
    //        delete rrtStar;
    //
    //        auto *smoothThetaStar = new SmoothThetaStar;
    //        if (smoothThetaStar->run()) {
    //            std::vector<Tpoint> path = smoothThetaStar->solutionPath();
    //            smoothThetaStarStats = PathEvaluation::evaluate(path, "Smooth
    //            Theta*", Qt::blue);
    //        }
    //        info["plans"]["smoothThetaStar"] = {
    //                {"curvature", smoothThetaStarStats.curvature},
    //                {"path_length", smoothThetaStarStats.path_length},
    //                {"steps", smoothThetaStar->steps()},
    //                {"time", smoothThetaStar->planningTime()},
    //                {"mean_clearing_distance",
    //                smoothThetaStarStats.mean_clearing_distance},
    //                {"median_clearing_distance",
    //                        smoothThetaStarStats.median_clearing_distance},
    //                {"min_clearing_distance",
    //                smoothThetaStarStats.min_clearing_distance},
    //                {"max_clearing_distance",
    //                smoothThetaStarStats.max_clearing_distance},
    //                {"path",
    //                Log::serializePath(smoothThetaStar->solutionPath())},
    //                {"trajectory",
    //                Log::serializeTrajectory(smoothThetaStar->solutionTrajectory())}
    //        };
    //        delete smoothThetaStar;

    OMPL_INFORM("Starting planner");
    //    auto *sbplPlanner = new
    //    SbplPlanner(SbplPlanner::SbplType::SBPL_ARASTAR); if
    //    (sbplPlanner->run()) {
    //      sbplStats =
    //          PathEvaluation::evaluate(sbplPlanner->solution(), "SBPL
    //          (ANA*)");
    //#if QT_SUPPORT
    //      sbplStats.color = Qt::darkGreen;
    //#endif
    //    }
    //    info["plans"]["sbpl"] = {
    //        {"curvature", sbplStats.curvature},
    //        {"path_length", sbplStats.path_length},
    //        {"steps", std::nan("N/A")},
    //        {"time", sbplPlanner->planningTime()},
    //        {"mean_clearing_distance", sbplStats.mean_clearing_distance},
    //        {"median_clearing_distance", sbplStats.median_clearing_distance},
    //        {"min_clearing_distance", sbplStats.min_clearing_distance},
    //        {"max_clearing_distance", sbplStats.max_clearing_distance},
    //        {"path", Log::serializePath(sbplPlanner->solutionPath())},
    //        {"trajectory",
    //        Log::serializeTrajectory(sbplPlanner->solution())}};
    //    delete sbplPlanner;

    //        printStats(thetaStarStats);
    //        std::cout << "\tSteps:\t\t\t" << thetaStar->steps() << std::endl;
    //        std::cout << "\tTime:\t\t\t" << thetaStar->planningTime() <<
    //        std::endl; printStats(gripsStats); std::cout << "\tSteps:\t\t\t"
    //        << thetaStar->steps() << std::endl; std::cout << "\tTime:\t\t\t"
    //        << PostSmoothing::smoothingTime + thetaStar->planningTime() <<
    //        std::endl; printStats(smoothThetaStarStats); std::cout <<
    //        "\tSteps:\t\t\t" << smoothThetaStar->steps() << std::endl;
    //        std::cout << "\tTime:\t\t\t" << smoothThetaStar->planningTime() <<
    //        std::endl;
    //
    Log::log(info);
  }

  Log::save();
  //    global::settings.environment->saveSbplConfigFile("env_" +
  //    Log::filename() + ".cfg");

#if DEBUG
  QtVisualizer::show();
  return QtVisualizer::exec();
#else
  return EXIT_SUCCESS;
#endif
}

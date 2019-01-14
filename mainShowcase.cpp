//#define DEBUG 1 // TODO activate DEBUG in PlannerSettings.h

#include <planners/ThetaStar.h>
#include <planners/Chomp.h>
#include "base/PlannerSettings.h"

#include "steer_functions/POSQ/POSQSteering.h"

#include "metrics/PathLengthMetric.h"

#include "planners/OMPLPlanner.hpp"
#include "planners/SmoothThetaStar.h"
#include "planners/SbplPlanner.h"

#include "gui/PathEvaluation.h"

#include "PostSmoothing.h"


namespace og = ompl::geometric;

void printStats(const PathStatistics &stats)
{
    std::cout << stats.planner << std::endl;
    std::cout << "\tPath length:   \t" << stats.pathLength << std::endl;
    std::cout << "\tMax curvature: \t" << stats.curvature << std::endl;
}

int main(int argc, char **argv)
{
    PlannerSettings::steeringType = Steering::STEER_TYPE_REEDS_SHEPP;
//    PlannerSettings::CarTurningRadius = 1.5;
    PlannerSettings::initializeSteering();
    PathEvaluation::initialize();

    QtVisualizer::initialize();

//    std::vector<Rectangle> obstacles;
//    obstacles.emplace_back(Rectangle(10, 0, 15, 14));
//    obstacles.emplace_back(Rectangle(26, 10, 31, 25));
//    PlannerSettings::environment = Environment::createFromObstacles(obstacles, 40, 25);
//    PlannerSettings::environment->setStart(Tpoint(5, 3));
//    PlannerSettings::environment->setGoal(Tpoint(36, 22));
//    PlannerSettings::environment = Environment::createRandom(50, 50, 0.1, 1542671305);

    PlannerSettings::environment = Environment::createRandomCorridor(50, 50, 3, 30, //1540486476); //1540445576); //1502484532); //1502407983); //1502323408); //1502316103); //1502231684); //1502227898); //1501893283); //1501892155);//1501089540); //1501089410 );//1500660612);// 1500551721);// 1500550472);
                                                                     (unsigned int) (time(nullptr) + 123));

    Log::instantiateRun();

    for (unsigned int i = 0; i < 10; ++i) {
//        PlannerSettings::environment = Environment::createRandom(50, 50, 0.1, 1542671305 + i);
        PlannerSettings::environment = Environment::createRandomCorridor(50, 50, 3, 30, 1540486476 + i + 1); //1540486476); //1540445576); //1502484532); //1502407983); //1502323408); //1502316103); //1502231684); //1502227898); //1501893283); //1501892155);//1501089540); //1501089410 );//1500660612);// 1500551721);// 1500550472);
//                                                                         (unsigned int) (time(nullptr) + 123));
        QtVisualizer::visualize(PlannerSettings::environment, 0);
        PathStatistics thetaStarStats, rrtStarStats, gripsStats, smoothThetaStarStats, sbplStats, chompStats;

        std::vector<Tpoint> gripsPath;
        std::vector<GNode> gripsTrajectory;

        auto info = nlohmann::json({{"plans", {}}, {"environment", PlannerSettings::environment->asJSON()}});

        ChompPlanner chompPlanner;
        if (chompPlanner.run()) {
            std::vector<Tpoint> path = chompPlanner.solutionPath();
            chompStats = PathEvaluation::evaluate(path, "CHOMP", Qt::darkCyan);
        }
        info["plans"]["chomp"] = {
                {"curvature", chompStats.curvature},
                {"pathLength", chompStats.pathLength},
                {"steps", std::nan("N/A")},
                {"time", chompPlanner.planningTime()},
                {"meanClearingDistance", chompStats.meanClearingDistance},
                {"medianClearingDistance", chompStats.medianClearingDistance},
                {"minClearingDistance", chompStats.minClearingDistance},
                {"maxClearingDistance", chompStats.maxClearingDistance},
                {"path", chompPlanner.solutionTrajectory().empty() ? Log::serializePath({}) : Log::serializePath(chompPlanner.solutionPath())},
                {"trajectory", Log::serializeTrajectory(chompPlanner.solutionTrajectory())}
        };

        auto *thetaStar = new ThetaStar;
        if (thetaStar->run()) {
            std::vector<Tpoint> path = thetaStar->solutionPath();
            thetaStarStats = PathEvaluation::evaluate(path, "Theta*", Qt::black);

            std::vector<GNode> trajectory = thetaStar->solutionTrajectory();
            gripsTrajectory = trajectory;
            PostSmoothing::smooth(gripsTrajectory, path);
            gripsPath = PlannerUtils::toSteeredTrajectoryPoints(gripsTrajectory);
            gripsStats = PathEvaluation::evaluate(gripsPath, "GRIPS", Qt::red);
        } else {
            OMPL_ERROR("Theta* couldn't find a solution.");
        }
        info["plans"]["thetaStar"] = {
                {"curvature", thetaStarStats.curvature},
                {"pathLength", thetaStarStats.pathLength},
                {"steps", thetaStar->steps()},
                {"time", thetaStar->planningTime()},
                {"meanClearingDistance", thetaStarStats.meanClearingDistance},
                {"medianClearingDistance", thetaStarStats.medianClearingDistance},
                {"minClearingDistance", thetaStarStats.minClearingDistance},
                {"maxClearingDistance", thetaStarStats.maxClearingDistance},
                {"path", Log::serializePath(thetaStar->solutionPath())},
                {"trajectory", Log::serializeTrajectory(thetaStar->solutionTrajectory())}
        };
        info["plans"]["grips"] = {
                {"curvature", gripsStats.curvature},
                {"pathLength", gripsStats.pathLength},
                {"steps", thetaStar->steps()},
                {"time", PostSmoothing::smoothingTime + thetaStar->planningTime()},
                {"meanClearingDistance", gripsStats.meanClearingDistance},
                {"medianClearingDistance", gripsStats.medianClearingDistance},
                {"minClearingDistance", gripsStats.minClearingDistance},
                {"maxClearingDistance", gripsStats.maxClearingDistance},
                {"path", Log::serializePath(gripsPath)},
                {"trajectory", Log::serializeTrajectory(gripsTrajectory)}
        };
        delete thetaStar;

        auto *rrtStar = new RRTstarPlanner;
        if (rrtStar->run()) {
            std::vector<Tpoint> path = rrtStar->solutionPath();
            rrtStarStats = PathEvaluation::evaluate(path, "RRT*", Qt::black);
        } else {
            OMPL_ERROR("RRT* couldn't find a solution.");
        }
        info["plans"]["rrtStar"] = {
                {"curvature", rrtStarStats.curvature},
                {"pathLength", rrtStarStats.pathLength},
                {"steps", std::nan("N/A")},
                {"time", rrtStar->planningTime()},
                {"meanClearingDistance", rrtStarStats.meanClearingDistance},
                {"medianClearingDistance", rrtStarStats.medianClearingDistance},
                {"minClearingDistance", rrtStarStats.minClearingDistance},
                {"maxClearingDistance", rrtStarStats.maxClearingDistance},
                {"path", Log::serializePath(rrtStar->solutionPath())},
                {"trajectory", Log::serializeTrajectory(rrtStar->solutionTrajectory())}
        };
        delete rrtStar;

        auto *smoothThetaStar = new SmoothThetaStar;
        if (smoothThetaStar->run()) {
            std::vector<Tpoint> path = smoothThetaStar->solutionPath();
            smoothThetaStarStats = PathEvaluation::evaluate(path, "Smooth Theta*", Qt::blue);
        }
        info["plans"]["smoothThetaStar"] = {
                {"curvature", smoothThetaStarStats.curvature},
                {"pathLength", smoothThetaStarStats.pathLength},
                {"steps", smoothThetaStar->steps()},
                {"time", smoothThetaStar->planningTime()},
                {"meanClearingDistance", smoothThetaStarStats.meanClearingDistance},
                {"medianClearingDistance",
                        smoothThetaStarStats.medianClearingDistance},
                {"minClearingDistance", smoothThetaStarStats.minClearingDistance},
                {"maxClearingDistance", smoothThetaStarStats.maxClearingDistance},
                {"path", Log::serializePath(smoothThetaStar->solutionPath())},
                {"trajectory", Log::serializeTrajectory(smoothThetaStar->solutionTrajectory())}
        };
        delete smoothThetaStar;

        auto *sbplPlanner = new SbplPlanner(SbplPlanner::SbplType::SBPL_ARASTAR);
        if (sbplPlanner->run()) {
            std::vector<Tpoint> path = sbplPlanner->solutionPath();
            sbplStats = PathEvaluation::evaluate(path, "SBPL (ANA*)", Qt::darkGreen);
        }
        info["plans"]["sbpl"] = {
                {"curvature", sbplStats.curvature},
                {"pathLength", sbplStats.pathLength},
                {"steps", std::nan("N/A")},
                {"time", sbplPlanner->planningTime()},
                {"meanClearingDistance", sbplStats.meanClearingDistance},
                {"medianClearingDistance", sbplStats.medianClearingDistance},
                {"minClearingDistance", sbplStats.minClearingDistance},
                {"maxClearingDistance", sbplStats.maxClearingDistance},
                {"path", sbplPlanner->solutionTrajectory().empty() ? Log::serializePath({}) : Log::serializePath(sbplPlanner->solutionPath())},
                {"trajectory", Log::serializeTrajectory(sbplPlanner->solutionTrajectory())}
        };
        delete sbplPlanner;

//        printStats(thetaStarStats);
//        std::cout << "\tSteps:\t\t\t" << thetaStar->steps() << std::endl;
//        std::cout << "\tTime:\t\t\t" << thetaStar->planningTime() << std::endl;
//        printStats(gripsStats);
//        std::cout << "\tSteps:\t\t\t" << thetaStar->steps() << std::endl;
//        std::cout << "\tTime:\t\t\t" << PostSmoothing::smoothingTime + thetaStar->planningTime() << std::endl;
//        printStats(smoothThetaStarStats);
//        std::cout << "\tSteps:\t\t\t" << smoothThetaStar->steps() << std::endl;
//        std::cout << "\tTime:\t\t\t" << smoothThetaStar->planningTime() << std::endl;
//
        Log::log(info);
    }

    Log::save();
    PlannerSettings::environment->saveSbplConfigFile("env_" + Log::filename() + ".cfg");

#if DEBUG
    QtVisualizer::show();
    return QtVisualizer::exec();
#else
    return EXIT_SUCCESS;
#endif
}

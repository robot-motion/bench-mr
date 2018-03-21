#define DEBUG // TODO activate DEBUG in PlannerSettings.h

#include <planners/ThetaStar.h>
#include "base/PlannerSettings.h"

#include "steer_functions/POSQ/POSQSteering.h"

#include "metrics/PathLengthMetric.h"

#include "planners/OMPLPlanner.hpp"
#include "planners/SmoothThetaStar.h"

#include "gui/PathEvaluation.h"

#include "PostSmoothing.h"


namespace og = ompl::geometric;

int main(int argc, char **argv)
{
    PlannerSettings::initializeSteering();
    PathEvaluation::initialize();

    QtVisualizer::initialize();

    std::vector<Rectangle> obstacles;
    obstacles.emplace_back(Rectangle(10, 0, 15, 15));
    obstacles.emplace_back(Rectangle(26, 10, 31, 25));

    PlannerSettings::environment = Environment::createFromObstacles(obstacles, 40, 25);
    PlannerSettings::environment->setStart(Tpoint(5, 3));
    PlannerSettings::environment->setGoal(Tpoint(36, 22));

    QtVisualizer::visualize(*PlannerSettings::environment, 0);
    PathStatisticsAggregator statsAggregator;

    Log::instantiateRun();
    statsAggregator.add(PathEvaluation::add(new ThetaStar, "Theta*", Qt::black));
    statsAggregator.add(PathEvaluation::add(new SmoothThetaStar, "Smooth Theta*", Qt::blue));
    Log::storeRun();
    Log::save();

    statsAggregator.showSummary();

//    std::vector<GNode> smoothed(trajectory);
//    PostSmoothing::smooth(smoothed, path);
//    auto smoothedTrajPoints = PlannerUtils::toSteeredTrajectoryPoints(smoothed);
//
//    QColor c(Qt::magenta);
//    c.setAlpha(150);
//    QPen oPen(c, 3.);
//    QtVisualizer::drawPath(smoothedTrajPoints, oPen);

    QtVisualizer::show();

    return QtVisualizer::exec();
}
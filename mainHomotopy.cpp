#include "base/PlannerSettings.h"
#include "gui/PathEvaluation.h"
#include "gui/QtVisualizer.h"
#include "planners/AStar.hpp"

namespace og = ompl::geometric;

int main(int argc, char **argv) {
  PathEvaluation::initialize();

  settings.steer.steering_type = Steering::STEER_TYPE_REEDS_SHEPP;
  settings.CarTurningRadius = 3.5;
  settings.initializeSteering();

  settings.VisualizeSmoothing1 = false;
  settings.VisualizeSmoothing2 = false;
  settings.VisualizeSmoothing3 = false;
  settings.VisualizeSmoothing4 = false;

  PathStatisticsAggregator statsAggregator{};

  QtVisualizer::initialize();

  settings.environment = Environment::createRandom(
      Environment::DefaultWidth, Environment::DefaultHeight, 0.1, 1234);
  settings.environment->setStart(Tpoint(2, 12));
  settings.environment->setGoal(
      Tpoint(Environment::DefaultWidth - 1, Environment::DefaultHeight - 12));

  QtVisualizer::visualize(settings.environment, 0);

  statsAggregator.add(PathEvaluation::add(new ThetaStar, "Theta*", Qt::black));
  statsAggregator.add(PathEvaluation::add(new AStar, "A*", Qt::gray));
  QtVisualizer::show();

  statsAggregator.showSummary();

  return QtVisualizer::exec();
}

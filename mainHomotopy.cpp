#include "PlannerSettings.h"
#include "gui/PathEvaluation.h"
#include "gui/QtVisualizer.h"
#include "planners/AStar.hpp"

namespace og = ompl::geometric;

int main(int argc, char **argv) {
  PathEvaluation::initialize();

  global::settings.steer.steering_type = Steering::STEER_TYPE_REEDS_SHEPP;
  global::settings.CarTurningRadius = 3.5;
  global::settings.initializeSteering();

  global::settings.VisualizeSmoothing1 = false;
  global::settings.VisualizeSmoothing2 = false;
  global::settings.VisualizeSmoothing3 = false;
  global::settings.VisualizeSmoothing4 = false;

  PathStatisticsAggregator statsAggregator{};

  QtVisualizer::initialize();

  global::settings.environment = Environment::createRandom(
      Environment::DefaultWidth, Environment::DefaultHeight, 0.1, 1234);
  global::settings.environment->setStart(Tpoint(2, 12));
  global::settings.environment->setGoal(
      Tpoint(Environment::DefaultWidth - 1, Environment::DefaultHeight - 12));

  QtVisualizer::visualize(global::settings.environment, 0);

  statsAggregator.add(PathEvaluation::add(new ThetaStar, "Theta*", Qt::black));
  statsAggregator.add(PathEvaluation::add(new AStar, "A*", Qt::gray));
  QtVisualizer::show();

  statsAggregator.showSummary();

  return QtVisualizer::exec();
}

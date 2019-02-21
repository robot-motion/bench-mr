#include "Environment.h"
#include <planners/thetastar/ThetaStar.h>
#include "PlannerUtils.hpp"

bool Environment::collides(const ompl::geometric::PathGeometric &trajectory) {
  for (auto &p : Point::fromPath(PlannerUtils::interpolated(trajectory))) {
    if (collides(p.x, p.y)) {
#if defined(DEBUG) && defined(QT_SUPPORT)
      QtVisualizer::drawNode(p.x, p.y, QColor(255, 255, 0, 150), .5);
#endif
      return true;
    }
  }
  return false;
}

double Environment::bilinearDistance(double x, double y, double cellSize) {
  const double xi =
      std::floor(std::max(std::min(width(), x), 0.) / cellSize) * cellSize;
  const double yi =
      std::floor(std::max(std::min(height(), y), 0.) / cellSize) * cellSize;
  const double xp =
      std::floor(std::max(std::min(width(), x + cellSize), 0.) / cellSize) *
      cellSize;
  const double yp =
      std::floor(std::max(std::min(height(), y + cellSize), 0.) / cellSize) *
      cellSize;
  const double u_ratio = x - xi;
  const double v_ratio = y - yi;
  const double u_opposite = cellSize - u_ratio;
  const double v_opposite = cellSize - v_ratio;
  const double tl = distance(xi, yi), tr = distance(xp, yi);
  const double bl = distance(xi, yp), br = distance(xp, yp);
  return (tl * u_opposite / cellSize + tr * u_ratio / cellSize) * v_opposite /
             cellSize +
         (bl * u_opposite / cellSize + br * u_ratio / cellSize) * v_ratio /
             cellSize;
}

bool Environment::distanceGradient(double x, double y, double &dx, double &dy,
                                   double p, double cellSize) {
  if (x < 0 || y < 0 || x > width() || y > height()) return false;
  // compute distances left, right, top, bottom
  const double dl = bilinearDistance(x - p, y, cellSize),
               dr = bilinearDistance(x + p, y, cellSize);
  const double db = bilinearDistance(x, y - p, cellSize),
               dt = bilinearDistance(x, y + p, cellSize);
  dx = (dl - dr) / (p * 2.);
  dy = (dt - db) / (p * 2.);
  return true;
}

void Environment::estimateStartGoalOrientations() {
  const auto cacheSteeringType = settings.steer.steering_type;
  const auto cacheEstimateTheta = settings.estimate_theta;
  const auto cacheCollisionMode = settings.collision_model;
  settings.steer.steering_type = Steering::STEER_TYPE_LINEAR;
  settings.estimate_theta = false;
  settings.collision_model = robot::ROBOT_POINT;
  settings.steer.initializeSteering();
  auto *thetaStar = new ThetaStar;
  if (thetaStar->run()) {
    std::vector<Point> path = thetaStar->solutionPath();
#if QT_SUPPORT
    QtVisualizer::drawPath(path, Qt::black);
#endif
    _start_theta = std::atan2(path[1].y - path[0].y, path[1].x - path[0].x);
    const auto n = path.size() - 1;
    _goal_theta =
        std::atan2(path[n].y - path[n - 1].y, path[n].x - path[n - 1].x);
    _thetas_defined = true;
  }
  delete thetaStar;
  settings.steer.steering_type = cacheSteeringType;
  settings.estimate_theta = cacheEstimateTheta;
  settings.collision_model = cacheCollisionMode;
  settings.steer.initializeSteering();
}

ompl::base::ScopedState<ob::SE2StateSpace> Environment::startScopedState() const {
  ompl::base::ScopedState<ob::SE2StateSpace> state(settings.ompl.state_space);
  state[0] = _start.x;
  state[1] = _start.y;
  state[2] = _start_theta;
  return state;
}

ompl::base::ScopedState<ob::SE2StateSpace> Environment::goalScopedState() const {
  ompl::base::ScopedState<ob::SE2StateSpace> state(settings.ompl.state_space);
  state[0] = _goal.x;
  state[1] = _goal.y;
  state[2] = _goal_theta;
  return state;
}

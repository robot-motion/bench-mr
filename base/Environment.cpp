#include "Environment.h"

#include <planners/thetastar/ThetaStar.h>

#include "utils/PlannerUtils.hpp"

bool Environment::collides(const ompl::geometric::PathGeometric &trajectory) {
  _collision_timer.resume();
  for (auto &p : Point::fromPath(PlannerUtils::interpolated(trajectory))) {
    if (collides(p.x, p.y)) {
#if defined(DEBUG) && defined(QT_SUPPORT)
      QtVisualizer::drawNode(p.x, p.y, QColor(255, 255, 0, 150), .5);
#endif
      _collision_timer.stop();
      return true;
    }
  }
  _collision_timer.stop();
  return false;
}

bool Environment::checkValidity(const ob::State *state) {
  _collision_timer.resume();
  if (global::settings.env.collision.collision_model == robot::ROBOT_POINT) {
    const auto *s = state->as<ob::SE2StateSpace::StateType>();
    const double x = s->getX(), y = s->getY();
    bool valid = !collides(x, y);
    _collision_timer.stop();
    return valid;
  } else {
    bool valid = !global::settings.environment->collides(
        global::settings.env.collision.robot_shape.value().transformed(state));
    _collision_timer.stop();
    // if (!valid) {
    //   const auto *s = state->as<ob::SE2StateSpace::StateType>();
    //   OMPL_DEBUG("State [%.2f %.2f %.2f] is invalid.", s->getX(), s->getY(),
    //              s->getYaw());
    // }
    return valid;
  }
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
  dx = (dr - dl) / (p * 2.);
  dy = (dt - db) / (p * 2.);
  return true;
}

void Environment::estimateStartGoalOrientations() {
  const auto cacheSteeringType = global::settings.steer.steering_type;
  const auto cacheEstimateTheta = global::settings.estimate_theta;
  const auto cacheCollisionMode =
      global::settings.env.collision.collision_model;
  global::settings.steer.steering_type = Steering::STEER_TYPE_LINEAR;
  global::settings.estimate_theta = false;
  global::settings.env.collision.collision_model = robot::ROBOT_POINT;
  global::settings.steer.initializeSteering();
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
    global::settings.env.start.theta = _start_theta;
    global::settings.env.goal.theta = _goal_theta;
  }
  delete thetaStar;
  global::settings.steer.steering_type = cacheSteeringType;
  global::settings.estimate_theta = cacheEstimateTheta;
  global::settings.env.collision.collision_model = cacheCollisionMode;
  global::settings.steer.initializeSteering();
}

ompl::base::ScopedState<ob::SE2StateSpace> Environment::startScopedState()
    const {
  ompl::base::ScopedState<ob::SE2StateSpace> state(
      global::settings.ompl.state_space);
  state[0] = _start.x;
  state[1] = _start.y;
  state[2] = _start_theta;
  return state;
}

ompl::base::ScopedState<ob::SE2StateSpace> Environment::goalScopedState()
    const {
  ompl::base::ScopedState<ob::SE2StateSpace> state(
      global::settings.ompl.state_space);
  state[0] = _goal.x;
  state[1] = _goal.y;
  state[2] = _goal_theta;
  return state;
}

void Environment::setThetas(double start, double goal) {
  _start_theta = start;
  _goal_theta = goal;
  _thetas_defined = true;
  global::settings.env.start.theta = _start_theta;
  global::settings.env.goal.theta = _goal_theta;
}

void Environment::setStart(const Point &point) {
  _start = point;
  global::settings.env.start.x = _start.x;
  global::settings.env.start.y = _start.y;
}

void Environment::setGoal(const Point &point) {
  _goal = point;
  global::settings.env.goal.x = _goal.x;
  global::settings.env.goal.y = _goal.y;
}

Environment::Environment()
    : _start(global::settings.env.start.x, global::settings.env.start.y),
      _goal(global::settings.env.goal.x, global::settings.env.goal.y),
      _start_theta(global::settings.env.start.theta),
      _goal_theta(global::settings.env.goal.theta) {}

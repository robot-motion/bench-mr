#pragma once

#include <ompl/base/ScopedState.h>

#include "Primitives.h"
#include "utils/Stopwatch.hpp"

class Environment {
 public:
  Environment();
  virtual ~Environment() = default;

  void setStart(const Point &point);
  const Point &start() const { return _start; }

  void setGoal(const Point &point);
  const Point &goal() const { return _goal; }

  virtual bool collides(double x, double y) const = 0;
  virtual bool collides(const Polygon &polygon) const = 0;
  bool collides(const Point &p) const { return collides(p.x, p.y); }
  bool collides(const ompl::geometric::PathGeometric &trajectory) const;
  bool collides(const ob::State *state) const {
    const auto *s = state->as<State>();
    return collides(s->getX(), s->getY());
  }

  /**
   * Used by planners to determine if the state is valid or not.
   */
  virtual bool checkValidity(const ob::State *state);

  inline const ob::RealVectorBounds &bounds() const { return _bounds; }
  inline double width() const { return _bounds.high.at(0) - _bounds.low.at(0); }
  inline double height() const {
    return _bounds.high.at(1) - _bounds.low.at(1);
  }

  virtual double distance(double x, double y) const { return -1; }

  /**
   * Bilinear filtering of distance (only relevant for grid-based environment).
   */
  virtual double bilinearDistance(double x, double y) const {
    return distance(x, y);
  }
  /**
   * Bilinear filtering of distance (only relevant for grid-based environment).
   */
  double bilinearDistance(const Point &point) const {
    return bilinearDistance(point.x, point.y);
  }

  /**
   * Computes negative gradient of distance field at position x, y.
   * @param x Position coordinate x.
   * @param y Position coordinate y.
   * @param dx Resulting gradient coordinate x.
   * @param dy Resulting gradient coordinate y.
   * @param p Sampling precision.
   * @return True, if x and y are within grid boundaries.
   */
  virtual bool distanceGradient(double x, double y, double &dx, double &dy,
                                double p = 0.1) const;

  virtual std::string name() const { return "Base map"; }

  /**
   * Estimates potentially suitable theta values for the start and goal state by
   * running a simple line search from start to goal. This feature is useful for
   * approaches that operate on the SE2 state space and cannot find good start /
   * goal angles themselves (e.g. SBPL).
   */
  void estimateStartGoalOrientations();

  bool thetasDefined() const { return _thetas_defined; }
  void setThetas(double start, double goal);

  ompl::base::State *startState() const { return _start.toState(_start_theta); }
  ompl::base::State *goalState() const { return _goal.toState(_goal_theta); }

  ompl::base::ScopedState<ob::SE2StateSpace> startScopedState() const;
  ompl::base::ScopedState<ob::SE2StateSpace> goalScopedState() const;

  double startTheta() const { return _start_theta; }
  double goalTheta() const { return _goal_theta; }

  virtual void to_json(nlohmann::json &j) const {
    j["type"] = "base";
    j["width"] = width();
    j["height"] = height();
    j["start"] = start();
    j["goal"] = goal();
    j["name"] = name();
  }

  /**
   * Unit, e.g. used by Theta* to determine neighboring states.
   */
  virtual double unit() const { return 1; }

  void resetCollisionTimer() { _collision_timer.reset(); }
  double elapsedCollisionTime() const { return _collision_timer.elapsed(); }

 protected:
  Point _start;
  Point _goal;
  double _start_theta{0};
  double _goal_theta{0};
  bool _thetas_defined{false};

  ob::RealVectorBounds _bounds{2};

  mutable Stopwatch _collision_timer;
};

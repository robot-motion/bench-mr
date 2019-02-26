#pragma once

#include <ompl/base/ScopedState.h>
#include "Primitives.h"

class Environment {
 public:
    Environment();
  virtual ~Environment() = default;

  void setStart(const Point &point);
  const Point &start() const { return _start; }

  void setGoal(const Point &point);
  const Point &goal() const { return _goal; }

  virtual bool collides(double x, double y) { return true; }
  virtual bool collides(const Polygon &polygon) { return true; }
  bool collides(const Point &p) { return collides(p.x, p.y); }
  bool collides(const ompl::geometric::PathGeometric &trajectory);
  bool collides(const ob::State *state) {
    const auto *s = state->as<State>();
    return collides(s->getX(), s->getY());
  }

  inline const ob::RealVectorBounds &bounds() const { return _bounds; }
  inline double width() const { return _bounds.high.at(0); }
  inline double height() const { return _bounds.high.at(1); }

  virtual double distance(double x, double y) { return -1; }

  /**
   * Bilinear filtering of distance.
   */
  double bilinearDistance(double x, double y, double cellSize = 1);
  double bilinearDistance(const Point &point, double cellSize = 1) {
    return bilinearDistance(point.x, point.y, cellSize);
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
  bool distanceGradient(double x, double y, double &dx, double &dy,
                        double p = 0.1, double cellSize = 1);

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

  virtual void to_json(nlohmann::json &j) {
    j["type"] = "base";
    j["width"] = width();
    j["height"] = height();
    j["start"] = start();
    j["goal"] = goal();
    j["name"] = name();
  }

 protected:
  Point _start;
  Point _goal;
  double _start_theta{0};
  double _goal_theta{0};
  bool _thetas_defined{false};

  ob::RealVectorBounds _bounds{2};
};

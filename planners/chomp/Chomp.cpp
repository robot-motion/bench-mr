#include "Chomp.h"
#include <planners/thetastar/ThetaStar.h>
#include <base/PlannerUtils.hpp>

Map2D *ChompPlanner::_map = new Map2D;

ChompPlanner::ChompPlanner() = default;

void ChompPlanner::_initializeStraightLine(int N, const Map2D &map,
                                           const vec2f &p0, const vec2f &p1,
                                           MatX &xi, MatX &q0, MatX &q1) {
  xi.resize(N, 2);
  q0.resize(1, 2);
  q1.resize(1, 2);

  q0 << p0.x(), p0.y();
  q1 << p1.x(), p1.y();

  for (int i = 0; i < N; ++i) {
    float u = float(i + 1) / (N + 1);
    vec2f pi = p0 + u * (p1 - p0);
    xi(i, 0) = pi.x();
    xi(i, 1) = pi.y();
  }
}

void ChompPlanner::_initializeThetaStar(int N, const vec2f &p0, const vec2f &p1,
                                        MatX &xi, MatX &q0, MatX &q1,
                                        bool extraClearing) {
  xi.resize(N, 2);
  q0.resize(1, 2);
  q1.resize(1, 2);

  q0 << p0.x(), p0.y();
  q1 << p1.x(), p1.y();

  Steering::SteeringType stCopy = settings.steer.steering_type;
  settings.steer.steering_type = Steering::SteeringType::STEER_TYPE_LINEAR;
  settings.steer.initializeSteering();
  auto *thetaStar = new ThetaStar;
  if (thetaStar->run()) {
    auto path = thetaStar->solutionPath();
    if (extraClearing) {
      // apply gradient descent to increase clearing distance
      PlannerUtils::gradientDescent(path, 15u, 0.5, 0.9);
    }
    const auto samples =
        PlannerUtils::equidistantSampling(path, static_cast<size_t>(N + 2));
#if DEBUG
    QtVisualizer::drawPath(path, Qt::black);
    QtVisualizer::drawNodes(path, Qt::black);
    QtVisualizer::drawNodes(samples);
#endif
    xi.resize(samples.size() - 2, 2);
    for (unsigned int i = 0; i < samples.size() - 2; ++i) {
      xi(i, 0) = samples[i + 1].x;
      xi(i, 1) = samples[i + 1].y;
    }
  } else {
    OMPL_ERROR("Could not generate Theta* initialization path for CHOMP.");
  }
  delete thetaStar;

  // revert steering function
  settings.steer.steering_type = stCopy;
  settings.steer.initializeSteering();
}

ob::PlannerStatus ChompPlanner::run() {
  _path.clear();
  OMPL_DEBUG("Computing CHOMP grid...");
  _map->grid.clear();
  const float csz = 1.f;
  const Environment &env = *settings.environment;
  vec3f min(0, 0, 0);
  vec3f max(env.width(), env.height(), 1.f);
  _map->grid.resize(min, max, DtGridf::AXIS_Z, csz);
  for (unsigned int y = 0; y <= env.height(); ++y) {
    for (unsigned int x = 0; x <= env.width(); ++x) {
      _map->grid(vec3u(x, y, 0)) = (env.occupiedCell(x, y) ? -1 : 1);
    }
  }
  _map->grid.computeDistsFromBinary();
  _map->eps = settings.chomp.epsilon;

  MatX q0, q1, xi;

  Map2DCHelper mhelper(*_map);
  chomp::ChompCollGradHelper cghelper(&mhelper, settings.chomp.gamma);

  vec2f p0(static_cast<float>(settings.environment->start().x),
           static_cast<float>(settings.environment->start().y));
  vec2f p1(static_cast<float>(settings.environment->goal().x),
           static_cast<float>(settings.environment->goal().y));
  if (p0.x() == p0.y() && p0 == p1) {
    p0 = _map->grid.bbox().p0.trunc();
    p1 = _map->grid.bbox().p1.trunc();
  }

  switch (settings.chomp.initialization) {
    case chomp::STRAIGHT_LINE:
      OMPL_DEBUG("Initializing CHOMP nodes using straight line...");
      _initializeStraightLine(settings.chomp.nodes, *_map, p0, p1, xi,
                              q0, q1);
      break;
    case chomp::THETA_STAR:
      OMPL_DEBUG("Initializing CHOMP nodes using Theta*...");
      _initializeThetaStar(settings.chomp.nodes, p0, p1, xi, q0, q1,
                           false);
      break;
    case chomp::THETA_STAR_X_CLEARING:
      OMPL_DEBUG(
          "Initializing CHOMP nodes using Theta* (with extra clearing)...");
      _initializeThetaStar(settings.chomp.nodes, p0, p1, xi, q0, q1,
                           true);
      break;
  }

  auto n = static_cast<int>(xi.rows());
  //    for (int i = -1; i <= n; ++i) {
  //        MatX pi;
  //        if (i < 0) {
  //            pi = q0;
  //        } else if (i >= (unsigned int) xi.rows()) {
  //            pi = q1;
  //        } else {
  //            pi = xi.row(i);
  //        }
  //        _path.emplace_back(Tpoint(pi(0), pi(1)));
  //    }
  //
  //    if (_path.size() != settings.chomp.nodes)
  //        OMPL_WARN("CHOMP terminated with %d nodes although %d nodes are
  //        requested.", _path.size(), settings.chomp.nodes);

  OMPL_DEBUG("Running CHOMP...");
  chomp::Chomp chomper(nullptr, xi, q0, q1, n, settings.chomp.alpha,
                       settings.chomp.error_tolerance,
                       settings.chomp.max_iterations,
                       settings.chomp.max_iterations);
  chomper.objective_type = settings.chomp.objective_type;
  chomper.ghelper = &cghelper;

  chomp::DebugChompObserver dobs;
  chomper.observer = &dobs;

  chomper.prepareChomp();
  chomper.prepareChompIter();

  _timer.start();
  chomper.solve(true, true);
  _timer.stop();

  for (int i = -1; i <= chomper.N; ++i) {
    MatX pi;
    if (i < 0) {
      pi = chomper.q0;
    } else if (i >= chomper.N) {
      pi = chomper.q1;
    } else {
      pi = chomper.xi.row(i);
    }
    _path.emplace_back(Point(pi(0), pi(1)));
  }

  OMPL_INFORM("CHOMP solution has %i nodes.", _path.size());

  //#if DEBUG
  //    QtVisualizer::drawPath(_path, Qt::blue);
  //    QtVisualizer::drawNodes(_path, Qt::blue, 0.2);
  //#endif

  return {!_path.empty(), false};
}

og::PathGeometric ChompPlanner::solution() const {
  og::PathGeometric path(settings.ompl.space_info);
  for (auto &p : _path) path.append(base::StateFromXYT(p.x, p.y, 0));
  return path;
}

std::vector<Point> ChompPlanner::solutionPath() const { return _path; }

bool ChompPlanner::hasReachedGoalExactly() const { return true; }

double ChompPlanner::planningTime() const { return _timer.elapsed(); }

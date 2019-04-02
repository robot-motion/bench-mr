#include "CHOMP.h"

#include <planners/thetastar/ThetaStar.h>
#include <utils/PlannerUtils.hpp>

Map2D *CHOMP::_map = new Map2D;

CHOMP::CHOMP() = default;

void initialize(const og::PathGeometric &path, unsigned int N, const vec2f &p0,
                const vec2f &p1, MatX &xi, MatX &q0, MatX &q1) {
  // TODO this implementation of CHOMP cannot handle yaw angles
  xi.resize(N, 2);
  q0.resize(1, 2);
  q1.resize(1, 2);

  og::PathGeometric p(path);
  q0 << p0.x(), p0.y();
  q1 << p1.x(), p1.y();

  p.interpolate(N + 2);
  assert(p.getStateCount() == N + 2);

#if defined(DEBUG) && defined(QT_SUPPORT)
  QtVisualizer::drawPath(path, Qt::black);
  QtVisualizer::drawNodes(path, Qt::black);
#endif
  OMPL_DEBUG("Samples: %d  --  N: %d", p.getStateCount(), N);
  xi.resize(p.getStateCount() - 2, 2);
  // Initialize with a straight line
  for (unsigned int i = 1; i < p.getStateCount() - 1; ++i) {
    const auto *state = p.getState(i)->as<State>();
    xi(i - 1, 0) = state->getX();
    xi(i - 1, 1) = state->getY();
  }
}

ob::PlannerStatus CHOMP::run(const og::PathGeometric &path) {
  _path.clear();
  OMPL_DEBUG("Computing CHOMP grid...");
  OMPL_DEBUG("Incoming path has %d nodes.", path.getStateCount());
  _map->grid.clear();
  const float csz = 1.f;
  Environment &env = *global::settings.environment;
  vec3f min(0, 0, 0);
  vec3f max(static_cast<float>(env.width()), static_cast<float>(env.height()),
            1.f);
  _map->grid.resize(min, max, DtGridf::AXIS_Z, csz);
  for (unsigned int y = 0; y < env.height(); ++y) {
    for (unsigned int x = 0; x < env.width(); ++x) {
      _map->grid(vec3u(x, y, 0)) =
          (env.collides((double)x, (double)y) ? -1 : 1);
    }
  }
  _map->grid.computeDistsFromBinary();
  _map->eps = global::settings.smoothing.chomp.epsilon;

  MatX q0, q1, xi;

  Map2DCHelper mhelper(*_map);
  chomp::ChompCollGradHelper cghelper(&mhelper,
                                      global::settings.smoothing.chomp.gamma);

  vec2f p0(static_cast<float>(global::settings.environment->start().x),
           static_cast<float>(global::settings.environment->start().y));
  vec2f p1(static_cast<float>(global::settings.environment->goal().x),
           static_cast<float>(global::settings.environment->goal().y));
  if (p0.x() == p0.y() && p0 == p1) {
    p0 = _map->grid.bbox().p0.trunc();
    p1 = _map->grid.bbox().p1.trunc();
  }

  OMPL_DEBUG("Initializing CHOMP nodes from trajectory...");
  initialize(path, global::settings.smoothing.chomp.nodes, p0, p1, xi, q0, q1);

  auto n = static_cast<int>(xi.rows());

  OMPL_DEBUG("Running CHOMP...");
  chomp::Chomp chomper(nullptr, xi, q0, q1, n,
                       global::settings.smoothing.chomp.alpha,
                       global::settings.smoothing.chomp.error_tolerance,
                       global::settings.smoothing.chomp.max_iterations,
                       global::settings.smoothing.chomp.max_iterations);
  chomper.objective_type = global::settings.smoothing.chomp.objective_type;
  chomper.ghelper = &cghelper;

#if DEBUG
  chomp::DebugChompObserver dobs;
  chomper.observer = &dobs;
#endif

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
    if (!pi.hasNaN()) _path.emplace_back(Point(pi(0), pi(1)));
  }

  OMPL_INFORM("CHOMP solution has %i nodes.", _path.size());

#if defined(DEBUG) && defined(QT_SUPPORT)
  QtVisualizer::drawPath(_path, Qt::blue);
  QtVisualizer::drawNodes(_path, Qt::blue, 0.2);
#endif

  return {!_path.empty(), false};
}

og::PathGeometric CHOMP::solution() const {
  og::PathGeometric path(global::settings.ompl.space_info);
  for (auto &p : _path) path.append(base::StateFromXYT(p.x, p.y, 0));
  return path;
}

std::vector<Point> CHOMP::solutionPath() const { return _path; }

double CHOMP::planningTime() const { return _timer.elapsed(); }

#include "GRIPS.h"
#include <base/PathStatistics.hpp>

#if QT_SUPPORT
#include "gui/QtVisualizer.h"
#endif

int GRIPS::insertedNodes = 0;
int GRIPS::pruningRounds = 0;
std::vector<int> GRIPS::nodesPerRound;
std::vector<GRIPS::RoundStats> GRIPS::statsPerRound;
GRIPS::RoundStats GRIPS::roundStats;
double GRIPS::smoothingTime = 0;
Stopwatch GRIPS::stopWatch;

bool GRIPS::smooth(ompl::geometric::PathGeometric &path,
                   const std::vector<Point> &originalPathIntermediaries) {
  const bool AverageAngles = true;

  insertedNodes = 0;
  pruningRounds = 0;
  nodesPerRound.clear();
  statsPerRound.clear();

  // register original path statistics
  beginRound(ROUND_ORIGINAL);
  endRound(path);

  smoothingTime = 0;
  stopWatch.start();

  PlannerUtils::updateAngles(path, AverageAngles);
#ifdef DEBUG
#if QT_SUPPORT
  QtVisualizer::drawTrajectory(path, Qt::lightGray);
  for (auto &o : originalPathIntermediaries)
    QtVisualizer::drawNode(o, Qt::darkGreen, 0.03);
  QtVisualizer::show();
  QtVisualizer::exec();
#endif
#endif

  double dx, dy;
  double eta =
      global::settings.smoothing.grips.eta;  // gradient descent step size
  for (int round = 0;
       round < global::settings.smoothing.grips.gradient_descent_rounds;
       ++round) {
    beginRound(ROUND_GD);
    OMPL_DEBUG("GRIPS: GD Round %d...", round + 1);
    // gradient descent along distance field
    for (auto i = 1u; i < path.getStateCount() - 1; ++i) {
      // compute gradient
      auto *s = path.getState(i)->as<State>();
      global::settings.environment->distanceGradient(s->getX(), s->getY(), dx,
                                                     dy, 1.);
      double distance =
          global::settings.environment->bilinearDistance(s->getX(), s->getY());
      distance = std::max(.1, distance);
      s->setX(s->getX() + eta * dx / distance);
      s->setY(s->getY() + eta * dy / distance);
    }
    eta *= global::settings.smoothing.grips.eta_discount;  // discount factor

    PlannerUtils::updateAngles(path, AverageAngles);

    // add/remove nodes if necessary
    auto tpath =
        PlannerUtils::toSteeredPoints(path.getState(0u), path.getState(1u));
    double lastDistance =
        global::settings.environment->bilinearDistance(tpath[0].x, tpath[0].y);
    double lastDistance2 =
        global::settings.environment->bilinearDistance(tpath[1].x, tpath[1].y);
    double lastDifference = lastDistance2 - lastDistance;
    ompl::geometric::PathGeometric npath(global::settings.ompl.space_info);
    Point lastNodePosition(tpath[0].x, tpath[0].y);

    for (auto i = 0u; i < path.getStateCount() - 1; ++i) {
      const auto *current = path.getState(i)->as<State>();
      const auto *next = path.getState(i + 1)->as<State>();

      lastNodePosition = Point(current);
      const Point nextNodePosition(next);

      npath.append(path.getState(i));

      tpath = PlannerUtils::toSteeredPoints(current, next);

      const double avg_theta =
          0.5 * (PlannerUtils::normalizeAngle(current->getYaw()) +
                 PlannerUtils::normalizeAngle(next->getYaw()));

      for (auto &p : tpath) {
        const double distance =
            global::settings.environment->bilinearDistance(p.x, p.y);
        const double difference = distance - lastDistance;
        if (lastDifference < 0 && difference > 0 &&
            lastNodePosition.distance(p.x, p.y) >=
                global::settings.smoothing.grips.min_node_distance &&
            nextNodePosition.distance(p.x, p.y) >=
                global::settings.smoothing.grips.min_node_distance) {
          // local minimum
          npath.append(p.toState(avg_theta));
          lastNodePosition = Point(p.x, p.y);

          ++insertedNodes;
#ifdef DEBUG
#if QT_SUPPORT
          QtVisualizer::drawNode(p.x, p.y, QColor(255, 150, 0, 180), 0.5);
#endif
#endif
        }
        lastDifference = difference;
        lastDistance = distance;
      }
    }
    npath.append(
        path.getState(static_cast<unsigned int>(path.getStateCount() - 1)));
    path = npath;

    PlannerUtils::updateAngles(path, AverageAngles);
#ifdef DEBUG
#if QT_SUPPORT
    QtVisualizer::drawTrajectory(path, Qt::blue, 0.2f);
    QtVisualizer::show();
    QtVisualizer::exec();
#endif
#endif

    endRound(path);
  }

  // try to remove nodes
  size_t lastPathLength;
  unsigned int pruningRound = 1;
  int fixes = 0;
  nodesPerRound.push_back((int)(path.getStateCount()));
  do {
    beginRound(ROUND_PRUNING);
    if (pruningRound >= global::settings.smoothing.grips.max_pruning_rounds) {
      OMPL_ERROR(
          "Giving up pruning after %i rounds. The smoothed trajectory most "
          "likely collides.",
          pruningRound);
      stopWatch.stop();
      smoothingTime = stopWatch.elapsed();
      return false;
    }

    lastPathLength = path.getStateCount();
    OMPL_DEBUG("GRIPS: Pruning Round  %i", pruningRound++);
    ++pruningRounds;

    fixes = 0;

    // determine irremovable nodes
    std::vector<unsigned int> irremovable;
    std::vector<unsigned int> local_irremovable{0};
    for (unsigned int i = 1; i < path.getStateCount() - 1; ++i) {
#if DEBUG
      std::cout << "PlannerUtils::collides(["
                << path.getState(i - 1)->as<State>()->getX() << " "
                << path.getState(i - 1)->as<State>()->getY() << "], ["
                << path.getState(i + 1)->as<State>()->getX() << " "
                << path.getState(i + 1)->as<State>()->getY() << "]) ? "
                << PlannerUtils::collides(path.getState(i - 1),
                                          path.getState(i + 1))
                << std::endl;
#endif
      if (PlannerUtils::collides(path.getState(local_irremovable.back()),
                                 path.getState(i + 1))) {
        local_irremovable.push_back(i);

#ifdef DEBUG
        OMPL_DEBUG("%i <--> %i WOULD COLLIDE (%.2f %.2f)", i - 1, i + 1,
                   path.getState(i)->as<State>()->getX(),
                   path.getState(i)->as<State>()->getY());
#endif
      }
    }
    local_irremovable.push_back((unsigned int)(path.getStateCount() - 1));
    irremovable = local_irremovable;

#ifdef DEBUG
    OMPL_DEBUG("GRIPS: we have %d irremovable nodes (before: %d).",
               irremovable.size(), path.getStateCount());

#if QT_SUPPORT
    for (auto i : irremovable) {
      QtVisualizer::drawNode(path.getState(i)->as<State>()->getX(),
                             path.getState(i)->as<State>()->getY(), Qt::darkRed,
                             0.4);
      OMPL_INFORM("irremovable %.2f %.2f",
                  path.getState(i)->as<State>()->getX(),
                  path.getState(i)->as<State>()->getY());
    }
#endif
#endif
    PlannerUtils::updateAngles(path, AverageAngles, true);
#ifdef DEBUG
#if QT_SUPPORT
    for (unsigned int i = 0; i < path.getStateCount(); ++i) {
      QtVisualizer::drawNode(path.getState(i), QColor(0, 0, 0, 100), 0.3,
                             false);
    }
#endif
#endif

    // compute final trajectory
    std::vector<ob::State *> finalPath;
    for (unsigned int ui = 1; ui < irremovable.size(); ++ui) {
      const auto i = irremovable[ui - 1];
      const auto j = irremovable[ui];

      if (finalPath.empty() ||
          !PlannerUtils::equals(path.getState(i), finalPath.back()))
        finalPath.emplace_back(path.getState(i));

      if (j - i <= 1) continue;  // no intermediary nodes

      std::vector<double> distances(j - i + 1,
                                    std::numeric_limits<double>::max());
      std::vector<unsigned int> predecessors(j - i + 1);

      for (unsigned int pi = 0; pi < predecessors.size(); ++pi)
        predecessors[pi] = pi == 0 ? 0 : pi - 1;

      distances[0] = 0;  // source weight is zero

      // run Bellman-Ford to determine best path from source (i) to sink (j)
      for (auto u = i; u <= j - 1; ++u) {
        for (auto v = u + 1; v <= j; ++v) {
          og::PathGeometric uv(global::settings.ompl.space_info);
          uv = PlannerUtils::interpolated(uv);
          if (PlannerUtils::collides(path.getState(u), path.getState(v), uv)) {
#ifdef DEBUG
#if QT_SUPPORT
            QtVisualizer::drawTrajectory(path.getState(u), path.getState(v),
                                         Qt::red, 3);
#endif
#endif
            continue;
          } else {
#ifdef DEBUG
#if QT_SUPPORT
            QtVisualizer::drawTrajectory(path.getState(u), path.getState(v),
                                         QColor(0, 150, 0, 100), 5);
#endif
#endif
          }

          const double edgeWeight = uv.length();
          if (distances[u - i] + edgeWeight < distances[v - i]) {
            distances[v - i] = distances[u - i] + edgeWeight;
            predecessors[v - i] = u - i;
          }
        }
      }

      unsigned int k = j - i;
      auto insertPosition = finalPath.size();
      while (k > 0) {
        if (!PlannerUtils::equals(path.getState(k + i), finalPath.back()))
          finalPath.insert(finalPath.begin() + insertPosition,
                           path.getState(k + i));
        if (k == predecessors[k]) {
          OMPL_ERROR("Failed to prune path due to loop in shortest path.");
          break;
        }
        k = predecessors[k];
      }
    }
    if (!PlannerUtils::equals(path.getStates().back(), finalPath.back()))
      finalPath.emplace_back(path.getStates().back());

    path.getStates() = finalPath;
    nodesPerRound.push_back((int)path.getStateCount());
    endRound(path);

    if (lastPathLength != path.getStateCount())
      OMPL_DEBUG(
          "Continuing pruning because lastPathLength (%i) != "
          "path.getStateCount() (%i)",
          (int)lastPathLength, (int)path.getStateCount());
    if (fixes > 0)
      OMPL_DEBUG("Continuing pruning because fixes (%i) > 0", fixes);
  } while (lastPathLength != path.getStateCount() || fixes > 0);

  stopWatch.stop();
  smoothingTime += stopWatch.elapsed();

#ifdef DEBUG
#if QT_SUPPORT
  QtVisualizer::drawTrajectory(path, QColor(200, 0, 180), 1.f);
  QtVisualizer::drawNodes(path, false, Qt::magenta, 0.2f);
  OMPL_INFORM("Path Length after GRIPS: %f", PathLengthMetric::evaluate(path));
#endif
#endif

  OMPL_INFORM("Post-smoothing SUCCEEDED after %i pruning rounds.",
              pruningRound);

  return true;
}

void GRIPS::beginRound(GRIPS::RoundType type) {
#ifdef STATS
  roundStats.stopWatch.start();
  roundStats.type = type;
#endif
}

void GRIPS::endRound(const ompl::geometric::PathGeometric &path) {
#ifdef STATS
  stopWatch.pause();
  static std::vector<double> nodeDistances, trajDistances;
  roundStats.stopWatch.stop();
  roundStats.time = roundStats.stopWatch.elapsed();
  roundStats.path_length = path.length();
  roundStats.max_curvature = CurvatureMetric::evaluate(path);
  roundStats.nodes = (int)path.getStateCount();
  nodeDistances.clear();
  for (auto i = 0u; i < path.getStateCount(); ++i)
    nodeDistances.push_back(global::settings.environment->bilinearDistance(
        Point(path.getState(i))));
  roundStats.median_node_obstacle_distance = stat::median(nodeDistances);
  roundStats.mean_node_obstacle_distance = stat::mean(nodeDistances);
  roundStats.min_node_obstacle_distance = stat::min(nodeDistances);
  roundStats.max_node_obstacle_distance = stat::max(nodeDistances);
  roundStats.std_node_obstacle_distance = stat::std(nodeDistances);
  trajDistances.clear();
  const auto interpolated = PlannerUtils::interpolated(path);
  for (auto i = 0u; i < interpolated.getStateCount(); ++i)
    trajDistances.push_back(global::settings.environment->bilinearDistance(
        Point(interpolated.getState(i))));
  roundStats.median_traj_obstacle_distance = stat::median(trajDistances);
  roundStats.mean_traj_obstacle_distance = stat::mean(trajDistances);
  roundStats.min_traj_obstacle_distance = stat::min(trajDistances);
  roundStats.max_traj_obstacle_distance = stat::max(trajDistances);
  roundStats.std_traj_obstacle_distance = stat::std(trajDistances);
  statsPerRound.push_back(roundStats);
  stopWatch.start();
#endif
}

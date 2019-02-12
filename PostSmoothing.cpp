#include "PostSmoothing.h"
#include <base/PathStatistics.hpp>

#if QT_SUPPORT
#include "gui/QtVisualizer.h"
#endif

//#define DEBUG 0
#undef DEBUG

int PostSmoothing::insertedNodes = 0;
int PostSmoothing::pruningRounds = 0;
int PostSmoothing::collisionFixAttempts = 0;
int PostSmoothing::roundsWithCollisionFixAttempts = 0;
std::vector<int> PostSmoothing::nodesPerRound;
std::vector<PostSmoothing::RoundStats> PostSmoothing::statsPerRound;
PostSmoothing::RoundStats PostSmoothing::roundStats;
double PostSmoothing::smoothingTime = 0;
Stopwatch PostSmoothing::stopWatch;

bool PostSmoothing::smooth(
    ompl::geometric::PathGeometric &path,
    const std::vector<Point> &originalPathIntermediaries) {
  const bool AverageAngles = true;

  insertedNodes = 0;
  pruningRounds = 0;
  collisionFixAttempts = 0;
  roundsWithCollisionFixAttempts = 0;
  nodesPerRound.clear();
  statsPerRound.clear();

  // register original path statistics
  beginRound(ROUND_ORIGINAL);
  endRound(path);

  smoothingTime = 0;
  stopWatch.start();

  PlannerUtils::updateAngles(path, AverageAngles);

  double dx, dy;
  double eta = PlannerSettings::gripsEta;  // gradient descent step size
  for (int round = 0; round < PlannerSettings::gripsGradientDescentRounds;
       ++round) {
    beginRound(ROUND_GD);
    // gradient descent along distance field
    for (auto i = 1u; i < path.getStateCount() - 1; ++i) {
      // compute gradient
      auto *s = path.getState(i)->as<State>();
      PlannerSettings::environment->distanceGradient(s->getX(), s->getY(), dx,
                                                     dy, 1.);
      double distance =
          PlannerSettings::environment->bilinearDistance(s->getX(), s->getY());
      distance = std::max(.1, distance);
      s->setX(s->getX() - eta * dx / distance);
      s->setY(s->getY() + eta * dy / distance);
    }
    eta *= PlannerSettings::gripsEtaDiscount;  // discount factor

    PlannerUtils::updateAngles(path, AverageAngles);

#ifdef DEBUG
#if QT_SUPPORT
    QtVisualizer::drawTrajectory(path, QColor(150, 150, 150, 200));
    QtVisualizer::drawNodes(path, false, QColor(70, 180, 250, 150), .2);
#endif
#endif

    // add/remove nodes if necessary
    auto tpath =
        PlannerUtils::toSteeredPoints(path.getState(0u), path.getState(1u));
    double lastDistance =
        PlannerSettings::environment->bilinearDistance(tpath[0].x, tpath[0].y);
    double lastDistance2 =
        PlannerSettings::environment->bilinearDistance(tpath[1].x, tpath[1].y);
    double lastDifference = lastDistance2 - lastDistance;
    ompl::geometric::PathGeometric npath(PlannerSettings::spaceInfo);
    Point lastNodePosition(tpath[0].x, tpath[0].y);

    for (auto i = 0u; i < path.getStateCount() - 1; ++i) {
      const auto *current = path.getState(i)->as<State>();
      const auto *next = path.getState(i + 1)->as<State>();

      lastNodePosition = Point(current);
      const Point nextNodePosition(next);

      npath.append(path.getState(i));

      tpath = PlannerUtils::toSteeredPoints(current, next);

      for (auto &p : tpath) {
        double distance =
            PlannerSettings::environment->bilinearDistance(p.x, p.y);
        double difference = distance - lastDistance;
        if (lastDifference < 0 && difference > 0 &&
            lastNodePosition.distance(p.x, p.y) >=
                PlannerSettings::gripsMinNodeDistance &&
            nextNodePosition.distance(p.x, p.y) >=
                PlannerSettings::gripsMinNodeDistance) {
          // local minimum
          npath.append(p.toState());
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
    endRound(path);
  }

#ifdef DEBUG
#if QT_SUPPORT
  for (auto &o : originalPathIntermediaries)
    QtVisualizer::drawNode(o, Qt::darkGreen, 0.1);
#endif
#endif

  // try to remove nodes
  size_t lastPathLength;
  unsigned int pruningRound = 1;
  int fixes = 0;
  nodesPerRound.push_back((int)(path.getStateCount()));
  do {
    beginRound(ROUND_PRUNING);
    if (pruningRound >= PlannerSettings::gripsMaxPruningRounds) {
      OMPL_ERROR(
          "Giving up pruning after %i rounds. The smoothed trajectory most "
          "likely collides.",
          pruningRound);
      stopWatch.stop();
      smoothingTime = stopWatch.time;
      return false;
    }

    lastPathLength = path.getStateCount();
    OMPL_DEBUG("#### PRUNING ROUND %i", pruningRound++);
    ++pruningRounds;

    fixes = 0;

    // determine unremovable nodes
    std::vector<unsigned int> unremovable;

    std::vector<unsigned int> local_unremovable{0};
    for (unsigned int i = 1; i < path.getStateCount() - 1; ++i) {
      if (PlannerUtils::collides(path.getState(i - 1), path.getState(i + 1))) {
        local_unremovable.push_back(i);

#ifdef DEBUG
        OMPL_DEBUG("%i <--> %i WOULD COLLIDE (%.2f %.2f)", i - 1, i + 1,
                   path.getState(i).x_r, path.getState(i).y_r);
#endif
      }
    }
    local_unremovable.push_back((unsigned int)(path.getStateCount() - 1));
    unremovable = local_unremovable;

#ifdef DEBUG
#if QT_SUPPORT
    for (auto i : unremovable) {
      QtVisualizer::drawNode(path.getState(i).x_r, path.getState(i).y_r,
                             Qt::darkRed, 0.4);
      OMPL_INFORM("UNREMOVABLE %.2f %.2f", path.getState(i).x_r,
                  path.getState(i).y_r);
    }
#endif
#endif

    PlannerUtils::updateAngles(path, AverageAngles, true);

#ifdef DEBUG
#if QT_SUPPORT
    for (unsigned int i = 0; i < path.getStateCount()(); ++i) {
      QtVisualizer::drawNode(path.getState(i), QColor(0, 0, 0, 100), 0.3,
                             false);
      //            QtVisualizer::drawLabel(std::to_string(i),
      //            path.getState(i).x_r + 0.2, path.getState(i).y_r + 0.2);
    }
#endif
#endif

    // compute final trajectory
    ompl::geometric::PathGeometric finalPath(PlannerSettings::spaceInfo);
    for (unsigned int ui = 1; ui < unremovable.size(); ++ui) {
      const auto i = unremovable[ui - 1];
      const auto j = unremovable[ui];

      if (finalPath.getStateCount() == 0 ||
          !PlannerUtils::equals(path.getState(i),
                                finalPath.getState(static_cast<unsigned int>(
                                    finalPath.getStateCount() - 1))))
        finalPath.append(path.getState(i));

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
          if (PlannerUtils::collides(path.getState(u), path.getState(v)))
            continue;  // a break has the same effect for linear steering and
                       // would be more efficient

          double edgeWeight =
              PathLengthMetric::evaluate(ompl::geometric::PathGeometric(
                  PlannerSettings::spaceInfo, path.getState(u),
                  path.getState(v)));

#ifdef DEBUG
#if QT_SUPPORT
//                    double dX = path.getState(v).x_r - path.getState(u).x_r;
//                    double dY = path.getState(v).y_r - path.getState(u).y_r;
//                    double rad = std::atan2(dY, dX);
//                    double plusMinus = v % 2 == 0 ? 1 : -1;
//                    double x = (path.getState(u).x_r + path.getState(v).x_r) *
//                    0.5 + plusMinus * std::pow(edgeWeight, .7)*std::cos(M_PI_2
//                    + rad); double y = (path.getState(u).y_r +
//                    path.getState(v).y_r) * 0.5 + plusMinus *
//                    std::pow(edgeWeight, .7)*std::sin(M_PI_2 + rad);
//                    QtVisualizer::drawPath(vector<Tpoint>({
//                                                                  Tpoint(path.getState(u).x_r,
//                                                                  path.getState(u).y_r),
//                                                                  Tpoint(x,
//                                                                  y),
//                                                                  Tpoint(path.getState(v).x_r,
//                                                                  path.getState(v).y_r)}),
//                                                                  Qt::black);
//                    QtVisualizer::drawLabel(std::to_string(edgeWeight), x-2,
//                    y, Qt::black);
#endif
#endif
          if (distances[u - i] + edgeWeight < distances[v - i]) {
            distances[v - i] = distances[u - i] + edgeWeight;
            predecessors[v - i] = u - i;
          }
        }
      }

      unsigned int k = j - i;
      auto insertPosition = finalPath.getStateCount();
      while (k > 0) {
        if (!PlannerUtils::equals(path.getState(k + i),
                                  finalPath.getState(static_cast<unsigned int>(
                                      finalPath.getStateCount() - 1))))
          finalPath.getStates().insert(
              finalPath.getStates().begin() + insertPosition,
              path.getState(k + i));
        if (k == predecessors[k]) {
          OMPL_ERROR("Failed to prune path due to loop in shortest path.");
          break;
        }
        k = predecessors[k];
      }
    }
    if (!PlannerUtils::equals(path.getStates().back(),
                              finalPath.getStates().back()))
      finalPath.append(path.getStates().back());

    path = finalPath;
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
  smoothingTime += stopWatch.time;

#ifdef DEBUG
#if QT_SUPPORT
  //    for (auto &n : path)
  //            QtVisualizer::drawNode(n, Qt::darkGreen, .1);

  //        QtVisualizer::drawTrajectory(path, QColor(200, 0, 180), 3.f);
  QtVisualizer::drawNodes(path, false, Qt::magenta, 0.2f);

  OMPL_INFORM("Path Length after our PS: %f", PathLengthMetric::evaluate(path));
//    OMPL_INFORM("Speed Arc Length: %f", SpeedArcLengthMetric::evaluate(path,
//    this, PlannerSettings::steering)); OMPL_INFORM("Peaks: %f",
//    PeaksMetric::evaluate(path, this, PlannerSettings::steering));
#endif
#endif
  OMPL_INFORM("Post-smoothing SUCCEEDED after %i pruning rounds.",
              pruningRound);

  return true;
}

void PostSmoothing::beginRound(PostSmoothing::RoundType type) {
#ifdef STATS
  roundStats.stopWatch.start();
  roundStats.type = type;
#endif
}

void PostSmoothing::endRound(const ompl::geometric::PathGeometric &path) {
#ifdef STATS
  stopWatch.pause();
  static std::vector<double> nodeDistances, trajDistances;
  roundStats.stopWatch.stop();
  roundStats.time = roundStats.stopWatch.time;
  roundStats.pathLength = path.length();
  roundStats.maxCurvature = CurvatureMetric::evaluate(path);
  roundStats.nodes = (int)path.getStateCount();
  nodeDistances.clear();
  for (auto i = 0u; i < path.getStateCount(); ++i)
    nodeDistances.push_back(PlannerSettings::environment->bilinearDistance(
        Point(path.getState(i))));
  roundStats.medianNodeObstacleDistance = stat::median(nodeDistances);
  roundStats.meanNodeObstacleDistance = stat::mean(nodeDistances);
  roundStats.minNodeObstacleDistance = stat::min(nodeDistances);
  roundStats.maxNodeObstacleDistance = stat::max(nodeDistances);
  roundStats.stdNodeObstacleDistance = stat::std(nodeDistances);
  trajDistances.clear();
  const auto interpolated = PlannerUtils::interpolated(path);
  for (auto i = 0u; i < interpolated.getStateCount(); ++i)
    trajDistances.push_back(PlannerSettings::environment->bilinearDistance(
        Point(interpolated.getState(i))));
  roundStats.medianTrajObstacleDistance = stat::median(trajDistances);
  roundStats.meanTrajObstacleDistance = stat::mean(trajDistances);
  roundStats.minTrajObstacleDistance = stat::min(trajDistances);
  roundStats.maxTrajObstacleDistance = stat::max(trajDistances);
  roundStats.stdTrajObstacleDistance = stat::std(trajDistances);
  statsPerRound.push_back(roundStats);
  stopWatch.resume();
#endif
}

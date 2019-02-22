#include "metrics/ClearingMetric.h"

#include "PathEvaluation.h"
#include "base/PlannerUtils.hpp"

double PathEvaluation::_maxPathLength = 0;
double PathEvaluation::_maxCurvature = 0;
double PathEvaluation::_maxTime = 0;

bool PathEvaluation::_smoothCollides = false;

double PathEvaluation::findMedian(const std::vector<double> &distances,
                                  size_t l, size_t r) {
  size_t count = r - l;
  if (count % 2 == 1) return distances[count / 2 + l];

  double right = distances[count / 2 + l];
  double left = distances[count / 2 - 1 + l];
  return (right + left) / 2.0;
}

std::vector<double> PathEvaluation::computeObstacleDistances(
    const std::vector<Point> &path) {
  std::vector<double> distances;
  for (unsigned int i = 0; i < path.size() - 1; ++i) {
    for (auto &point : PlannerUtils::linearInterpolate(path[i], path[i + 1])) {
      double distance = global::settings.environment->bilinearDistance(point);
      distances.push_back(distance);
    }
  }
  return distances;
}

void PathEvaluation::initialize() {
  _maxPathLength = 0;
  _maxCurvature = 0;
  _maxTime = 0;
  _smoothCollides = false;
}

//#if QT_SUPPORT
//PathStatistics PathEvaluation::add(AbstractPlanner *planner,
//                                   const std::string &label,
//                                   const QColor &color) {
//  //    OMPLPlanner<PLANNER> planner;
//  PathStatistics stats(label, color);
//#else
//PathStatistics PathEvaluation::add(AbstractPlanner *planner,
//                                   std::string label) {
//  //    OMPLPlanner<PLANNER> planner;
//  PathStatistics stats(label);
//#endif
//  if (!planner->run()) {
//    stats.pathFound = false;
//    OMPL_ERROR("Planner %s couldn't find a solution.", label.c_str());
//    return stats;
//  }
//  stats.pathFound = true;
//  stats.exactGoalPath = planner->hasReachedGoalExactly();
//  std::vector<Point> path = planner->solutionPath();
//  auto trajectory = planner->solution();
//
//  double planningTime = planner->planningTime();
//  stats.planningTime = planningTime;
//  //    auto begin_time = ros::WallTime::now().toSec();
//  ompl::geometric::PathGeometric smoothed(trajectory);
//  //    OMPL_INFORM("Running our smoothing method on %s ...", label.c_str());
//  //    PostSmoothing::smooth(smoothed, path);
//  //    auto postsmoothing_time = planningTime + PostSmoothing::smoothingTime;
//  //    auto smoothedTrajPoints =
//  //    PlannerUtils::toSteeredPoints(smoothed); auto *smoothedTraj =
//  //    new Trajectory(smoothedTrajPoints); double smoothedPathLength =
//  //    PathLengthMetric::evaluate(smoothedTraj); double smoothedCurvature =
//  //    CurvatureMetric::evaluateMetric(smoothedTraj, 0, false);
//  //        QtVisualizer::drawNodes(smoothedTrajPoints, QColor(255, 150, 0),
//  //        .03f); QtVisualizer::drawNodes(smoothed, false, Qt::red, .03f);
//  //        QtVisualizer::drawNodes(smoothed, false, Qt::red, .03f);
//
//  //  QPen oPen(color, 1.5);
//  //  QtVisualizer::addLegendEntry(LegendEntry(label, oPen));
//  //        QtVisualizer::drawNodes(path, color, .03f);
//  //  QtVisualizer::drawPath(path, oPen);
//
//  //    QPen ourPen(color, 1.5f);
//  //
//  //    auto *lines = new QLineSeries;
//  //    lines->setColor(color);
//  //    lines->setName(QString::fromStdString(label));
//  //    lines->setPen(oPen);
//  //
//  //    _labels << QString::fromStdString(label);
//
//  //    std::vector<double> distances = computeObstacleDistances(path);
//  //    stats.pathCollides = false;
//  //    for (unsigned int i = 0; i < distances.size(); ++i)
//  //    {
//  //        lines->append((i + 1.f) / distances.size(), distances[i]);
//  //        if (distances[i] <= 0.0f)
//  //            stats.pathCollides = true;
//  //    }
//  //
//  //    _distances->addSeries(lines);
//  //
//  //    auto *smoothedLines = new QLineSeries;
//  //    smoothedLines->setName(QString::fromStdString(label) + " smoothed");
//  //    smoothedLines->setColor(color);
//  //
//  //    std::vector<double> smoothedDistances =
//  //    computeObstacleDistances(smoothedTrajPoints);
//  stats.ourSmoothingCollides = false;
//  //    for (unsigned int i = 0; i < smoothedDistances.size(); ++i)
//  //    {
//  //        smoothedLines->append((i + 1.f) / smoothedDistances.size(),
//  //        smoothedDistances[i]); if (smoothedDistances[i] <= 0.f)
//  //        {
//  //            _smoothCollides = true;
//  //            stats.ourSmoothingCollides = true;
//  //        }
//  //    }
//  //    stats.ourSmoothingCollides = PlannerUtils::collides(smoothedTrajPoints);
//  //    if (stats.ourSmoothingCollides)
//  //        OMPL_WARN("%s collides with our smoothing!", label.c_str());
//  //
//  //    if (stats.exactGoalPath && !stats.ourSmoothingCollides)
//  //    {
//  //        stats.ourSmoothingTime = postsmoothing_time;
//  //        stats.ourSmoothingPathLength = smoothedPathLength;
//  //        stats.ourSmoothingCurvature = smoothedCurvature;
//  //        QtVisualizer::addLegendEntry(LegendEntry(label + " - Our Smoothing",
//  //        ourPen)); QtVisualizer::drawPath(smoothedTrajPoints, ourPen);
//  //    }
//  //
//  //    _distances->addSeries(smoothedLines);
//  //
//  //    QBoxSet *box = computeBox(distances, label);
//  //    box->setBrush(QBrush(color, Qt::BrushStyle::BDiagPattern));
//  //    _boxSeries->append(box);
//  //
//  //    QBoxSet *boxSmoothed = computeBox(smoothedDistances, label + "
//  //    smoothed"); boxSmoothed->setBrush(QBrush(color));
//  //    _boxSeries->append(boxSmoothed);
//
//  double pathLength = PathLengthMetric::evaluate(trajectory);
//  double curvature = CurvatureMetric::evaluate(trajectory);
//
//  if (pathLength > _maxPathLength) _maxPathLength = pathLength;
//  if (curvature > _maxCurvature) _maxCurvature = curvature;
//  //    if (postsmoothing_time > _maxTime)
//  //        _maxTime = postsmoothing_time;
//
//  //    QBarSet *pathLengthBarSet = new QBarSet(QString::fromStdString(label));
//  //    pathLengthBarSet->setColor(color);
//  //    *pathLengthBarSet << pathLength;
//  //    *pathLengthBarSet << smoothedPathLength;
//  //    std::cout << "Path length of " << label << " path: "
//  //              << pathLength << std::endl;
//  //    _pathLengthSeries->append(pathLengthBarSet);
//
//  stats.curvature = curvature;
//  stats.pathLength = pathLength;
//
//  //    delete smoothedTraj;
//  return stats;
//
//  //  OMPL_INFORM("Running B-Spline smoothing method on %s...", label.c_str());
//  //  auto smoothed1 = planner->smoothBSpline();
//  //  if (!smoothed1.trajectory.empty()) {
//  //    auto omplSmoothedTraj = new Trajectory(smoothed1.trajectory);
//  //    stats.omplSmoothing1Collides =
//  //        global::settings.environment->collides(*omplSmoothedTraj);
//  //    if (stats.omplSmoothing1Collides)
//  //      OMPL_WARN("%s collides with smoothed B-spline", label.c_str());
//  //    else if (stats.exactGoalPath) {
//  //      stats.omplSmoothing1Time = planningTime + smoothed1.time;
//  //      stats.omplSmoothing1PathLength =
//  //          PathLengthMetric::evaluate(omplSmoothedTraj);
//  //      stats.omplSmoothing1Curvature =
//  //          CurvatureMetric::evaluateMetric(omplSmoothedTraj, 0, false);
//  //      if (stats.omplSmoothing1PathLength > _maxPathLength)
//  //        _maxPathLength = stats.omplSmoothing1PathLength;
//  //      if (stats.omplSmoothing1Curvature > _maxCurvature)
//  //        _maxCurvature = stats.omplSmoothing1Curvature;
//  //      if (stats.omplSmoothing1Time > _maxTime)
//  //        _maxTime = stats.omplSmoothing1Time;
//  //#if QT_SUPPORT
//  //      if (global::settings.VisualizeSmoothing1) {
//  //        int h, s, l, a;
//  //        color.getHsl(&h, &s, &l, &a);
//  //        QPen sPen(QColor::fromHsl((h - 40 + 360) % 360, s, l * 0.8,
//  //        a), 1.5f,
//  //                  Qt::PenStyle::DashLine);
//  //        QtVisualizer::addLegendEntry(LegendEntry(label + " - B-spline",
//  //        sPen));
//  //        //            QtVisualizer::drawNodes(smoothed1.trajectory,
//  //        //            QColor::fromHsl((h-40+255)%255, s, l, a), .03);
//  //        QtVisualizer::drawPath(smoothed1.trajectory, sPen);
//  //      }
//  //#endif
//  //    }
//  //    delete omplSmoothedTraj;
//  //  }
//  //
//  //  OMPL_INFORM("Running SimplifyMax smoothing method on %s...",
//  //  label.c_str()); auto smoothed2 = planner->simplifyMax(); if
//  //  (!smoothed2.trajectory.empty()) {
//  //    auto omplSmoothedTraj = new Trajectory(smoothed2.trajectory);
//  //    stats.omplSmoothing2Collides =
//  //        global::settings.environment->collides(*omplSmoothedTraj);
//  //    if (stats.omplSmoothing2Collides)
//  //      OMPL_WARN("%s collides with simplify max", label.c_str());
//  //    else if (stats.exactGoalPath) {
//  //      stats.omplSmoothing2Time = planningTime + smoothed2.time;
//  //      stats.omplSmoothing2PathLength =
//  //          PathLengthMetric::evaluate(omplSmoothedTraj);
//  //      stats.omplSmoothing2Curvature =
//  //          CurvatureMetric::evaluateMetric(omplSmoothedTraj, 0, false);
//  //      if (stats.omplSmoothing2PathLength > _maxPathLength)
//  //        _maxPathLength = stats.omplSmoothing2PathLength;
//  //      if (stats.omplSmoothing2Curvature > _maxCurvature)
//  //        _maxCurvature = stats.omplSmoothing2Curvature;
//  //      if (stats.omplSmoothing2Time > _maxTime)
//  //        _maxTime = stats.omplSmoothing2Time;
//  //#if QT_SUPPORT
//  //      if (global::settings.VisualizeSmoothing2) {
//  //        int h, s, l, a;
//  //        color.getHsl(&h, &s, &l, &a);
//  //
//  //        QPen sPen(QColor::fromHsl((h - 40 + 360) % 360, s, l), 1.5f,
//  //                  Qt::PenStyle::DashDotLine);
//  //        QtVisualizer::addLegendEntry(
//  //            LegendEntry(label + " - SimplifyMax", sPen));
//  //        //            QtVisualizer::drawNodes(smoothed1.trajectory,
//  //        //            QColor::fromHsl((h-40+255)%255, s, l, a), .03);
//  //        QtVisualizer::drawPath(smoothed2.trajectory, sPen);
//  //        //            QtVisualizer::drawNodes(smoothed2.trajectory, Qt::red,
//  //        //            0.3f); //1.5f, Qt::PenStyle::DashDotLine);
//  //
//  //        //                auto geoPath = planner->geometricPath();
//  //        //                for (auto *state : geoPath.getStates())
//  //        //                {
//  //        ////            const auto *state2D =
//  //        /// state->as<ob::RealVectorStateSpace::StateType>(); / //
//  //        /// Extract the robot's (x,y) position from its state / double x =
//  //        /// state2D->values[0]; /            double y = state2D->values[1];
//  //        //                    const auto *s =
//  //        //                    state->as<ob::SE2StateSpace::StateType>();
//  //        double
//  //        //                    x=s->getX(), y=s->getY();
//  //        //                    QtVisualizer::drawNode(x, y, Qt::darkGreen);
//  //        //                }
//  //      }
//  //#endif
//  //    }
//  //    delete omplSmoothedTraj;
//  //  }
//  //
//  //  OMPL_INFORM("Running shortcut smoothing method on %s...", label.c_str());
//  //  auto smoothed3 = planner->shortcutPath();
//  //  if (!smoothed3.trajectory.empty()) {
//  //    auto omplSmoothedTraj = new Trajectory(smoothed3.trajectory);
//  //    stats.omplSmoothing3Collides =
//  //        global::settings.environment->collides(*omplSmoothedTraj);
//  //    if (stats.omplSmoothing3Collides)
//  //      OMPL_WARN("%s collides with shortcut", label.c_str());
//  //    else if (stats.exactGoalPath) {
//  //      stats.omplSmoothing3Time = planningTime + smoothed3.time;
//  //      stats.omplSmoothing3PathLength =
//  //          PathLengthMetric::evaluate(omplSmoothedTraj);
//  //      stats.omplSmoothing3Curvature =
//  //          CurvatureMetric::evaluateMetric(omplSmoothedTraj, 0, false);
//  //      if (stats.omplSmoothing3PathLength > _maxPathLength)
//  //        _maxPathLength = stats.omplSmoothing3PathLength;
//  //      if (stats.omplSmoothing3Curvature > _maxCurvature)
//  //        _maxCurvature = stats.omplSmoothing3Curvature;
//  //      if (stats.omplSmoothing3Time > _maxTime)
//  //        _maxTime = stats.omplSmoothing3Time;
//  //#if QT_SUPPORT
//  //      if (global::settings.VisualizeSmoothing3) {
//  //        int h, s, l, a;
//  //        color.getHsl(&h, &s, &l, &a);
//  //        QPen sPen(QColor::fromHsl((h + 30) % 360, s, l * 0.8), 1.5f,
//  //                  Qt::PenStyle::DashDotDotLine);
//  //        QtVisualizer::addLegendEntry(
//  //            LegendEntry(label + " - ShortcutPath", sPen));
//  //        QtVisualizer::drawPath(smoothed3.trajectory, sPen);
//  //        //            QtVisualizer::drawNodes(smoothed3.trajectory,
//  //        //            QColor::fromHsl((h+30+255)%255, s, l, a), 0.03f);
//  //        //1.5f,
//  //        //            Qt::PenStyle::DashDotLine);
//  //      }
//  //#endif
//  //    }
//  //    delete omplSmoothedTraj;
//  //  }
//  //
//  //  OMPL_INFORM("Running Anytime Path Shortening on %s...", label.c_str());
//  //  auto smoothed4 = planner->anytimePathShortening();
//  //  if (!smoothed4.trajectory.empty()) {
//  //    stats.omplSmoothing4Found = true;
//  //    stats.exactGoalSmoothedPath =
//  //        (smoothed4.status == ob::PlannerStatus::EXACT_SOLUTION);
//  //    auto omplSmoothedTraj = new Trajectory(smoothed4.trajectory);
//  //    stats.omplSmoothing4Collides =
//  //        global::settings.environment->collides(*omplSmoothedTraj);
//  //    if (stats.omplSmoothing4Collides)
//  //      OMPL_WARN("%s collides with Anytime PS", label.c_str());
//  //    else if (stats.exactGoalSmoothedPath) {
//  //      stats.omplSmoothing4Time = planningTime + smoothed4.time;
//  //      stats.omplSmoothing4PathLength =
//  //          PathLengthMetric::evaluate(omplSmoothedTraj);
//  //      stats.omplSmoothing4Curvature =
//  //          CurvatureMetric::evaluateMetric(omplSmoothedTraj, 0, false);
//  //      if (stats.omplSmoothing4PathLength > _maxPathLength)
//  //        _maxPathLength = stats.omplSmoothing4PathLength;
//  //      if (stats.omplSmoothing4Curvature > _maxCurvature)
//  //        _maxCurvature = stats.omplSmoothing4Curvature;
//  //      if (stats.omplSmoothing4Time > _maxTime)
//  //        _maxTime = stats.omplSmoothing4Time;
//  //#if QT_SUPPORT
//  //      if (global::settings.VisualizeSmoothing4) {
//  //        int h, s, l, a;
//  //        color.getHsl(&h, &s, &l, &a);
//  //        QPen sPen(QColor::fromHsl((h - 40 + 360) % 360, s, l * 0.8), 1.5f,
//  //                  Qt::PenStyle::DashDotDotLine);
//  //        QtVisualizer::addLegendEntry(LegendEntry(label + " - AnytimePS",
//  //        sPen)); QtVisualizer::drawPath(smoothed4.trajectory, sPen);
//  //        //            QtVisualizer::drawNodes(smoothed4.trajectory,
//  //        //            QColor::fromHsl((h+40+255)%255, s, l, a), 0.04f);
//  //        //1.5f,
//  //        //            Qt::PenStyle::DashDotLine);
//  //      }
//  //#endif
//  //    }
//  //    delete omplSmoothedTraj;
//  //  } else
//  //    stats.omplSmoothing4Found = false;
//  //
//  //#if QT_SUPPORT
//  //  QtVisualizer::drawStats(stats);
//  //#endif
//  //
//  //  //    delete smoothedTraj;
//  //  delete traj;
//  //  return stats;
//}

#if QT_SUPPORT
PathStatistics PathEvaluation::evaluate(
    const ompl::geometric::PathGeometric &path, const std::string &label,
    const QColor &color) {
  PathStatistics stats(label, color);
#else
PathStatistics PathEvaluation::evaluate(PathStatistics &stats,
    const ompl::geometric::PathGeometric &path, const AbstractPlanner *planner) {
#endif
  stats.path_length = PathLengthMetric::evaluate(path);
  stats.curvature = CurvatureMetric::evaluate(path);
  stats.smoothness = path.smoothness();

  if (global::settings.evaluate_clearing) {
    auto clearings = ClearingMetric::clearingDistances(path);
    stats.mean_clearing_distance = stat::mean(clearings);
    stats.median_clearing_distance = stat::median(clearings);
    stats.min_clearing_distance = stat::min(clearings);
    stats.max_clearing_distance = stat::max(clearings);
  }

#if QT_SUPPORT
  QPen oPen(color, 1.5);
  QtVisualizer::addLegendEntry(LegendEntry(label, oPen));
  QtVisualizer::drawPath(Point::fromPath(path), oPen);
#endif

  return stats;
}

#pragma once

#include <cmath>

#include "PlannerSettings.h"
#include "Primitives.h"

#if QT_SUPPORT
#include "gui/QtVisualizer.h"
#endif

namespace ob = ompl::base;
namespace og = ompl::geometric;

class PlannerUtils {
 public:
  static double slope(double x1, double y1, double x2, double y2) {
    double dy = y2 - y1;
    double dx = x2 - x1;
    double x = std::atan2(dy, dx);
    return x;
  }

  static double slope(const Point &a, const Point &b) {
    double dy = b.y - a.y;
    double dx = b.x - a.x;
    double x = std::atan2(dy, dx);
    return x;
  }

  static double slope(const ompl::base::State *a, const ompl::base::State *b) {
    double dy = b->as<State>()->getY() - a->as<State>()->getY();
    double dx = b->as<State>()->getX() - a->as<State>()->getX();
    double x = std::atan2(dy, dx);
    return x;
  }

  static bool equals(const ompl::base::State *a, const ompl::base::State *b) {
    double dy = b->as<State>()->getY() - a->as<State>()->getY();
    double dx = b->as<State>()->getX() - a->as<State>()->getX();
    double dt = b->as<State>()->getYaw() - a->as<State>()->getYaw();
    return std::abs(dx) <= settings.ompl.state_equality_tolerance &&
           std::abs(dy) <= settings.ompl.state_equality_tolerance &&
           std::abs(dt) <= settings.ompl.state_equality_tolerance;
  }

  static std::vector<Point> toSteeredPoints(const ob::State *a,
                                            const ob::State *b) {
    return Point::fromPath(og::PathGeometric(settings.ompl.space_info, a, b));
  }

  static bool collides(const std::vector<Point> &path) {
    for (unsigned int i = 0; i < path.size(); ++i) {
      if (settings.environment->occupied(path[i].x, path[i].y)) {
#ifdef DEBUG
        // QtVisualizer::drawPath(path, QColor(255, 100, 0, 170));
#endif
        return true;
      }

      // check intermediary points
      if (i < path.size() - 1) {
        double dx = (path[i + 1].x - path[i].x);
        double dy = (path[i + 1].y - path[i].y);
        double size = std::sqrt(dx * dx + dy * dy);
        const double scale = 1.5;
        dx = dx / size * scale;
        dy = dy / size * scale;

        auto steps = (int)(size / std::sqrt(dx * dx + dy * dy));

        for (int j = 1; j <= steps; ++j) {
          if (settings.environment->occupied(path[i].x + dx * j,
                                                     path[i].y + dy * j))
          //  || settings.environment->occupied(path[i].x + dx * j + .5,
          //  path[i].y + dy * j
          //  + .5))
          {
#if QT_SUPPORT
//                        QtVisualizer::drawNode(path[i].x + dx * j, path[i].y +
//                        dy * j,
//                                               QColor(255*.8, 255*0, 255*.9),
//                                               0.3);
#ifdef DEBUG
            // QtVisualizer::drawPath(path, QColor(250, 0, 0, 70));
#endif
#endif
            return true;
          }
        }
      }
    }
#if QT_SUPPORT
#ifdef DEBUG
    // QtVisualizer::drawPath(path, QColor(70, 150, 0, 120));
#endif
#endif
    return false;
  }

  static bool collides(const ompl::base::State *a, const ompl::base::State *b) {
    ompl::geometric::PathGeometric p(settings.ompl.space_info, a, b);
    p.interpolate();
    const auto path = Point::fromPath(p);
    return collides(path);
  }

  static bool collides(const std::vector<Point> &path,
                       std::vector<Point> &collisions) {
    collisions.clear();
    for (unsigned int i = 1; i < path.size(); ++i) {
      if (settings.environment->occupied(path[i].x, path[i].y)) {
#if QT_SUPPORT
#ifdef DEBUG
        // QtVisualizer::drawPath(path, QColor(255, 100, 0, 170));
#endif
#endif
        collisions.emplace_back(path[i]);
        continue;
      }

      // check intermediary points
      if (i < path.size() - 1) {
        double dx = (path[i + 1].x - path[i].x);
        double dy = (path[i + 1].y - path[i].y);
        double size = std::sqrt(dx * dx + dy * dy);
        const double scale = 0.15;  // 1.5;
        dx = dx / size * scale;
        dy = dy / size * scale;

        auto steps = (int)(size / std::sqrt(dx * dx + dy * dy));

        for (int j = 1; j <= steps; ++j) {
          if (settings.environment->occupied(path[i].x + dx * j,
                                                     path[i].y + dy * j))
          //  || settings.environment->occupied(path[i].x + dx * j + .5,
          //  path[i].y + dy * j
          //  + .5))
          {
#if QT_SUPPORT
//                        QtVisualizer::drawNode(path[i].x + dx * j, path[i].y +
//                        dy * j,
//                                               QColor(255*.8, 255*0, 255*.9),
//                                               0.3);
#ifdef DEBUG
            // QtVisualizer::drawPath(path, QColor(250, 0, 0, 70));
#endif
#endif
            collisions.emplace_back(path[j]);
            continue;
          }
        }
      }
    }
#ifdef DEBUG
    // QtVisualizer::drawPath(path, QColor(150, 200, 0, 70));
#endif
    return !collisions.empty();
  }

  static bool collides(const ompl::base::State *a, const ompl::base::State *b,
                       std::vector<Point> &collisions) {
    ompl::geometric::PathGeometric p(settings.ompl.space_info, a, b);
    p.interpolate();
    const auto path = Point::fromPath(p);
    return collides(path, collisions);
  }

  static ompl::geometric::PathGeometric interpolated(
      ompl::geometric::PathGeometric path) {
    path.interpolate();
    return path;
  }

  static void updateAngles(ompl::geometric::PathGeometric &path,
                           bool AverageAngles = true,
                           bool preventCollisions = true) {
    if (path.getStateCount() < 2) return;

    double theta_old = path.getStates()[0]->as<State>()->getYaw();
    path.getStates()[0]->as<State>()->setYaw(
        slope(path.getStates()[0], path.getStates()[1]));
    if (preventCollisions && collides(path.getStates()[0], path.getStates()[1]))
      path.getStates()[0]->as<State>()->setYaw(theta_old);  // revert setting
    for (int i = 1; i < path.getStateCount() - 1; ++i) {
      theta_old = path.getStates()[i]->as<State>()->getYaw();
      if (AverageAngles) {
        double l = slope(path.getStates()[i - 1], path.getStates()[i]);
        double r = slope(path.getStates()[i], path.getStates()[i + 1]);
        if (std::abs(l - r) >= M_PI) {
          if (l > r)
            l += 2. * M_PI;
          else
            r += 2. * M_PI;
        }
        path.getStates()[i]->as<State>()->setYaw((l + r) * 0.5);
      } else
        path.getStates()[i]->as<State>()->setYaw(
            slope(path.getStates()[i - 1], path.getStates()[i]));

      if (preventCollisions &&
          (collides(path.getStates()[i - 1], path.getStates()[i]) ||
           collides(path.getStates()[i], path.getStates()[i + 1])))
        path.getStates()[i]->as<State>()->setYaw(theta_old);  // revert setting
    }
    theta_old =
        path.getStates()[path.getStateCount() - 1]->as<State>()->getYaw();
    path.getStates()[path.getStateCount() - 1]->as<State>()->setYaw(
        slope(path.getStates()[path.getStateCount() - 2],
              path.getStates()[path.getStateCount() - 1]));
    if (preventCollisions &&
        collides(path.getStates()[path.getStateCount() - 1],
                 path.getStates()[path.getStateCount() - 2]))
      path.getStates()[path.getStateCount() - 1]->as<State>()->setYaw(
          theta_old);  // revert setting
  }

  static void gradientDescent(ompl::geometric::PathGeometric &path,
                              unsigned int rounds, double eta,
                              double discount = 1.) {
    double dx, dy;
    for (int round = 0; round < rounds; ++round) {
      // gradient descent along distance field, excluding start/end nodes
      for (int i = 1; i < path.getStateCount() - 1; ++i) {
        // compute gradient
        settings.environment->distanceGradient(
            path.getStates()[i]->as<State>()->getX(),
            path.getStates()[i]->as<State>()->getY(), dx, dy, 1.);
        double distance = settings.environment->bilinearDistance(
            path.getStates()[i]->as<State>()->getX(),
            path.getStates()[i]->as<State>()->getY());
        distance = std::max(.1, distance);
        path.getStates()[i]->as<State>()->setX(
            path.getStates()[i]->as<State>()->getX() - eta * dx / distance);
        path.getStates()[i]->as<State>()->setY(
            path.getStates()[i]->as<State>()->getY() + eta * dy / distance);
      }
      eta *= discount;
    }
  }

  static void gradientDescent(std::vector<Point> &path, unsigned int rounds,
                              double eta, double discount = 1.) {
    double dx, dy;
    for (int round = 0; round < rounds; ++round) {
      // gradient descent along distance field, excluding start/end nodes
      for (int i = 1; i < path.size() - 1; ++i) {
        // compute gradient
        settings.environment->distanceGradient(path[i].x, path[i].y, dx,
                                                       dy, 1.);
        double distance = settings.environment->bilinearDistance(
            path[i].x, path[i].y);
        distance = std::max(.1, distance);
        path[i].x -= eta * dx / distance;
        path[i].y += eta * dy / distance;
      }
      eta *= discount;
    }
  }

  static std::vector<Point> linearInterpolate(const Point &a, const Point &b,
                                              double dt = 0.1) {
    std::vector<Point> points;
    double dx = (b.x - a.x);
    double dy = (b.y - a.y);
    double size = std::sqrt(dx * dx + dy * dy);
    if (size == 0) return std::vector<Point>{a};
    dx = dx / size * dt;
    dy = dy / size * dt;
    auto steps = (int)(size / std::sqrt(dx * dx + dy * dy));

    for (int j = 1; j <= steps; ++j)
      points.emplace_back(a.x + dx * j, a.y + dy * j);

    if (points.empty()) points.emplace_back(a);

    return points;
  }

  static std::vector<Point> linearInterpolate(ompl::base::State *a,
                                              ompl::base::State *b,
                                              double dt = 0.1) {
    return linearInterpolate(
        Point(a->as<State>()->getX(), a->as<State>()->getY()),
        Point(b->as<State>()->getX(), b->as<State>()->getY()), dt);
  }

  static ompl::base::State *closestPoint(Point x,
                                         const std::vector<Point> &points) {
    if (points.size() == 1) return points[0].toState();
    unsigned int closest = 0;
    double dist = points[closest].distanceSquared(x);
    for (unsigned int i = 1; i < points.size() - 1; ++i) {
      if (settings.environment->occupied(points[i].x, points[i].y))
        continue;
      const double d = points[i].distanceSquared(x);
      if (d < dist) {
        dist = d;
        closest = i;
      }
    }
    double theta;
    if (closest == 0)
      theta = slope(points[0], points[1]);
    else
      theta = slope(points[closest - 1], points[closest + 1]);
    return points[closest].toState(theta);
  }

  static double totalLength(const std::vector<Point> &path) {
    double l = 0;
    for (size_t i = 1; i < path.size(); ++i) {
      const double dx = (path[i].x - path[i - 1].x);
      const double dy = (path[i].y - path[i - 1].y);
      l += std::sqrt(dx * dx + dy * dy);
    }
    return l;
  }

  static std::vector<Point> equidistantSampling(const std::vector<Point> &path,
                                                size_t targetSize) {
    std::vector<Point> result;
    const double total = totalLength(path);
    const double targetSegment = total / targetSize;
    result.emplace_back(path[0]);
    double sofar = 0;
    double segment = 0;
    size_t resultCounter = 1;
    for (size_t i = 1; i < path.size(); ++i) {
      double dx = (path[i].x - path[i - 1].x);
      double dy = (path[i].y - path[i - 1].y);
      const double l = std::sqrt(dx * dx + dy * dy);
      if (std::abs(l) < 1e-3) continue;
      dx /= l;
      dy /= l;
      if (segment + l < targetSegment) {
        segment += l;
        OMPL_DEBUG("EquidistantSampling: Segment too short between %i and %i",
                   i - 1, i);
        continue;
      }
      double start = std::fmod(segment + l, targetSegment) - segment;
      segment = 0;
      const auto segmentSteps =
          static_cast<unsigned int>(std::floor((segment + l) / targetSegment));
      for (unsigned int j = 0; j < segmentSteps; ++j) {
        const double alpha = start + j * targetSegment;
        const Point p(path[i - 1].x + dx * alpha, path[i - 1].y + dy * alpha);
        result.emplace_back(p);
      }
      sofar += segmentSteps * targetSegment;
      segment = l - segmentSteps * targetSegment;
    }
    result.emplace_back(path.back());
    return result;
  }
};

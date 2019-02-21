#pragma once

#include "Environment.h"
#include "utils/SvgPolygonLoader.hpp"

#include <collision2d/sat.hpp>

/**
 * Implements a maze consisting of convex shapes as obstacles.
 */
class PolygonMaze : public Environment {
 public:
  std::string name() const override { return _name; }

  const std::vector<Polygon> &obstacles() const { return _obstacles; }

  static PolygonMaze loadFromSvg(const std::string &filename) {
    PolygonMaze maze;
    maze._name += " " + filename;
    maze._obstacles = SvgPolygonLoader::load(filename);
    if (maze._obstacles.empty()) {
      OMPL_ERROR(
          ("Could not find any obstacles in \"" + filename + "\".").c_str());
      return maze;
    }
    auto min = maze._obstacles[0].min();
    auto max = maze._obstacles[0].max();
    for (const auto &o : maze._obstacles) {
      const auto new_min = o.min();
      const auto new_max = o.max();
      if (new_min.x < min.x) min.x = new_min.x;
      if (new_max.x > max.x) max.x = new_max.x;
      if (new_min.y < min.y) min.y = new_min.y;
      if (new_max.y > max.y) max.y = new_max.y;
    }
    maze._bounds.setLow(0, min.x);
    maze._bounds.setLow(1, min.y);
    maze._bounds.setHigh(0, max.x);
    maze._bounds.setHigh(1, max.y);
    OMPL_INFORM(("Loaded polygon maze from \"" + filename + "\".").c_str());
    OMPL_INFORM("\tBounds:  [%.2f %.2f] -- [%.2f %.2f]", min.x, min.y, max.x,
                max.y);
    return maze;
  }

  bool collides(double x, double y) override {
    int i = 0;
    for (const auto &poly : _obstacles) {
      if (collision2d::intersect(collision2d::Point<double>{x, y},
                                 (collision2d::Polygon<double>)poly)) {
        OMPL_INFORM("[%.2f %.2f] collides with polygon %d.", x, y, i);
//        for (const auto &point : poly.points)
//            std::cout << point << " ";
//        std::cout << std::endl;
        return true;
      }
      ++i;
    }
    return false;
  }
  bool collides(const Polygon &polygon) override {
    for (const auto &poly : _obstacles) {
      if (collision2d::intersect((collision2d::Polygon<double>)polygon,
                                 (collision2d::Polygon<double>)poly))
        return true;
    }
    return false;
  }

 private:
  PolygonMaze() = default;

  std::string _name{"polygon_maze"};
  std::vector<Polygon> _obstacles;
};

inline void to_json(nlohmann::json &j, const PolygonMaze &m) {
  j["type"] = "polygon";
  j["obstacles"] = m.obstacles();
  j["start"] = {m.start().x, m.start().y, m.startTheta()};
  j["goal"] = {m.goal().x, m.goal().y, m.goalTheta()};
  j["name"] = m.name();
}

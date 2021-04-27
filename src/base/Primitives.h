#pragma once

#include <ompl/base/State.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/PathControl.h>
#include <ompl/geometric/PathGeometric.h>

#include <nlohmann/json.hpp>

namespace ob = ompl::base;

typedef ob::SE2StateSpace::StateType State;

namespace base {
ob::State *StateFromXYT(double x, double y, double theta);
ob::State *StateFromXY(double x, double y);
}  // namespace base

struct Rectangle {
  double x1{0}, y1{0};
  double x2{0}, y2{0};

  Rectangle() = default;
  Rectangle(double x1, double y1, double x2, double y2)
      : x1(x1), y1(y1), x2(x2), y2(y2) {}

  inline double x() const { return std::min(x1, x2); }
  inline double y() const { return std::min(y1, y2); }

  inline double width() const { return std::abs(x1 - x2); }
  inline double height() const { return std::abs(y1 - y2); }

  /**
   * Ensures that x1, y1 is the bottom-left corner (minimum coordinates).
   */
  void correct() {
    if (x1 > x2) std::swap(x1, x2);
    if (y1 > y2) std::swap(y1, y2);
  }
};

struct Point {
  double x{0}, y{0};

  Point() = default;
  Point(double x, double y) : x(x), y(y) {}
  Point(const ob::State *state) {
    if (state == nullptr) {
      OMPL_WARN("Cannot create point from NULL state.");
      return;
    }

    {
      x = state->as<State>()->getX();
      y = state->as<State>()->getY();
    }
  }

  static std::vector<Point> fromPath(const ompl::geometric::PathGeometric &p,
                                     bool interpolate = false);

  static std::vector<Point> fromPath(const ompl::control::PathControl &p,
                                     bool interpolate = false);

  static Point centroid(const std::vector<Point> &points) {
    double x = 0, y = 0;
    for (auto &p : points) {
      x += p.x;
      y += p.y;
    }
    return {x / points.size(), y / points.size()};
  }

  double distance(const Point &p) const {
    const double dx = p.x - x;
    const double dy = p.y - y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double distance(double x, double y) const {
    const double dx = this->x - x;
    const double dy = this->y - y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double distanceSquared(const Point &p) const {
    const double dx = p.x - x;
    const double dy = p.y - y;
    return dx * dx + dy * dy;
  }

  double distanceSquared(double x, double y) const {
    const double dx = this->x - x;
    const double dy = this->y - y;
    return dx * dx + dy * dy;
  }

  operator Eigen::Matrix<double, 2, 1>() const { return {x, y}; }
  Point &operator=(const Eigen::Matrix<double, 2, 1> &p) {
    x = p(0);
    y = p(1);
    return *this;
  }

  Point &operator+=(const Point &p) {
    x += p.x;
    y += p.y;
    return *this;
  }
  Point operator+(const Point &p) const { return {x + p.x, y + p.y}; }
  Point &operator-=(const Point &p) {
    x -= p.x;
    y -= p.y;
    return *this;
  }
  Point operator-(const Point &p) const { return {x - p.x, y - p.y}; }

  Point &operator*=(double d) {
    x *= d;
    y *= d;
    return *this;
  }
  Point operator*(double d) const { return {x * d, y * d}; }

  Point &operator/=(double d) {
    x /= d;
    y /= d;
    return *this;
  }
  Point operator/(double d) const { return {x / d, y / d}; }

  ompl::base::State *toState(double theta = 0) const;

  friend std::ostream &operator<<(std::ostream &stream, const Point &p) {
    return stream << '[' << p.x << ' ' << p.y << ']';
  }
};

inline void to_json(nlohmann::json &j, const Point &p) { j = {p.x, p.y}; }
inline void from_json(const nlohmann::json &j, Point &p) {
  p.x = j.at(0);
  p.y = j.at(1);
}

struct Polygon {
  std::vector<Point> points;

  operator std::vector<Point> &() { return points; }
  operator const std::vector<Point> &() const { return points; }

  Polygon(const std::vector<Point> &points = {}) : points(points) {}

  Point centroid() const {
    Point c;
    for (const auto &p : points) c += p;
    c /= points.size();
    return c;
  }

  /**
   * Translates points such that the centroid becomes (0, 0).
   */
  void center() {
    const auto c = centroid();
    for (auto &p : points) p -= c;
  }

  void scale(double scaling) {
    for (auto &p : points) p *= scaling;
  }

  Polygon scaled(double scaling) {
    Polygon poly(*this);
    for (auto &p : poly.points) p *= scaling;
    return poly;
  }

  /**
   * Loads polygon from a path tag inside an SVG file.
   */
  static Polygon loadFromSvgPathStr(std::string path_str) {
    std::replace(path_str.begin(), path_str.end(), ',', ' ');
    std::vector<std::string> tokens;
    std::stringstream ss(path_str);
    std::string word;
    // split with space
    while (ss >> word) tokens.emplace_back(word);
    std::vector<Point> points;
    Point point;
    // flip the vertical coordinates to match Inkscape's rendering
    for (size_t i = 0; i < tokens.size(); i++) {
      const auto &token = tokens[i];
      if (token == "m" || token == "l") {
        for (size_t j = i + 1; j < tokens.size() - 1; j += 2) {
          if (!is_number(tokens[j])) break;
          point += {stod(tokens[j]), -stod(tokens[j + 1])};
          points.emplace_back(point);
          i += 2;
        }
      }
      if (token == "M" || token == "L") {
        for (size_t j = i + 1; j < tokens.size() - 1; j += 2) {
          if (!is_number(tokens[j])) break;
          point = Point(stod(tokens[j]), -stod(tokens[j + 1]));
          points.emplace_back(point);
          i += 2;
        }
      }
      if (token == "h" || token == "v") {
        for (size_t j = i + 1; j < tokens.size(); j++) {
          if (!is_number(tokens[j])) break;
          if (token == "h")
            point.x += stod(tokens[j]);
          else
            point.y -= stod(tokens[j]);
          points.emplace_back(point);
          i++;
        }
      }
      if (token == "H" || token == "V") {
        for (size_t j = i + 1; j < tokens.size(); j++) {
          if (!is_number(tokens[j])) break;
          if (token == "H")
            point.x = stod(tokens[j]);
          else
            point.y = -stod(tokens[j]);
          points.emplace_back(point);
          i++;
        }
      }
    }
    return points;
  }

  void translate(const Point &t) {
    for (auto &p : points) p += t;
  }

  /**
   * Rotates polygon counterclockwise about the origin.
   */
  void rotate(double angle) {
    double c = std::cos(angle), s = std::sin(angle);
    for (auto &p : points) {
      const Point temp(p);
      p.x = temp.x * c - temp.y * s;
      p.y = temp.x * s + temp.y * c;
    }
  }

  Polygon transformed(const ompl::base::State *state) const {
    Polygon p(*this);
    // std::cout << state->as<State>()->getYaw() << " "
    //           << state->as<State>()->getX() << " " <<
    //           state->as<State>()->getY()
    //           << std::endl;
    p.rotate(state->as<State>()->getYaw());
    p.translate({state->as<State>()->getX(), state->as<State>()->getY()});
    // std::cout << p << std::endl;
    return p;
  }

  /**
   * Minimum x/y coordinates of all points.
   */
  Point min() const {
    Point m = points[0];
    for (const auto &p : points) {
      if (p.x < m.x) m.x = p.x;
      if (p.y < m.y) m.y = p.y;
    }
    return m;
  }
  /**
   * Maximum x/y coordinates of all points.
   */
  Point max() const {
    Point m = points[0];
    for (const auto &p : points) {
      if (p.x > m.x) m.x = p.x;
      if (p.y > m.y) m.y = p.y;
    }
    return m;
  }

  bool isConvex() const;

  Polygon convexHull() const;

  /**
   * Cast to the polygon format compatible with the `collision2d` library.
   */
  operator std::vector<Eigen::Matrix<double, 2, 1>>() const {
    std::vector<Eigen::Matrix<double, 2, 1>> v;
    for (const auto &p : points)
      v.emplace_back(Eigen::Matrix<double, 2, 1>{p.x, p.y});
    return v;
  }

  friend std::ostream &operator<<(std::ostream &stream, const Polygon &p) {
    for (const auto &point : p.points) stream << point << ' ';
    return stream;
  }

 private:
  static bool is_number(const std::string &s) {
    try {
      std::stod(s);
    } catch (...) {
      return false;
    }
    return true;
  }
};

inline void to_json(nlohmann::json &j, const Polygon &p) { j = p.points; }
inline void from_json(const nlohmann::json &j, Polygon &p) {
  p.points.clear();
  for (const auto &element : j) p.points.emplace_back(element);
}

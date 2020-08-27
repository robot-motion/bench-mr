#include "GridMaze.h"

#include <ompl/util/Console.h>
#include <planners/thetastar/ThetaStar.h>
#include <stdio.h>
#include <stdlib.h>

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include <third_party/stb/stb_image.h>
#include <third_party/stb/stb_image_resize.h>

#include <collision2d/sat.hpp>
#include <fstream>
#include <iostream>

#include "PlannerSettings.h"
#include "utils/PlannerUtils.hpp"

#ifdef QT_SUPPORT
#include "gui/QtVisualizer.h"

#endif

#if XML_SUPPORT
#include <pugixml/pugixml.hpp>
#endif

#if ROS_SUPPORT
#include <nav_msgs/GridCells.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#endif

GridMaze::GridMaze(unsigned int seed, unsigned int width, unsigned int height,
                   double voxelSize)
    : Environment(),
      _voxels_x(width),
      _voxels_y(height),
      _voxelSize(voxelSize),
      _empty(false),
      _seed(seed),
      _distances(nullptr),
      _type("unknown") {
  _grid = new bool[cells()];
  _bounds.setLow(0, 0);
  _bounds.setLow(1, 0);
  _bounds.setHigh(0, _voxels_x * _voxelSize);
  _bounds.setHigh(1, _voxels_y * _voxelSize);
  bool *g = _grid;
  for (unsigned int i = 0; i < cells(); ++i) *g++ = false;
}

GridMaze::GridMaze(const GridMaze &environment)
    : Environment(), _distances(nullptr) {
  _grid = new bool[environment.cells()];
  bool *g = _grid;
  for (unsigned int i = 0; i < environment.cells(); ++i)
    *g++ = environment._grid[i];

  _seed = environment._seed;
  _bounds = environment._bounds;
  _empty = environment._empty;
  _voxels_x = environment._voxels_x;
  _voxels_y = environment._voxels_y;
  _voxelSize = environment._voxelSize;
  _type = environment._type;
  _name = environment._name;
}

GridMaze::~GridMaze() {
  delete[] _grid;
  delete[] _distances;
}

void GridMaze::fill(double x, double y, bool value) {
  _grid[coord2key(x, y)] = value;
}

void GridMaze::fill(const Rectangle &r, bool value) {
  for (int x = (int)std::floor(std::max(0., std::min(r.x1, r.x2)));
       x < std::ceil(std::min(width(), std::max(r.x1, r.x2))); ++x) {
    for (int y = (int)std::floor(std::max(0., std::min(r.y1, r.y2)));
         y < std::ceil(std::min(height(), std::max(r.y1, r.y2))); ++y) {
      _grid[coord2key(x, y)] = value;
    }
  }
}

void GridMaze::fillBorder(bool value, int size) {
  if (size < 1) return;
  fill(Rectangle(0, 0, width(), size), value);
  fill(Rectangle(0, height() - size, width(), height()), value);
  fill(Rectangle(0, 0, size, height()), value);
  fill(Rectangle(width() - size, 0, width(), height()), value);
}

GridMaze *GridMaze::createRandomCorridor(unsigned int width,
                                         unsigned int height, double radius,
                                         int branches, unsigned int seed,
                                         int borderSize) {
  OMPL_INFORM("Generating environment with seed %i", seed);
  srand(seed);
  auto *environment = new GridMaze(seed, width, height);
  environment->_type = "corridor";

  for (unsigned int i = 0; i < environment->cells(); ++i)
    environment->_grid[i] = true;

  typedef Eigen::Matrix<double, 3, 1> Vector3;
  std::vector<Vector3> nodes{Vector3{width / 2., height / 2., 0.}};
  for (int k = 0; k < branches; ++k) {
    const int x = borderSize + (int)radius +
                  rand() % (width - (int)radius * 2 - 2 * borderSize - 1);
    const int y = borderSize + (int)radius +
                  rand() % (height - (int)radius * 2 - 2 * borderSize - 1);

    // find closest vertex
    double minDistance = std::numeric_limits<double>::max();
    Point closest;
    for (const auto &node : nodes) {
      const Point pos(node.x(), node.y());
      double d = pos.distance(x, y);
      if (d < minDistance) {
        minDistance = d;
        closest = pos;
      }
    }

    if (std::abs(x - closest.x) < std::abs(y - closest.y)) {
      // connect vertically
      environment->fill(
          Rectangle(closest.x - std::floor(radius),
                    std::min(closest.y, (double)y) - std::floor(radius),
                    closest.x + std::ceil(radius),
                    std::max(closest.y, (double)y) + std::ceil(radius)),
          false);
      const double yaw = closest.y > y ? -M_PI_2 : M_PI_2;
      nodes.emplace_back(Vector3{closest.x, static_cast<double>(y), yaw});
    } else {
      // connect horizontally
      environment->fill(
          Rectangle(std::min(closest.x, (double)x) - std::floor(radius),
                    closest.y - std::floor(radius),
                    std::max(closest.x, (double)x) + std::ceil(radius),
                    closest.y + std::ceil(radius)),
          false);
      const double yaw = closest.x > x ? -M_PI : 0;
      nodes.emplace_back(Vector3{static_cast<double>(x), closest.y, yaw});
    }
  }
  environment->fillBorder(true, borderSize);

  // find start / goal positions
  double max_dist = 0;
  for (auto &p : nodes) {
    if (environment->occupied(p.x(), p.y())) {
#ifdef DEBUG
      QtVisualizer::drawNode(p.x, p.y);
#endif
      OMPL_WARN("(%f %f) is occupied.", p.x(), p.y());
      continue;
    }
    const Point start(p.x(), p.y());
    for (auto &node : nodes) {
      const Point q(node.x(), node.y());
      if (environment->occupied(q.x, q.y)) continue;
      double dist = start.distance(q);
      if (dist > max_dist) {
        max_dist = dist;
        environment->setStart(start);
        environment->setGoal(q);
        // flip start yaw angle to be "inwards" the map
        environment->setThetas(PlannerUtils::normalizeAngle(p(2) - M_PI),
                               PlannerUtils::normalizeAngle(node(2)));
      }
    }
  }
  std::cout << *environment << std::endl;
  environment->computeDistances();
  return environment;
}

GridMaze *GridMaze::createRandom(unsigned int width, unsigned int height,
                                 double obsRatio, unsigned int seed,
                                 int borderSize) {
  OMPL_INFORM("Generating environment with seed %i", seed);
  srand(seed);
  auto *environment = new GridMaze(seed, width, height);
  environment->_type = "random";
  // make borders occupied
  environment->fillBorder(true, borderSize);
  double x, y;
  for (int i = 0; i < width * height * obsRatio; ++i) {
    x = rand() * 1. / RAND_MAX * width;
    y = rand() * 1. / RAND_MAX * height;
    environment->fill(x, y, true);
    //        environment->_checker->add_obstacle(Tobstacle(x, y, 0, 1, 1));
  }

  do {
    x = int(rand() * 1. / RAND_MAX * (width / 8.));
    y = int(rand() * 1. / RAND_MAX * (height / 8.));
    environment->setStart(Point(x, y));
  } while (environment->occupied(x, y));

  do {
    x = int(width * 7. / 8. + rand() * 1. / RAND_MAX * (width / 8.));
    y = int(height * 7. / 8. + rand() * 1. / RAND_MAX * (height / 8.));
    environment->setGoal(Point(x, y));
  } while (environment->occupied(x, y));

  return environment;
}

GridMaze *GridMaze::createFromObstacles(const std::vector<Rectangle> &obstacles,
                                        unsigned int width, unsigned int height,
                                        int borderSize) {
  auto *environment = new GridMaze(0, width, height);
  for (auto &obs : obstacles) environment->fill(obs, true);
  environment->fillBorder(true, borderSize);
  environment->_type = "obstacles - " + std::to_string(obstacles.size());
  return environment;
}

#if XML_SUPPORT
GridMaze *GridMaze::loadFromXml(std::string filename) {
  pugi::xml_document doc;
  doc.load_file(filename.c_str());

  double maxx = std::numeric_limits<double>::min();
  double maxy = std::numeric_limits<double>::min();
  std::vector<Rectangle> obstacles;
  for (auto &node : doc.select_nodes("/scenario/obstacle")) {
    double x1 = node.node().attribute("x1").as_double();
    double x2 = node.node().attribute("x2").as_double();
    double y1 = node.node().attribute("y1").as_double();
    double y2 = node.node().attribute("y2").as_double();
    maxx = std::max(maxx, std::max(x1, x2));
    maxy = std::max(maxy, std::max(y1, y2));
    obstacles.push_back(Rectangle(x1, y1, x2, y2));
  }

  auto environment =
      createFromObstacles(obstacles, (unsigned int)std::round(maxx),
                          (unsigned int)std::round(maxy));
  environment->_start = Tpoint(3, 3, 0);
  environment->_goal = Tpoint(44.5, 36.50, 0);
  environment->_type = "XML - " + filename;

  return environment;
}
#endif

#if ROS_SUPPORT
void GridMaze::publish(ros::NodeHandle &nodeHandle) const {
  ros::Rate loop_rate(5);
  auto pub_obstacles_ =
      nodeHandle.advertise<nav_msgs::GridCells>("static_obstacles", 0);
  auto pub_obstacle_cells_ =
      nodeHandle.advertise<visualization_msgs::MarkerArray>(
          "static_obstacle_cells", 0);
  auto pub_start_goal_ =
      nodeHandle.advertise<visualization_msgs::Marker>("start_goal", 0);
  ros::spinOnce();
  loop_rate.sleep();

  nav_msgs::GridCells obstacles;
  obstacles.header.frame_id = "world";
  obstacles.cellwidth() = 1;
  obstacles.cellheight() = 1;

  visualization_msgs::MarkerArray cells;
  visualization_msgs::Marker removeAllCells;
  removeAllCells.action = 3;
  removeAllCells.id = 0;
  removeAllCells.header.frame_id = "world";
  cells.markers.push_back(removeAllCells);
  for (int x = 0; x <= width(); ++x) {
    for (int y = 0; y <= height(); ++y) {
      if (!_grid[coord2key(x, y)]) continue;

      geometry_msgs::Point p;
      p.x = x + 0.5;
      p.y = y + 0.5;
      p.z = 0.0;
      obstacles.cells.push_back(p);

      visualization_msgs::Marker cell;
      cell.action = 0;
      cell.id = width() * (y + 1) + x;
      cell.header.frame_id = "world";
      cell.type = visualization_msgs::Marker::CUBE;
      cell.pose.position.x = x + 0.5;
      cell.pose.position.y = y + 0.5;
      cell.pose.position.z = 0;
      cell.scale.x = 1;
      cell.scale.y = 1;
      cell.scale.z = 1;
      cell.color.a = 1;
      cell.color.r = .4;
      cell.color.g = .4;
      cell.color.b = .4;
      cells.markers.push_back(cell);
    }
  }

  visualization_msgs::Marker s, g;
  s.header.frame_id = "world";
  s.id = 0;
  s.type = visualization_msgs::Marker::SPHERE;
  s.color.a = 1;
  s.color.r = 0.10;
  s.color.g = 0.70;
  s.color.b = 0.10;
  s.scale.x = 1;
  s.scale.y = 1;
  s.scale.z = 1;
  s.action = 0;  // add or modify
  s.pose.position.x = _start.x;
  s.pose.position.y = _start.y;
  s.pose.position.z = 0;

  g.header.frame_id = "world";
  g.id = 1;
  g.type = visualization_msgs::Marker::SPHERE;
  g.color.a = 1;
  g.color.r = 0.20;
  g.color.g = 0.40;
  g.color.b = 0.70;
  g.scale.x = 1;
  g.scale.y = 1;
  g.scale.z = 1;
  g.action = 0;  // add or modify
  g.pose.position.x = _goal.x;
  g.pose.position.y = _goal.y;
  g.pose.position.z = 0;

  ROS_INFO("Start: %f %f", _start.x, _start.y);
  ROS_INFO("Goal:  %f %f", _goal.x, _goal.y);

  for (int i = 0; i < 2; ++i) {
    pub_obstacles_.publish(obstacles);
    pub_obstacle_cells_.publish(cells);

    pub_start_goal_.publish(s);
    pub_start_goal_.publish(g);

    ros::spinOnce();
    loop_rate.sleep();
  }
}
#endif

bool GridMaze::collides(double x, double y) { return occupied(x, y); }

bool GridMaze::collides(const Polygon &polygon) {
  _collision_timer.resume();
  typedef std::vector<Eigen::Matrix<double, 2, 1>> PG;
  const auto poly = (PG)polygon;

  // first retrieve submap that overlaps with polygon
  const auto min = polygon.min();
  const auto max = polygon.max();
  const auto start_x =
      std::min(_voxels_x, static_cast<unsigned int>(std::max(
                              0, static_cast<int>(min.x / _voxelSize))));
  const auto start_y =
      std::min(_voxels_y, static_cast<unsigned int>(std::max(
                              0, static_cast<int>(min.y / _voxelSize))));
  const auto end_x =
      std::min(_voxels_x, static_cast<unsigned int>(std::max(
                              0, static_cast<int>(max.x / _voxelSize) + 1)));
  const auto end_y =
      std::min(_voxels_y, static_cast<unsigned int>(std::max(
                              0, static_cast<int>(max.y / _voxelSize) + 1)));

  for (auto y = start_y; y < end_y; ++y) {
    for (auto x = start_x; x < end_x; ++x) {
      // TODO use more efficient polygon representation of GridMaze to speed up
      // the collision checks against polygon shapes
      if (occupiedCell(x, y)) {
        // create Polygon for this cell and do collision check
        const Polygon cell({{x * _voxelSize, y * _voxelSize},
                            {(x + 1) * _voxelSize, y * _voxelSize},
                            {(x + 1) * _voxelSize, (y + 1) * _voxelSize},
                            {x * _voxelSize, (y + 1) * _voxelSize}});
        if (collision2d::intersect(poly, (PG)cell)) {
          _collision_timer.stop();
          return true;
        }
      }
    }
  }
  _collision_timer.stop();
  return false;
}

std::vector<Rectangle> GridMaze::obstacles() const {
  std::vector<Rectangle> obs;
  for (unsigned int x = 0; x < width(); ++x) {
    for (unsigned int y = 0; y < height(); ++y) {
      if (!occupiedCell(x, y)) continue;
      obs.emplace_back(x, y, x + 1, y + 1);
    }
  }
  return obs;
}

std::vector<Rectangle> GridMaze::obstacles(double x1, double y1, double x2,
                                           double y2) const {
  std::vector<Rectangle> obs;
  for (auto x = (unsigned int)std::round(x1); x <= std::round(x2); ++x) {
    for (auto y = (unsigned int)std::round(y1); y <= std::round(y2); ++y) {
      if (!occupiedCell(x, y)) continue;
      obs.emplace_back(x, y, x + 1, y + 1);
    }
  }
  return obs;
}

void GridMaze::computeDistances() {
  OMPL_INFORM(("Computing distances via " +
               distance_computation::to_string(distanceComputationMethod()))
                  .c_str());
  delete[] _distances;
  _distances = new double[(_voxels_x + 1) * (_voxels_y + 1)];
  if (distanceComputationMethod() == distance_computation::DEAD_RECKONING) {
    // more efficient, but less accurate Dead Reckoning Algorithm
    //
    // The "Dead reckoning" signed distance transform
    // George J. Grevera
    // Journal Computer Vision and Image Understanding (2004)
    //
    // http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.102.7988

    static const double d1{
        1};  // horizontal/vertical distance between two adjacent cells
    static const double d2{
        std::sqrt(2.)};  // diagonal distance between two cells

    // border points corresponding to each cell
    auto *P = new Point[(_voxels_x + 1) * (_voxels_y + 1)];

    // initialize distances, immediate interior & exterior elements
    for (int y = 0; y < _voxels_y; ++y) {
      for (int x = 0; x < _voxels_x; ++x) {
        const auto key = coord2key(x, y);
        bool here = _grid[key];
        if ((x > 0 && _grid[coord2key(x - 1, y)] != here) ||
            (x < _voxels_x - 1 && _grid[coord2key(x + 1, y)] != here) ||
            (y > 0 && _grid[coord2key(x, y - 1)] != here) ||
            (y < _voxels_y - 1 && _grid[coord2key(x, y + 1)] != here)) {
          _distances[key] = 0;
          P[key] = Point(x, y);
        } else {
          _distances[key] = std::numeric_limits<double>::max();
          P[key] = Point();
        }
      }
    }

    // first pass
    for (int y = 0; y < _voxels_y; ++y) {
      for (int x = 0; x < _voxels_x; ++x) {
        const auto key = coord2key(x, y);
        auto &d = _distances[key];

        if (x > 0 && y > 0 && _distances[coord2key(x - 1, y - 1)] + d2 < d) {
          P[key] = P[coord2key(x - 1, y - 1)];
          const double dx = x - P[key].x;
          const double dy = y - P[key].y;
          d = std::sqrt(dx * dx + dy * dy);
        }
        if (y > 0 && _distances[coord2key(x, y - 1)] + d1 < d) {
          P[key] = P[coord2key(x, y - 1)];
          const double dx = x - P[key].x;
          const double dy = y - P[key].y;
          d = std::sqrt(dx * dx + dy * dy);
        }
        if (x < _voxels_x - 1 && y > 0 &&
            _distances[coord2key(x + 1, y - 1)] + d2 < d) {
          P[key] = P[coord2key(x + 1, y - 1)];
          const double dx = x - P[key].x;
          const double dy = y - P[key].y;
          d = std::sqrt(dx * dx + dy * dy);
        }
        if (x > 0 && _distances[coord2key(x - 1, y)] + d1 < d) {
          P[key] = P[coord2key(x - 1, y)];
          const double dx = x - P[key].x;
          const double dy = y - P[key].y;
          d = std::sqrt(dx * dx + dy * dy);
        }
      }
    }

    // final pass
    for (int y = _voxels_y - 1; y >= 0; --y) {
      for (int x = _voxels_x - 1; x >= 0; --x) {
        const auto key = coord2key(x, y);
        auto &d = _distances[key];

        if (x < _voxels_x - 1 && _distances[coord2key(x + 1, y)] + d1 < d) {
          P[key] = P[coord2key(x + 1, y)];
          const double dx = x - P[key].x;
          const double dy = y - P[key].y;
          d = std::sqrt(dx * dx + dy * dy);
        }
        if (x > 0 && y < _voxels_y - 1 &&
            _distances[coord2key(x - 1, y + 1)] + d2 < d) {
          P[key] = P[coord2key(x - 1, y + 1)];
          const double dx = x - P[key].x;
          const double dy = y - P[key].y;
          d = std::sqrt(dx * dx + dy * dy);
        }
        if (y < _voxels_y - 1 && _distances[coord2key(x, y + 1)] + d1 < d) {
          P[key] = P[coord2key(x, y + 1)];
          const double dx = x - P[key].x;
          const double dy = y - P[key].y;
          d = std::sqrt(dx * dx + dy * dy);
        }
        if (x < _voxels_x - 1 && y < _voxels_y - 1 &&
            _distances[coord2key(x + 1, y + 1)] + d2 < d) {
          P[key] = P[coord2key(x + 1, y + 1)];
          const double dx = x - P[key].x;
          const double dy = y - P[key].y;
          d = std::sqrt(dx * dx + dy * dy);
        }
      }
    }

    delete[] P;

    // indicate inside (0)
    for (unsigned int y = 0; y < _voxels_y; ++y) {
      for (unsigned int x = 0; x < _voxels_x; ++x) {
        const auto key = coord2key(x, y);
        if (_grid[key]) _distances[key] = 0;
      }
    }
  } else {
    // Brute-Force Algorithm
    for (unsigned int x = 0; x < _voxels_x; ++x) {
      for (unsigned int y = 0; y < _voxels_y; ++y) {
        if (_grid[coord2key(x, y)]) {
          _distances[coord2key(x, y)] = 0;
          continue;
        }
        double minDistance = std::numeric_limits<double>::max();
        for (unsigned int dx = 0; dx < _voxels_x; ++dx) {
          for (unsigned int dy = 0; dy < _voxels_y; ++dy) {
            if (_grid[coord2key(dx, dy)]) {
              double d = std::sqrt(std::pow((double)dx - x, 2.) +
                                   std::pow((double)dy - y, 2.));
              minDistance = std::min(d, minDistance);
            }
          }
        }
        _distances[coord2key(x, y)] = minDistance;
      }
    }
  }
}

GridMaze *GridMaze::createSimple() {
  auto *environment = new GridMaze(0, DefaultWidth, DefaultHeight);
  environment->fill(Rectangle(18, 18, 34, 34), true);
  environment->setStart(Point(18, 45));
  environment->setGoal(Point(45, 18));
  environment->_type = "simple";
  return environment;
}

// Moving Ai File test Constructor
GridMaze *GridMaze::createFromMovingAiScenario(Scenario &scenario) {
  auto *environment = new GridMaze(0, scenario.map_width, scenario.map_height);
  // set start and goal points
  environment->setStart(Point(scenario.start_x, scenario.start_y));
  environment->setGoal(Point(scenario.goal_x, scenario.goal_y));
  environment->_voxels_x = scenario.map_width;
  environment->_voxels_y = scenario.map_height;
  environment->_type = "moving_ai " + scenario.mapName;
  environment->_name =
      scenario.filename + "[" + std::to_string(scenario.id) + "]";

  // construct map
  const auto &grid = scenario.getMap();
  for (unsigned int x = 0; x < scenario.map_width; x++) {
    for (unsigned int y = 0; y < scenario.map_height; y++)
      // XXX the map is transposed!
      environment->fill(x, y, grid[y][x] != '.');
  }

  if (global::settings.benchmark.moving_ai.create_border.value())
    environment->fillBorder(true);

  std::cout << "Loaded scenario " << scenario << std::endl;
  //  std::cout << *environment << std::endl;

  return environment;
}

GridMaze *GridMaze::createFromImage(const std::string &filename,
                                    double occupancy_threshold, int width,
                                    int height) {
  int actual_width, actual_height, n;
  unsigned char *data =
      stbi_load(filename.c_str(), &actual_width, &actual_height, &n, 1);
  if (!data) {
    fprintf(stderr, "Error: failed to load image %s: %s\n", filename.c_str(),
            stbi_failure_reason());
    return nullptr;
  }
  printf("Loaded grid map from image %s (%ix%i, %i channel(s)).\n",
         filename.c_str(), actual_width, actual_height, n);
  fflush(stdout);

  if (width > 0 && height > 0 &&
      (actual_width != width || actual_height != height)) {
    unsigned char *resized_data = new unsigned char[width * height];
    int result = stbir_resize_uint8(data, actual_width, actual_height, 0, resized_data,
                       width, height, 0, 1);
    if (!result || !resized_data) {
      fprintf(stderr,
              "Warning: failed to resize image %s from %ix%i to %ix%i: %s\n",
              filename.c_str(), actual_width, actual_height, width, height,
              stbi_failure_reason());
      fflush(stderr);
    } else {
      printf("Resized image %s from %ix%i to %ix%i.\n", filename.c_str(),
             actual_width, actual_height, width, height);
      fflush(stdout);
      stbi_image_free(data);
      data = resized_data;
    }
  } else {
    width = actual_width;
    height = actual_height;
  }

  auto *environment = new GridMaze(0, width, height);
  environment->_type = "image: " + filename;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      double pixel = data[y * width + x] / 255.;
      // flip image vertically
      environment->_grid[(height - y - 1) * width + x] = pixel <= occupancy_threshold;
    }
  }

  stbi_image_free(data);
  return environment;
}

double GridMaze::obstacleRatio() const {
  int occ = 0;
  for (unsigned int x = 0; x < _voxels_x; ++x) {
    for (unsigned int y = 0; y < _voxels_y; ++y) occ += (int)occupiedCell(x, y);
  }
  return (double)(occ) / (double)(cells());
}

std::string GridMaze::generatorType() const { return _type; }

bool GridMaze::saveSbplConfigFile(const std::string &filename) const {
  auto file = std::fstream(filename, std::ios::out);
  if (file.bad()) return false;

  file << "discretization(cells): " << width() << " " << height() << std::endl;
  file << "start(cells): " << 0 << " " << 0 << std::endl;
  file << "end(cells): " << width() - 1 << " " << height() - 1 << std::endl;
  file << "environment:" << std::endl;
  for (unsigned int x = 0; x <= width(); ++x) {
    for (unsigned int y = 0; y <= height(); ++y)
      file << (occupiedCell(x, y) ? "1 " : "0 ");
    file << std::endl;
  }

  char *absFilename = nullptr;
  absFilename = realpath(filename.c_str(), absFilename);
  OMPL_INFORM(("Saved sbpl environment cfg-file at " + std::string(absFilename))
                  .c_str());
  return true;
}

void GridMaze::mapData(unsigned char *data, double resolution) {
  if (resolution == 1) {
    for (unsigned int x = 0; x < _voxels_x; ++x) {
      for (unsigned int y = 0; y < _voxels_y; ++y)
        data[x + y * _voxels_x] =
            static_cast<unsigned char>(occupiedCell(x, y) ? 20u : 0u);
    }
  } else {
    auto w = static_cast<unsigned int>(width() / resolution);
    auto h = static_cast<unsigned int>(height() / resolution);
    for (unsigned int y = 0; y <= h; ++y) {
      for (unsigned int x = 0; x <= w; ++x)
        data[x + y * w] = static_cast<unsigned char>(
            occupied(x * resolution, y * resolution) ? 20u : 0u);
    }
  }
  OMPL_DEBUG("Generated SBPL map data");
}

std::string GridMaze::mapString() const {
  std::string data;
  for (unsigned int y = 0; y < _voxels_y; ++y) {
    for (unsigned int x = 0; x < _voxels_x; ++x)
      data += occupiedCell(x, y) ? '1' : '0';
  }
  return data;
}

std::vector<double> GridMaze::mapDistances() {
  vector<double> distances(cells());
  for (unsigned int y = 0; y < _voxels_y; ++y) {
    for (unsigned int x = 0; x < _voxels_x; ++x)
      distances[x + y * _voxels_x] = distance(x, y);
  }
  return distances;
}

#pragma once

#include <utils/ScenarioLoader.h>

#include <ctime>
#include <iostream>
#include <nlohmann/json.hpp>
#include <vector>

#include "Environment.h"
#include "PlannerSettings.h"

#define ROS_SUPPORT 0
#define XML_SUPPORT 0

#if ROS_SUPPORT
#include <ros/ros.h>
#endif

class GridMaze : public Environment {
 public:
  GridMaze() = default;
  GridMaze(const GridMaze &environment);

  ~GridMaze();

  static const unsigned int DefaultWidth = 50;
  static const unsigned int DefaultHeight = 50;

  unsigned int seed() const { return _seed; }

  bool empty() const { return _empty; }

  double voxelSize() const { return _voxelSize; }

  inline bool occupied(unsigned int index) const { return _grid[index]; }
  inline bool occupied(double x, double y) {
    _collision_timer.resume();
    //    std::cout << "occupied ?   " << x << "\t" << y << std::endl;
    if (x < 0 || y < 0 || x > width() || y > height()) {
      _collision_timer.stop();
      return true;
    }
    //    if (!fastCollisionCheck) {
    //#if QT_SUPPORT
    //      //            bool o = bilinearDistance(x, y) <= 0.05;
    //      //            QtVisualizer::drawNode(x, y, o ? Qt::red :
    //      Qt::darkGreen);
    //#endif
    //      return _grid[coord2key(x, y)] || bilinearDistance(x, y) <= 0.1;
    //    }
    bool c = _grid[coord2key(x, y)] || _grid[coord2key(x + .15, y)] ||
             _grid[coord2key(x, y + .15)] ||
             _grid[coord2key(x + .15, y + .15)] ||
             _grid[coord2key(x - .15, y)] || _grid[coord2key(x, y - .15)] ||
             _grid[coord2key(x - .15, y - .15)];

    _collision_timer.stop();
    return c;
  }

  inline bool occupiedCell(unsigned int xi, unsigned int yi) const {
    return _grid[yi * _voxels_x + xi];
  }

  /**
   * Computes distances field if necessary, and returns the distance
   * to the nearest obstacle.
   */
  inline double distance(double x, double y) override {
    if (_distances == nullptr) computeDistances();
    return _distances[coord2key(x, y)];
  }

  /**
   * Computes distances field if necessary, and returns the distance
   * to the nearest obstacle.
   */
  inline double distance(unsigned int index) {
    if (_distances == nullptr) computeDistances();
    return _distances[index];
  }

  /**
   * Computes distances field if necessary, and returns the distance
   * to the nearest obstacle.
   */
  inline double distance(unsigned int xi, unsigned int yi) {
    if (_distances == nullptr) computeDistances();
    return _distances[yi * _voxels_x + xi];
  }

  friend std::ostream &operator<<(std::ostream &stream, const GridMaze &m) {
    for (unsigned int y = 0; y < m._voxels_y; ++y) {
      for (unsigned int x = 0; x < m._voxels_x; ++x) {
        if (std::round(m._goal.x) == x && std::round(m._goal.y) == y)
          stream << 'G';
        else if (std::round(m._start.x) == x && std::round(m._start.y) == y)
          stream << 'S';
        else if (m.occupiedCell(x, y))
          stream << '#';
        else
          stream << ' ';
      }
      stream << std::endl;
    }
    return stream;
  }

  /**
   * Writes cfg file for planners based on sbpl.
   * @param filename The filename of the ".cfg" file.
   * @return True if saving succeeded, false otherwise.
   */
  bool saveSbplConfigFile(const std::string &filename) const;

  void mapData(unsigned char *data, double resolution = 1.);
  std::string mapString() const;
  std::vector<double> mapDistances();

  std::vector<Rectangle> obstacles() const;
  std::vector<Rectangle> obstacles(double x1, double y1, double x2,
                                   double y2) const;

  bool collides(double x, double y) override;
  bool collides(const Polygon &polygon) override;

  static GridMaze *createRandom(unsigned int width = DefaultWidth,
                                unsigned int height = DefaultHeight,
                                double obsRatio = 0.3,
                                unsigned int seed = (unsigned int)time(nullptr),
                                int borderSize = 1);
  static GridMaze *createRandomCorridor(
      unsigned int width = DefaultWidth, unsigned int height = DefaultHeight,
      double radius = 2, int branches = 30,
      unsigned int seed = (unsigned int)time(nullptr), int borderSize = 1);
  static GridMaze *createFromObstacles(const std::vector<Rectangle> &obstacles,
                                       unsigned int width = DefaultWidth,
                                       unsigned int height = DefaultHeight,
                                       int borderSize = 1);
  static GridMaze *createSimple();

  static GridMaze *createFromMovingAiScenario(Scenario &scenario);

  /**
   * Creates a map from a grayscale image where tones below the occupancy
   * threshold are considered obstacles. Optional resizing is applied if the
   * provided width and height are nonzero.
   */
  static GridMaze *createFromImage(const std::string &filename,
                                   double occupancy_threshold = 0.5,
                                   int width = 0, int height = 0);

#if XML_SUPPORT
  static GridMaze *loadFromXml(std::string filename);
#endif
#if ROS_SUPPORT
  void publish(ros::NodeHandle &nodeHandle) const;
#endif

  double obstacleRatio() const;
  std::string generatorType() const;

  unsigned int cells() const { return _voxels_x * _voxels_y; }

  unsigned int voxels_x() const { return _voxels_x; }
  unsigned int voxels_y() const { return _voxels_y; }

  std::string name() const override { return _name; }
  std::string &name() { return _name; }

  /**
   * Brute-force, quadratic in the number of cells, algorithm
   * to compute the distance field, i.e. distance to the nearest
   * obstacle for every voxel.
   */
  void computeDistances();

  void to_json(nlohmann::json &j) override {
    j["type"] = "grid";
    j["generator"] = generatorType();
    j["width"] = voxels_x();
    j["height"] = voxels_y();
    j["obstacleRatio"] = obstacleRatio();
    j["seed"] = seed();
    j["start"] = {start().x, start().y, startTheta()};
    j["goal"] = {goal().x, goal().y, goalTheta()};
    j["map"] = mapString();
    j["name"] = name();
    if (global::settings.log_env_distances) j["distances"] = mapDistances();
    j["distance_computation_method"] =
        distance_computation::to_string(distanceComputationMethod());
  }

 protected:
  using Environment::_collision_timer;

  GridMaze(unsigned int seed, unsigned int width, unsigned int height,
           double voxelSize = 1.);

  inline unsigned int coord2key(double x, double y) const {
    return (unsigned int)std::max(
        0., std::min(std::round(y), _voxels_y - 1.) * _voxels_x +
                std::min(std::round(x), _voxels_x - 1.));
  }
  void fill(double x, double y, bool value);
  void fill(const Rectangle &r, bool value);
  void fillBorder(bool value, int size = 1);

  distance_computation::Method distanceComputationMethod() const {
    if (global::settings.auto_choose_distance_computation_method) {
      if (cells() > global::settings.fast_odf_threshold)
        return distance_computation::DEAD_RECKONING;
      return distance_computation::BRUTE_FORCE;
    }
    return global::settings.distance_computation_method;
  }

 private:
  // true means occupied
  bool *_grid{nullptr};

  unsigned int _voxels_x{0};
  unsigned int _voxels_y{0};

  double *_distances{nullptr};
  double _voxelSize{1.0};
  bool _empty{true};
  unsigned int _seed{0};
  std::string _type{"undefined"};
  std::string _name{"grid"};
};

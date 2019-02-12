#pragma once

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

/**
 * Load MovingAI scenario files.
 */
struct Scenario {
  int bucket{0};
  std::string mapName{""};
  unsigned int map_width{0};
  unsigned int map_height{0};
  unsigned int start_x{0};
  unsigned int start_y{0};
  unsigned int goal_x{0};
  unsigned int goal_y{0};
  double optimal_length{0};

  void loadMap();

  inline const std::vector<std::vector<char>> &getMap() {
    loadMap();
    return _grid;
  }

 private:
  std::vector<std::vector<char>> _grid;
};

class ScenarioLoader {
 public:
  ScenarioLoader() = default;

  void load(const std::string &fileName);

  inline int version() { return _version; }

  std::vector<Scenario> &scenarios() { return _scenarios; }

  // debug, prints data to terminal
  inline void printData() {
    for (const auto &scenario : _scenarios) {
      std::cout << scenario.bucket;
      std::cout << "  " << scenario.mapName;
      std::cout << "  " << scenario.map_width;
      std::cout << "  " << scenario.map_height;
      std::cout << "  " << scenario.start_x;
      std::cout << "  " << scenario.start_y;
      std::cout << "  " << scenario.goal_x;
      std::cout << "  " << scenario.goal_y;
      std::cout << std::fixed << std::setprecision(8) << "  "
                << scenario.optimal_length;
      std::cout << std::endl;
    }
  }

 private:
  int _version;
  std::string _fileName;
  std::vector<Scenario> _scenarios;
};

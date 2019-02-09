// Load scenario files for testing for MPB

#ifndef SCLOADER_H
#define SCLOADER_H

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

struct Scenario {
  int _bucket;
  std::string _mapName;
  int _map_width;
  int _map_height;
  int _start_x;
  int _start_y;
  int _goal_x;
  int _goal_y;
  double _optimal_length;
  std::vector<std::vector<char>> _grid;

  Scenario() = default;
  Scenario(int bucket, const std::string &mapName, int map_width,
           int map_height, int start_x, int start_y, int goal_x, int goal_y,
           double optimal_length)
      : _bucket(bucket),
        _mapName(mapName),
        _map_width(map_width),
        _map_height(map_height),
        _start_x(start_x),
        _start_y(start_y),
        _goal_x(goal_x),
        _goal_y(goal_y),
        _optimal_length(optimal_length){};

  void loadMap();

  inline const std::vector<std::vector<char>>& getMap() {
    loadMap();
    return _grid;
  }
};

class ScenarioLoader {
 public:
  ScenarioLoader() = default;
  ScenarioLoader(const std::string &fileName);

  ~ScenarioLoader();

  inline int get_Version() { return _version; }

  // get scenario by index
  Scenario *getScenario(size_t num);

  //iterate through list one by one
  Scenario *getScenario();
  
  inline unsigned int getSize(){
    return size;
  }

  // debug, prints data to terminal
  inline void printData() {
    for (size_t i = 0; i < scenario_list.size(); i++) {
      std::cout << scenario_list[i]->_bucket;
      std::cout << "  " << scenario_list[i]->_mapName;
      std::cout << "  " << scenario_list[i]->_map_width;
      std::cout << "  " << scenario_list[i]->_map_height;
      std::cout << "  " << scenario_list[i]->_start_x;
      std::cout << "  " << scenario_list[i]->_start_y;
      std::cout << "  " << scenario_list[i]->_goal_x;
      std::cout << "  " << scenario_list[i]->_goal_y;
      std::cout << std::fixed << std::setprecision(8) << "  "
                << scenario_list[i]->_optimal_length;
      std::cout << std::endl;
    }
  }

 private:
  int _version;

  int currIndex;

  unsigned int size;

  std::string _fileName;

  std::vector<Scenario *> scenario_list;
};

#endif

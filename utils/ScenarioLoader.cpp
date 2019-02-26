// File Reader for MPB

#include "ScenarioLoader.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <ompl/util/Console.h>

using namespace std;

// give a .scen file name, read the file and open corresponding map file
void ScenarioLoader::load(const string& fileName) {
  _scenarios.clear();
  _fileName = fileName;

  ifstream input_file(_fileName);

  if (input_file.fail()) {
    cerr << "Failed to open MovingAI scenario file " << fileName << endl;
    return;
  }

  // read in version number
  string temp;
  input_file >> temp;
  if (temp != "version" && temp != "Version") {
    cerr << "Scenario File Missing Version Number." << endl;
    return;
  }
  input_file >> _version;

  string line;
  // read scenarios and create Scenario objects
  while (getline(input_file, line)) {
    if (line.empty()) continue;
    stringstream ss(line);
    Scenario scenario;
    scenario.filename = fileName;
    ss >> scenario.id >> scenario.mapName >> scenario.map_width >>
        scenario.map_height >> scenario.start_x >> scenario.start_y >>
        scenario.goal_x >> scenario.goal_y >> scenario.optimal_length;
    // flip y coordinates of start and goal
//    scenario.start_y = scenario.map_height - scenario.start_y;
//    scenario.goal_y = scenario.map_height - scenario.goal_y;

    _scenarios.push_back(scenario);
  }

  OMPL_INFORM("Loaded %d Moving AI scenarios from %s.", _scenarios.size(), fileName.c_str());
}

void Scenario::loadMap() {
  ifstream map_file(mapName);
  if (map_file.fail() && filename.find('/') != std::string::npos) {
    mapName = filename.substr(0, filename.find_last_of('/')) + "/" + mapName;
    map_file = ifstream(mapName);
  }
  if (map_file.fail()) {
    cerr << "Failed to load map file from " << mapName << endl;
  }
  string temp;
  getline(map_file, temp);

  size_t height;
  size_t width;

  map_file >> temp >> height;
  map_file >> temp >> width;
  map_file >> temp;
  getline(map_file, temp);

  for (size_t i = 0; i < height; i++) {
    vector<char> new_row;
    _grid.push_back(new_row);
    string row;
    getline(map_file, row);
    for (size_t j = 0; j < width; j++) {
      _grid[i].push_back(row[j]);
    }
  }
}
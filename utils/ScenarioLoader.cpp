// File Reader for MPB

#include "ScenarioLoader.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

// give a .scen file name, read the file and open corresponding map file
void ScenarioLoader::load(const string& fileName) {
  _scenarios.clear();
  _fileName = fileName;

  ifstream input_file(_fileName);

  if (input_file.fail()) {
    cerr << "Failed to open MovingAI scenario file." << endl;
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
    ss >> scenario.bucket >> scenario.mapName >> scenario.map_width >>
        scenario.map_height >> scenario.start_x >> scenario.start_y >>
        scenario.goal_x >> scenario.goal_y >> scenario.optimal_length;
    _scenarios.push_back(scenario);
  }
}

void Scenario::loadMap() {
  ifstream map_file(mapName);
  if (map_file.fail()) {
    cerr << "Invalid Map File." << endl;
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
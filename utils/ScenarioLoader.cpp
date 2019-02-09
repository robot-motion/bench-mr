// File Reader for MPB

#include "ScenarioLoader.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

// give a .scen file name, read the file and open corresponding map file
ScenarioLoader::ScenarioLoader(const string& fileName) {
  _fileName = fileName;

  currIndex = 0;

  ifstream input_file(_fileName);

  if (input_file.fail()) {
    cerr << "Failed to Open Scenario File." << endl;
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
    if (line == "") {
      continue;
    }

    stringstream ss(line);

    int bucket;
    std::string mapName;
    int map_width;
    int map_height;
    int start_x;
    int start_y;
    int goal_x;
    int goal_y;
    double optimal_length;

    ss >> bucket >> mapName >> map_width >> map_height >> start_x >> start_y >>
        goal_x >> goal_y >> optimal_length;

    Scenario* curr_scen =
        new Scenario(bucket, mapName, map_width, map_height, start_x, start_y,
                     goal_x, goal_y, optimal_length);
    scenario_list.push_back(curr_scen);
  }

  size = scenario_list.size();
}

// get scenario by index
Scenario* ScenarioLoader::getScenario(size_t num) {
  if (num < 0 || num > scenario_list.size() - 1) {
    cerr << "Required Scenario Number Out of Range." << endl;
    return nullptr;
  }
  return scenario_list[num];
}

//iterate through entire senario list
Scenario* ScenarioLoader::getScenario() {
  if (currIndex > scenario_list.size() - 1) {
    cout << "Reached end of current list. Restarting from beginning" << endl;
    currIndex = 0;
  }

  return scenario_list[currIndex++];
}

// destructor
ScenarioLoader::~ScenarioLoader() {
  for (size_t i = 0; i < scenario_list.size(); i++) {
    delete scenario_list[i];
  }
}

void Scenario::loadMap() {
  ifstream map_file(_mapName);
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
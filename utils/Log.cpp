#include "Log.h"

#include <stdio.h>
#include <stdlib.h>

#include "PostSmoothing.h"

nlohmann::json Log::_json = {{"runs", nlohmann::json::array()}};
nlohmann::json Log::_currentRun;

void Log::instantiateRun() {
  auto time =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  char mbstr[100];
  std::strftime(mbstr, sizeof(mbstr), "%F %T", std::localtime(&time));
  std::string tstr(mbstr);
  tstr = tstr.substr(0, tstr.length() - 1);
  _currentRun = {{"globals",
                  {
                      {"time", tstr},
                  }},
                 {"runs", nlohmann::json::array()}};

  _currentRun.update(nlohmann::json(global::settings));
  _currentRun["settings"]["steering"] =
      Steering::to_string(global::settings.steer.steering_type);
}

void Log::log(const PathStatistics &stats) {
  nlohmann::json runStats = stats;
  runStats["ps_roundStats"] = PostSmoothing::statsPerRound;
  _currentRun["runs"].push_back(runStats);
}

void Log::log(const nlohmann::json &stats) {
  _currentRun["runs"].push_back(stats);
}

void Log::save(std::string filename, std::string path) {
  if (filename.empty()) filename = Log::filename() + (std::string) ".json";
  std::ofstream o(path + filename);
  o << std::setw(4) << _currentRun << std::endl;

  char *absFilename = nullptr;
  absFilename = realpath((path + filename).c_str(), absFilename);
  OMPL_INFORM("Saved path statistics log file at %s.", absFilename);
}

void Log::storeRun() { _json["runs"].push_back(_currentRun); }

std::string Log::filename() {
  return _currentRun["settings"]["steering"].get<std::string>() + " " +
         std::to_string(global::settings.environment->width()) + "x" +
         std::to_string(global::settings.environment->height()) + " " +
         global::settings.environment->name() + " " +
         _currentRun["globals"]["time"].get<std::string>();
}

std::vector<std::array<double, 2>> Log::serializePath(
    const std::vector<Point> &path) {
  std::vector<std::array<double, 2>> r;
  for (auto &p : path) r.push_back({p.x, p.y});
  return r;
}

std::vector<std::array<double, 3>> Log::serializeTrajectory(
    const ompl::geometric::PathGeometric &t) {
  const auto traj = PlannerUtils::interpolated(t);
  std::vector<std::array<double, 3>> r;
  for (auto i = 0u; i < traj.getStateCount(); ++i) {
    const auto *s = traj.getState(i)->as<State>();
    r.push_back({s->getX(), s->getY(), s->getYaw()});
  }
  return r;
}

#pragma once

#include <chrono>
#include <nlohmann/json.hpp>

#include <base/Primitives.h>
#include <ompl/control/PathControl.h>
#include "base/PathStatistics.hpp"

class Log {
 public:
  static void instantiateRun();

  static void log(const PathStatistics &stats);
  static void log(const nlohmann::json &stats);

  static void storeRun();

  static void save(std::string filename = "", const std::string &path = "log/");

  static std::string filename();

  static std::vector<std::array<double, 2>> serializePath(
      const std::vector<Point> &path);
  static std::vector<std::array<double, 3>> serializeTrajectory(
      const ompl::geometric::PathGeometric &traj, bool interpolate = true);

  static std::vector<std::array<double, 3>> serializeTrajectory(
      const ompl::control::PathControl &traj, bool interpolate = true);

 private:
  static nlohmann::json _json;
  static nlohmann::json _currentRun;
};

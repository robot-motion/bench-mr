#include <chrono>
#include <base/Trajectory.h>
#include <base/gnode.h>

#include "base/PathStatistics.hpp"

#include "json.hpp"


class Log
{
public:
    static void instantiateRun();

    static void log(const PathStatistics &stats);
    static void log(const nlohmann::json &stats);

    static void storeRun();

    static void save(std::string filename = "",
                     std::string path = "log/");

    static std::string filename();

    static std::vector<std::vector<double> > serializePath(const std::vector<Tpoint> &path);
    static std::vector<std::vector<double> > serializeTrajectory(const std::vector<GNode> &traj);

private:
    static nlohmann::json _json;
    static nlohmann::json _currentRun;

};

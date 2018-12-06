#include <base/PlannerUtils.hpp>
#include "SbplPlanner.h"


ob::PlannerStatus SbplPlanner::run() {
    _solution.clear();
    std::vector<int> stateIDs;
    Stopwatch stopwatch;
    stopwatch.start();
    OMPL_DEBUG("Replanning");
    if (dynamic_cast<ARAPlanner*> (_sbPlanner) != NULL) {
        ((ARAPlanner*)_sbPlanner)->costs_changed(); //use by ARA* planner (non-incremental)
    }
//    else if (dynamic_cast<ADPlanner*> (_sbPlanner) != NULL) {
//        // get the affected states
//        environment_navxythetalat.GetPredsofChangedEdges(&changedcellsV, &preds_of_changededgesIDV);
//        // let know the incremental planner about them
//        //use by AD* planner (incremental)
//        ((ADPlanner*)_sbPlanner)->update_preds_of_changededges(&preds_of_changededgesIDV);
//        printf("%d states were affected\n", (int)preds_of_changededgesIDV.size());
//    }
    OMPL_DEBUG("Notified of changes");
    int result = _sbPlanner->replan(PlannerSettings::PlanningTime, &stateIDs);
    OMPL_DEBUG("SBPL finished.");
    _planningTime = stopwatch.stop();
    if (result) {
        OMPL_INFORM("SBPL found a solution!");
//        int x, y;
//        for (auto id : stateIDs) {
//            _env2d.GetCoordFromState(id, x, y);
//            _solution.emplace_back(GNode(x, y));
//        }
        std::vector<sbpl_xy_theta_pt_t> xythetaPath;
        _env->ConvertStateIDPathintoXYThetaPath(&stateIDs, &xythetaPath);
        _solution.resize(xythetaPath.size());
        for (auto &xyt : xythetaPath)
            _solution.emplace_back(GNode(
                    xyt.x / PlannerSettings::sbplResolution,
                    xyt.y / PlannerSettings::sbplResolution,
                    xyt.theta));
    }
    return {result == 1, false};
}

std::vector<GNode> SbplPlanner::solutionTrajectory() const {
    return _solution;
}

std::vector<Tpoint> SbplPlanner::solutionPath() const {
    return PlannerUtils::toSteeredTrajectory(_solution).getPath();
}

og::PathGeometric SbplPlanner::geometricPath() const {
    og::PathGeometric path(ss->getSpaceInformation());
    if (_solution.empty())
    {
        OMPL_ERROR("SmoothTheta*: The computed path contains no GNodes!");
        return path;
    }
    for (auto &node : _solution)
    {
        auto *state = ss->getStateSpace()->allocState()->as<ob::SE2StateSpace::StateType>();
        state->setXY(node.x_r, node.y_r);
        state->setYaw(node.theta);
        path.append(state);
    }
    return path;
}

bool SbplPlanner::hasReachedGoalExactly() const {
    return !_solution.empty()
           && _solution.back().x_r == PlannerSettings::environment->goal().x
           && _solution.back().y_r == PlannerSettings::environment->goal().y;
}

double SbplPlanner::planningTime() const {
    return _planningTime;
}

SbplPlanner::SbplPlanner(SbplPlanner::SbplType type) {
    _env = new EnvironmentNAVXYTHETALAT;
//    const auto *mapData = PlannerSettings::environment->mapData();
//    std::cout << "config: " << (int) PlannerSettings::environment->start().x << " " <<
//            (int) PlannerSettings::environment->start().y << " " <<
//            (int) PlannerSettings::environment->goal().x << " " <<
//            (int) PlannerSettings::environment->goal().y << std::endl;
////    _env.InitializeEnvironment();
//    _env2d.InitializeEnv(PlannerSettings::environment->width(),
//                         PlannerSettings::environment->height(),
//                         mapData,
//                         (int) PlannerSettings::environment->start().x,
//                         (int) PlannerSettings::environment->start().y,
//                         (int) PlannerSettings::environment->goal().x,
//                         (int) PlannerSettings::environment->goal().y,
//                         1u
//    );
    // set the perimeter of the robot (it is given with 0,0,0 robot ref. point for which planning is done)
    vector<sbpl_2Dpt_t> perimeterptsV;
    sbpl_2Dpt_t pt_m;
    double halfwidth = 0.01; //0.3;
    double halflength = 0.01; //0.45;
    pt_m.x = -halflength;
    pt_m.y = -halfwidth;
    perimeterptsV.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = -halfwidth;
    perimeterptsV.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = halfwidth;
    perimeterptsV.push_back(pt_m);
    pt_m.x = -halflength;
    pt_m.y = halfwidth;
    perimeterptsV.push_back(pt_m);
    // clear the footprint
    perimeterptsV.clear();

    OMPL_DEBUG("Constructing SBPL Planner");
    std::cout << "Motion primitive filename: " << PlannerSettings::sbplMotionPrimitiveFilename << std::endl;
//    _mapData = PlannerSettings::environment->mapData();
    EnvNAVXYTHETALAT_InitParms params{};
    params.numThetas = PlannerSettings::sbplNumThetaDirs;
//    params.mapdata = _mapData;
    params.startx = PlannerSettings::environment->start().x * PlannerSettings::sbplResolution;
    params.starty = PlannerSettings::environment->start().y * PlannerSettings::sbplResolution;
    params.goalx = PlannerSettings::environment->goal().x * PlannerSettings::sbplResolution;
    params.goaly = PlannerSettings::environment->goal().y * PlannerSettings::sbplResolution;
    params.goaltol_x = PlannerSettings::sbplGoalToleranceX * PlannerSettings::sbplResolution;
    params.goaltol_y = PlannerSettings::sbplGoalToleranceY * PlannerSettings::sbplResolution;
    params.goaltol_theta = PlannerSettings::sbplGoalToleranceTheta;

    OMPL_DEBUG("Size: %d", sizeof(_mapData));

//    EnvironmentNAVXYTHETALAT env3;
    _env->InitializeEnv(PlannerSettings::environment->width(),
                       PlannerSettings::environment->height(),
                       perimeterptsV,
                       PlannerSettings::sbplResolution, // cell size
                       1., //PlannerSettings::sbplFordwardVelocity,
                       1., //PlannerSettings::sbplTimeToTurn45DegsInPlace,
                       1u, // obstacle threshold
                       PlannerSettings::sbplMotionPrimitiveFilename,
                       params
    );
    OMPL_DEBUG("Initialized SBPL environment");

    // Initialize MDP Info
//    MDPConfig MDPCfg;
//    if (!_env.InitializeMDPCfg(&MDPCfg)) {
//        throw SBPL_Exception("ERROR: InitializeMDPCfg failed");
//    }

    switch (type) {
        case SBPL_ARASTAR:
            _sbPlanner = new ARAPlanner(&_env2d, ForwardSearch);
            break;
        case SBPL_ADSTAR:
            _sbPlanner = new ADPlanner(&_env2d, ForwardSearch);
            break;
        case SBPL_RSTAR:
            _sbPlanner = new RSTARPlanner(&_env2d, ForwardSearch);
            break;
        case SBPL_ANASTAR:
            _sbPlanner = new anaPlanner(&_env2d, ForwardSearch);
            break;
    }

    OMPL_DEBUG("Initialized SBPL Planner");

    _sbPlanner->set_search_mode(PlannerSettings::sbplSearchUntilFirstSolution);
    _sbPlanner->set_initialsolution_eps(PlannerSettings::sbplInitialSolutionEps);
}

SbplPlanner::~SbplPlanner() {
    delete _sbPlanner;
    delete _env;
}

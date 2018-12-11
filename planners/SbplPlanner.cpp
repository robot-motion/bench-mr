#include <base/PlannerUtils.hpp>
#include "SbplPlanner.h"


SbplPlanner::SbplPlanner(SbplPlanner::SbplType type) {
    _env = new EnvironmentNAVXYTHETALAT;

    // set the perimeter of the robot (it is given with 0,0,0 robot ref. point for which planning is done)
    vector<sbpl_2Dpt_t> perimeterptsV;
    sbpl_2Dpt_t pt_m;
    double halfwidth = 0.3; //0.3;
    double halflength = 0.3; //0.45;
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
    _mapData = new unsigned char[PlannerSettings::environment->cells(PlannerSettings::sbplResolution)];
    PlannerSettings::environment->mapData(_mapData, PlannerSettings::sbplResolution);

    EnvNAVXYTHETALAT_InitParms params{};
    params.numThetas = PlannerSettings::sbplNumThetaDirs;
    params.mapdata = _mapData;  // TODO reactivate
    auto w = static_cast<unsigned int>(PlannerSettings::environment->width() / PlannerSettings::sbplResolution);
    auto h = static_cast<unsigned int>(PlannerSettings::environment->height() / PlannerSettings::sbplResolution);
    for (unsigned int y = 0; y <= h; ++y) {
        for (unsigned int x = 0; x <= w; ++x)
            std::cout << static_cast<unsigned>(_mapData[x + y * w]);
        std::cout << std::endl;
    }
    params.startx = PlannerSettings::environment->start().x;
    params.starty = PlannerSettings::environment->start().y;
    params.goalx = PlannerSettings::environment->goal().x;
    params.goaly = PlannerSettings::environment->goal().y;
    params.goaltol_x = PlannerSettings::sbplGoalToleranceX;
    params.goaltol_y = PlannerSettings::sbplGoalToleranceY;
    params.goaltol_theta = PlannerSettings::sbplGoalToleranceTheta;

//    OMPL_DEBUG("Size: %d", sizeof(_mapData));

//    EnvironmentNAVXYTHETALAT env3;
    _env->InitializeEnv(static_cast<int>(PlannerSettings::environment->width() / PlannerSettings::sbplResolution + 1),
                        static_cast<int>(PlannerSettings::environment->height() / PlannerSettings::sbplResolution + 1),
                        perimeterptsV,
                        PlannerSettings::sbplResolution, // cell size
                        PlannerSettings::sbplFordwardVelocity,
                        PlannerSettings::sbplTimeToTurn45DegsInPlace,
                        1u, // obstacle threshold
                        PlannerSettings::sbplMotionPrimitiveFilename,
                        params
    );
    OMPL_DEBUG("Initialized SBPL environment");

    // Initialize MDP Info
    MDPConfig MDPCfg{};
    if (!_env->InitializeMDPCfg(&MDPCfg)) {
        throw SBPL_Exception("ERROR: InitializeMDPCfg failed");
    }

    switch (type) {
        case SBPL_ARASTAR:
            _sbPlanner = new ARAPlanner(_env, ForwardSearch);
            break;
        case SBPL_ADSTAR:
            _sbPlanner = new ADPlanner(_env, ForwardSearch);
            break;
        case SBPL_RSTAR:
            _sbPlanner = new RSTARPlanner(_env, ForwardSearch);
            break;
        case SBPL_ANASTAR:
            _sbPlanner = new anaPlanner(_env, ForwardSearch);
            break;
    }

    OMPL_DEBUG("Initialized SBPL Planner");

    _sbPlanner->set_search_mode(PlannerSettings::sbplSearchUntilFirstSolution);
    _sbPlanner->set_initialsolution_eps(PlannerSettings::sbplInitialSolutionEps);

    _sbPlanner->set_start(_env->GetStateFromCoord(static_cast<int>(PlannerSettings::environment->start().x / PlannerSettings::sbplResolution),
                                                  static_cast<int>(PlannerSettings::environment->start().y / PlannerSettings::sbplResolution),
                                                  0));
    _sbPlanner->set_goal(_env->GetStateFromCoord(static_cast<int>(PlannerSettings::environment->goal().x / PlannerSettings::sbplResolution),
                                                 static_cast<int>(PlannerSettings::environment->goal().y / PlannerSettings::sbplResolution),
                                                 0));
}

SbplPlanner::~SbplPlanner() {
    delete _sbPlanner;
    delete _env;
    delete [] _mapData;
}

ob::PlannerStatus SbplPlanner::run() {
    _solution.clear();
    std::vector<int> stateIDs;
    Stopwatch stopwatch;
    stopwatch.start();
    OMPL_DEBUG("Replanning");
    if (dynamic_cast<ARAPlanner*> (_sbPlanner) != nullptr) {
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
//    _sbPlanner->InitializeSearchStateSpace();
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
//        _solution.resize(xythetaPath.size());
        for (auto &xyt : xythetaPath) {
            if (std::abs(xyt.x) < 1e-3 && std::abs(xyt.y) < 1e-3)
                continue;
            _solution.emplace_back(GNode(
                    xyt.x,  // / PlannerSettings::sbplResolution,
                    xyt.y,  // / PlannerSettings::sbplResolution,
                    xyt.theta));
        }
    } else {
        OMPL_WARN("SBPL found no solution.");
    }
    return {result == 1, false};
}

std::vector<GNode> SbplPlanner::solutionTrajectory() const {
    return _solution;
}

std::vector<Tpoint> SbplPlanner::solutionPath() const {
    // XXX SBPL already returns all the steered points
    std::vector<Tpoint> p;
    for (auto &node : _solution)
        p.emplace_back(Tpoint(node.x_r, node.y_r));
    return p;
//    return PlannerUtils::toSteeredTrajectory(_solution).getPath();
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

#include "SbplPlanner.h"
#include <base/PlannerUtils.hpp>

SbplPlanner::SbplPlanner(SbplPlanner::SbplType type)
    : _solution(og::PathGeometric(PlannerSettings::spaceInfo)) {
  _env = new EnvironmentNAVXYTHETALAT;

  // set the perimeter of the robot (it is given with 0,0,0 robot ref. point for
  // which planning is done)
  vector<sbpl_2Dpt_t> perimeterptsV;
  sbpl_2Dpt_t pt_m;
  double halfwidth = 0.3;   // 0.3;
  double halflength = 0.3;  // 0.45;
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

  std::cout << "Motion primitive filename: "
            << PlannerSettings::sbplMotionPrimitiveFilename << std::endl;

  _env->InitializeEnv(static_cast<int>(PlannerSettings::environment->width() *
                                       PlannerSettings::sbplScaling),
                      static_cast<int>(PlannerSettings::environment->height() *
                                       PlannerSettings::sbplScaling),
                      nullptr,  // mapdata
                      0, 0, 0,  // start (x, y, theta, t)
                      0, 0, 0,  // goal (x, y, theta)
                      0, 0, 0,  // goal tolerance
                      perimeterptsV,
                      PlannerSettings::sbplResolution,  // cell size
                      PlannerSettings::sbplFordwardVelocity,
                      PlannerSettings::sbplTimeToTurn45DegsInPlace,
                      20u,  // obstacle threshold
                      PlannerSettings::sbplMotionPrimitiveFilename);
  for (int ix(0); ix < PlannerSettings::environment->width() *
                           PlannerSettings::sbplScaling;
       ++ix)
    for (int iy(0); iy < PlannerSettings::environment->height() *
                             PlannerSettings::sbplScaling;
         ++iy)
      _env->UpdateCost(
          ix, iy,
          static_cast<unsigned char>(PlannerSettings::environment->occupied(
                                         ix / PlannerSettings::sbplScaling,
                                         iy / PlannerSettings::sbplScaling)
                                         ? 20u
                                         : 1u));
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

  int startTheta = 0, goalTheta = 0;
  if (PlannerSettings::estimateTheta) {
    auto orientations =
        PlannerSettings::environment->estimateStartGoalOrientations();
    // handle weird behavior when start and goal nodes appear on different sides
    // as usual
    // TODO verify
    if (PlannerSettings::environment->start().x <
        PlannerSettings::environment->goal().x) {
      startTheta =
          static_cast<int>(std::round((orientations.first) / M_PI *
                                      PlannerSettings::sbplNumThetaDirs)) %
          PlannerSettings::sbplNumThetaDirs;
      goalTheta =
          static_cast<int>(std::round((orientations.second) / M_PI *
                                      PlannerSettings::sbplNumThetaDirs)) %
          PlannerSettings::sbplNumThetaDirs;
    } else {
      startTheta =
          static_cast<int>(std::round((orientations.first + M_PI / 2) / M_PI *
                                      PlannerSettings::sbplNumThetaDirs)) %
          PlannerSettings::sbplNumThetaDirs;
      goalTheta =
          static_cast<int>(std::round((orientations.second + M_PI / 2) / M_PI *
                                      PlannerSettings::sbplNumThetaDirs)) %
          PlannerSettings::sbplNumThetaDirs;
    }

    std::cout << "startTheta: " << (orientations.first * 180. / M_PI)
              << " deg   " << startTheta << std::endl;
    std::cout << "goalTheta: " << (orientations.second * 180. / M_PI)
              << " deg   " << goalTheta << std::endl;

//#if DEBUG
#if QT_SUPPORT
    QtVisualizer::drawNode(PlannerSettings::environment->start().x,
                           PlannerSettings::environment->start().y,
                           orientations.first);
    QtVisualizer::drawNode(PlannerSettings::environment->goal().x,
                           PlannerSettings::environment->goal().y,
                           orientations.second);
#endif
    //#endif
  }

  _sbPlanner->set_search_mode(PlannerSettings::sbplSearchUntilFirstSolution);
  _sbPlanner->set_initialsolution_eps(PlannerSettings::sbplInitialSolutionEps);

  _sbPlanner->set_start(_env->GetStateFromCoord(
      static_cast<int>(std::round(PlannerSettings::environment->start().x *
                                  PlannerSettings::sbplScaling)),
      static_cast<int>(std::round(PlannerSettings::environment->start().y *
                                  PlannerSettings::sbplScaling)),
      startTheta));
  _sbPlanner->set_goal(_env->GetStateFromCoord(
      static_cast<int>(std::round(PlannerSettings::environment->goal().x *
                                  PlannerSettings::sbplScaling)),
      static_cast<int>(std::round(PlannerSettings::environment->goal().y *
                                  PlannerSettings::sbplScaling)),
      goalTheta));
}

SbplPlanner::~SbplPlanner() {
  delete _sbPlanner;
  delete _env;
}

ob::PlannerStatus SbplPlanner::run() {
  _solution.clear();
  std::vector<int> stateIDs;
  Stopwatch stopwatch;
  stopwatch.start();
  OMPL_DEBUG("Replanning");
  if (dynamic_cast<ARAPlanner *>(_sbPlanner) != nullptr) {
    ((ARAPlanner *)_sbPlanner)
        ->costs_changed();  // use by ARA* planner (non-incremental)
  }
  //    else if (dynamic_cast<ADPlanner*> (_sbPlanner) != NULL) {
  //        // get the affected states
  //        environment_navxythetalat.GetPredsofChangedEdges(&changedcellsV,
  //        &preds_of_changededgesIDV);
  //        // let know the incremental planner about them
  //        //use by AD* planner (incremental)
  //        ((ADPlanner*)_sbPlanner)->update_preds_of_changededges(&preds_of_changededgesIDV);
  //        printf("%d states were affected\n",
  //        (int)preds_of_changededgesIDV.size());
  //    }
  OMPL_DEBUG("Notified of changes");
  //    _sbPlanner->InitializeSearchStateSpace();
  int result = _sbPlanner->replan(PlannerSettings::PlanningTime, &stateIDs);
  OMPL_DEBUG("SBPL finished.");
  _planningTime = stopwatch.stop();
  if (result) {
    OMPL_INFORM("SBPL found a solution!");
    std::vector<sbpl_xy_theta_pt_t> xythetaPath;
    _env->ConvertStateIDPathintoXYThetaPath(&stateIDs, &xythetaPath);
    for (auto &xyt : xythetaPath) {
      if (std::abs(xyt.x) < 1e-3 && std::abs(xyt.y) < 1e-3) continue;
      _solution.append(
          base::StateFromXYT(xyt.x / PlannerSettings::sbplResolution /
                                 PlannerSettings::sbplScaling,
                             xyt.y / PlannerSettings::sbplResolution /
                                 PlannerSettings::sbplScaling,
                             xyt.theta));
    }
  } else {
    OMPL_WARN("SBPL found no solution.");
  }
  return {result == 1, false};
}

og::PathGeometric SbplPlanner::solution() const { return _solution; }

bool SbplPlanner::hasReachedGoalExactly() const {
  if (_solution.getStateCount() == 0) return false;
  const auto *last =
      _solution
          .getState(static_cast<unsigned int>(_solution.getStateCount() - 1))
          ->as<State>();
  return last->getX() == PlannerSettings::environment->goal().x &&
         last->getY() == PlannerSettings::environment->goal().y;
}

double SbplPlanner::planningTime() const { return _planningTime; }

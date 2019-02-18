#include "SbplPlanner.h"
#include <base/PlannerUtils.hpp>

SbplPlanner::SbplPlanner(SbplPlanner::SbplType type)
    : _solution(og::PathGeometric(settings.ompl.space_info)) {
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
            << settings.sbpl.motion_primitive_filename << std::endl;

  _env->InitializeEnv(
      static_cast<int>(settings.environment->width() * settings.sbpl.scaling),
      static_cast<int>(settings.environment->height() * settings.sbpl.scaling),
      nullptr,  // mapdata
      0, 0, 0,  // start (x, y, theta, t)
      0, 0, 0,  // goal (x, y, theta)
      0, 0, 0,  // goal tolerance
      perimeterptsV,
      settings.sbpl.resolution,  // cell size
      settings.sbpl.fordward_velocity,
      settings.sbpl.time_to_turn_45_degs_in_place,
      20u,  // obstacle threshold
      settings.sbpl.motion_primitive_filename.value().c_str());
  for (int ix(0); ix < settings.environment->width() * settings.sbpl.scaling;
       ++ix)
    for (int iy(0); iy < settings.environment->height() * settings.sbpl.scaling;
         ++iy)
      _env->UpdateCost(
          ix, iy,
          static_cast<unsigned char>(
              settings.environment->occupied(ix / settings.sbpl.scaling,
                                             iy / settings.sbpl.scaling)
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
  if (settings.estimate_theta) {
    auto orientations = settings.environment->estimateStartGoalOrientations();
    // handle weird behavior when start and goal nodes appear on different sides
    // as usual
    // TODO verify
    if (settings.environment->start().x < settings.environment->goal().x) {
      startTheta = static_cast<int>(std::round((orientations.first) / M_PI *
                                               settings.sbpl.num_theta_dirs)) %
                   settings.sbpl.num_theta_dirs;
      goalTheta = static_cast<int>(std::round((orientations.second) / M_PI *
                                              settings.sbpl.num_theta_dirs)) %
                  settings.sbpl.num_theta_dirs;
    } else {
      startTheta =
          static_cast<int>(std::round((orientations.first + M_PI / 2) / M_PI *
                                      settings.sbpl.num_theta_dirs)) %
          settings.sbpl.num_theta_dirs;
      goalTheta =
          static_cast<int>(std::round((orientations.second + M_PI / 2) / M_PI *
                                      settings.sbpl.num_theta_dirs)) %
          settings.sbpl.num_theta_dirs;
    }

    std::cout << "startTheta: " << (orientations.first * 180. / M_PI)
              << " deg   " << startTheta << std::endl;
    std::cout << "goalTheta: " << (orientations.second * 180. / M_PI)
              << " deg   " << goalTheta << std::endl;

//#if DEBUG
#if QT_SUPPORT
    QtVisualizer::drawNode(settings.environment->start().x,
                           settings.environment->start().y, orientations.first);
    QtVisualizer::drawNode(settings.environment->goal().x,
                           settings.environment->goal().y, orientations.second);
#endif
    //#endif
  }

  _sbPlanner->set_search_mode(settings.sbpl.search_until_first_solution);
  _sbPlanner->set_initialsolution_eps(settings.sbpl.initial_solution_eps);

  _sbPlanner->set_start(_env->GetStateFromCoord(
      static_cast<int>(
          std::round(settings.environment->start().x * settings.sbpl.scaling)),
      static_cast<int>(
          std::round(settings.environment->start().y * settings.sbpl.scaling)),
      startTheta));
  _sbPlanner->set_goal(_env->GetStateFromCoord(
      static_cast<int>(
          std::round(settings.environment->goal().x * settings.sbpl.scaling)),
      static_cast<int>(
          std::round(settings.environment->goal().y * settings.sbpl.scaling)),
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
  int result = _sbPlanner->replan(settings.ompl.max_planning_time, &stateIDs);
  OMPL_DEBUG("SBPL finished.");
  _planningTime = stopwatch.stop();
  if (result) {
    OMPL_INFORM("SBPL found a solution!");
    std::vector<sbpl_xy_theta_pt_t> xythetaPath;
    _env->ConvertStateIDPathintoXYThetaPath(&stateIDs, &xythetaPath);
    for (auto &xyt : xythetaPath) {
      if (std::abs(xyt.x) < 1e-3 && std::abs(xyt.y) < 1e-3) continue;
      _solution.append(base::StateFromXYT(
          xyt.x / settings.sbpl.resolution / settings.sbpl.scaling,
          xyt.y / settings.sbpl.resolution / settings.sbpl.scaling, xyt.theta));
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
  return last->getX() == settings.environment->goal().x &&
         last->getY() == settings.environment->goal().y;
}

double SbplPlanner::planningTime() const { return _planningTime; }

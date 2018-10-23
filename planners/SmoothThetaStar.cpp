#include <base/PlannerUtils.hpp>
#include "SmoothThetaStar.h"
#include "smooth_stl_thetastar.h"


#define DEBUG_LISTS 1

SmoothThetaStar::SmoothThetaStar() : AbstractPlanner(), ob::Planner(ss->getSpaceInformation(), "Smooth Theta*")
{
    curr_traj = new Trajectory();

    srand((unsigned int) (time(nullptr)));

    /// Euclidean Cost
    COST_SEARCH = true;

    // USE_ASTAR
    USE_ASTAR = false;

    USE_GRANDPARENT = true;

    _planningTime = 0;

    pdef_ = std::make_shared<ompl::base::ProblemDefinition>(ss->getSpaceInformation());
}

SmoothThetaStar::SmoothThetaStar(bool astar, std::string name) : AbstractPlanner(), ob::Planner(ss->getSpaceInformation(),
                                                                                    std::move(name))
{
    curr_traj = new Trajectory();

    srand((unsigned int) (time(nullptr)));

    /// Euclidean Cost
    COST_SEARCH = true;

    // USE_ASTAR
    USE_ASTAR = astar;

    USE_GRANDPARENT = true;

    _planningTime = 0;

    pdef_ = std::make_shared<ompl::base::ProblemDefinition>(ss->getSpaceInformation());
}

bool SmoothThetaStar::initialize()
{
    return true;
}

bool SmoothThetaStar::search(std::vector<std::vector<GNode> > &paths, GNode start, GNode goal)
{
    paths.clear();

    OMPL_DEBUG("Smooth Theta*: Start: %d, %d --- Goal: %d, %d ", start.x, start.y, goal.x, goal.y);

    std::vector<GNode> sol;
    std::vector<std::vector<GNode> > path_sol;

    SmoothThetaStarSearch<GNode> thetastarsearch(COST_SEARCH);

    if (USE_ASTAR)
        thetastarsearch.useAstar();

    if (USE_GRANDPARENT)
        thetastarsearch.use_connectGrandParent();

    unsigned int SearchCount = 0;

    const unsigned int NumSearches = 1;

    GNode *p, *lastOpen = nullptr;
    while (SearchCount < NumSearches)
    {
        // Set Start and goal states
        thetastarsearch.SetStartAndGoalStates(start, goal);

        unsigned int SearchState;
        unsigned int SearchSteps = 0;
        do
        {
            SearchState = thetastarsearch.SearchStep();

            SearchSteps++;

            if (SearchState != SmoothThetaStarSearch<GNode>::SEARCH_STATE_SEARCHING)
                break;
#if DEBUG_LISTS
            OMPL_INFORM("SmoothTheta* Step: %d", (int)SearchSteps);

            int len = 0;

//            OMPL_INFORM("Open:");
            p = thetastarsearch.GetOpenListStart();
            lastOpen = p;
            if (p == nullptr)
                OMPL_INFORM("SmoothTheta*: No open nodes");
//            else
//                QtVisualizer::drawNode(*p);

            QtVisualizer::saveScene();
            double dx, dy;
            double eta = 0.25;
            while (p)
            {
                len++;

                if (gradientDescentOpenVertices)
                {
                    PlannerSettings::environment->distanceGradient(p->x_r, p->y_r, dx, dy, 1.);
                    double distance = PlannerSettings::environment->bilinearDistance(p->x_r, p->y_r);
                    distance = std::max(.1, distance);
                    p->x_r -= eta * dx / distance;
                    p->y_r += eta * dy / distance;
                }

//#if !DEBUG_LIST_LENGTHS_ONLY
//                //p->PrintNodeInfo();
//                if (len > 1)
                    QtVisualizer::drawNode(*p, Qt::darkYellow, 0.3, false);

//#endif

                p = thetastarsearch.GetOpenListNext();
            }
//            OMPL_INFORM("Open list has %d",len);

//            len = 0;

//            OMPL_INFORM("Closed");
            p = thetastarsearch.GetClosedListStart();
            while (p)
            {
                len++;

//#if !DEBUG_LIST_LENGTHS_ONLY
//                //p->PrintNodeInfo();
//                QtVisualizer::drawNode(*p);
//#endif
                QtVisualizer::drawNode(*p, Qt::gray, 0.3, false);

                p = thetastarsearch.GetClosedListNext();

            }
//            OMPL_INFORM("Closed list has %d nodes",len);

            SmoothThetaStarSearch<GNode>::Node *node = thetastarsearch.GetCurrentBestRawNode(); //thetastarsearch.GetSolutionStart();
            if (node == nullptr)
                continue;
//            sol.push_back(GNode(node->m_UserState.x, node->m_UserState.y, node->m_UserState.theta));
            bool repeating = false;
//            _publisher.publish(_publisher._createNodeMarker(&(node->m_UserState), .1f, .7f, .1f));
            std::vector<GNode> reached;
            reached.push_back(node->m_UserState);
            std::vector<SmoothThetaStarSearch<GNode>::Node *> path;
            path.insert(path.begin(), node);
            while ((node = node->parent) && !repeating)
            {
//                OMPL_INFORM("Found parent at %d %d", node->m_UserState.x, node->m_UserState.y);
                for (GNode &r : reached)
                {
                    if (r.x == node->m_UserState.x && r.y == node->m_UserState.y)
                    {
                        repeating = true;
                        break;
                    }
                }
                if (repeating)
                    break;
//                sol.push_back(GNode(node->m_UserState.x_r, node->m_UserState.y_r, node->m_UserState.theta));
                reached.push_back(node->m_UserState);
                path.insert(path.begin(), node);
//                pub_open_closed_.publish(_createNodeMarker(&(node->m_UserState), .1f, .7f, .1f));
            }

            bool preventCollisions = true;
            bool AverageAngles = true;
            if (averageAngles && path.size() > 2)
            {
                double theta_old = path[0]->m_UserState.theta;
                path[0]->m_UserState.theta = PlannerUtils::slope(path[0]->m_UserState, path[1]->m_UserState);
                if (preventCollisions && PlannerUtils::collides(path[0]->m_UserState, path[1]->m_UserState))
                    path[0]->m_UserState.theta = theta_old; // revert setting
                for (int i = 1; i < path.size() - 1; ++i)
                {
                    theta_old = path[i]->m_UserState.theta;
                    if (AverageAngles)
                    {
                        double l = PlannerUtils::slope(path[i - 1]->m_UserState, path[i]->m_UserState);
                        double r = PlannerUtils::slope(path[i]->m_UserState, path[i + 1]->m_UserState);
                        if (std::abs(l - r) >= M_PI)
                        {
                            if (l > r)
                                l += 2. * M_PI;
                            else
                                r += 2. * M_PI;
                        }
                        path[i]->m_UserState.theta = (l + r) * 0.5;
                    }
                    else
                        path[i]->m_UserState.theta = PlannerUtils::slope(path[i - 1]->m_UserState, path[i]->m_UserState);

                    if (preventCollisions && (PlannerUtils::collides(path[i-1]->m_UserState, path[i]->m_UserState) || PlannerUtils::collides(path[i]->m_UserState, path[i+1]->m_UserState)))
                        path[i]->m_UserState.theta = theta_old; // revert setting
                }
                theta_old = path[path.size() - 1]->m_UserState.theta;
                path[path.size() - 1]->m_UserState.theta = PlannerUtils::slope(path[path.size() - 2]->m_UserState, path[path.size() - 1]->m_UserState);
                if (preventCollisions && PlannerUtils::collides(path[path.size() - 1]->m_UserState, path[path.size() - 2]->m_UserState))
                    path[path.size() - 1]->m_UserState.theta = theta_old; // revert setting
            }
            else if (averageAngles && path.size() == 2)
            {
                double theta_old0 = path[0]->m_UserState.theta;
                double theta_old1 = path[1]->m_UserState.theta;
                // try to set angle like a straight line connecting both vertices
                path[0]->m_UserState.theta = PlannerUtils::slope(path[0]->m_UserState, path[1]->m_UserState);
                path[1]->m_UserState.theta = path[0]->m_UserState.theta;
                if (preventCollisions && PlannerUtils::collides(path[0]->m_UserState, path[1]->m_UserState))
                {
                    path[0]->m_UserState.theta = theta_old0;
                    path[1]->m_UserState.theta = theta_old1;
                    OMPL_WARN("REVERTING avg angles for path of length 2.");
                }
                OMPL_INFORM("Averaged angles for path length 2.");
            }

            reached.clear();
            for (auto *n : path)
            {
                reached.insert(reached.begin(), n->m_UserState);
            }

            //TODO reactivate PNG saving
            QtVisualizer::drawTrajectory(reached, Qt::blue, 1.1f);
            QtVisualizer::drawNodes(reached, Qt::blue);

            QtVisualizer::savePng(QString("log/step_%1.png").arg((int)SearchSteps, 5, 10, QChar('0')));
            QtVisualizer::restoreScene();
//
//            OMPL_INFORM("Generated partial path of length %d", (int)sol.size());
////            if (sol.size() > 1)
////                _publisher.publishGlobalTraj(sol, false, 1);
//
//            sol.clear();
#endif

#if INTERACTIVE_STEP_THROUGH
            OMPL_INFORM("Press key to proceed");
            std::cin.get();
#endif

#if STEP_DELAY
            usleep(100000); // in microseconds
#endif
        }
        while (SearchState == SmoothThetaStarSearch<GNode>::SEARCH_STATE_SEARCHING);


        if (SearchState == SmoothThetaStarSearch<GNode>::SEARCH_STATE_SUCCEEDED)
        {
            OMPL_DEBUG("SmoothTheta* search found goal state.");

            GNode *node = thetastarsearch.GetSolutionStart();
            int steps = 0;
            int xs, ys;

            xs = node->x;
            ys = node->y;
            sol.emplace_back(GNode(xs, ys, node->theta, node->steer, node->steer_cost, node->costs, node->orientations));

            while ((node = thetastarsearch.GetSolutionNext()))
            {
                xs = (int) node->x_r;
                ys = (int) node->y_r;
                sol.emplace_back(*node);
                GNode s(xs, ys,
                        node->theta,
                        node->steer,
                        node->steer_cost,
                        node->costs,
                        node->orientations,
                        node->x_r,
                        node->y_r);
                steps++;
            }

            if (averageAngles)
            {
                PlannerUtils::updateAngles(sol);
            }

            // Once you're done with the solution you can free the nodes up
            thetastarsearch.FreeSolutionNodes();
        }
        else if (SearchState == SmoothThetaStarSearch<GNode>::SEARCH_STATE_FAILED)
        {
            OMPL_ERROR("SmoothTheta* search terminated. Did not find goal state.");
        }

        // Display the number of loops the search went through
        OMPL_DEBUG("SmoothTheta* SearchSteps: %d ", (int) SearchSteps);

        SearchCount++;

        thetastarsearch.EnsureMemoryFreed();
    }

    paths.push_back(sol);

    unsigned long path_size = paths.size();
    OMPL_INFORM("SmoothTheta* found %d path(s).", (int)path_size);

    return path_size > 0;
}


ob::PlannerStatus SmoothThetaStar::run()
{
    ob::ScopedState<> start(ss->getStateSpace());
    start[0] = PlannerSettings::environment->start().x;
    start[1] = PlannerSettings::environment->start().y;
    start[2] = 0;
    ob::ScopedState<> goal(ss->getStateSpace());
    goal[0] = PlannerSettings::environment->goal().x;
    goal[1] = PlannerSettings::environment->goal().y;
    goal[2] = 0;

    pdef_->setStartAndGoalStates(start, goal);

    return ob::Planner::solve(PlannerSettings::PlanningTime);
}

og::PathGeometric SmoothThetaStar::geometricPath() const
{
    og::PathGeometric path(ss->getSpaceInformation());
    auto gnodes = global_paths[0];
    if (gnodes.empty())
    {
        OMPL_ERROR("SmoothTheta*: The computed path contains no GNodes!");
        return path;
    }
    for (auto &node : gnodes)
    {
        auto *state = ss->getStateSpace()->allocState()->as<ob::SE2StateSpace::StateType>();
        state->setXY(node.x_r, node.y_r);
        state->setYaw(node.theta);
        path.append(state);
    }
    return path;
}

std::vector<GNode> SmoothThetaStar::solutionTrajectory() const
{
    return global_paths[0];
}

std::vector<Tpoint> SmoothThetaStar::solutionPath() const
{
    return PlannerUtils::toSteeredTrajectory(global_paths[0]).getPath();
}

bool SmoothThetaStar::hasReachedGoalExactly() const
{
    return true;
}

double SmoothThetaStar::planningTime() const
{
    return _planningTime;
}

void SmoothThetaStar::setProblemDefinition(const ob::ProblemDefinitionPtr &pdef)
{
    pdef_ = pdef;
}

ob::PlannerStatus SmoothThetaStar::solve(const ob::PlannerTerminationCondition &ptc)
{
    pdef_->clearSolutionPaths();
    pdef_->clearSolutionNonExistenceProof();

    auto *goal = dynamic_cast<ob::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return ob::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return ob::PlannerStatus::INVALID_GOAL;
    }

    auto *goalState = ss->getStateSpace()->allocState();
    goal->sampleGoal(goalState);
    GNode goalNode(goalState->as<ob::SE2StateSpace::StateType>()->getX(),
                   goalState->as<ob::SE2StateSpace::StateType>()->getY());

    auto *startState = pdef_->getStartState(0)->as<ob::SE2StateSpace::StateType>();
    GNode startNode(startState->getX(), startState->getY());

    curr_traj->reset();
    global_paths.clear();

    PlannerSettings::steering->clearInternalData();

    OMPL_DEBUG("SmoothTheta*: Generate a new global path");
    Stopwatch sw;
    sw.start();
    search(global_paths, startNode, goalNode);
    sw.stop();
    _planningTime = sw.time;

    OMPL_INFORM("SmoothTheta* search finished");
    OMPL_DEBUG("Global path size: %d", (int) global_paths[0].size());
    if ((int) global_paths[0].size() == 0)
    {
        OMPL_WARN("SmoothTheta*: No Path found");
        return ob::PlannerStatus::ABORT;
    }

    for (auto &gnodes : global_paths)
    {
        auto path(std::make_shared<og::PathGeometric>(ss->getSpaceInformation()));
        for (auto &node: gnodes)
        {
            auto *state = ss->getStateSpace()->allocState()->as<ob::SE2StateSpace::StateType>();
            state->setXY(node.x_r, node.y_r);
            state->setYaw(node.theta);
            path->append(state);
        }
        pdef_->addSolutionPath(path, false, 0.0, getName());
    }

    return ob::PlannerStatus::EXACT_SOLUTION;
}

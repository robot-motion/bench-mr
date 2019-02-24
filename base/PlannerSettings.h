#pragma once

#include <chomp/Chomp.h>
#include <params.hpp>

#include "Environment.h"
#include "steer_functions/Steering.h"

using namespace params;

namespace chomp {
enum ChompInitialization { STRAIGHT_LINE, THETA_STAR, THETA_STAR_X_CLEARING };
// clang-format off
//NLOHMANN_JSON_SERIALIZE_ENUM(ChompInitialization, {
//  {STRAIGHT_LINE, "straight_line"},
//  {THETA_STAR, "theta_star"},
//  {THETA_STAR_X_CLEARING, "theta_star_xclearing"},
//})
// clang-format on
}  // namespace chomp

namespace sbpl {
enum Planner { SBPL_ARASTAR, SBPL_ADSTAR, SBPL_RSTAR, SBPL_ANASTAR };
}

namespace robot {
enum Model { ROBOT_POINT, ROBOT_POLYGON };
}

namespace distance_computation {
enum Method { BRUTE_FORCE, DEAD_RECKONING };
inline std::string to_string(Method m) {
  switch (m) {
    case BRUTE_FORCE:
      return "BRUTE_FORCE";
    default:
      return "DEAD_RECKONING";
  }
}
}  // namespace distance_computation

namespace PlannerSettings {
/**
 * Global settings.
 */
struct GlobalSettings : public Group {
  using Group::Group;
  Environment *environment{nullptr};

  /**
   * Whether to log the distances precomputed by the grid mazes.
   */
  Property<bool> log_env_distances{false, "log_env_distances", this};

  Property<bool> auto_choose_distance_computation_method{
      false, "auto_choose_distance_computation_method", this};
  Property<distance_computation::Method> distance_computation_method{
      distance_computation::BRUTE_FORCE, "distance_computation_method",
      this};

  Property<double> max_planning_time{15, "max_planning_time", this};

  /**
   * For maps with more cells than this threshold, a fast, approximate algorithm
   * is used to compute the obstacle distance field (necessary for clearance
   * evaluations and GRIPS).
   */
  Property<unsigned int> fast_odf_threshold{100 * 100, "fast_odf_threshold",
                                            this};

  /**
   * Whether to estimate the orientation of the start and goal states for
   * planners that need them (e.g. SBPL).
   */
  Property<bool> estimate_theta{false, "estimate_theta", this};

  /**
   * Whether to compute stats on clearing distances of the found solutions.
   */
  Property<bool> evaluate_clearing{true, "evaluate_clearing", this};

  /**
   * Which model is used for collision checking.
   */
  Property<robot::Model> collision_model{robot::ROBOT_POINT, "collision_model",
                                         this};

  Property<Polygon> robot_shape{Polygon(), "robot_shape", this};

  /**
   * Radius threshold to evaluate whether the exact goal has been found.
   */
  Property<double> exact_goal_radius{1e-2, "exact_goal_radius", this};

  /**
   * Settings related to OMPL.
   */
  struct OmplSettings : public Group {
    using Group::Group;

    ompl::base::StateSpacePtr state_space{nullptr};
    ompl::base::SpaceInformationPtr space_info{nullptr};
    ompl::base::OptimizationObjectivePtr objective{nullptr};

    Property<double> state_equality_tolerance{1e-4, "state_equality_tolerance",
                                              this};
    Property<double> cost_threshold{100, "cost_threshold", this};

    Property<unsigned int> seed{0, "seed", this};
  } ompl{"ompl", this};

  struct SteerSettings : public Group {
    using Group::Group;

    /**
     * Initializes OMPL state space, space information and optimization
     * objective for the given steer function global::settings.
     */
    void initializeSteering() const;

    Property<Steering::SteeringType> steering_type{
        Steering::STEER_TYPE_REEDS_SHEPP, "steering_type", this};
    Property<double> car_turning_radius{4, "car_turning_radius", this};

    /**
     * Distance between states sampled using the steer function for collision
     * detection, rendering and evaluation.
     */
    Property<double> sampling_resolution{0.015, "sampling_resolution", this};

    struct HC_CC_Settings : public Group {
      using Group::Group;

      Property<double> kappa{5, "kappa", this};
      Property<double> sigma{0.5, "sigma", this};
    } hc_cc{"hc_cc", this};
  } steer{"steer", this};

  struct SbplSettings : public Group {
    using Group::Group;

    /**
     * Search until it finds a solution (even if allotted time is over)?
     */
    Property<bool> search_until_first_solution{
        false, "search_until_first_solution", this};
    /**
     * TODO verify description
     * How much deviation from optimal solution is acceptable (>1)? Ignored by
     * planners that don't have notion of epsilon, 1 means optimal search.
     */
    Property<double> initial_solution_eps{30, "initial_solution_eps", this};
    Property<double> fordward_velocity{0.4, "fordward_velocity",
                                       this};  // in meters/sec
    Property<double> time_to_turn_45_degs_in_place{
        0.6, "time_to_turn_45_degs_in_place", this};  // in sec
    //    Property<std::string> motion_primitive_filename{
    //        "./sbpl_mprim/pr2.mprim", "motion_primitive_filename", this};
    Property<std::string> motion_primitive_filename{
        "./sbpl_mprim/unicycle_0.25.mprim", "motion_primitive_filename", this};

    /**
     * These tolerances are most likely ignored by SPBL at the moment.
     */
    Property<double> goal_tolerance_x{1, "goal_tolerance_x", this};
    Property<double> goal_tolerance_y{1, "goal_tolerance_y", this};
    Property<double> goal_tolerance_theta{2 * M_PI, "goal_tolerance_theta",
                                          this};

    /**
     * XXX Important: resolution must match resolution in motion primitive
     * definition file.
     */
    //    Property<double> resolution{0.025, "resolution", this};
    Property<double> resolution{0.25, "resolution", this};

    /**
     * Scale environment to accommodate extents of SBPL's motion primitives.
     * Intuition: the smaller this number the larger the turning radius.
     */
    Property<double> scaling{1.5, "scaling", this};

    /**
     * XXX Important: number of theta directions must match resolution in motion
     * primitive definition file.
     */
    Property<unsigned int> num_theta_dirs{16u, "num_theta_dirs", this};

    Property<sbpl::Planner> planner{sbpl::SBPL_ARASTAR, "planner", this};
  } sbpl{"sbpl", this};

  struct ChompSettings : public Group {
    using Group::Group;

    Property<unsigned int> nodes{300u, "nodes", this};
    Property<double> alpha{0.05, "alpha", this};
    /**
     * Obstacle importance.
     */
    Property<float> epsilon{4, "epsilon", this};
    Property<double> gamma{0.8, "gamma", this};
    Property<double> error_tolerance{1e-6, "error_tolerance", this};
    /**
     * Here, the number for global and local iterations is the same.
     */
    Property<unsigned int> max_iterations{1500u, "max_iterations", this};
    Property<chomp::ChompObjectiveType> objective_type{chomp::MINIMIZE_VELOCITY,
                                                       "objective_type", this};
    Property<chomp::ChompInitialization> initialization{chomp::THETA_STAR,
                                                        "initialization", this};
  } chomp{"chomp", this};

  struct GripsSettings : public Group {
    using Group::Group;

    /*
     * Minimum distance to be maintained between two consecutive nodes.
     */
    Property<double> min_node_distance{3, "min_node_distance", this};
    /*
     * Gradient descent rate.
     */
    Property<double> eta{0.5, "eta", this};
    /*
     * Discount factor for gradient descent rate.
     */
    Property<double> eta_discount{0.8, "eta_discount", this};

    /**
     * Number of gradient descent rounds.
     */
    Property<unsigned int> gradient_descent_rounds{5, "gradient_descent_rounds",
                                                   this};
    /**
     * Maximum number of pruning rounds after which the algorithm
     * should terminate.
     */
    Property<unsigned int> max_pruning_rounds{100, "max_pruning_rounds", this};
  } grips{"grips", this};
  struct SmoothThetaStarSettings : public Group {
    using Group::Group;

    Property<bool> gradient_descent_open_nodes{
        true, "gradient_descent_open_nodes", this};
    Property<bool> annealedGradientdescent_open_nodes{
        true, "annealedGradientdescent_open_nodes", this};
    Property<bool> gradient_descent_current{false, "gradient_descent_current",
                                            this};
    Property<bool> gradient_descent_successors{
        false, "gradient_descent_successors", this};
    Property<double> gradient_descent_eta{0.5, "gradient_descent_eta", this};
    Property<double> gradient_descent_eta_discount{
        0.8, "gradient_descent_eta_discount", this};
    Property<unsigned int> gradient_descent_rounds{
        10u, "gradient_descent_rounds", this};
    Property<bool> average_angles{true, "average_angles", this};

    /**
     * This setting is relevant for Theta* and Smooth Theta*.
     */
    Property<int> number_edges{10, "number_edges", this};
  } smoothThetaStar{"smooth_theta_star", this};
};
}  // namespace PlannerSettings

struct global {
  static PlannerSettings::GlobalSettings settings;
};

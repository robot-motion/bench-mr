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

/**
 * Global settings.
 */
inline struct GlobalSettings : public Group {
  using Group::Group;
  Environment *environment{nullptr};

  /**
   * Whether to estimate the orientation of the start and goal states for
   * planners that need them (e.g. SBPL).
   */
  Property<bool> estimate_theta{true, "estimate_theta", this};

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
    Property<double> max_planning_time{15.0, "max_planning_time", this};
  } ompl{"ompl", this};

  struct SteerSettings : public Group {
    using Group::Group;

    void initializeSteering() const;

    Property<Steering::SteeringType> steering_type{
        Steering::STEER_TYPE_REEDS_SHEPP, "steering_type", this};
    Property<double> car_turning_radius{4, "car_turning_radius", this};
    //    Property<double> linear_steering_delta{3, "linear_steering_delta",
    //    this};

    /**
     * Distance between states sampled using the steer function for collision
     * detection, rendering and evaluation.
     */
    Property<double> sampling_resolution{0.1, "sampling_resolution", this};

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
    Property<double> initial_solution_eps{3, "initial_solution_eps", this};
    Property<double> fordward_velocity{0.4, "fordward_velocity",
                                       this};  // in meters/sec
    Property<double> time_to_turn_45_degs_in_place{
        0.6, "time_to_turn_45_degs_in_place", this};  // in sec
    Property<std::string> motion_primitive_filename{
        "./sbpl_mprim/unicycle_0.125.mprim", "motion_primitive_filename", this};

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
    Property<double> resolution{0.125, "resolution", this};

    /**
     * Scale environment to accommodate extents of SBPL's motion primitives.
     */
    Property<double> scaling{5, "scaling", this};

    /**
     * XXX Important: number of theta directions must match resolution in motion
     * primitive definition file.
     */
    Property<unsigned int> num_theta_dirs{16u, "num_theta_dirs", this};
  } sbpl{"sbpl", this};

  struct ChompSettings : public Group {
    using Group::Group;

    Property<unsigned int> nodes{127u, "nodes", this};
    Property<double> alpha{0.05, "alpha", this};
    /**
     * Obstacle importance.
     */
    Property<float> epsilon{2, "epsilon", this};
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
} settings{"settings"};

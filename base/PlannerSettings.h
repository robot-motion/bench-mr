#pragma once

#include <chomp/Chomp.h>
#include <params.hpp>

#include "Environment.h"
#include "steer_functions/Steering.h"

using namespace params;

namespace sbpl {
enum Planner {
  SBPL_ARASTAR,
  SBPL_ADSTAR,
  SBPL_RSTAR,
  SBPL_ANASTAR,
  SBPL_MHA,
  SBPL_LAZY_ARA
};
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
struct StateSettings : public Group {
  using Group::Group;
  Property<double> x{0, "x", this};
  Property<double> y{0, "y", this};
  Property<double> theta{0, "theta", this};
};

/**
 * Global settings.
 */
struct GlobalSettings : public Group {
  using Group::Group;
  Environment *environment{nullptr};

  struct EnvironmentSettings : public Group {
    using Group::Group;
    /**
     * What type of environment to load/generate ("grid"/"polygon").
     */
    Property<std::string> type{"grid", "type", this};

    StateSettings start{"start", this};
    StateSettings goal{"goal", this};

    void createEnvironment();

    struct GridSettings : public Group {
      using Group::Group;
      /**
       * Generator for grid environments ("corridor", "random", "moving_ai").
       */
      Property<std::string> generator{"corridor", "generator", this};
      Property<unsigned int> width{50, "width", this};
      Property<unsigned int> height{50, "height", this};
      Property<unsigned int> seed{1, "seed", this};

      struct CorridorSettings : public Group {
        using Group::Group;
        Property<double> radius{5, "radius", this};
        Property<int> branches{50, "branches", this};
      } corridor{"corridor", this};

      struct RandomSettings : public Group {
        using Group::Group;
        Property<double> obstacle_ratio{0.1, "obstacle_ratio", this};
      } random{"random", this};
    } grid{"grid", this};

    struct PolygonSettings : public Group {
      using Group::Group;

      /**
       * Generator for grid environments ("corridor", "random", "moving_ai").
       */
      Property<std::string> source{"polygon_mazes/parking1.svg", "source",
                                   this};

      /**
       * Scale polygons from InkScape.
       */
      Property<double> scaling{1. / 22., "scaling", this};

    } polygon{"polygon", this};

    /**
     * Settings pertaining to the collision checker.
     */
    struct CollisionSettings : public Group {
      using Group::Group;
      /**
       * Which model is used for collision checking.
       */
      Property<robot::Model> collision_model{robot::ROBOT_POLYGON,
                                             "collision_model", this};

      Property<Polygon> robot_shape{Polygon(), "robot_shape", this};
      /**
       * SVG file name of robot shape.
       */
      Property<std::string> robot_shape_source{"polygon_mazes/car.svg",
                                               "robot_shape_source", this};

      void initializeCollisionModel();
    } collision{"collision", this};
  } env{"env", this};

  /**
   * Whether to log the distances precomputed by the grid mazes.
   */
  Property<bool> log_env_distances{false, "log_env_distances", this};

  Property<bool> auto_choose_distance_computation_method{
      true, "auto_choose_distance_computation_method", this};
  Property<distance_computation::Method> distance_computation_method{
      distance_computation::BRUTE_FORCE, "distance_computation_method", this};

  Property<double> max_planning_time{15, "max_planning_time", this};

  /**
   * For maps with more cells than this threshold, a fast approximating
   * algorithm is used to compute the obstacle distance field (necessary for
   * clearance evaluations and GRIPS).
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
   * Radius threshold to evaluate whether the exact goal has been found.
   */
  Property<double> exact_goal_radius{1e-2, "exact_goal_radius", this};

  /**
   * Any og::PathGeometric with more nodes will not get interpolated.
   */
  Property<unsigned int> interpolation_limit{500u, "interpolation_limit", this};

  /**
   * Maximal length a og::PathGeometric can have to be interpolated.
   */
  Property<double> max_path_length{10000, "max_path_length", this};

  /**
   * Threshold in radians of the difference between consecutive yaw angles to be
   * considered a cusp.
   */
  Property<double> cusp_angle_threshold{60 * M_PI / 180.,
                                        "cusp_angle_threshold", this};

  /**
   * Settings related to benchmarking.
   */
  struct BenchmarkSettings : public Group {
    using Group::Group;

    Property<int> runs{10, "runs", this};
    Property<std::string> log_file{"", "log_file", this};

    /**
     * If a list of steer functions is given, they will each be tested on every
     * run.
     */
    Property<std::vector<Steering::SteeringType>> steer_functions{
        {//           Steering::STEER_TYPE_REEDS_SHEPP,
         //           Steering::STEER_TYPE_DUBINS,
         // TODO reactivate other steer functions
         //           Steering::STEER_TYPE_POSQ,
         //           Steering::STEER_TYPE_HC_REEDS_SHEPP,
         Steering::STEER_TYPE_CC_REEDS_SHEPP},
        "steer_functions",
        this};

    /**
     * Time intervals used for the anytime planner benchmark.
     */
    Property<std::vector<double>> anytime_intervals{
        {1., 3., 5., 10., 15., 20., 25., 30.}, "anytime_intervals", this};

    struct MovingAiSettings : public Group {
      using Group::Group;

      /**
       * Whether to run a Moving AI scenario.
       */
      Property<bool> active{false, "active", this};

      Property<std::string> scenario{"Berlin_0_256.map.scen", "scenario", this};
      /**
       * Start index of problem in this scenario. Python-style negative indices
       * are allowed.
       */
      Property<int> start{-10, "start", this};
      /**
       * End index of problem in this scenario. Python-style negative indices
       * are allowed.
       */
      Property<int> end{-1, "end", this};

      /**
       * Create a border of width 1 to prevent solutions leading outside the map
       * boundaries.
       */
      Property<bool> create_border{true, "create_border", this};
    } moving_ai{"moving_ai", this};

    /**
     * Select certain smoothing algorithms to be evaluated.
     */
    struct SmoothingSettings : public Group {
      using Group::Group;
      Property<bool> grips{false, "grips", this};
      Property<bool> chomp{false, "chomp", this};
      Property<bool> ompl_shortcut{false, "ompl_shortcut", this};
      Property<bool> ompl_bspline{false, "ompl_bspline", this};
      Property<bool> ompl_simplify_max{false, "ompl_simplify_max", this};
    } smoothing{"smoothing", this};

    /**
     * Select certain planning algorithms to be evaluated.
     */
    struct PlanningSettings : public Group {
      using Group::Group;
      Property<bool> theta_star{true, "theta_star", this};
      Property<bool> rrt{true, "rrt", this};
      Property<bool> rrt_star{true, "rrt_star", this};
      Property<bool> bit_star{true, "bit_star", this};
      Property<bool> cforest{true, "cforest", this};
      Property<bool> rrt_sharp{true, "rrt_sharp", this};
      Property<bool> sorrt_star{true, "sorrt_star", this};
      Property<bool> informed_rrt_star{true, "informed_rrt_star", this};
      Property<bool> sbpl_arastar{true, "sbpl_arastar", this};
      Property<bool> sbpl_adstar{true, "sbpl_adstar", this};
      Property<bool> sbpl_anastar{false, "sbpl_anastar", this};
      Property<bool> sbpl_lazy_ara{false, "sbpl_lazy_ara", this};
      Property<bool> sbpl_mha{true, "sbpl_mha", this};
      Property<bool> prm{true, "prm", this};
      Property<bool> prm_star{true, "prm_star", this};
      Property<bool> est{true, "est", this};
      Property<bool> sbl{false, "sbl", this};
      Property<bool> fmt{false, "fmt", this};
      Property<bool> bfmt{true, "bfmt", this};
      Property<bool> sst{true, "sst", this};
      Property<bool> kpiece{true, "kpiece", this};
      Property<bool> stride{false, "stride", this};
      Property<bool> spars{true, "spars", this};
      Property<bool> spars2{true, "spars2", this};
      Property<bool> pdst{true, "pdst", this};
    } planning{"planning", this};
  } benchmark{"benchmark", this};

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

    Property<unsigned int> seed{1, "seed", this};

    struct RRTstarSettings : public Group {
      using Group::Group;

      /**
       * Probability of selecting the goal state during the exploration.
       */
      Property<double> goal_bias{0.05, "goal_bias", this};

      /**
       * Maximum length of a motion to be added to the tree.
       */
      Property<double> max_distance{0., "max_distance", this};
    } rrt_star{"rrt_star", this};
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
    Property<double> sampling_resolution{0.005, "sampling_resolution", this};

    struct HC_CC_Settings : public Group {
      using Group::Group;

      Property<double> kappa{0.2, "kappa", this};
      Property<double> sigma{0.2, "sigma", this};
    } hc_cc{"hc_cc", this};

    /**
     * Settings related to the POSQ steer function.
     */
    struct PosqSettings : public Group {
      using Group::Group;

      Property<double> alpha{3, "alpha", this};
      Property<double> phi{-1, "phi", this};
      Property<double> rho{1, "rho", this};
      Property<double> rho_end_condition{0.005, "rho_end_condition", this};
      Property<double> v{1, "v", this};
      Property<double> v_max{1, "v_max", this};

      /**
       * Length of the wheel axis.
       */
      Property<double> axis_length{0.54, "axis_length", this};

      /**
       * Integration time step.
       */
      Property<double> dt{0.1, "dt", this};
    } posq{"posq", this};
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
    Property<double> fordward_velocity{0.2, "fordward_velocity",
                                       this};  // in meters/sec
    Property<double> time_to_turn_45_degs_in_place{
        0.6, "time_to_turn_45_degs_in_place", this};  // in sec

    Property<std::string> motion_primitive_filename{
        "./sbpl_mprim/unicycle_0.25.mprim", "motion_primitive_filename", this};
    /**
     * XXX Important: number of theta directions must match resolution in motion
     * primitive definition file.
     */
    Property<unsigned int> num_theta_dirs{16u, "num_theta_dirs", this};
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
    Property<double> scaling{8, "scaling", this};

    /**
     * These tolerances are most likely ignored by SPBL at the moment.
     */
    Property<double> goal_tolerance_x{1, "goal_tolerance_x", this};
    Property<double> goal_tolerance_y{1, "goal_tolerance_y", this};
    Property<double> goal_tolerance_theta{2 * M_PI, "goal_tolerance_theta",
                                          this};
  } sbpl{"sbpl", this};

  struct SmoothingSettings : public Group {
    using Group::Group;

    struct GripsSettings : public Group {
      using Group::Group;

      /*
       * Minimum distance to be maintained between two consecutive nodes.
       */
      Property<double> min_node_distance{3, "min_node_distance", this};
      /*
       * Gradient descent rate.
       */
      Property<double> eta{0.9, "eta", this};
      /*
       * Discount factor for gradient descent rate.
       */
      Property<double> eta_discount{0.8, "eta_discount", this};

      /**
       * Number of gradient descent rounds.
       */
      Property<unsigned int> gradient_descent_rounds{
          5, "gradient_descent_rounds", this};
      /**
       * Maximum number of pruning rounds after which the algorithm
       * should terminate.
       */
      Property<unsigned int> max_pruning_rounds{100, "max_pruning_rounds",
                                                this};
    } grips{"grips", this};

    struct OmplSettings : public Group {
      using Group::Group;

      Property<unsigned int> bspline_max_steps{5, "bspline_max_steps", this};
      Property<double> bspline_epsilon{0.005, "bspline_epsilon", this};

      Property<unsigned int> shortcut_max_steps{0, "shortcut_max_steps", this};
      Property<unsigned int> shortcut_max_empty_steps{
          0, "shortcut_max_empty_steps", this};
      Property<double> shortcut_range_ratio{0.33, "shortcut_range_ratio", this};
      Property<double> shortcut_snap_to_vertex{0.005, "shortcut_snap_to_vertex",
                                               this};
    } ompl{"ompl", this};

    struct ChompSettings : public Group {
      using Group::Group;

      Property<unsigned int> nodes{100u, "nodes", this};
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
      Property<chomp::ChompObjectiveType> objective_type{
          chomp::MINIMIZE_VELOCITY, "objective_type", this};
    } chomp{"chomp", this};

  } smoothing{"smoothing", this};

  struct ThetaStarSettings : public Group {
    using Group::Group;

    /**
     * This setting is relevant for Theta* and Smooth Theta*.
     */
    Property<int> number_edges{10, "number_edges", this};
  } thetaStar{"theta_star", this};

  GlobalSettings() : Group("settings") { ompl::RNG::setSeed(ompl.seed); }
};
}  // namespace PlannerSettings

struct global {
  static PlannerSettings::GlobalSettings settings;
};

stat_names = {
    'max_curvature': 'Maximum Curvature',
    'normalized_curvature': 'Normalized Curvature',
    'aol': 'AOL',
    'max_clearing_distance': 'Maximum Clearing',
    'mean_clearing_distance': 'Mean Clearing',
    'median_clearing_distance': 'Median Clearing',
    'min_clearing_distance': 'Minimum Clearing',
    'path_length': 'Path Length',
    'smoothness': 'Smoothness',
    'planning_time': 'Computation Time',
    'cusps': 'Cusps',
    'aggregate': 'Aggregate',
}

metric_properties = {
    'path_found': {
        'sum': True
    },
    'planning_time': {
        'show_std': True,
        'highlight_optimum': True
    },
    'path_length': {
        'show_std': True
    },
    'mean_clearing_distance': {
        'show_std': True
    },
    'min_clearing_distance': {
        'show_std': True
    },
    'max_clearing_distance': {
        'show_std': True
    },
    'median_clearing_distance': {
        'show_std': True
    },
    'max_curvature': {
        'show_std': True,
        'minimize': True
    },
    'normalized_curvature': {
        'show_std': True,
        'minimize': True
    },
    'aol': {
        'show_std': True,
        'minimize': True
    },
    'cusps': {
        'minimize': True,
        'sum': True
    }
}

steer_function_names = {
    'reeds_shepp': 'Reeds-Shepp',
    'dubins': 'Dubins',
    'posq': 'POSQ',
    'clothoid': 'G1 Clothoid',
    'linear': 'Linear',
    'cc_dubins': 'CC Dubins',
    'hc_reeds_shepp': 'HC Reeds-Shepp',
    'cc_reeds_shepp': 'CC Reeds-Shepp'
}

steer_functions = [
    'reeds_shepp',
    'dubins',
    'posq',
    'clothoid',
    'linear',
    'cc_dubins',
    'hc_reeds_shepp',
    'cc_reeds_shepp'
]

robot_models = [
    'kinematic_car',
    'kinematic_single_track'
]

robot_models_names = {
    'kinematic_car': 'Kinematic Car',
    'kinematic_single_track': 'Kinematic Single Track'
}

smoother_names = {
    'grips': 'GRIPS',
    'ompl_bspline': 'B-Spline',
    'ompl_shortcut': 'Shortcut',
    'ompl_simplify_max': 'SimplifyMax'
}

smoothers = list(smoother_names.values())

sampling_planners = ['rrt', 'est', 'sbl', 'prm',
                     'theta_star', 'sst', 'kpiece', 'pdst', 'stride']
anytime_planners = ['rrt_star', 'rrt_sharp', 'informed_rrt_star', 'sorrt_star', 'prm_star', 'fmt', 'bfmt', 'cforest',
                    'bit_star', 'spars', 'spars2']
controlbased_planners = ['fpkpiece', 'fpest', 'fpsst', 'fprrt', 'fppdst']
sbpl_planners = ['sbpl_adstar', 'sbpl_anastar',
                 'sbpl_arastar', 'sbpl_lazy_ara', 'sbpl_mha']
all_planners = sampling_planners + anytime_planners + sbpl_planners

# Mapping internal planner names to their printable counterparts
planner_names = {
    'rrt': 'RRT',
    'est': 'EST',
    'sbl': 'SBL',
    'prm': 'PRM',
    'theta_star': 'Theta*',
    'sst': 'SST',
    'fmt': 'FMT',
    'kpiece': 'KPIECE',
    'pdst': 'PDST',
    'stride': 'STRIDE',
    'rrt_star': 'RRT*',
    'rrt_sharp': 'RRT#',
    'informed_rrt_star': 'Informed RRT*',
    'sorrt_star': 'SORRT*',
    'prm_star': 'PRM*',
    'bfmt': 'BFMT',
    'cforest': 'CForest',
    'bit_star': 'BIT*',
    'spars': 'SPARS',
    'spars2': 'SPARS2',
    'sbpl_adstar': 'SBPL AD*',
    'sbpl_anastar': 'SBPL ANA*',
    'sbpl_arastar': 'SBPL ARA*',
    'sbpl_lazy_ara': 'SBPL Lazy ARA*',
    'sbpl_mha': 'SBPL MHA*',
    'fpkpiece': 'FP KPIECE',
    'fpest': 'FP EST',
    'fpsst': 'FP SST',
    'fprrt': 'FP RRT',
    'fppdst': 'FP PDST'
}

# Names internally used by MPB/OMPL to appear in the "plans" dictionary of the results files
planner_internal_names = {
    'rrt': 'RRT',
    'est': 'EST',
    'sbl': 'SBL',
    'prm': 'PRM',
    'theta_star': 'Theta*',
    'sst': 'SST',
    'fmt': 'FMT',
    'kpiece': 'KPIECE1',
    'pdst': 'PDST',
    'stride': 'STRIDE',
    'rrt_star': 'RRTstar',
    'rrt_sharp': 'RRT#',
    'informed_rrt_star': 'InformedRRTstar',
    'sorrt_star': 'SORRTstar',
    'prm_star': 'PRMstar',
    'bfmt': 'BFMT',
    'cforest': 'CForest',
    'bit_star': 'kBITstar',
    'spars': 'SPARS',
    'spars2': 'SPARStwo',
    'sbpl_adstar': 'SBPL_ADstar',
    'sbpl_anastar': 'SBPL_ANAstar',
    'sbpl_arastar': 'SBPL_ARAstar',
    'sbpl_lazy_ara': 'SBPL_Lazy_ARA',
    'sbpl_mha': 'SBPL_MHA',
    'fpkpiece': 'FP KPIECE',
    'fpest': 'FP EST',
    'fpsst': 'FP SST',
    'fprrt': 'FP RRT',
    'fppdst': 'FP PDST'
}

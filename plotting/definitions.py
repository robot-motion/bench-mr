stat_names = {
    'curvature': 'Curvature',
    'max_clearing_distance': 'Maximum Clearing Distance',
    'mean_clearing_distance': 'Mean Clearing Distance',
    'median_clearing_distance': 'Median Clearing Distance',
    'min_clearing_distance': 'Minimum Clearing Distance',
    'path_length': 'Path Length',
    'smoothness': 'Smoothness',
    'planning_time': 'Computation Time',
    'cost': 'Path Length',
    'cusps': 'Cusps',
    'aggregate': 'Aggregate'
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

steer_functions = list(steer_function_names.keys())

smoother_names = {
    'chomp': 'CHOMP',
    'grips': 'GRIPS',
    'ompl_bspline': 'B-Spline',
    'ompl_shortcut': 'Shortcut',
    'ompl_simplify_max': 'SimplifyMax'
}

smoothers = list(smoother_names.values())

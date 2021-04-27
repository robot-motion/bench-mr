---
layout: default
title:  "Add a Custom Smoother"
date:   2021-01-04 13:20:59 +0100
parent: "Post Smoothers"
grand_parent: Components
nav_order: 1
---

# Add a Custom Smoother

The following steps are necessary to add a new post-smoothing or path improvement algorithm to Bench-MR.

Your implementation needs to provide an OMPL-compatible function that modifies a `ompl::geometric::PathGeometric&` reference inplace.

## Register Smoother in `PlanningSettings`

Register your smoother as a new entry in `global::settings.benchmark.smoothing` in `base/PlannerSettings.h` to allow the benchmark configuration to enable the use of this smoother. Each entry follows the same format:

```cpp
Property<bool> smoother{true, "smoother", this};
```

where `smoother` is the name of your smoother.

## Add Smoother to Benchmark Function

To execute the smoother when its corresponding flag in `PlanningSettings` is active, we need to add the following condition to the `evaluateSmoothers` function in `utils/PathEvaluation.hpp`:

```cpp
if (global::settings.benchmark.smoothing.<smoother>) {
  ompl::geometric::PathGeometric path(planner->solution());
  // TODO: evaluate your smoother here by modifying `path` inplace
  PathStatistics path_stats;
  evaluate(path_stats, path, planner);
  j["<smoother>"] = {
      {"time", /* TODO: insert elapsed time for smoothing */},
      {"collision_time",
       global::settings.environment->elapsedCollisionTime()},
      {"steering_time", global::settings.ompl.steering_timer.elapsed()},
      {"name", "<smoother name>"},
      {"cost", path.length()},
      {"trajectory", Log::serializeTrajectory(path)},
      {"path", Log::serializeTrajectory(path, false)},
      {"stats", nlohmann::json(path_stats)["stats"]},
      // add any additional smoother stats / properties you would like to log
  };
}
```

where `<smoother>` is the name of your smoother.

## Update Front-end Definitions

Go to `python/definitions.py` to optionally register a user-friendly title in the `smoother_names` dictionary that is displayed in the plots inplace of the smoother name.

```py
smoother_names = {
	# ...
	'<smoother>': 'Desired title used in plots'
}
```

After compiling the `benchmark` executable, the new smoother is now available from `MPB`, e.g. via the `MPB.set_smoothers(["<smoother>"])` helper function.
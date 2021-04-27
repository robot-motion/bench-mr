---
layout: default
title:  "Add a Custom Planner"
date:   2021-01-04 13:20:59 +0100
parent: Planners
grand_parent: Components
nav_order: 1
---

# Add a Custom Planner

The following steps are necessary to add a new planner to Bench-MR.

## Inherit from `AbstractPlanner`

Your planner needs to be implemented in C++ and provide an interface that inherits from `AbstractPlanner`. The corresponding abstract functions need to implemented:

```cpp
std::string name() const {}
ompl::base::PlannerStatus run()
ompl::geometric::PathGeometric solution() const
bool hasReachedGoalExactly() const
double planningTime() const
```

As shown, the path solution needs to be returned as a `ompl::geometric::PathGeometric` from OMPL.

## Register Planner in `PlanningSettings`

Register your planner as a new entry in the `PlanningSettings` in `base/PlannerSettings.h` to allow the benchmark configuration to enable the use of this planner. Each entry follows the same format:

```cpp
Property<bool> planner{true, "planner", this};
```

where `planner` is the name of your planner.

## Add Planner to Benchmark Function

To execute the planner when its corresponding flag in `PlanningSettings` is active, we need to add the following term to the `evaluatePlanners` function in `experiments/benchmark.cpp`:

```cpp
if (global::settings.benchmark.planning.planner)
  PathEvaluation::evaluateSmoothers<Planner>(info);
```

where `Planner` is the type of your planner (the one inherited from `AbstractPlanner`).

## Update Front-end Definitions

Go to `python/definitions.py` to optionally register a user-friendly title in the `planner_names` dictionary that is displayed in the plots inplace of the planner name.

```py
planner_names = {
	# ...
	'planner': 'Desired title used in plots'
}
```

After compiling the `benchmark` executable, the new planner is now available from `MPB`, e.g. via the `MPB.set_planners(["planner"])` helper function.
---
layout: default
title:  "Metrics"
date:   2021-01-04 13:20:59 +0100
parent: "Components"
has_children: true
nav_order: 3
---

# Metrics
{: .no_toc }

Bench-MR includes various metrics on which the planning algorithms can be evaluated.
{: .fs-6 .fw-300 }

The following statistics are gathered for each benchmark in Bench-MR:

| Metric Name    | Metric Title        |
|:----------------|:-------------------|
| `max_curvature`	|	Maximum Curvature |
| `normalized_curvature`	|	Normalized Curvature |
| `max_clearing_distance`	|	Maximum Clearing |
| `mean_clearing_distance`	|	Mean Clearing |
| `median_clearing_distance`	|	Median Clearing |
| `min_clearing_distance`	|	Minimum Clearing |
| `path_length`	|	Path Length |
| `smoothness`	|	Smoothness |
| `planning_time`	|	Computation Time |
| `cusps`	|	Cusps |
| `aggregate`	|	Aggregate |

This mapping is defined in the `stat_names` dictionary in `plotting/definitions.py`.

## Visualize Statistics

Use the `MPB.plot_planner_stats` function with the optional `metrics` keyword to provide a string of comma-separated metric names to plot:


#### Example

```python
mpb = MPB()
mpb.set_corridor_grid_env(radius = 3)
mpb.set_planners(['rrt', 'rrt_star', 'informed_rrt_star'])
mpb.set_steer_functions(['reeds_shepp'])
mpb.run(id='test_run', runs=3);
mpb.plot_planner_stats(metrics=", ".join(stat_names.keys()))
```

<img src="{{ site.baseurl }}/assets/frontend/metrics/output_7_1.png" style="width:100% !important;max-height:none"/>
    

## Computation Time Drill-Down

Examine the computation times within the different phases of planning:


```python
mpb.plot_planner_timings()
```
    
![png]({{ site.baseurl }}/assets/frontend/metrics/output_9_0.png) | ![png]({{ site.baseurl }}/assets/frontend/metrics/output_9_1.png) | ![png]({{ site.baseurl }}/assets/frontend/metrics/output_9_2.png)
    
[View Jupyter Notebook](https://github.com/eric-heiden/mpb/blob/master/plotting/Metrics.ipynb){: .btn .btn-green }

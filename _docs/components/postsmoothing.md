---
layout: default
title:  "Post Smoothers"
date:   2021-01-04 13:20:59 +0100
parent: "Components"
has_children: true
nav_order: 5
---

# Post Smoothers
{: .no_toc }

Given a solution from a motion planner, post smoothing, or path improvement algorithms attempt to find a better (e.g., shorter, smoother) path.
{: .fs-6 .fw-300 }


A list of smoothers can be provided to the `MPB` instance which will run a benchmark replicating all the settings across each selected smoother. The user-friendly function is the following that will enable the respective smoothers in under the `benchmark.planning` group:

```py
MPB.set_smoothers(smoothers: [str])
```

The *smoother names* are parsed from the list of strings, showing an error if any entry could not be unified with the available smoothers in Bench-MR. Check the `smoother_names` dictionary in `definitions.py` for the mapping of smoother names to be given to the `set_smoothers` function, and their respective printable titles:

| Smoother Name    | Smoother Title        |
|:----------------|:-------------------|
| `grips`	|	GRIPS |
| `ompl_bspline`	|	B-Spline |
| `ompl_shortcut`	|	Shortcut |
| `ompl_simplify_max`	|	SimplifyMax |

#### Example

```python
mpb = MPB()
mpb.set_corridor_grid_env(radius = 3)
mpb.set_planners(['rrt'])
mpb.set_steer_functions(['reeds_shepp'])
mpb.set_smoothers(['grips',
                   'ompl_bspline',
                   'ompl_shortcut',
                   'ompl_simplify_max'])
mpb.run(runs=5);
```

Visualize trajectories (original solution and smoothed paths):

```python
mpb.visualize_trajectories(show_smoother=True);
```

![png]({{ site.baseurl }}/assets/frontend/smoothers/output_3_1.png)
    
Plot statistics for the 5 runs comparing the original plan against the post-smoothing solutions:

```python
mpb.plot_smoother_stats();
```
    
![png]({{ site.baseurl }}/assets/frontend/smoothers/output_4_1.png)

[View Jupyter Notebook](https://github.com/eric-heiden/mpb/blob/master/plotting/Smoothing.ipynb){: .btn .btn-green }

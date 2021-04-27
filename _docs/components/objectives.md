---
layout: default
title:  "Optimization Objectives"
date:   2021-01-04 13:20:59 +0100
parent: "Components"
nav_order: 7
---

# Optimization Objectives
{: .no_toc }

The OMPL-based planners can be configured to optimize for certain path quality criteria.

<script type="text/javascript" id="MathJax-script" async
  src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-chtml.js">
</script>

The optimization objective is defined by the `ompl.optimization_objective` setting:

| Value           | Description        |
|:----------------|:-------------------|
| `min_pathlength`       | Minimize path length |
| `min_smoothness`       | Minimize smoothness as [defined by OMPL](https://ompl.kavrakilab.org/classompl_1_1geometric_1_1PathGeometric.html#a10b135957051a6923a8a1f4e23b10453): <br/><br/> $$ \mbox{smoothness} = \sum\limits_{i=2}^{n-1}\left(\frac{2\left(\pi - \arccos\left(\frac{a_i^2+b_i^2-c_i^2}{2 a_i b_i}\right)\right)}{a_i + b_i}\right)^2 $$ |
| `min_curvature`        | Minimize normalized curvature along the path (normalized by the path length) <br/> $$\kappa_\mathrm{norm}=\sum_i\int_{\sigma_i}\kappa(\dot{\sigma}_i(t))\parallel\dot{p}_{\sigma_i}(t)\parallel_2\,dt$$,<br/> where $$\sigma_i$$ are the continuous curvature segments (between the cusps) of the full trajectory $$\sigma$$ and $$\kappa(\dot{\sigma}(t))$$ computes the curvature at a point $$\sigma(t)$$ of the trajectory and $$p_\sigma$$ denotes the $$x$$ and $$y$$ components of $$\sigma$$ |
| `max_minclearance`     | Maximize minimum clearance (distance to nearest obstacle along the entire path) |


#### Example

```python
def create_mpb(optimization_objective):
    mpb = MPB()
    mpb["max_planning_time"] = 2
    mpb.set_planners(['prm'])
    mpb["ompl.sampler"] = "halton"
    mpb.set_steer_functions(['reeds_shepp'])
    mpb['ompl.cost_threshold'] = 0
    mpb.set_corridor_grid_env(radius=5, branches = 15)
    mpb["ompl.seed"] = 0
    mpb.set_id(optimization_objective)
    mpb["ompl.optimization_objective"] = optimization_objective
    return mpb

pool = MultipleMPB()
pool.benchmarks.append(create_mpb("min_pathlength"))
pool.benchmarks.append(create_mpb("min_smoothness"))
pool.benchmarks.append(create_mpb("min_curvature"))
pool.benchmarks.append(create_mpb("max_minclearance"))
pool.run_parallel(runs=25, id='optimization_objectives', show_plot=False)
pool.merge('optimization_objectives/optimization_objectives.json', 
           plan_names=['PRM (min_pathlength)',
                       'PRM (min_smoothness)',
                       'PRM (min_curvature)',
                       'PRM (max_minclearance)'])
```



```python
from trajectory import visualize
visualize('optimization_objectives/optimization_objectives.json', num_colors=10)
```
    
<img src="{{ site.baseurl }}/assets/frontend/objectives/output_3_1.png" style="width:100% !important;max-height:none"/>


```python
from plot_stats import plot_planner_stats
plot_planner_stats('optimization_objectives/optimization_objectives.json', num_colors=10)
```

<img src="{{ site.baseurl }}/assets/frontend/objectives/output_4_1.png" style="width:100% !important;max-height:none"/>

[View Jupyter Notebook](https://github.com/eric-heiden/mpb/blob/master/plotting/Optimization%20Objective.ipynb){: .btn .btn-green }

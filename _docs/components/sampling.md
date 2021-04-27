---
layout: default
title:  "Sampling"
date:   2021-01-04 13:20:59 +0100
parent: "Components"
nav_order: 8
---

# Sampling
{: .no_toc }

The planners can be configured to use **random** (default) or **deterministic (Halton) sampling**.

The sampling is defined by the `ompl.sampler` setting:

| Value           | Description        |
|:----------------|:-------------------|
| `iid`          | Random (i.i.d.) sampling |
| `halton`       | Halton sampling (deterministic) |

#### Example

```python
mpb = MPB()
mpb.set_planners(['prm'])
mpb.set_steer_functions(['reeds_shepp'])
mpb.set_corridor_grid_env(radius=3)
mpb["ompl.seed"] = 0

# Random sampling
mpb_iid = deepcopy(mpb)
mpb_iid.set_id('iid')
mpb_iid["ompl.sampler"] = "iid"

# Deterministic sampling
mpb_halton = deepcopy(mpb)
mpb_halton.set_id('halton')
mpb_halton["ompl.sampler"] = "halton"

pool = MultipleMPB()
pool.benchmarks.append(mpb_iid)
pool.benchmarks.append(mpb_halton)
pool.run_parallel(runs=10, id='samplers', show_plot=False)
pool.merge('samplers/samplers.json', plan_names=['PRM (iid)', 'PRM (Halton)'])
```


```python
from trajectory import visualize
visualize('samplers/samplers.json')
```
    
![png]({{ site.baseurl }}/assets/frontend/sampling/output_3_1.png)
    

```python
from plot_stats import plot_planner_stats
plot_planner_stats('samplers/samplers.json')
```
    
![png]({{ site.baseurl }}/assets/frontend/sampling/output_4_1.png)

[View Jupyter Notebook](https://github.com/eric-heiden/mpb/blob/master/plotting/Random%20vs%20Deterministic.ipynb){: .btn .btn-green }

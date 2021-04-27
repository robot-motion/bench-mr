---
layout: default
title:  "Planners"
date:   2021-01-04 13:20:59 +0100
parent: "Components"
has_children: true
nav_order: 2
---

# Planners
{: .no_toc }

Bench-MR supports several motion-planning algorithms, supporting implementations from OMPL and SBPL out of the box.
{: .fs-6 .fw-300 }

A list of planners can be provided to the `MPB` instance which will run a benchmark replicating all the settings across each selected planner. The user-friendly function is the following that will enable the respective planners in under the `benchmark.planning` group:

```py
MPB.set_planners(planners: [str])
```

The *planner names* are parsed from the list of strings, showing an error if any entry could not be unified with the available planners in Bench-MR. Check the `planner_names` dictionary in `definitions.py` for the mapping of planner names to be given to the `set_planners` function, and their respective printable titles:

| Planner Name    | Planner Title        |
|:----------------|:-------------------|
| `rrt`	|	RRT |
| `est`	|	EST |
| `sbl`	|	SBL |
| `prm`	|	PRM |
| `theta_star`	|	Theta* |
| `sst`	|	SST |
| `fmt`	|	FMT |
| `kpiece`	|	KPIECE |
| `pdst`	|	PDST |
| `stride`	|	STRIDE |
| `rrt_star`	|	RRT* |
| `rrt_sharp`	|	RRT# |
| `informed_rrt_star`	|	Informed RRT* |
| `sorrt_star`	|	SORRT* |
| `prm_star`	|	PRM* |
| `bfmt`	|	BFMT |
| `cforest`	|	CForest |
| `bit_star`	|	BIT* |
| `spars`	|	SPARS |
| `spars2`	|	SPARS2 |
| `sbpl_adstar`	|	SBPL AD* |
| `sbpl_anastar`	|	SBPL ANA* |
| `sbpl_arastar`	|	SBPL ARA* |
| `sbpl_lazy_ara`	|	SBPL Lazy ARA* |
| `sbpl_mha`	|	SBPL MHA* |
| `fpkpiece`	|	FP KPIECE |
| `fpest`	|	FP EST |
| `fpsst`	|	FP SST |
| `fprrt`	|	FP RRT |
| `fppdst`	|	FP PDST |

[View Tutorial]({{ site.baseurl }}/docs/getting-started/tutorial/){: .btn .btn-green }

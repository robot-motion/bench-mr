---
layout: default
title:  "Extend Functions"
date:   2021-01-04 13:20:59 +0100
parent: "Components"
has_children: true
nav_order: 4
---

# Extend Functions
{: .no_toc }

Bench-MR has support for various *steer functions* used by sampling-based planners, and kinodynamic robot models for *forward-propagating* planners.

## Steer Functions

The steer functions can be provided by a list of function names (see below).

```py
MPB.set_steer_functions(steerings: [str])
```

This function will set the configuration `benchmark.steer_functions` to the provided list of steer functions, and set the parameter `benchmark.control_planners_on` to `False`.

The following steer functions are provided with Bench-MR at the moment:

| Steer function name          | Description        |
|:-----------------------------|:-------------------|
| `reeds_shepp`          | Reeds-Shepp (from OMPL) |
| `dubins`       | Dubins (from OMPL) |
| `posq`       | POSQ (from [ palmieri/posq](https://github.com/palmieri/posq)) |
| `linear`       | Linear (straight-line) steering |
| `cc_dubins`       | Continuous-curvature Dubins (from [hbanzhaf/steering_functions](https://github.com/hbanzhaf/steering_functions)) |
| `hc_reeds_shepp`       | Hybrid-curvature Reeds-Shepp (from [hbanzhaf/steering_functions](https://github.com/hbanzhaf/steering_functions)) |
| `cc_reeds_shepp`       | Continuous-curvature Reeds-Shepp (from [hbanzhaf/steering_functions](https://github.com/hbanzhaf/steering_functions)) |

## Robot Models

The robot models can be provided by a list of names (see below).

```py
MPB.set_robot_models_functions(robot_models: [str])
```

This function will set the configuration `benchmark.forward_propagations` to the provided list of robot models, and set the parameter `benchmark.control_planners_on` to `True`.

The following robot models are provided with Bench-MR at the moment:

| Robot model name          | Description        |
|:-----------------------------|:-------------------|
| `kinematic_car`          | Kinematic Car |
| `kinematic_single_track`       | Kinematic Single Track |

Depending on the `benchmark.control_planners_on` setting, each `MPB` instance runs all the specified steer functions or robot model functions in sequence.

## Example

```python
def create_mpb(planner : str, extend_function : str, planning_time):
    mpb = MPB()
    mpb["max_planning_time"] = planning_time
    mpb.set_planners([planner])
    if(extend_function in steer_functions):
        mpb.set_steer_functions([extend_function])
    if(extend_function in robot_models):
        mpb.set_robot_models_functions([extend_function])
    mpb.set_corridor_grid_env(radius=5, branches = 15)
    mpb["ompl.seed"] = 0
    mpb.set_id('comparison_extend_function_%s_%s' % (planner, extend_function))
    return mpb
```

### Comparing RRT with different extend function
In the following we compare the classical RRT algorithm, with 2 different extend function, namely the Reeds-Shepp steer function and the kinematic car model (used in forward propagation).


```python
pool = MultipleMPB()
pool.benchmarks.append(create_mpb('rrt', 'reeds_shepp', 15))
pool.benchmarks.append(create_mpb('fprrt', 'kinematic_car', 15))
pool.run_parallel(runs=5, id='comparison_extend_functions', show_plot=True)
pool.merge('comparison_extend_functions.json')
```
   
![png]({{ site.baseurl }}/assets/frontend/extendfun/output_4_4.png)
    

```python
visualize('comparison_extend_functions.json', num_colors=10)
```
  
![png]({{ site.baseurl }}/assets/frontend/extendfun/output_5_1.png)


```python
plot_planner_stats('comparison_extend_functions.json', num_colors=10)
```
 
![png]({{ site.baseurl }}/assets/frontend/extendfun/output_6_1.png)

We add now in the comparison also SST, an asymptotically near-optimal incremental version of RRT.

```python
pool = MultipleMPB()
pool.benchmarks.append(create_mpb('rrt', 'reeds_shepp', 30))
pool.benchmarks.append(create_mpb('fprrt', 'kinematic_car', 30))
pool.benchmarks.append(create_mpb('fpsst', 'kinematic_car', 30))
pool.run_parallel(runs=5, id='comparison_extend_functions_with_sst', show_plot=True)
pool.merge('comparison_extend_functions_with_sst.json')
```
    
![png]({{ site.baseurl }}/assets/frontend/extendfun/output_8_5.png)
    

```python
visualize('comparison_extend_functions_with_sst.json', num_colors=10)
```
    
![png]({{ site.baseurl }}/assets/frontend/extendfun/output_9_1.png)

```python
plot_planner_stats('comparison_extend_functions_with_sst.json', num_colors=10)
```
    
![png]({{ site.baseurl }}/assets/frontend/extendfun/output_10_1.png)
    
[View Jupyter Notebook](https://github.com/eric-heiden/mpb/blob/master/plotting/Steering%20and%20Forward%20Propagation.ipynb){: .btn .btn-green }

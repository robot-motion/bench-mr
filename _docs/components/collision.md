---
layout: default
title:  "Collision Checking"
date:   2021-01-04 13:20:59 +0100
parent: "Components"
nav_order: 3
---

# Collision Checking
{: .no_toc }

Bench-MR supports collision detection for a robot represented by a point or a convex polygon.
{: .fs-6 .fw-300 }

The `env.collision.collision_model` defines the collision model:

| Value           | Description        |
|:----------------|:-------------------|
| `0`       | Point-based collision model |
| `1`       | Polygon-based collision model |

For polygon-based collision detection, the SVG file representing the robot shape can be optionally set via the `env.collision.robot_shape_source` setting (see below).


### Example

Set up `warehouse2` scenario with RRT as the planner.


```python
m = MPB()
m.set_planners(['rrt'])
m["max_planning_time"] = 60
m["env.start"] = {"theta": -1.58, "x": 7.5, "y": -10}
m["env.goal"] = {"theta": -1.58, "x": 116, "y": -70}
m["env.type"] = "polygon"
m["env.polygon.source"] = "polygon_mazes/warehouse2.svg"
```

#### Point-based collision model

```python
m["env.collision.collision_model"] = 0
m.run(runs=1)
m.visualize_trajectories(draw_start_goal_thetas=True, plot_every_nth_polygon=10, silence=True)
```
    
![png]({{ site.baseurl }}/assets/frontend/collision/output_5_2.png)
    


#### Polygon-based collision model

We use the `env.collision.robot_shape_source` setting to define a collision shape for the robot to load from a SVG file.

```python
m["env.collision.collision_model"] = 1
m["env.collision.robot_shape_source"] = "polygon_mazes/warehouse_robot.svg"
m.run(runs=1)
m.visualize_trajectories(draw_start_goal_thetas=True, plot_every_nth_polygon=10, silence=True)
```

![png]({{ site.baseurl }}/assets/frontend/collision/output_7_2.png)

[View Jupyter Notebook](https://github.com/eric-heiden/mpb/blob/master/plotting/Collision%20Detection.ipynb){: .btn .btn-green }

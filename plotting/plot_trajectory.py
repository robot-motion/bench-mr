#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon


def plot_trajectory(traj, planner: str, settings, color, add_label=True, alpha: float = 1.):
    traj = np.array(traj)
    if settings["collision_model"] == 0:
        # point collision model
        if add_label:
            plt.plot(traj[:, 0], traj[:, 1], '-', color=color, label=planner)
        else:
            plt.plot(traj[:, 0], traj[:, 1], '-', color=color)
    else:
        # polygon collision model
        points = np.array(settings["robot_shape"])
        if add_label:
            plt.plot([], '-', color=color, label=planner)
        for i in range(traj.shape[0]):
            state = traj[i, :]
            c, s = np.cos(state[2]), np.sin(state[2])
            rotation = np.array([[c, -s], [s, c]])
            poly = Polygon(state[:2] + np.matmul(points, rotation), True, fill=False, linestyle='--', edgecolor=color,
                           alpha=alpha)
            plt.gca().add_patch(poly)

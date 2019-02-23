#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
from bitarray import bitarray


def plot_env(env, run_id: int = -1, colors=(None, None), draw_start_goal=True, draw_start_goal_thetas=True,
             set_title=True):
    """
    Plots the json branch for an environment.
    :param set_title:
    :param run_id:
    :param run:
    :param draw_start_goal_thetas:
    :param draw_start_goal:
    :param env:
    :param colors:
    :return:
    """
    title = ''
    if run_id >= 0:
        title += 'Run %i ' % run_id
    if colors is None:
        colors = [None, None]
    plt.gca().set_axisbelow(True)
    if env["type"] == "grid":
        w = env["width"] + 1
        h = env["height"] + 1
        ax = plt.gca()

        if w * h > 100 * 100:
            major_ticks = np.arange(0, max(w, h), 25)
            minor_ticks = np.arange(0, max(w, h), 5)
        else:
            major_ticks = np.arange(0, max(w, h), 10)
            minor_ticks = np.arange(0, max(w, h), 1)

        ax.set_xticks(major_ticks)
        ax.set_xticks(minor_ticks, minor=True)
        ax.set_yticks(major_ticks)
        ax.set_yticks(minor_ticks, minor=True)

        ax.grid(which='both')
        ax.grid(which='minor', alpha=0.2)
        ax.grid(which='major', alpha=0.5)

        map_data = np.array(list(bitarray(env["map"]))).reshape((w, h))
        map_data = 1. - np.flip(map_data, axis=0)
        plt.imshow(map_data, cmap='gray', vmin=-1, vmax=1, extent=[0, w, 0, h], alpha=0.5)
        ax.set_xlim([0, w])
        ax.set_ylim([0, h])

        title += '(%i$\\times$%i %s' % (w - 1, h - 1, env["generator"])
        if env["seed"] > 0:
            title += ' %i' % env["seed"]
    elif env["type"] == "polygon":
        polygons = []
        for points in env["obstacles"]:
            points = np.array(points)
            polygons.append(Polygon(points, True))
        plt.gca().add_collection(PatchCollection(polygons, color='lightgray', edgecolor='gray', alpha=0.8))
        plt.grid()

        title += env["name"]
    plt.axis('equal')

    if draw_start_goal:
        start = env["start"]
        goal = env["goal"]
        plt.scatter([start[0]], [start[1]], label="Start", color=colors[0])
        plt.scatter([goal[0]], [goal[1]], label="Goal", color=colors[1])
        if draw_start_goal_thetas:
            plt.arrow(start[0], start[1], math.sin(start[2]) * 20, math.cos(start[2]) * 20, width=0.01, head_width=5,
                      label="start", color=colors[0], edgecolor=colors[0])
            plt.arrow(goal[0], goal[1], math.sin(goal[2]) * 20, math.cos(goal[2]) * 20, width=0.01, head_width=5,
                      label="start",
                      color=colors[1], edgecolor=colors[0])
    if set_title:
        plt.title(title)

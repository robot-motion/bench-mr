#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import math
import click
import json
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
from bitarray import bitarray


def plot_env(env, run_id: int = -1, colors=(None, None), draw_start_goal=True, draw_start_goal_thetas=True,
             set_title=True, show_distances=False):
    """
    Plots the json branch for an environment.
    :param show_distances:
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
        w = env["width"]
        h = env["height"]
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

        if show_distances and "distances" in env:
            map_data = np.array(env["distances"]).reshape((w, h))
            print(map_data)
            print("Maximum distance:", map_data.max())
            plt.imshow(np.flip(map_data, axis=0), cmap='jet', vmin=0, vmax=map_data.max(), extent=[0, w, 0, h])
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


@click.command()
@click.option('--json_file', help='Name of the JSON file of a benchmarking run.')
@click.option('--run_id', default='0', help='ID numbers of the the runs ("all" or comma-separated list on integers).')
@click.option('--show_distances', default=False)
@click.option('--draw_start_goal', default=True)
@click.option('--draw_start_goal_thetas', default=True)
@click.option('--set_title', default=True)
def main(json_file: str, run_id: str, draw_start_goal=True, draw_start_goal_thetas=True,
         set_title=True, show_distances=True):
    data = json.load(open(json_file, 'r'))
    if run_id.lower() == "all":
        run_ids = list(range(len(data["runs"])))
    else:
        run_ids = [int(s.strip()) for s in run_id.split(',')]

    for i in run_ids:
        print("Plotting run %i" % i)
        plt.figure("Run %i" % i)
        plot_env(data["runs"][i]["environment"], i, draw_start_goal=draw_start_goal,
                 draw_start_goal_thetas=draw_start_goal_thetas,
                 set_title=set_title, show_distances=show_distances)
    plt.show()


if __name__ == '__main__':
    main()

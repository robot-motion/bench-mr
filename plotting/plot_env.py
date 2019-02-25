#!/usr/bin/env python3

import click
import numpy as np
import math
import click
import json
from bitarray import bitarray

from utils import add_options, group

plot_env_options = [
    click.option('--show_distances', default=False, type=bool),
    click.option('--draw_start_goal', default=True, type=bool),
    click.option('--draw_start_goal_thetas', default=False, type=bool),
    click.option('--set_title', default=True, type=bool)
]


@add_options(plot_env_options)
def plot_env(env, run_id: int = -1, colors=(None, None), draw_start_goal=True, draw_start_goal_thetas=True,
             set_title=True, show_distances=False, **_):
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
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    from matplotlib.collections import PatchCollection

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

        if show_distances:
            if "distances" not in env:
                click.echo('Environment contains no distance information.', err=True)
            else:
                map_data = np.array(env["distances"]).reshape((w, h))
                click.echo(map_data)
                click.echo("Maximum distance:", map_data.max())
                plt.imshow(np.flip(map_data, axis=0), cmap='jet', vmin=0, vmax=map_data.max(), extent=[0, w, 0, h])
        map_data = np.array(list(bitarray(env["map"]))).reshape((w, h))
        map_data = 1. - np.flip(map_data, axis=0)
        plt.imshow(map_data, cmap='gray', vmin=-1, vmax=1, extent=[0, w, 0, h], alpha=0.5)
        ax.set_xlim([0, w])
        ax.set_ylim([0, h])

        title += '(%i$\\times$%i %s %i)' % (w, h, env["generator"], env["seed"])
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
            radius = max(env["width"], env["height"]) / 20
            head_width = max(env["width"], env["height"]) / 100
            plt.arrow(start[0], start[1], math.cos(start[2]) * radius, math.sin(start[2]) * radius, width=0.01,
                      head_width=head_width,
                      label="start", color=colors[0])
            plt.arrow(goal[0], goal[1], math.cos(goal[2]) * radius, math.sin(goal[2]) * radius, width=0.01,
                      head_width=head_width,
                      label="start",
                      color=colors[1])
    if set_title:
        plt.title(title)


@click.command()
@click.option('--json_file', type=str, help='Name of the JSON file of a benchmarking run.')
@click.option('--run_id', default='0', type=str,
              help='ID numbers of the the runs ("all" or comma-separated list on integers).')
@click.option('--show_distances', default=False, type=bool)
@click.option('--draw_start_goal', default=True, type=bool)
@click.option('--draw_start_goal_thetas', default=True, type=bool)
@click.option('--set_title', default=True, type=bool)
@click.option('--headless', default=False, type=bool)
@click.option('--save_file', default=None, type=str)
@click.option('--dpi', default=200, type=int)
def main(json_file: str, run_id: str, draw_start_goal=True, draw_start_goal_thetas=True,
         set_title=True, show_distances=True, headless=False, save_file: str = None, dpi: int = 200):
    if headless:
        import matplotlib
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    data = json.load(open(json_file, 'r'))
    if run_id.lower() == "all":
        run_ids = list(range(len(data["runs"])))
    else:
        run_ids = [int(s.strip()) for s in run_id.split(',')]

    for i in run_ids:
        print("Plotting run %i" % i)
        env = data["runs"][i]["environment"]
        aspect_ratio = env["width"] / env["height"]
        plt.figure("Run %i" % i, figsize=(5, 5 * aspect_ratio))
        plot_env(env, i, draw_start_goal=draw_start_goal,
                 draw_start_goal_thetas=draw_start_goal_thetas,
                 set_title=set_title, show_distances=show_distances)
        if save_file is not None:
            plt.tight_layout()
            ext = save_file.rindex('.')
            filename = save_file[:ext] + '_%i' % i + save_file[ext:]
            plt.savefig(filename, dpi=dpi)
    if not headless:
        plt.show()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import json
import click
import matplotlib.pyplot as plt

from plot_env import plot_env
from plot_trajectory import plot_trajectory
from color import get_color, get_colors


@click.command()
@click.option('--json_file', help='Name of the JSON file of a benchmarking run.')
@click.option('--limit_runs', default=0, help='Number of runs to visualize (0 means all).')
@click.option('--max_plots_per_line', default=5, help='Number of runs to visualize (0 means all).')
def visualize(json_file: str, limit_runs: int = 0, max_plots_per_line: int = 5):
    data = json.load(open(json_file, "r"))
    max_plots_per_line = min(max_plots_per_line, len(data["runs"]))
    axes_h = max_plots_per_line
    axes_v = len(data["runs"]) // max_plots_per_line

    plt.figure(figsize=(axes_h * 7.5, axes_v * 6))
    for i, run in enumerate(data["runs"]):
        if 0 < limit_runs <= i:
            break
        plt.subplot(axes_v, axes_h, i + 1)
        plot_env(run["environment"], run_id=(i if len(data["runs"]) > 1 else -1),
                 draw_start_goal_thetas=data["settings"]["estimate_theta"])
        for j, (planner, plan) in enumerate(run["plans"].items()):
            plot_trajectory(plan["trajectory"], planner, data["settings"], color=get_color(j))

    plt.show()


if __name__ == '__main__':
    visualize()

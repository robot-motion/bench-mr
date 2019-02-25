#!/usr/bin/env python3
import json
import click
import matplotlib.pyplot as plt

from plot_env import plot_env
from plot_trajectory import plot_trajectory
from color import get_color, get_colors


@click.command(name='mpb-plotting', context_settings=dict(
    ignore_unknown_options=True,
    allow_extra_args=True,
))
@click.option('--json_file', help='Name of the JSON file of a benchmarking run.')
@click.option('--run_id', default='0', help='ID numbers of the the runs ("all" or comma-separated list on integers).')
@click.option('--plan', default='RRTstar', help='Name of the planner whose trajectory should be plotted.')
@click.pass_context
def visualize(ctx, json_file: str, run_id: str = "0", plan: str = "RRTstar"):
    data = json.load(open(json_file, "r"))

    kwargs = {ctx.args[i][2:]: ctx.args[i + 1] for i in range(0, len(ctx.args), 2)}
    print(kwargs)

    if run_id.lower() == "all":
        run_ids = list(range(len(data["runs"])))
    else:
        run_ids = [int(s.strip()) for s in run_id.split(',')]

    for i in run_ids:
        run = data["runs"][i]
        planner = run["plans"][plan]
        print("Plotting run %i" % i)
        colors = get_colors(len(planner["intermediary_solutions"]) + 1, "jet")
        plt.figure("Run %i (%s)" % (i, plan))
        plot_env(run["environment"], run_id=(i if len(data["runs"]) > 1 else -1),
                 draw_start_goal_thetas=data["settings"]["estimate_theta"], **kwargs)
        for j, intermediary in enumerate(planner["intermediary_solutions"]):
            plot_trajectory(intermediary["trajectory"], planner, data["settings"], color=colors[j], alpha=0.1, **kwargs)
        plot_trajectory(planner["trajectory"], planner, data["settings"], color=colors[-1], alpha=0.1, **kwargs)

    plt.show()


if __name__ == '__main__':
    visualize()

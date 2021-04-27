#!/usr/bin/env python3
import json
import click
import math
import sys

from plot_env import plot_env, plot_env_options
from plot_trajectory import plot_trajectory, plot_nodes, plot_trajectory_options
from color import get_color, get_colors, color_options

from utils import add_options, group, parse_run_ids, parse_planners, print_run_info, convert_planner_name


@group.command()
@click.option('--json_file', help='Name of the JSON file of a benchmarking run.', type=click.Path(exists=True))
@click.option('--run_id', default='all', type=str,
              help='ID numbers of the the runs ("all" or comma-separated list on integers).')
@click.option('--fig_width', default=6, type=float)
@click.option('--fig_height', default=6, type=float)
@click.option('--custom_min_x', default=None, type=float)
@click.option('--custom_min_y', default=None, type=float)
@click.option('--custom_max_x', default=None, type=float)
@click.option('--custom_max_y', default=None, type=float)
@click.option('--draw_nodes', default=False, type=bool)
@click.option('--draw_final_solution', default=True, type=bool)
@click.option('--max_plots_per_line', default=5, help='Number of runs to visualize (0 means all).')
@click.option('--headless', default=False, type=bool)
@click.option('--combine_views', default=True, type=bool)
@click.option('--silence', default=False, type=bool)
@click.option('--save_file', default=None, type=str)
@click.option('--dpi', default=200, type=int)
@click.option('--planners', default='', type=str)
@add_options(plot_env_options)
@add_options(plot_trajectory_options)
@add_options(color_options)
def main(**kwargs):
    print(kwargs)
    visualize_traj_convergence(**kwargs)


def visualize_traj_convergence(json_file: str, run_id: str = 'all',
                               draw_nodes=True,
                               max_plots_per_line: int = 5, headless=False,
                               combine_views=False,
                               save_file: str = None,
                               planners='',
                               draw_final_solution: bool = True,
                               fig_width: float = 7.4, fig_height: float = 6,
                               custom_min_x: float = None,
                               custom_min_y: float = None,
                               custom_max_x: float = None,
                               custom_max_y: float = None,
                               silence=False,
                               dpi: int = 200, **kwargs):
    inputs = locals()
    for key, item in inputs.items():
        kwargs[key] = item
    if not silence:
        click.echo("Visualizing %s..." % click.format_filename(json_file))

    if headless and 'matplotlib' not in sys.modules:
        import matplotlib
        matplotlib.use('Agg')
        click.echo("Running headless")
    import matplotlib.pyplot as plt

    import matplotlib as mpl
    mpl.rcParams['mathtext.fontset'] = 'cm'
    mpl.rcParams['pdf.fonttype'] = 42  # make sure to not use Level-3 fonts

    planners = parse_planners(planners)

    file = open(json_file, "r")
    data = json.load(file)
    file.close()
    run_ids = parse_run_ids(run_id, len(data["runs"]))

    if combine_views:
        max_plots_per_line = min(max_plots_per_line, len(run_ids))
        axes_h = max_plots_per_line
        axes_v = int(math.ceil(len(run_ids) / max_plots_per_line))
        plt.figure("MPB %s" % json_file, figsize=(axes_h * fig_width, axes_v * fig_height))

    plot_counter = 1
    for i in run_ids:
        run = data["runs"][i]
        if not silence:
            print_run_info(data, i, run_ids)
        if combine_views:
            plt.subplot(axes_v, axes_h, plot_counter)
            plot_counter += 1
        else:
            plt.figure("Run %i - %s" % (i, json_file), figsize=(fig_width, fig_height))
        kwargs['run_id'] = (i if len(data["runs"]) > 1 else -1)
        env = run["environment"]
        plot_env(env, **kwargs)
        if "settings" in run:
            settings = run["settings"]
            if not silence:
                print("Using settings from run %i." % i)
        else:
            settings = data["settings"]
        for j, (planner, plan) in enumerate(run["plans"].items()):
            if len(planners) > 0 and planner.lower() not in planners:
                continue
            if draw_final_solution:
                label = "%s (final)" % convert_planner_name(planner)
                plot_trajectory(plan["trajectory"], label, settings, color='k', **kwargs)
                if draw_nodes:
                    plot_nodes(plan["path"], label, settings, color='k', **kwargs)
            colors = get_colors(len(plan["intermediary_solutions"]), **kwargs)
            for si, solution in enumerate(plan["intermediary_solutions"]):
                label = '%s (%.2f s)' % (convert_planner_name(planner), solution["time"])
                plot_trajectory(solution["trajectory"], label, settings, color=colors[si], **kwargs)
                if draw_nodes:
                    plot_nodes(solution["path"], label, settings, color=colors[si], **kwargs)

        if not combine_views or plot_counter % axes_h == 0:
            # Shrink current axis by 20%
            ax = plt.gca()
            box = ax.get_position()
            ax.set_position([box.x0, box.y0, box.width * 0.8, box.height])
            plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

        if custom_min_x is not None and custom_max_y is not None:
            plt.gca().set_xlim([custom_min_x, custom_max_x])
            plt.gca().set_ylim([custom_min_y, custom_max_y])
        elif "min_x" in env and "max_y" in env:
            plt.gca().set_xlim([env["min_x"], env["max_x"]])
            plt.gca().set_ylim([env["min_y"], env["max_y"]])
        elif "width" in env and "height" in env:
            plt.gca().set_xlim([0, env["width"]])
            plt.gca().set_ylim([0, env["height"]])

        if not combine_views and save_file is not None:
            plt.tight_layout()
            ext = save_file.rindex('.')
            if len(run_ids) > 1:
                filename = save_file[:ext] + '_%i' % i + save_file[ext:]
            else:
                filename = save_file[:ext] + save_file[ext:]
            plt.savefig(filename, dpi=dpi, bbox_inches='tight')
            if not silence:
                click.echo("Saved %s." % filename)

    if combine_views and save_file is not None:
        plt.tight_layout()
        plt.savefig(save_file, dpi=dpi, bbox_inches='tight')
        if not silence:
            click.echo("Saved %s." % save_file)
    if not headless:
        plt.show()


if __name__ == '__main__':
    main()

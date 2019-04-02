#!/usr/bin/env python3
import json
import click
import math

from plot_env import plot_env, plot_env_options
from plot_trajectory import plot_trajectory, plot_nodes, plot_trajectory_options
from color import get_color, get_colors, color_options

from utils import add_options, group, parse_run_ids, parse_planners, parse_smoothers, show_legend


@group.command()
@click.option('--json_file', help='Name of the JSON file of a benchmarking run.', type=click.Path(exists=True))
@click.option('--run_id', default='all', type=str,
              help='ID numbers of the the runs ("all" or comma-separated list on integers).')
@click.option('--show_smoother', default=True, type=bool)
@click.option('--fig_width', default=6, type=float)
@click.option('--fig_height', default=6, type=float)
@click.option('--custom_min_x', default=None, type=float)
@click.option('--custom_min_y', default=None, type=float)
@click.option('--custom_max_x', default=None, type=float)
@click.option('--custom_max_y', default=None, type=float)
@click.option('--draw_nodes', default=False, type=bool)
@click.option('--draw_cusps', default=False, type=bool)
@click.option('--cusp_radius', default=1, type=float)
@click.option('--max_plots_per_line', default=5, help='Number of runs to visualize (0 means all).')
@click.option('--headless', default=False, type=bool)
@click.option('--combine_views', default=True, type=bool)
@click.option('--silence', default=False, type=bool)
@click.option('--save_file', default=None, type=str)
@click.option('--dpi', default=200, type=int)
@click.option('--ignore_planners', default='', type=str)
@add_options(plot_env_options)
@add_options(plot_trajectory_options)
@add_options(color_options)
def main(**kwargs):
    print(kwargs)
    visualize(**kwargs)


def visualize(json_file: str, run_id: str = 'all',
              show_smoother=False,
              show_only_smoother=False,
              draw_nodes=True,
              draw_cusps=False,
              cusp_radius: float = 1,
              max_plots_per_line: int = 5,
              headless=False,
              combine_views=False,
              save_file: str = None,
              ignore_planners='',
              ignore_smoothers='',
              fig_width: float = 6, fig_height: float = 6,
              custom_min_x: float = None,
              custom_min_y: float = None,
              custom_max_x: float = None,
              custom_max_y: float = None,
              silence=False,
              dpi: int = 200, **kwargs):
    kwargs.update(locals())
    if not silence:
        click.echo("Visualizing %s..." % click.format_filename(json_file))

    if headless:
        import matplotlib
        matplotlib.use('Agg')
        click.echo("Running headless")
    import matplotlib.pyplot as plt

    from matplotlib import patches
    from matplotlib.collections import PatchCollection

    import matplotlib as mpl
    mpl.rcParams['mathtext.fontset'] = 'cm'
    mpl.rcParams['pdf.fonttype'] = 42  # make sure to not use Level-3 fonts

    ignore_planners = parse_planners(ignore_planners)
    if len(ignore_planners) > 0 and not silence:
        click.echo('Ignoring the following planner(s): %s' % ', '.join(ignore_planners))

    ignore_smoothers = parse_smoothers(ignore_smoothers)
    if len(ignore_smoothers) > 0 and not silence:
        click.echo('Ignoring the following smoother(s): %s' % ', '.join(ignore_smoothers))

    data = json.load(open(json_file, "r"))
    run_ids = parse_run_ids(run_id, len(data["runs"]))

    if combine_views:
        max_plots_per_line = min(max_plots_per_line, len(run_ids))
        axes_h = max_plots_per_line
        axes_v = int(math.ceil(len(run_ids) / max_plots_per_line))
        plt.figure("MPB %s" % json_file, figsize=(axes_h * fig_width, axes_v * fig_height))

    plot_labels = []
    for i in run_ids:
        run = data["runs"][i]
        for j, (planner, plan) in enumerate(run["plans"].items()):
            if planner.lower() in ignore_planners:
                continue
            if not show_only_smoother:
                plot_labels.append(planner)
            if show_smoother and "smoothing" in plan and plan["smoothing"] is not None:
                for k, (smoother, smoothing) in enumerate(plan["smoothing"].items()):
                    if smoothing["name"] in ignore_smoothers:
                        continue
                    plot_labels.append("%s (%s)" % (planner, smoother))
    colors = get_colors(len(plot_labels), **kwargs)

    plot_counter = 1
    for i in run_ids:
        color_counter = 0
        run = data["runs"][i]
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
            if planner.lower() in ignore_planners:
                continue

            if not show_only_smoother:
                if draw_cusps:
                    circles = []
                    for cusp in plan["stats"]["cusps"]:
                        circle = patches.Circle(cusp, cusp_radius, ec="none")
                        circles.append(circle)
                    collection = PatchCollection(circles, alpha=0.5, color=colors[color_counter])
                    plt.gca().add_collection(collection)

                plot_trajectory(plan["trajectory"], planner, settings, color=colors[color_counter], **kwargs)
                color_counter += 1
            if draw_nodes:
                plot_nodes(plan["path"], planner, settings, color=colors[color_counter], **kwargs)

            if show_smoother and "smoothing" in plan and plan["smoothing"] is not None:
                for k, (smoother, smoothing) in enumerate(plan["smoothing"].items()):
                    if smoothing["name"] in ignore_smoothers:
                        continue
                    plot_trajectory(smoothing["trajectory"], "%s (%s)" % (planner, smoothing['name']), settings,
                                    color=colors[color_counter], **kwargs)
                    if draw_cusps:
                        circles = []
                        for cusp in smoothing["stats"]["cusps"]:
                            circle = patches.Circle(cusp, cusp_radius, ec="none")
                            circles.append(circle)
                        collection = PatchCollection(circles, alpha=0.5, color=colors[color_counter])
                        plt.gca().add_collection(collection)
                    if draw_nodes and "path" in smoothing:
                        plot_nodes(smoothing["path"], "%s (%s)" % (planner, smoother), settings,
                                   color=colors[color_counter], **kwargs)
                    color_counter += 1

        plt.gca().autoscale(False)
        plt.gca().set_aspect('equal', 'box')
        if custom_min_x is not None and custom_max_y is not None:
            plt.gca().set_xlim([custom_min_x, custom_max_x])
            plt.gca().set_ylim([custom_min_y, custom_max_y])
        elif "min_x" in env and "max_y" in env:
            plt.gca().set_xlim([env["min_x"], env["max_x"]])
            plt.gca().set_ylim([env["min_y"], env["max_y"]])
        elif "width" in env and "height" in env:
            plt.gca().set_xlim([0, env["width"]])
            plt.gca().set_ylim([0, env["height"]])

        if not combine_views or plot_counter % axes_h == axes_h-1:
            show_legend(**kwargs)

        if not combine_views and save_file is not None:
            ext = save_file.rindex('.')
            if len(run_ids) > 1:
                filename = save_file[:ext] + '_%i' % i + save_file[ext:]
            else:
                filename = save_file[:ext] + save_file[ext:]
            plt.savefig(filename, dpi=dpi, bbox_inches='tight')
            if not silence:
                click.echo("Saved %s." % filename)

    if combine_views and save_file is not None:
        plt.subplots_adjust(wspace=0.1, hspace=0.1)
        plt.savefig(save_file, dpi=dpi, bbox_inches='tight')
        if not silence:
            click.echo("Saved %s." % save_file)
    if not headless:
        plt.show()


if __name__ == '__main__':
    main()

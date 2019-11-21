#!/usr/bin/env python3
import json
import click
import math
import sys

import definitions
from definitions import smoother_names
from plot_env import plot_env, plot_env_options
from plot_trajectory import plot_trajectory, plot_nodes, plot_trajectory_options
from color import get_color, get_colors, color_options

from utils import add_options, group, parse_run_ids, parse_planners, parse_smoothers, show_legend, convert_planner_name


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


def visualize(json_file: str,
              run_id: str = 'all',
              show_smoother=False,
              show_only_smoother=False,
              draw_nodes=True,
              draw_cusps=False,
              cusp_radius: float = 1,
              draw_collisions=True,
              collision_radius: float = 1.2,
              max_plots_per_line: int = 5,
              headless=False,
              combine_views=True,
              save_file: str = None,
              ignore_planners='',
              ignore_smoothers='',
              fig_width: float = 6, fig_height: float = 6,
              custom_min_x: float = None,
              custom_min_y: float = None,
              custom_max_x: float = None,
              custom_max_y: float = None,
              silence=False,
              show_legend_once=True,
              use_existing_subplot=False,
              dpi: int = 200, **kwargs):
    kwargs.update(locals())
    if not silence:
        click.echo("Visualizing %s" % click.format_filename(json_file))

    if headless:
        import matplotlib
        matplotlib.use('Agg')
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

    file = open(json_file, "r")
    data = json.load(file)
    file.close()
    run_ids = parse_run_ids(run_id, len(data["runs"]))

    axes_h, axes_v = 1, 1
    if combine_views:
        max_plots_per_line = min(max_plots_per_line, len(run_ids))
        axes_h = max_plots_per_line
        axes_v = int(math.ceil(len(run_ids) / max_plots_per_line))
        if not use_existing_subplot:
            plt.figure("MPB %s" % json_file, figsize=(axes_h * fig_width, axes_v * fig_height))

    planners = []
    for run_id in run_ids:
        for planner in data["runs"][run_id]["plans"]:
            if planner in ignore_planners:
                continue
            if planner not in planners:
                planners.append(planner)
    planners = sorted(planners, key=convert_planner_name)

    plot_labels = []
    for i in run_ids:
        run = data["runs"][i]
        if run["plans"] is None:
            print("No plans were found in %s at run #%i." % (json_file, i))
            continue
        for j, planner in enumerate(planners):
            if planner.lower() in ignore_planners:
                continue
            if planner not in run["plans"]:
                continue
            plan = run["plans"][planner]
            if not show_only_smoother and convert_planner_name(planner) not in plot_labels:
                plot_labels.append(convert_planner_name(planner))
            if show_smoother and "smoothing" in plan and plan["smoothing"] is not None:
                for k, (smoother, smoothing) in enumerate(plan["smoothing"].items()):
                    if smoothing["name"] in ignore_smoothers:
                        continue
                    plot_label = "%s (%s)" % (convert_planner_name(planner), smoother_names[smoother])
                    if plot_label not in plot_labels:
                        plot_labels.append(plot_label)
    colors = get_colors(len(plot_labels), **kwargs)

    plot_counter = 1
    legend_shown = False
    for i in run_ids:
        color_counter = 0
        run = data["runs"][i]
        if not use_existing_subplot:
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
            # if not silence:
            #     print("Using settings from run %i." % i)
        else:
            settings = data["settings"]
        if run["plans"] is None:
            continue
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
                if draw_collisions and "collisions" in plan["stats"]:
                    circles = []
                    for collision in plan["stats"]["collisions"]:
                        circle = patches.Circle(collision, collision_radius, ec="none")
                        circles.append(circle)
                    collection = PatchCollection(circles, alpha=0.5, color=colors[color_counter])
                    plt.gca().add_collection(collection)

                plot_trajectory(plan["trajectory"], planner, settings, color=colors[color_counter], add_label=False,
                                **kwargs)
                if draw_nodes:
                    plot_nodes(plan["path"], planner, settings, color=colors[color_counter], **kwargs)
                color_counter += 1

            if show_smoother and "smoothing" in plan and plan["smoothing"] is not None:
                for k, (smoother, smoothing) in enumerate(plan["smoothing"].items()):
                    if smoothing["name"] in ignore_smoothers:
                        continue
                    plot_trajectory(smoothing["trajectory"], "%s (%s)" % (planner, smoothing['name']), settings,
                                    color=colors[color_counter], add_label=False, **kwargs)
                    if draw_cusps:
                        circles = []
                        for cusp in smoothing["stats"]["cusps"]:
                            circle = patches.Circle(cusp, cusp_radius, ec="none")
                            circles.append(circle)
                        collection = PatchCollection(circles, alpha=0.5, color=colors[color_counter])
                        plt.gca().add_collection(collection)
                    if draw_collisions and "collisions" in plan["stats"]:
                        circles = []
                        for collision in smoothing["stats"]["collisions"]:
                            circle = patches.Circle(collision, collision_radius, ec="none")
                            circles.append(circle)
                        collection = PatchCollection(circles, alpha=0.5, color=colors[color_counter])
                        plt.gca().add_collection(collection)
                    if draw_nodes and "path" in smoothing:
                        plot_nodes(smoothing["path"], "%s (%s)" % (planner, smoother), settings,
                                   color=colors[color_counter], **kwargs)
                    color_counter += 1
        # print("combine_views:", combine_views, "plot_counter:", plot_counter, "axes_h:", axes_h)
        if combine_views and (plot_counter - 1 == axes_h or axes_h == 1):
            for label, color in zip(plot_labels, colors):
                plt.plot([], [], color=color, label=label)
            show_legend(**kwargs)
            legend_shown = True

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

        if not combine_views and plot_counter % axes_h == axes_h - 1:
            for label, color in zip(plot_labels, colors):
                plt.plot([], [], color=color, label=label)
            if not show_legend_once or not legend_shown:
                show_legend(**kwargs)
            legend_shown = True

        if not combine_views and save_file is not None:
            ext = save_file.rindex('.')
            if len(run_ids) > 1:
                filename = save_file[:ext] + '_%i' % i + save_file[ext:]
            else:
                filename = save_file[:ext] + save_file[ext:]
            plt.savefig(filename, dpi=dpi, bbox_inches='tight')
            if not silence:
                click.echo("Saved %s." % filename)

    if combine_views:
        plt.subplots_adjust(wspace=0.4 / fig_width, hspace=0.7 / fig_height)
        if save_file is not None:
            plt.savefig(save_file, dpi=dpi, bbox_inches='tight')
            if not silence:
                click.echo("Saved %s." % save_file)
    if not headless:
        plt.show()


def visualize_grid(json_file: str,
                   run_id: str = 'all',
                   suptitle: str = None,
                   show_smoother=False,
                   show_only_smoother=False,
                   draw_nodes=True,
                   draw_cusps=False,
                   cusp_radius: float = 1,
                   draw_collisions=True,
                   show_stats=True,
                   collision_radius: float = 1.2,
                   max_plots_per_line: int = 5,
                   headless=False,
                   combine_views=True,
                   save_file: str = None,
                   ignore_planners='',
                   ignore_smoothers='',
                   fig_size: float = 6,
                   custom_min_x: float = None,
                   custom_min_y: float = None,
                   custom_max_x: float = None,
                   custom_max_y: float = None,
                   silence=False,
                   show_legend_once=True,
                   use_existing_subplot=False,
                   dpi: int = 200, **kwargs):
    kwargs.update(locals())
    if not silence:
        click.echo("Visualizing %s" % click.format_filename(json_file))

    if headless:
        import matplotlib
        matplotlib.use('Agg')
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

    file = open(json_file, "r")
    data = json.load(file)
    file.close()
    run_ids = parse_run_ids(run_id, len(data["runs"]))

    planners = []
    steer_functions = []
    for run_id in run_ids:
        run = data["runs"][run_id]
        for planner in run["plans"]:
            if planner in ignore_planners:
                continue
            if planner not in planners:
                planners.append(planner)
        s = run["settings"]["steer"]["steering_type"]
        if s not in steer_functions:
            steer_functions.append(s)
    planners = sorted(planners, key=convert_planner_name)

    # used to determine width/height ratio
    env_width, env_height = -1, -1
    
    valid_smoother_names = []

    plot_labels = []
    for i in run_ids:
        run = data["runs"][i]
        if env_width < 0 or env_height < 0:
            env_width = run["environment"]["width"]
            env_height = run["environment"]["height"]
        if run["plans"] is None:
            print("No plans were found in %s at run #%i." % (json_file, i))
            continue
        for j, planner in enumerate(planners):
            if planner.lower() in ignore_planners:
                continue
            if planner not in run["plans"]:
                continue
            plan = run["plans"][planner]
            if not show_only_smoother: # and convert_planner_name(planner) not in plot_labels:
                plot_labels.append(convert_planner_name(planner))
            if show_smoother and "smoothing" in plan and plan["smoothing"] is not None:
                for k, (smoother, smoothing) in enumerate(plan["smoothing"].items()):
                    if smoothing["name"] in ignore_smoothers:
                        continue
                    valid_smoother_names.append(smoothing["name"])
                    plot_label = "%s (%s)" % (convert_planner_name(planner), smoother_names[smoother])
#                     if plot_label not in plot_labels:
                    plot_labels.append(plot_label)
    colors = get_colors(len(planners) * (1 + len(valid_smoother_names)), **kwargs)

    total_plots = len(plot_labels) * len(steer_functions) * (1 + len(valid_smoother_names))

    if env_width > 0 and env_height > 0:
        if env_width > env_height:
            fig_width = fig_size
            fig_height = fig_size * env_height / env_width
        else:
            fig_width = fig_size * env_width / env_height
            fig_height = fig_size
    else:
        fig_width = fig_height = fig_size

    axes_h, axes_v = 1, 1
    if combine_views:
        max_plots_per_line = min(max_plots_per_line, len(planners))
        axes_h = max_plots_per_line
        axes_v = int(math.ceil(total_plots / max_plots_per_line))
        dpi = 200
        # assume 200 dpi resolution
        if axes_v * fig_height >= 2**16 / dpi:
            new_axes_v = int(2**16 / dpi // fig_height)
            print("Figure height must not be greater than 2^16 pixels, cropping figure from %i vertical axes to %i." \
                  % (axes_v, new_axes_v), file=sys.stderr)
            axes_v = new_axes_v
            total_plots = axes_h * axes_v
        if not use_existing_subplot:
            fig = plt.figure("MPB %s" % json_file, figsize=(axes_h * fig_width, axes_v * fig_height), dpi=dpi)
            if suptitle is not None:
                plt.suptitle(suptitle, fontsize=20, fontweight="bold", ha="left", y=1 + 1 / (axes_v * fig_height), x=0,
                             transform=fig.transFigure)

    plots_drawn = set()

    plot_counter = 0
    legend_shown = False
    for i in run_ids:
        run = data["runs"][i]
        kwargs['run_id'] = (i if len(data["runs"]) > 1 else -1)
        env = run["environment"]
        if "settings" in run:
            settings = run["settings"]
            # if not silence:
            #     print("Using settings from run %i." % i)
        else:
            settings = data["settings"]
        if run["plans"] is None:
            continue
        for j, planner in enumerate(planners):
            if planner.lower() in ignore_planners:
                continue
            if planner not in run["plans"]:
                continue
            plan = run["plans"][planner]

            s = run["settings"]["steer"]["steering_type"]
            plot_counter += 1 # steer_functions.index(s) * len(plot_labels) + planners.index(planner) + 1
            if plot_counter in plots_drawn or plot_counter > total_plots:
                continue
            else:
                plots_drawn.add(plot_counter)

            if combine_views:
                plt.subplot(axes_v, axes_h, plot_counter)
            else:
                plt.figure("Run %i - %s" % (i, json_file), figsize=(fig_width, fig_height))

            plot_env(env, set_title=False, show_start_goal_labels=False, **kwargs)

            plt.title(convert_planner_name(planner), loc="left", fontweight="bold")
            right_title = definitions.steer_function_names[definitions.steer_functions[s]]
            if len(run_ids) > 1:
                right_title += " (%i/%i)" % (i + 1, max(run_ids) + 1)
            plt.title(right_title, loc="right")

            if (plot_counter - 1) % axes_h > 0:
                # hide left ticks for subplots that are not on the left
                plt.gca().axes.yaxis.set_ticklabels([])
            if int((plot_counter - 1) / axes_h) != axes_v - 1:
                # hide bottom ticks for subplots that are not at the bottom
                plt.gca().axes.xaxis.set_ticklabels([])

            color_counter = planners.index(planner)

            if not show_only_smoother:
                if draw_cusps:
                    circles = []
                    for cusp in plan["stats"]["cusps"]:
                        circle = patches.Circle(cusp, cusp_radius, ec="none")
                        circles.append(circle)
                    collection = PatchCollection(circles, alpha=0.5, color=colors[color_counter])
                    plt.gca().add_collection(collection)
                if draw_collisions and "collisions" in plan["stats"]:
                    circles = []
                    for collision in plan["stats"]["collisions"]:
                        circle = patches.Circle(collision, collision_radius, ec="none")
                        circles.append(circle)
                    collection = PatchCollection(circles, alpha=0.5, color=colors[color_counter])
                    plt.gca().add_collection(collection)

                plot_trajectory(plan["trajectory"], planner, settings, color=colors[color_counter], add_label=False,
                                **kwargs)
                if show_stats and "stats" in plan and plan["stats"] is not None and plan["trajectory"] is not None:
                    non_empty_legend = False
                    for metric in ("path_length", "planning_time", "curvature"):
                        if plan["stats"][metric] is not None:
                            label = "%s:  %.3f" % (definitions.stat_names[metric], plan["stats"][metric])
                            plt.plot([], [], label=label, ls='', marker='')
                            non_empty_legend = True
                    if plan["stats"]["path_collides"] is not None and plan["stats"]["path_collides"]:
                        plt.plot([], [], label="--- collides ---", ls='', marker='')
                        non_empty_legend = True
                    if plan["stats"]["exact_goal_path"] is not None and not plan["stats"]["exact_goal_path"]:
                        plt.plot([], [], label="--- goal not reached ---", ls='', marker='')
                        non_empty_legend = True
                    if non_empty_legend:
                        plt.legend(handletextpad=-1.5)
                if draw_nodes:
                    plot_nodes(plan["path"], planner, settings, color=colors[color_counter], **kwargs)

            if show_smoother and "smoothing" in plan and plan["smoothing"] is not None:
                for k, (smoother, smoothing) in enumerate(plan["smoothing"].items()):
                    if smoothing["name"] in ignore_smoothers:
                        continue
                    plot_trajectory(smoothing["trajectory"], "%s (%s)" % (planner, smoothing['name']), settings,
                                    color=colors[color_counter], add_label=False, **kwargs)
                    if show_stats and "stats" in smoothing and smoothing["stats"] is not None and smoothing["trajectory"] is not None:
                        non_empty_legend = False
                        for metric in ("path_length", "planning_time", "curvature"):
                            if smoothing["stats"][metric] is not None:
                                label = "%s:  %.3f" % (definitions.stat_names[metric], smoothing["stats"][metric])
                                plt.plot([], [], label=label, ls='', marker='')
                                non_empty_legend = True
                        if smoothing["stats"]["path_collides"] is not None and smoothing["stats"]["path_collides"]:
                            plt.plot([], [], label="--- collides ---", ls='', marker='')
                            non_empty_legend = True
                        if smoothing["stats"]["exact_goal_path"] is not None and not smoothing["stats"]["exact_goal_path"]:
                            plt.plot([], [], label="--- goal not reached ---", ls='', marker='')
                            non_empty_legend = True
                        if non_empty_legend:
                            plt.legend(handletextpad=-1.5)
                    if draw_cusps:
                        circles = []
                        for cusp in smoothing["stats"]["cusps"]:
                            circle = patches.Circle(cusp, cusp_radius, ec="none")
                            circles.append(circle)
                        collection = PatchCollection(circles, alpha=0.5, color=colors[color_counter])
                        plt.gca().add_collection(collection)
                    if draw_collisions and "collisions" in plan["stats"]:
                        circles = []
                        for collision in smoothing["stats"]["collisions"]:
                            circle = patches.Circle(collision, collision_radius, ec="none")
                            circles.append(circle)
                        collection = PatchCollection(circles, alpha=0.5, color=colors[color_counter])
                        plt.gca().add_collection(collection)
                    if draw_nodes and "path" in smoothing:
                        plot_nodes(smoothing["path"], "%s (%s)" % (planner, smoother), settings,
                                   color=colors[color_counter], **kwargs)
                    color_counter += 1
        # print("combine_views:", combine_views, "plot_counter:", plot_counter, "axes_h:", axes_h)
        if combine_views and (plot_counter - 1 == axes_h or axes_h == 1):
            for label, color in zip(plot_labels, colors):
                plt.plot([], [], color=color, label=label)
            show_legend(**kwargs)
            legend_shown = True

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

        if not combine_views and plot_counter % axes_h == axes_h - 1:
            for label, color in zip(plot_labels, colors):
                plt.plot([], [], color=color, label=label)
            if not show_legend_once or not legend_shown:
                show_legend(**kwargs)
            legend_shown = True

        if not combine_views and save_file is not None:
            ext = save_file.rindex('.')
            if len(run_ids) > 1:
                filename = save_file[:ext] + '_%i' % i + save_file[ext:]
            else:
                filename = save_file[:ext] + save_file[ext:]
            plt.savefig(filename, dpi=dpi, bbox_inches='tight')
            if not silence:
                click.echo("Saved %s." % filename)

    if combine_views:
        plt.tight_layout()
        if save_file is not None:
            plt.savefig(save_file, dpi=dpi, bbox_inches='tight')
            if not silence:
                click.echo("Saved %s." % save_file)
    if not headless:
        plt.show()


if __name__ == '__main__':
    main()

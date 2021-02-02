#!/usr/bin/env python3
import json
import click
import math
import numpy as np
import sys

from color import get_colors

from utils import group, parse_metrics, parse_run_ids, print_run_info, parse_planners, parse_smoothers, \
    convert_planner_name, show_legend
from definitions import stat_names, smoothers, smoother_names
from plot_aggregate import plot_aggregate, plot_smoother_aggregate


@group.command()
@click.option('--json_file', help='Name of the JSON file of a benchmarking run.', type=click.Path(exists=True))
@click.option('--run_id', default='all', type=str,
              help='ID numbers of the the runs ("all" or comma-separated list on integers).')
@click.option('--max_plots_per_line', default=2, help='Number of runs to visualize (0 means all).')
@click.option('--headless', default=False, type=bool)
@click.option('--combine_views', default=True, type=bool)
@click.option('--plot_violins', default=True, type=bool)
@click.option('--save_file', default=None, type=str)
@click.option('--metrics', default='curvature, cost', type=str)
@click.option('--dpi', default=200, type=int)
@click.option('--ignore_planners', default='', type=str)
@click.option('--ticks_rotation', default=90, type=float)
@click.option('--fig_width', default=6, type=float)
@click.option('--fig_height', default=6, type=float)
def main(**kwargs):
    print(kwargs)
    plot_planner_stats(**kwargs)


# each violin has to have at least 2 entries
def ensure_valid_violin(arr):
    if not arr or len(arr) == 1:
        return [float('nan'), float('nan')]
    return arr


def plot_planner_stats(json_file: str,
                       run_id: str = 'all',
                       plot_violins=True,
                       max_plots_per_line: int = 3,
                       headless=False,
                       combine_views=True,
                       save_file: str = None,
                       ignore_planners='',
                       silence=False,
                       ticks_rotation=90,
                       fig_width: float = 6,
                       fig_height: float = 6,
                       metrics='path_length, normalized_curvature, aol, planning_time, mean_clearing_distance, cusps, aggregate',
                       dpi: int = 200,
                       scatter_mark_size=40, **kwargs):
    kwargs.update(locals())
    if not silence:
        click.echo("Visualizing %s..." % click.format_filename(json_file))

    stat_keys = parse_metrics(metrics)

    if headless and 'matplotlib' not in sys.modules:
        import matplotlib
        matplotlib.use('Agg')
        click.echo("Running headless")
    import matplotlib.pyplot as plt
    import matplotlib as mpl
    mpl.rcParams['mathtext.fontset'] = 'cm'
    mpl.rcParams['pdf.fonttype'] = 42  # make sure to not use Level-3 fonts

    ignore_planners = parse_planners(ignore_planners)
    if len(ignore_planners) > 0 and not silence:
        click.echo('Ignoring the following planner(s): %s' %
                   ', '.join(ignore_planners))

    file = open(json_file, "r")
    data = json.load(file)
    file.close()
    run_ids = parse_run_ids(run_id, len(data["runs"]))

    if combine_views:
        max_plots_per_line = min(max_plots_per_line, len(stat_keys))
        axes_h = max_plots_per_line
        axes_v = int(math.ceil(len(stat_keys) / max_plots_per_line))
        plt.figure("MPB Stats %s" % json_file, figsize=(
            axes_h * fig_width, axes_v * fig_height))

    # obtain number of planners
    planners = []
    for run_id in run_ids:
        for planner in data["runs"][run_id]["plans"]:
            if planner in ignore_planners:
                continue
            if planner not in planners:
                planners.append(planner)
    planners = sorted(planners, key=convert_planner_name)
    if 'num_colors' not in kwargs:
        kwargs['num_colors'] = len(planners)
    violin_colors = get_colors(**kwargs)
    ticks = np.arange(len(planners)) + 0.5

    valid_planners = []

    for si, stat_key in enumerate(stat_keys):
        ax = None
        if combine_views:
            ax = plt.subplot(axes_v, axes_h, si + 1)
        else:
            plt.figure("Run %i - %s (%s)" % (run_id, json_file,
                                             stat_names[stat_key]), figsize=(fig_width, fig_height))
            ax = plt.gca()

        if stat_key == "aggregate":
            plot_aggregate(ax, [data["runs"][i] for i in run_ids],
                           planners=planners, show_legend=True, **kwargs)
        else:
            stats = {}
            for run_id in run_ids:
                run = data["runs"][run_id]
                for j, planner in enumerate(planners):
                    if planner.lower() in ignore_planners:
                        continue
                    if planner not in run["plans"]:
                        continue
                    plan = run["plans"][planner]
                    if plan is None:
                        continue
                    if planner not in stats:
                        stats[planner] = []
                    if planner not in valid_planners:
                        valid_planners.append(planner)
                    if stat_key not in plan["stats"]:
                        stat = np.nan
                    elif stat_key == "cusps":
                        stat = len(plan["stats"]["cusps"])
                        if not plan["stats"]["path_found"]:
                            stat = np.nan
                    else:
                        stat = plan["stats"][stat_key]
                    if stat is None:
                        stat = np.nan
                    if not np.isnan(stat):
                        stats[planner].append(stat)
                    if not plot_violins:
                        if len(data["runs"]) > 1:
                            offset = 0.25 + 0.5 * run_id / \
                                (len(data["runs"])-1)
                        else:
                            offset = 0.5
                        plt.scatter([planners.index(planner) + offset],
                                    [stat],
                                    color=violin_colors[planners.index(
                                        planner) % kwargs['num_colors']],
                                    s=scatter_mark_size)
            kwargs['run_id'] = run_id
            plt.grid()
            plt.gca().set_axisbelow(True)

            if plot_violins:
                ticks = np.arange(len(valid_planners)) + 0.5
                violins = [ensure_valid_violin(
                    stats[planner]) for planner in valid_planners]
                try:
                    vs = plt.violinplot(violins, ticks, points=50, widths=0.8,
                                        showmeans=True, showextrema=False, showmedians=True)
                    for i, body in enumerate(vs["bodies"]):
                        body.set_facecolor(
                            violin_colors[i % kwargs['num_colors']])
                        body.set_edgecolor((0, 0, 0, 0))
                    for partname in ('cmeans', 'cmedians'):
                        vs[partname].set_edgecolor("black")
                    vs['cmeans'].set_edgecolor('green')

                    if not combine_views or si % axes_h == axes_h - 1:
                        plt.plot([np.nan], [np.nan],
                                 color="green", label="Mean")
                        plt.plot([np.nan], [np.nan],
                                 color="black", label="Median")
                        show_legend(**kwargs)
                except:
                    pass

        plt.xticks(ticks, [convert_planner_name(p)
                           for p in valid_planners], rotation=ticks_rotation, fontsize=14)
        plt.gca().set_xlim([0, len(valid_planners)])
        plt.title(stat_names[stat_key], fontsize=18, pad=15)

        if not combine_views and save_file is not None:
            plt.tight_layout()
            ext = save_file.rindex('.')
            filename = save_file[:ext] + '_%s' % stat_key + save_file[ext:]
            plt.savefig(filename, dpi=dpi, bbox_inches='tight')
            click.echo("Saved %s." % filename)

    plt.tight_layout()
    if combine_views and save_file is not None:
        plt.savefig(save_file, dpi=dpi, bbox_inches='tight')
        click.echo("Saved %s." % save_file)
    if not headless:
        plt.show()


def plot_smoother_stats(json_file: str,
                        run_id: str = 'all',
                        plot_violins=True,
                        max_plots_per_line: int = 2,
                        headless=False,
                        combine_views=True,
                        save_file: str = None,
                        ignore_planners='',
                        ignore_smoothers='',
                        separate_planners=True,
                        show_planners=True,
                        silence=False,
                        ticks_rotation=90,
                        fig_width: float = 8,
                        fig_height: float = 6,
                        metrics='path_length, normalized_curvature, planning_time, mean_clearing_distance, cusps, aggregate',
                        dpi: int = 200,
                        scatter_mark_size=40, **kwargs):
    kwargs.update(locals())
    if not silence:
        click.echo("Visualizing %s..." % click.format_filename(json_file))

    stat_keys = parse_metrics(metrics)

    if headless and 'matplotlib' not in sys.modules:
        import matplotlib
        matplotlib.use('Agg')
        click.echo("Running headless")
    import matplotlib.pyplot as plt
    import matplotlib as mpl
    mpl.rcParams['mathtext.fontset'] = 'cm'
    mpl.rcParams['pdf.fonttype'] = 42  # make sure to not use Level-3 fonts

    ignore_planners = parse_planners(ignore_planners)
    if len(ignore_planners) > 0 and not silence:
        click.echo('Ignoring the following planner(s): %s' %
                   ', '.join(ignore_planners))

    ignore_smoothers = parse_smoothers(ignore_smoothers)
    if len(ignore_smoothers) > 0 and not silence:
        click.echo('Ignoring the following smoother(s): %s' %
                   ', '.join(ignore_smoothers))

    data = json.load(open(json_file, "r"))
    run_ids = parse_run_ids(run_id, len(data["runs"]))

    if combine_views:
        max_plots_per_line = min(max_plots_per_line, len(stat_keys))
        axes_h = max_plots_per_line
        axes_v = int(math.ceil(len(stat_keys) / max_plots_per_line))
        plt.figure("MPB Stats %s" % json_file, figsize=(
            axes_h * fig_width, axes_v * fig_height))

    # obtain number of planners
    planners = []
    for run_id in run_ids:
        for planner in data["runs"][run_id]["plans"]:
            if planner in ignore_planners:
                continue
            if planner not in planners:
                planners.append(planner)

    valid_smoothers = []
    # determine x-ticks names
    if separate_planners:
        bar_names = []
        for planner in planners:
            if separate_planners and show_planners:
                bar_names.append(planner)
            for smoother in smoothers:
                if smoother in ignore_smoothers:
                    continue
                if smoother not in valid_smoothers:
                    valid_smoothers.append(smoother)
                if len(planners) == 1:
                    bar_names.append(smoother)
                else:
                    bar_names.append("%s (%s)" %
                                     (convert_planner_name(planner), smoother))
    else:
        bar_names = smoothers
        for smoother in smoothers:
            if smoother in ignore_smoothers:
                continue
            if smoother not in valid_smoothers:
                valid_smoothers.append(smoother)

    if 'num_colors' not in kwargs:
        kwargs['num_colors'] = len(planners)
    violin_colors = get_colors(**kwargs)
    ticks = np.arange(len(bar_names)) + 0.5

    for si, stat_key in enumerate(stat_keys):
        ax = None
        if combine_views:
            ax = plt.subplot(axes_v, axes_h, si + 1)
        else:
            plt.figure("Run %i - %s (%s)" % (run_id, json_file,
                                             stat_names[stat_key]), figsize=(fig_width, fig_height))
            ax = plt.gca()

        if stat_key == "aggregate":
            plot_smoother_aggregate(ax, [data["runs"][i] for i in run_ids], planners=planners,
                                    smoothers=valid_smoothers, show_legend=True,
                                    **kwargs)
        else:
            stats = {}
            for run_id in run_ids:
                run = data["runs"][run_id]
                if not run["plans"]:
                    continue
                for j, (planner, plan) in enumerate(run["plans"].items()):
                    if planner in ignore_planners:
                        continue
                    if separate_planners and show_planners:
                        if planner not in stats:
                            stats[planner] = []
                        if stat_key not in plan["stats"]:
                            stat = np.nan
                        elif stat_key == "cusps":
                            stat = len(plan["stats"]["cusps"])
                            if not plan["stats"]["path_found"]:
                                stat = np.nan
                        else:
                            stat = plan["stats"][stat_key]
                        if stat is None:
                            stat = np.nan
                        if not np.isnan(stat):
                            stats[planner].append(stat)
                        if len(data["runs"]) > 1:
                            offset = 0.25 + 0.5 * run_id / \
                                (len(data["runs"])-1)
                        else:
                            offset = 0.5
                        if not plot_violins:
                            plt.scatter([bar_names.index(planner) + offset],
                                        [stat],
                                        color=violin_colors[bar_names.index(
                                            planner) % kwargs['num_colors']],
                                        s=scatter_mark_size)
                    if "smoothing" in plan:
                        if not plan["smoothing"]:
                            continue
                        for smoother, smoothing in plan["smoothing"].items():
                            if smoother in ignore_smoothers:
                                continue
                            if separate_planners and len(planners) > 1:
                                bar_name = "%s (%s)" % (convert_planner_name(
                                    planner), smoother_names[smoother])
                            else:
                                bar_name = smoother_names[smoother]

                            if bar_name not in stats:
                                stats[bar_name] = []
                            if stat_key not in smoothing["stats"]:
                                stat = np.nan
                            elif stat_key == "cusps":
                                stat = len(smoothing["stats"]["cusps"])
                                if not smoothing["stats"]["path_found"]:
                                    stat = np.nan
                            else:
                                stat = smoothing["stats"][stat_key]
                                if stat_key == "planning_time":
                                    # add smoothing time to planning time
                                    stat += smoothing["time"]
                            if stat is None:
                                stat = np.nan
                            if not np.isnan(stat):
                                stats[bar_name].append(stat)
                            if not plot_violins:
                                plt.scatter([bar_names.index(bar_name) + 0.85 - 0.5 * run_id / len(data["runs"])],
                                            [stat],
                                            color=violin_colors[bar_names.index(
                                                bar_name) % kwargs['num_colors']],
                                            s=scatter_mark_size)
            kwargs['run_id'] = run_id
            plt.grid()
            plt.gca().set_axisbelow(True)

            bar_names = list(stats.keys())
            ticks = np.arange(len(bar_names)) + 0.5

            if plot_violins:
                violins = [ensure_valid_violin(
                    stats[bar_name]) for bar_name in bar_names]
                try:
                    vs = plt.violinplot(violins, ticks, points=50, widths=0.8,
                                        showmeans=True, showextrema=False, showmedians=True)
                    for i, body in enumerate(vs["bodies"]):
                        body.set_facecolor(
                            violin_colors[i % kwargs['num_colors']])
                        body.set_edgecolor((0, 0, 0, 0))
                    for partname in ('cmeans', 'cmedians'):
                        vs[partname].set_edgecolor("black")
                    vs['cmeans'].set_edgecolor('green')

                    if not combine_views or si % axes_h == axes_h - 1:
                        plt.plot([np.nan], [np.nan],
                                 color="green", label="Mean")
                        plt.plot([np.nan], [np.nan],
                                 color="black", label="Median")
                        show_legend(**kwargs)
                except:
                    pass

            plt.xticks(ticks, bar_names, rotation=ticks_rotation, fontsize=14)
            plt.gca().set_xlim([0, len(bar_names)])
            plt.title(stat_names[stat_key], fontsize=18, pad=15)

        if not combine_views and save_file is not None:
            plt.tight_layout()
            ext = save_file.rindex('.')
            filename = save_file[:ext] + '_%s' % stat_key + save_file[ext:]
            plt.savefig(filename, dpi=dpi, bbox_inches='tight')
            click.echo("Saved %s." % filename)

    plt.tight_layout()
    if combine_views and save_file is not None:
        plt.savefig(save_file, dpi=dpi, bbox_inches='tight')
        click.echo("Saved %s." % save_file)
    if not headless:
        plt.show()


if __name__ == '__main__':
    main()

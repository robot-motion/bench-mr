#!/usr/bin/env python3
import json
import click
import math
import numpy as np

from color import get_colors

from utils import group, parse_metrics, parse_run_ids, print_run_info, parse_planners, convert_planner_name
from definitions import stat_names
from plot_aggregate import plot_aggregate


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


def plot_planner_stats(json_file: str,
                       run_id: str = 'all',
                       plot_violins=True,
                       max_plots_per_line: int = 2,
                       headless=False,
                       combine_views=False,
                       save_file: str = None,
                       ignore_planners='',
                       silence=False,
                       ticks_rotation=90,
                       fig_width: float = 8,
                       fig_height: float = 6,
                       metrics='path_length, curvature, planning_time, mean_clearing_distance, cusps, aggregate',
                       dpi: int = 200, **kwargs):
    kwargs.update(locals())
    if not silence:
        click.echo("Visualizing %s..." % click.format_filename(json_file))

    stat_keys = parse_metrics(metrics)

    if headless:
        import matplotlib
        matplotlib.use('Agg')
        click.echo("Running headless")
    import matplotlib.pyplot as plt
    import matplotlib as mpl
    mpl.rcParams['mathtext.fontset'] = 'cm'
    mpl.rcParams['pdf.fonttype'] = 42  # make sure to not use Level-3 fonts

    ignore_planners = parse_planners(ignore_planners)
    if len(ignore_planners) > 0 and not silence:
        click.echo('Ignoring the following planner(s): %s' % ', '.join(ignore_planners))

    data = json.load(open(json_file, "r"))
    run_ids = parse_run_ids(run_id, len(data["runs"]))

    if combine_views:
        max_plots_per_line = min(max_plots_per_line, len(stat_keys))
        axes_h = max_plots_per_line
        axes_v = int(math.ceil(len(stat_keys) / max_plots_per_line))
        plt.figure("MPB Stats %s" % json_file, figsize=(axes_h * fig_width, axes_v * fig_height))

    # obtain number of planners
    planners = []
    for run_id in run_ids:
        for planner in data["runs"][run_id]["plans"]:
            if planner in ignore_planners:
                continue
            if planner not in planners:
                planners.append(planner)
    violin_colors = get_colors(len(planners), **kwargs)
    ticks = np.arange(len(planners)) + 0.5

    for si, stat_key in enumerate(stat_keys):
        ax = None
        if combine_views:
            if stat_key == 'aggregate':
                ax = plt.subplot("%i%i%i" % (axes_v, axes_h, si + 1), projection='3d', proj_type='ortho')
            else:
                ax = plt.subplot(axes_v, axes_h, si + 1)
        else:
            plt.figure("Run %i - %s (%s)" % (run_id, json_file, stat_names[stat_key]))
            if stat_key == 'aggregate':
                ax = plt.axes(projection='3d', proj_type='ortho')
            else:
                ax = plt.gca()

        if stat_key == "aggregate":
            plot_aggregate(ax, [data["runs"][i] for i in run_ids], planners=planners, show_legend=True, **kwargs)
        else:
            stats = {}
            for run_id in run_ids:
                run = data["runs"][run_id]
                for j, (planner, plan) in enumerate(run["plans"].items()):
                    if planner in ignore_planners:
                        continue
                    if planner not in stats:
                        stats[planner] = []
                    if stat_key not in plan["stats"]:
                        stat = np.nan
                    elif stat_key == "cusps":
                        stat = len(plan["stats"]["cusps"])
                    else:
                        stat = plan["stats"][stat_key]
                    if stat is None:
                        stat = np.nan
                    stats[planner].append(stat)
                    if not plot_violins:
                        plt.scatter([planners.index(planner) + 0.85 - 0.5 * run_id / len(data["runs"])],
                                    [stat],
                                    color=violin_colors[planners.index(planner)],
                                    s=4)
            kwargs['run_id'] = run_id
            plt.title(stat_names[stat_key], fontsize=20)
            plt.grid()
            plt.gca().set_axisbelow(True)

            if plot_violins:
                violins = [stats[planner] or [] for planner in planners]
                vs = plt.violinplot(violins, ticks, points=150, widths=0.6,
                                    showmeans=True, showextrema=False, showmedians=True)
                for i, body in enumerate(vs["bodies"]):
                    body.set_facecolor(violin_colors[i])
                    body.set_edgecolor((0, 0, 0, 0))
                for partname in ('cmeans', 'cmedians'):
                    vs[partname].set_edgecolor("black")
                vs['cmeans'].set_edgecolor('green')

                if not combine_views or si % axes_h == axes_h-1:
                    plt.plot([np.nan], [np.nan], color="green", label="Mean")
                    plt.plot([np.nan], [np.nan], color="black", label="Median")
                    plt.legend(bbox_to_anchor=(1.08, 1), loc=2, borderaxespad=0.)

        plt.xticks(ticks, [convert_planner_name(p) for p in planners], rotation=ticks_rotation)
        plt.gca().set_xlim([0, len(planners)])

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

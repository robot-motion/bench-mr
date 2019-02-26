#!/usr/bin/env python3
import json
import click
import math

from color import get_color, get_colors

from utils import add_options, group


@group.command()
@click.option('--json_file', help='Name of the JSON file of a benchmarking run.', type=click.Path(exists=True))
@click.option('--run_id', default='all', type=str,
              help='ID numbers of the the runs ("all" or comma-separated list on integers).')
@click.option('--max_plots_per_line', default=2, help='Number of runs to visualize (0 means all).')
@click.option('--headless', default=False, type=bool)
@click.option('--combine_views', default=True, type=bool)
@click.option('--save_file', default=None, type=str)
@click.option('--dpi', default=200, type=int)
def main(**kwargs):
    print(kwargs)
    visualize(**kwargs)


def visualize(json_file: str, run_id: str = 'all',
              show_smoother=False,
              draw_nodes=True,
              max_plots_per_line: int = 5, headless=False,
              combine_views=False,
              save_file: str = None,
              dpi: int = 200, **kwargs):
    click.echo("Visualizing %s..." % click.format_filename(json_file))

    stat_names = {
        'curvature': 'Curvature',
        'max_clearing_distance': 'Maximum Clearing Distance',
        'mean_clearing_distance': 'Mean Clearing Distance',
        'median_clearing_distance': 'Median Clearing Distance',
        'min_clearing_distance': 'Minimum Clearing Distance',
        'path_length': 'Path Length',
        'smoothness': 'Smoothness',
        'cost': 'Path Length'
    }

    stat_keys = [
        'curvature',
        # 'max_clearing_distance',
        # 'mean_clearing_distance',
        # 'median_clearing_distance',
        # 'min_clearing_distance',
        # 'path_length',
        # 'smoothness',
        'cost'
    ]

    if headless:
        import matplotlib
        matplotlib.use('Agg')
        click.echo("Running headless")
    import matplotlib.pyplot as plt

    import matplotlib as mpl
    mpl.rcParams['mathtext.fontset'] = 'cm'
    mpl.rcParams['pdf.fonttype'] = 42  # make sure to not use Level-3 fonts

    data = json.load(open(json_file, "r"))
    if run_id.lower() == "all":
        run_ids = list(range(len(data["runs"])))
    else:
        run_ids = [int(s.strip()) for s in run_id.split(',')]

    if combine_views:
        max_plots_per_line = min(max_plots_per_line, len(stat_keys))
        axes_h = max_plots_per_line
        axes_v = int(math.ceil(len(stat_keys) / max_plots_per_line))
        plt.figure("MPB Convergence %s" % json_file, figsize=(axes_h * 5, axes_v * 5))

    run_id = run_ids[0]
    for si, stat_key in enumerate(stat_keys):
        run = data["runs"][run_id]
        if combine_views:
            plt.subplot(axes_v, axes_h, si + 1)
        else:
            plt.figure("Run %i - %s (%s)" % (run_id, json_file, stat_key))
        kwargs['run_id'] = run_id
        env = run["environment"]
        plt.title(stat_names[stat_key], fontsize=20)
        plt.grid()
        plt.gca().set_xlabel("Planning Time [sec]", fontsize=18)
        if "settings" in run:
            settings = run["settings"]
            print("Using settings from run", run_id)
        else:
            settings = data["settings"]
        for j, (planner, plan) in enumerate(run["plans"].items()):
            if "intermediary_solutions" in plan and len(plan["intermediary_solutions"]) > 0:
                times = []
                stats = []
                for sol in plan["intermediary_solutions"]:
                    times.append(sol["time"])
                    if stat_key == 'cost':
                        stats.append(sol["cost"])
                    else:
                        stats.append(sol["stats"][stat_key])
                plt.plot(times, stats, '.-', color=get_color(j), label=planner)

        if not combine_views or si % axes_h == 0:
            plt.legend()

        if not combine_views and save_file is not None:
            plt.tight_layout()
            ext = save_file.rindex('.')
            filename = save_file[:ext] + '_%s' % stat_key + save_file[ext:]
            plt.savefig(filename, dpi=dpi, bbox_inches='tight')
            click.echo("Saved %s." % filename)

    if combine_views and save_file is not None:
        plt.tight_layout()
        plt.savefig(save_file, dpi=dpi, bbox_inches='tight')
        click.echo("Saved %s." % save_file)
    if not headless:
        plt.show()


if __name__ == '__main__':
    main()

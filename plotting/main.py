#!/usr/bin/env python3
import json
import click

from plot_env import plot_env, plot_env_options
from plot_trajectory import plot_trajectory, plot_trajectory_options
from color import get_color, get_colors

from utils import add_options, group


@group.command()
@click.option('--json_file', help='Name of the JSON file of a benchmarking run.', type=click.Path(exists=True))
@click.option('--run_id', default='all', type=str,
              help='ID numbers of the the runs ("all" or comma-separated list on integers).')
@click.option('--show_smoother', default=True, type=bool)
@click.option('--max_plots_per_line', default=5, help='Number of runs to visualize (0 means all).')
@click.option('--headless', default=False, type=bool)
@click.option('--combine_views', default=True, type=bool)
@click.option('--save_file', default=None, type=str)
@click.option('--dpi', default=200, type=int)
@add_options(plot_env_options)
@add_options(plot_trajectory_options)
def main(**kwargs):
    print(kwargs)
    visualize(**kwargs)


def visualize(json_file: str, run_id: str = 'all', show_smoother=False, max_plots_per_line: int = 5, headless=False,
              combine_views=False,
              save_file: str = None,
              dpi: int = 200, **kwargs):
    click.echo("Visualizing %s..." % click.format_filename(json_file))

    if headless:
        import matplotlib
        matplotlib.use('Agg')
        click.echo("Running headless")
    import matplotlib.pyplot as plt

    data = json.load(open(json_file, "r"))
    if run_id.lower() == "all":
        run_ids = list(range(len(data["runs"])))
    else:
        run_ids = [int(s.strip()) for s in run_id.split(',')]

    if combine_views:
        max_plots_per_line = min(max_plots_per_line, len(run_ids))
        axes_h = max_plots_per_line
        axes_v = len(run_ids) // max_plots_per_line
        plt.figure("MPB %s" % json_file, figsize=(axes_h * 7.5, axes_v * 6))

    for i in run_ids:
        run = data["runs"][i]
        if combine_views:
            plt.subplot(axes_v, axes_h, i + 1)
        else:
            plt.figure("Run %i - %s" % (i, json_file))
        kwargs['run_id'] = (i if len(data["runs"]) > 1 else -1)
        plot_env(run["environment"], **kwargs)
        for j, (planner, plan) in enumerate(run["plans"].items()):
            plot_trajectory(plan["trajectory"], planner, data["settings"], color=get_color(j), **kwargs)
            if show_smoother and "smoothing" in plan:
                for k, (smoother, smoothing) in enumerate(plan["smoothing"].items()):
                    plot_trajectory(smoothing["trajectory"], "%s (%s)" % (planner, smoother), data["settings"],
                                    color=get_color(j + k + 1), **kwargs)

        plt.gca().set_xlim([0, run["environment"]["width"]])
        plt.gca().set_ylim([0, run["environment"]["height"]])

        if not combine_views or i % axes_h == 0:
            plt.legend()

        if not combine_views and save_file is not None:
            plt.tight_layout()
            ext = save_file.rindex('.')
            filename = save_file[:ext] + '_%i' % i + save_file[ext:]
            plt.savefig(filename, dpi=dpi)
            click.echo("Saved %s." % filename)

    if combine_views and save_file is not None:
        plt.tight_layout()
        plt.savefig(save_file, dpi=dpi)
        click.echo("Saved %s." % save_file)
    if not headless:
        plt.show()


if __name__ == '__main__':
    main()

import numpy as np

from utils import show_legend, convert_planner_name
from definitions import smoother_names


def plot_aggregate(ax, runs, planners: [str], ticks_rotation=90, **kwargs):
    planners = [convert_planner_name(planner) for planner in planners]
    found = {planner: 0 for planner in planners}
    collision_free = {planner: 0 for planner in planners}
    exact = {planner: 0 for planner in planners}

    for run in runs:
        for j, (planner, plan) in enumerate(run["plans"].items()):
            planner = convert_planner_name(planner)
            if planner not in planners:
                continue

            if plan["stats"]["path_found"]:
                found[planner] += 1
                if not plan["stats"]["path_collides"]:
                    collision_free[planner] += 1
                if plan["stats"]["exact_goal_path"]:
                    exact[planner] += 1

    plot_aggregate_stats(ax, len(runs), found, collision_free, exact, ticks_rotation, **kwargs)


def plot_smoother_aggregate(ax, runs, planners: [str], smoothers: [str], separate_planners=False, show_planners=True,
                            ticks_rotation=90, **kwargs):
    if separate_planners:
        bar_names = []
        for planner in planners:
            planner = convert_planner_name(planner)
            if separate_planners and show_planners:
                bar_names.append(planner)
            for smoother in smoothers:
                bar_names.append("%s (%s)" % (convert_planner_name(planner), smoother))
    else:
        bar_names = smoothers

    total_solutions = len(runs)
    found = {bar_name: 0 for bar_name in bar_names}
    collision_free = {bar_name: 0 for bar_name in bar_names}
    exact = {bar_name: 0 for bar_name in bar_names}

    for run in runs:
        for j, (planner, plan) in enumerate(run["plans"].items()):
            planner = convert_planner_name(planner)
            if planner not in planners:
                continue

            if plan["stats"]["path_found"]:
                if separate_planners and show_planners:
                    found[planner] += 1
                    if not plan["stats"]["path_collides"]:
                        collision_free[planner] += 1
                    if plan["stats"]["exact_goal_path"]:
                        exact[planner] += 1
                if "smoothing" not in plan or plan["smoothing"] is None:
                    continue
                for smoother, smoothing in plan["smoothing"].items():
                    if smoothing["name"] not in smoothers:
                        continue
                    if separate_planners:
                        bar_name = "%s (%s)" % (planner, smoother_names[smoother])
                    else:
                        bar_name = smoothing["name"]
                    if smoothing["stats"]["path_found"]:
                        found[bar_name] += 1
                        if not smoothing["stats"]["path_collides"]:
                            collision_free[bar_name] += 1
                        if smoothing["stats"]["exact_goal_path"]:
                            exact[bar_name] += 1

    plot_aggregate_stats(ax, total_solutions, found, collision_free, exact, ticks_rotation, **kwargs)


def plot_aggregate_stats(ax, total: int, found: {str: int}, collision_free: {str: int}, exact: {str: int},
                         ticks_rotation=90, show_aggregate_title=True, **kwargs):
    import matplotlib.pyplot as plt
    planners = list(found.keys())
    width = 0.25
    xs = np.arange(len(planners)) + 0.5
    ys = [total for _ in planners]
    ax.bar(xs, ys, width=0.71, linewidth=2, color="lightgray", edgecolor="black", linestyle="-", label="Total runs")
    ys = [found[planner] for planner in planners]
    ax.bar(xs, ys, width=0.7, label="Found solutions")
    ys = [collision_free[planner] for planner in planners]
    ax.bar(xs-0.15, ys, hatch='/', width=width, label="Collision-free")
    ys = [exact[planner] for planner in planners]
    ax.bar(xs+0.15, ys, hatch='\\', width=width, color="yellow", label="Exact solution")

    plt.xticks(xs, planners, rotation=ticks_rotation, fontsize=14)
    plt.gca().set_xlim([0, len(planners)])
    if show_aggregate_title:
        plt.title("Aggregate", fontsize=18, pad=15)

    ax.grid()
    show_legend(**kwargs)

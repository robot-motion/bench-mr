import numpy as np


def plot_aggregate(ax, runs, planners: [str], show_legend=True, **_):
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
    # ax = plt.axes(projection='3d')
    found = {planner: 0 for planner in planners}
    collision_free = {planner: 0 for planner in planners}
    exact = {planner: 0 for planner in planners}

    for run in runs:
        for j, (planner, plan) in enumerate(run["plans"].items()):
            if planner not in planners:
                continue

            if plan["stats"]["path_found"]:
                found[planner] += 1
                if not plan["stats"]["path_collides"]:
                    collision_free[planner] += 1
                if plan["stats"]["exact_goal_path"]:
                    exact[planner] += 1

    width = 0.25
    depth = 0.45
    x = np.arange(len(planners))
    y = np.ones(len(planners))
    bottom = np.zeros(len(planners))
    found_zs = [found[planner] for planner in planners]
    ax.bar3d(x+0.09, y + 1.0, bottom, width, depth, found_zs, shade=False)
    plt.plot([], [], label="Found solutions")
    collision_free_zs = [collision_free[planner] for planner in planners]
    ax.bar3d(x+0.39, y + 0.5, bottom, width, depth, collision_free_zs, shade=False)
    plt.plot([], [], label="Collision-free")
    exact_zs = [exact[planner] for planner in planners]
    ax.bar3d(x+0.69, y + 0.0, bottom, width, depth, exact_zs, shade=False)
    plt.plot([], [], label="Exact")
    plt.title("Aggregated", fontsize=20, pad=40)

    ax.set_yticks([])
    ax.view_init(0, -90)
    ax.dist = 5.8
    if show_legend:
        plt.legend(loc="center left")

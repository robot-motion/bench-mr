#!/usr/bin/env python3
import click
import json
import os
from definitions import steer_functions, steer_function_names, smoother_names, smoothers, planner_names, robot_models, robot_models_names
import numpy as np

# Fix random seed (used by kernel density estimation in violin plots)
np.random.seed(123)


def safe_mean(xs):
    return np.mean([x for x in xs if x is not None and not np.isnan(x)])


def safe_std(xs):
    return np.std([x for x in xs if x is not None and not np.isnan(x)])


def safe_sum(xs):
    return np.sum([x for x in xs if x is not None and not np.isnan(x)])


def safe_min(xs):
    return np.min([x for x in xs if x is not None and not np.isnan(x)])


def safe_max(xs):
    return np.max([x for x in xs if x is not None and not np.isnan(x)])


def add_options(options):
    def _add_options(func):
        for option in reversed(options):
            func = option(func)
        return func

    return _add_options


@click.group()
def group(**_):
    pass


def is_int(s: str):
    try:
        int(s)
        return True
    except ValueError:
        return False


def is_float(s: str):
    try:
        float(s)
        return True
    except ValueError:
        return False


def parse_run_ids(run_id: str, total: int) -> [int]:
    """
    Parses lists of run IDs, e.g., "all", "0-2, -1".
    :param run_id: String with run IDs.
    :param total: Total number of available runs.
    :return: List of run IDs.
    """
    run_id = run_id.replace(' ', '')
    if run_id == '' or run_id.lower() == "all":
        return list(range(total))

    def parse_int(s: str):
        return (int(s) + total) % total

    run_ids = []
    rs = [s.strip() for s in run_id.split(',')]
    for r in rs:
        if ':' in r:
            ri = r.index(':')
            start = 0 if ri == 0 else parse_int(r[:ri])
            end = total if ri == len(r) - 1 else parse_int(r[(ri + 1):])
            run_ids += list(range(start, end))
        else:
            run_ids.append(parse_int(r))
    return run_ids


def parse_steer_functions(sf: str) -> [int]:
    """
    Returns list of indices corresponding to the selected steer functions.
    :param sf: A string like "all", "0-2", "posq,dubins"
    :return: List of integers.
    """
    total = len(steer_functions)
    sf = sf.replace(' ', '')
    if sf == '' or sf.lower() == "all":
        return list(range(total))

    def parse_int(s: str):
        return (int(s) + total) % total

    sfs = []
    rs = [s.strip() for s in sf.split(',')]
    for r in rs:
        if ':' in r:
            ri = r.index(':')
            start = 0 if ri == 0 else parse_int(r[:ri])
            end = total if ri == len(r) - 1 else parse_int(r[(ri + 1):])
            sfs += list(range(start, end))
        elif is_int(r):
            sfs.append(parse_int(r))
        elif r in steer_functions:
            sfs.append(steer_functions.index(r))
        else:
            click.echo('Substring "%s" could not identify a steer function.' %
                       r,
                       err=True)
    return sfs


def parse_robot_models(rm: str) -> [int]:
    """
    Returns list of indices corresponding to the selected steer functions.
    :param rm: A string like "all", "0-2", "posq,dubins"
    :return: List of integers.
    """
    total = len(robot_models)
    rm = rm.replace(' ', '')
    if rm == '' or rm.lower() == "all":
        return list(range(total))

    def parse_int(s: str):
        return (int(s) + total) % total

    rms = []
    rs = [s.strip() for s in rm.split(',')]
    for r in rs:
        if ':' in r:
            ri = r.index(':')
            start = 0 if ri == 0 else parse_int(r[:ri])
            end = total if ri == len(r) - 1 else parse_int(r[(ri + 1):])
            rms += list(range(start, end))
        elif is_int(r):
            rms.append(parse_int(r))
        elif r in robot_models:
            rms.append(robot_models.index(r))
        else:
            click.echo('Substring "%s" could not identify a steer function.' %
                       r,
                       err=True)
    return rms


def parse_planners(planners: str) -> {str: str}:
    return {
        s.strip().lower(): s.strip()
        for s in planners.split(',') if len(s.strip()) > 0
    }


def parse_smoothers(ss: str) -> {str: str}:
    if ss == 'all':
        return smoothers
    ss = [
        s.strip().lower().replace(' ', '') for s in ss.split(',')
        if len(s.strip()) > 0
    ]
    result = {}
    for key, val in smoother_names.items():
        for s in ss:
            if (s in key.lower().replace(' ', '').replace('_', '').replace(
                    '-', '') or s in val.lower().replace(' ', '').replace(
                        '_', '').replace('-', '')):
                result[val] = s
    return result


def parse_metrics(metrics: str) -> [str]:
    from definitions import stat_names
    if metrics.lower().strip() == "all":
        return list(stat_names.values())
    return [
        s.strip().lower() for s in metrics.split(',')
        if s.strip().lower() in stat_names
    ]


def print_run_info(data, run_id: int, run_ids: [int]):
    run = data["runs"][run_id]
    title = '%s Run #%i (%i / %i) %s' % (
        '+' * 25, run_id, run_ids.index(run_id) + 1, len(run_ids), '+' * 25)
    click.echo(title)
    steering = steer_function_names[steer_functions[data["settings"]["steer"]
                                                    ["steering_type"]]]
    controlbased = data["settings"]["benchmark"]["control_planners_on"]

    robot_model = robot_models_names[robot_models[data["settings"]
                                                  ["forwardpropagation"]["forward_propagation_type"]]]
    if "settings" in run:
        steering = steer_function_names[steer_functions[
            run["settings"]["steer"]["steering_type"]]]
        robot_model = robot_models_names[robot_models[run["settings"]
                                                      ["forwardpropagation"]["forward_propagation_type"]]]

    if(controlbased):
        click.echo('+ Robot Model:        %s ' % robot_model)
    else:
        click.echo('+ Steering:        %s ' % steering)

    env = run["environment"]
    click.echo('+ Environment:     %s' % env["name"])
    click.echo('+ Planners:        %s' % (', '.join(run["plans"].keys())))
    total_count = len(run["plans"])
    found_count = len([
        plan for plan in run["plans"].values() if plan["stats"]["path_found"]
    ])
    exact_count = len([
        plan for plan in run["plans"].values()
        if plan["stats"]["path_found"] and plan["stats"]["exact_goal_path"]
    ])
    collision_count = len([
        plan for plan in run["plans"].values()
        if plan["stats"]["path_found"] and plan["stats"]["path_collides"]
    ])
    click.echo('+ Found solution:  %i / %i' % (found_count, total_count))
    click.echo('+ Exact solution:  %i / %i' % (exact_count, total_count))
    click.echo('+ Found colliding: %i / %i' % (collision_count, found_count))
    click.echo('+' * len(title) + '\n')


def convert_planner_name(planner: str) -> str:
    if planner in planner_names:
        return planner_names[planner]
    return planner.replace('star', '*').replace("two", "2").replace('_', ' ').replace('kBIT', 'BIT') \
        .replace('KPIECE1', 'KPIECE').replace('Informed', 'Informed ')


def show_legend(show_legend=True,
                show_legend_outside=True,
                legend_location="best",
                **_):
    if not show_legend:
        return
    try:
        import matplotlib.pyplot as plt
        if show_legend_outside:
            plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
        else:
            plt.legend(loc=legend_location)
    except:
        pass


def latexify(text: str) -> str:
    text = text.replace('#', '\\#')
    text = text.replace('%', '\\%')
    text = text.replace('*', '${}^*$')
    text = text.replace('_', '\\_')
    return text


def get_planners(results_filename: str) -> [str]:
    planners = []
    with open(results_filename, 'r') as rf:
        runs = json.load(rf)["runs"]
        for run in runs:
            if "plans" not in run:
                continue
            for planner in run["plans"].keys():
                if planner not in planners:
                    planners.append(planner)
    planners = sorted(planners, key=convert_planner_name)
    return planners


def get_aggregate_stats(results_filenames: [str]) -> dict:
    found = {}
    collision_free = {}
    exact = {}
    totals = {}
    for filename in results_filenames:
        if not os.path.exists(filename):
            continue
        try:
            with open(filename, 'r') as rf:
                runs = json.load(rf)["runs"]
                for run in runs:
                    for j, (planner, plan) in enumerate(run["plans"].items()):
                        planner = convert_planner_name(planner)
                        totals[planner] = totals.get(planner, 0) + 1
                        if plan["stats"]["path_found"]:
                            found[planner] = found.get(planner, 0) + 1
                            if not plan["stats"]["path_collides"]:
                                collision_free[planner] = collision_free.get(
                                    planner, 0) + 1
                            if plan["stats"]["exact_goal_path"]:
                                exact[planner] = exact.get(planner, 0) + 1
                        else:
                            if planner not in found:
                                found[planner] = 0
        except:
            pass
    return {
        "total": (max(totals.values()) if len(totals) > 0 else 0),
        "found": found,
        "collision_free": collision_free,
        "exact": exact
    }

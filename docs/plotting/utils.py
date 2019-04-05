#!/usr/bin/env python3
import click

from definitions import steer_functions, steer_function_names
import numpy as np

# Fix random seed (used by kernel density estimation in violin plots)
np.random.seed(123)


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
            click.echo('Substring "%s" could not identify a steer function.' % r, err=True)
    return sfs


def parse_planners(planners: str) -> {str: str}:
    return {s.strip().lower(): s.strip() for s in planners.split(',') if len(s.strip()) > 0}


def parse_metrics(metrics: str) -> [str]:
    from definitions import stat_names
    if metrics.lower().strip() == "all":
        return list(stat_names.values())
    return [s.strip().lower() for s in metrics.split(',') if s.strip().lower() in stat_names]


def print_run_info(data, run_id: int, run_ids: [int]):
    run = data["runs"][run_id]
    title = '%s Run #%i (%i / %i) %s' % ('+' * 25, run_id, run_ids.index(run_id)+1, len(run_ids), '+' * 25)
    click.echo(title)
    steering = steer_function_names[steer_functions[data["settings"]["steer"]["steering_type"]]]
    if "settings" in run:
        steering = steer_function_names[steer_functions[run["settings"]["steer"]["steering_type"]]]
    click.echo('+ Steering:        %s ' % steering)
    env = run["environment"]
    click.echo('+ Environment:     %s' % env["name"])
    total_count = len(run["plans"])
    found_count = len([plan for plan in run["plans"].values() if plan["stats"]["path_found"]])
    exact_count = len(
        [plan for plan in run["plans"].values() if plan["stats"]["path_found"] and plan["stats"]["exact_goal_path"]])
    collision_count = len(
        [plan for plan in run["plans"].values() if plan["stats"]["path_found"] and plan["stats"]["path_collides"]])
    click.echo('+ Found solution:  %i / %i' % (found_count, total_count))
    click.echo('+ Exact solution:  %i / %i' % (exact_count, total_count))
    click.echo('+ Found colliding: %i / %i' % (collision_count, found_count))
    click.echo('+' * len(title) + '\n')


def convert_planner_name(planner: str) -> str:
    return planner.replace('star', '*').replace("two", "2")

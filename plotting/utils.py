#!/usr/bin/env python3
import click

from definitions import steer_functions, steer_function_names


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
            end = total if ri == len(r)-1 else parse_int(r[(ri+1):])
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

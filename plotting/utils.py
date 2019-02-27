#!/usr/bin/env python3
import click


def add_options(options):
    def _add_options(func):
        for option in reversed(options):
            func = option(func)
        return func

    return _add_options


@click.group()
def group(**_):
    pass


def parse_run_ids(run_id: str, total: int):
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

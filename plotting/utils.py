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
    for range in rs:
        if ':' in range:
            ri = range.index(':')
            start = 0 if ri == 0 else parse_int(range[:ri])
            end = total if ri == len(range)-1 else parse_int(range[(ri+1):])
            run_ids += list(range(start, end))

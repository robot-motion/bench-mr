#!/usr/bin/env python3

import matplotlib
import matplotlib.cm as cmx
import matplotlib.pyplot as plt

import click
from utils import add_options

color_options = [
    click.option('--num_colors', default=10, type=int),
    click.option('--color_map_name', default='tab10', type=str)
]


@add_options(color_options)
def get_color(color_id, num_colors=20, color_map_name='tab20', **_):
    cm = plt.get_cmap(color_map_name)
    cNorm  = matplotlib.colors.Normalize(vmin=0, vmax=num_colors)
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=cm)
    return scalarMap.to_rgba(color_id)


@add_options(color_options)
def get_colors(num_colors=20, color_map_name='tab20', **_):
    cm = plt.get_cmap(color_map_name)
    cNorm  = matplotlib.colors.Normalize(vmin=0, vmax=num_colors)
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=cm)
    return [scalarMap.to_rgba(color_id) for color_id in range(num_colors)]

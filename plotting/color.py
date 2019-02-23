#!/usr/bin/env python3

import matplotlib
import matplotlib.cm as cmx
import matplotlib.pyplot as plt


def get_color(color_id, num_colors=10, color_map_name='tab10'):
    cm = plt.get_cmap(color_map_name)
    cNorm  = matplotlib.colors.Normalize(vmin=0, vmax=num_colors)
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=cm)
    return scalarMap.to_rgba(color_id)


def get_colors(num_colors=10, color_map_name='tab10'):
    cm = plt.get_cmap(color_map_name)
    cNorm  = matplotlib.colors.Normalize(vmin=0, vmax=num_colors)
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=cm)
    return [scalarMap.to_rgba(color_id) for color_id in range(num_colors)]

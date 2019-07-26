from typing import Union


class Series:
    """
    Series abstracts a diagram  or table where multiple data sequences pertaining to different metrics
    can be visualized. Each data source (e.g. a planner) can have multiple different data sequences
    which are summarized into a std interval, or plotted using min/max shades.
    Multiple metrics result in multiple plots, or multiple subtables.
    The x-coordinate for each data entry can be specified so that the min/max bound is visualized using
    the convex hull of all the points per data source and metric.
    """
    def __init__(self, metrics=None, xlabel: str = ""):
        if metrics is None:
            metrics = []
        self.sources = set()
        self.metrics = metrics
        self.xlabel = xlabel

    def add_sequence(self, source: str, values: [float], xs: [Union[str, float]] = None):
        pass

    def plot_lines(self, **kwargs):
        pass

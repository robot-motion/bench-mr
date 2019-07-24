import json
import subprocess
import datetime
import time
import itertools
import os
from utils import *
from tqdm import tqdm_notebook

MPB_BINARY = './benchmark'
MPB_BINARY_DIR = '../bin'


class MPB:
    def __init__(self, config_file: str = '../bin/benchmark_template.json'):
        self.config = json.load(open(config_file, 'r'))["settings"]
        self.id = None
        self._planners = []
        self._smoothers = []
        self._steer_functions = []
        self.set_planners(['rrt'])
        self.set_smoothers([])
        self.set_steer_functions(['reeds_shepp'])
        self.config_filename = None
        self.results_filename = None

    def __getitem__(self, item: str):
        c = self.config
        for s in item.split('.'):
            c = c[s]
        return c

    def __setitem__(self, item: str, value):
        c = self.config
        splits = item.split('.')
        for s in splits[:-1]:
            c = c[s]
        c[splits[-1]] = value
        return c

    def set_random_grid_env(self,
                            width: int = 50,
                            height: int = 50,
                            obstacle_ratio: float = 0.1,
                            seed: int = 1):
        self["env.type"] = "grid"
        self["env.grid.generator"] = "random"
        self["env.grid.width"] = width
        self["env.grid.height"] = height
        self["env.grid.seed"] = seed
        self["env.grid.random.obstacle_ratio"] = obstacle_ratio

    def set_corridor_grid_env(self,
                              width: int = 50,
                              height: int = 50,
                              branches: int = 40,
                              radius: float = 3.,
                              seed: int = 1):
        self["env.type"] = "grid"
        self["env.grid.generator"] = "corridor"
        self["env.grid.width"] = width
        self["env.grid.height"] = height
        self["env.grid.seed"] = seed
        self["env.grid.corridor.branches"] = branches
        self["env.grid.corridor.radius"] = radius

    def set_planners(self, planners: [str]):
        planners = list(set(itertools.chain.from_iterable(map(parse_planners, planners))))
        self._planners = []
        for p in self["benchmark.planning"].keys():
            if p in planners:
                self["benchmark.planning." + p] = True
                print("Using planner %s." % p)
                self._planners.append(p)
            else:
                self["benchmark.planning." + p] = False

    def set_steer_functions(self, steerings: [str]):
        self._steer_functions = list(set(itertools.chain.from_iterable(map(parse_steer_functions, steerings))))
        for name, p in zip(steerings, self._steer_functions):
            print("Using steer function %s." % name)
        self["benchmark.steer_functions"] = self._steer_functions

    def set_smoothers(self, smoothers: [str]):
        smoothers = list(set(itertools.chain.from_iterable(map(parse_smoothers, smoothers))))
        self._smoothers = []
        for p in self["benchmark.smoothing"].keys():
            if p in smoothers:
                self["benchmark.smoothing." + p] = True
                print("Using smoother %s." % p)
                self._smoothers.append(p)
            else:
                self["benchmark.smoothing." + p] = False

    def save_settings(self, filename: str):
        json.dump({
            "settings": self.config
        }, open(filename, 'w'), indent=2)

    def run(self, id: str = None, runs: int = 1):
        self["benchmark.runs"] = runs
        ts = time.time()
        if id:
            self.id = id
        else:
            self.id = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H-%M-%S')
        self.config_filename = self.id + "_config.json"
        self.results_filename = self.id + "_results.json"
        self["benchmark.log_file"] = os.path.abspath(self.results_filename)
        log_filename = self.id + ".log"
        logfile = open(log_filename, 'w')
        self.save_settings(self.config_filename)
        print("Running MPB with ID %s (log file at %s)..." % (self.id, log_filename))
        total_iterations = len(self._planners) * len(self._steer_functions) * runs * (1 + len(self._smoothers))
        print(total_iterations)
        pbar = tqdm_notebook(range(total_iterations))
        tsk = subprocess.Popen([MPB_BINARY, os.path.abspath(self.config_filename)],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               cwd=os.path.abspath(MPB_BINARY_DIR))
        iteration = 1
        while True:
            line = tsk.stdout.readline()
            if line is None:
                break
            line = line.decode('UTF-8')
            if line == '':
                break
            if 'Path evaluation complete.' in line:
                pbar.update(1)
                iteration += 1
            logfile.write(line)
        pbar.close()
        logfile.close()
        return tsk.returncode

    def visualize_trajectories(self, **kwargs):
        from trajectory import visualize
        visualize(self.results_filename, **kwargs)

    def plot_planner_stats(self, **kwargs):
        from plot_stats import plot_planner_stats
        plot_planner_stats(self.results_filename, **kwargs)

    def plot_smoother_stats(self, **kwargs):
        from plot_stats import plot_smoother_stats
        plot_smoother_stats(self.results_filename, **kwargs)

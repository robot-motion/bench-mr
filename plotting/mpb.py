import json
import subprocess
import datetime
import time
import itertools
import os
import sys
from typing import Optional

from utils import *
from multiprocessing import Pool
from tqdm import tqdm_notebook

MPB_BINARY = './benchmark'
MPB_BINARY_DIR = '../bin'


class MPB:
    def __init__(self,
                 config_file: str = '../bin/benchmark_template.json',
                 output_path: str = ''):
        self.config = json.load(open(config_file, 'r'))["settings"]  # type: dict
        self.output_path = output_path  # type: str
        self.id = None  # type: Optional[str]
        self._planners = []  # type: [str]
        self._smoothers = []  # type: [str]
        self._steer_functions = []  # type: [str]
        self.set_planners(['rrt'])
        self.set_smoothers([])
        self.set_steer_functions(['reeds_shepp'])
        self.config_filename = None  # type: Optional[str]
        self.results_filename = None  # type: Optional[str]

        bin_path = os.path.abspath(os.path.join(MPB_BINARY_DIR, MPB_BINARY))
        if not os.path.exists(bin_path):
            print('Error: Could not find benchmark binary file at %s. ' % bin_path +
                  'Make sure you have built it and set the correct MPB_BINARY_DIR variable in %s.' % __file__,
                  file=sys.stderr)

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

    def set_id(self, id: str):
        self.id = id
        self.config_filename = self.id + "_config.json"
        self.results_filename = self.id + "_results.json"
        self["benchmark.log_file"] = os.path.abspath(self.results_filename)

    def run(self, id: str = None, runs: Optional[int] = 1, show_progress_bar: bool = True):
        if runs:
            self["benchmark.runs"] = runs
        else:
            runs = self["benchmark.runs"]
        ts = time.time()
        if not id:
            id = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H-%M-%S')
        self.set_id(id)
        log_filename = self.id + ".log"
        logfile = open(log_filename, 'w')
        self.save_settings(self.config_filename)
        print("Running MPB with ID %s (log file at %s)..." % (self.id, log_filename))
        total_iterations = len(self._planners) * len(self._steer_functions) * runs * (1 + len(self._smoothers))
        if show_progress_bar:
            pbar = tqdm_notebook(range(total_iterations), desc=self.id)
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
            if '<stats>' in line:
                if show_progress_bar:
                    pbar.update(1)
                # print('%s: %i / %i' % (id, iteration, total_iterations))
                iteration += 1
            logfile.write(line)
        if show_progress_bar:
            pbar.close()
        logfile.close()
        return tsk.poll()

    def visualize_trajectories(self, **kwargs):
        from trajectory import visualize
        visualize(self.results_filename, **kwargs)

    def plot_planner_stats(self, **kwargs):
        from plot_stats import plot_planner_stats
        plot_planner_stats(self.results_filename, **kwargs)

    def plot_smoother_stats(self, **kwargs):
        from plot_stats import plot_smoother_stats
        plot_smoother_stats(self.results_filename, **kwargs)


class MultipleMPB:
    def __init__(self):
        self.id = None  # type: Optional[str]
        self.benchmarks = []  # type: [MPB]

    def __getitem__(self, item: str, index: int = 0):
        return self.benchmarks[index][item]

    def __setitem__(self, item: str, value):
        for i in range(len(self.benchmarks)):
            self.benchmarks[i][item] = value

    @staticmethod
    def run_(arg):
        config_filename, index, id = arg
        mpb = MPB(config_filename)
        mpb_id = "%s_%i" % (id, index)
        code = mpb.run(id=mpb_id,
                       runs=None,
                       show_progress_bar=True)
        if code == 0:
            print("Benchmark %i (%s) finished successfully." % (index, mpb_id))
        else:
            print("Benchmark %i (%s) failed. Return code: %i." % (index, mpb_id, code), file=sys.stderr)
        return code

    def run_parallel(self, id: str = None, runs: int = 1, processes: int = os.cpu_count()):
        self["benchmark.runs"] = runs
        ts = time.time()
        if id:
            self.id = id
        else:
            self.id = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H-%M-%S')
        config_files = []
        log_files = []
        for i, mpb in enumerate(self.benchmarks):
            filename = "%s_%i_config.json" % (self.id, i)
            mpb.save_settings(filename)
            config_files.append(filename)
            log_files.append("%s_%i.log" % (self.id, i))
            mpb.set_id("%s_%i" % (self.id, i))
        print("Creating pool of %i processes." % processes)
        with Pool(processes) as pool:
            results = pool.map(MultipleMPB.run_,
                               zip(config_files,
                                   range(len(self.benchmarks)),
                                   [self.id] * len(self.benchmarks)))
            if all([r == 0 for r in results]):
                print("All benchmarks succeeded.")
            else:
                print("Error(s) occurred, not all benchmarks succeeded.", file=sys.stderr)
                for i, code in enumerate(results):
                    if code == 0:
                        continue
                    print("Benchmark %i failed with return code %i. See log file %s."
                          % (i, code, log_files[i]), file=sys.stderr)

    def visualize_trajectories(self, **kwargs):
        import matplotlib.pyplot as plt
        plt.figure(figsize=(6 * len(self.benchmarks), 6))
        for i, m in enumerate(self.benchmarks):
            plt.subplot(1, len(self.benchmarks), i + 1)
            m.visualize_trajectories(headless=True, combine_views=False,
                                     use_existing_subplot=True, show_legend=False,
                                     silence=True, **kwargs)
            plt.title("%s" % m.id)
        plt.tight_layout()

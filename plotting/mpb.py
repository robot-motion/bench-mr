import json
import subprocess
import datetime
import time
import itertools
import os
import sys
import resource
from typing import Optional

from utils import *
from multiprocessing import Pool
from tqdm import tqdm_notebook

MPB_BINARY = './benchmark'
MPB_BINARY_DIR = '../bin'

# limit memory by this fraction of available memory if activated for parallel MPB execution
MEMORY_LIMIT_FRACTION = min(0.9, 5. / os.cpu_count())


class MPB:
    def __init__(self,
                 config_file: str = os.path.join(MPB_BINARY_DIR, 'benchmark_template.json'),
                 output_path: str = ''):
        bin_path = os.path.abspath(os.path.join(MPB_BINARY_DIR, MPB_BINARY))
        if not os.path.exists(bin_path):
            raise Exception('Error: Could not find benchmark binary file at %s. ' % bin_path +
                            'Make sure you have built it and set the correct MPB_BINARY_DIR variable in %s.' % __file__)
        self.config = MPB.get_config(config_file)  # type: dict
        self.output_path = output_path  # type: str
        self.id = None  # type: Optional[str]
        self._planners = [planner for planner, used in self["benchmark.planning"].items() if used]  # type: [str]
        self._smoothers = [smoother for smoother, used in self["benchmark.smoothing"].items() if used]  # type: [str]
        self._steer_functions = [steer_functions[index] for index in self["benchmark.steer_functions"]]  # type: [str]

        # print("planners:       \t", self._planners)
        # print("smoothers:      \t", self._smoothers)
        # print("steer_functions:\t", self._steer_functions)
        # self.set_planners(['rrt'])
        # self.set_smoothers([])
        # self.set_steer_functions(['reeds_shepp'])
        self.config_filename = config_file  # type: Optional[str]
        self.results_filename = None  # type: Optional[str]

    def __getitem__(self, item: str) -> dict:
        c = self.config
        for s in item.split('.'):
            c = c[s]
        return c

    def __setitem__(self, item: str, value: object) -> object:
        c = self.config
        splits = item.split('.')
        for s in splits[:-1]:
            c = c[s]
        c[splits[-1]] = value
        return value

    def update(self, config: dict) -> dict:
        for key, value in config.items():
            self[key] = value
        return self.config

    @staticmethod
    def get_config(config_file: str = os.path.join(MPB_BINARY_DIR, 'benchmark_template.json')) -> dict:
        if not os.path.exists(config_file):
            raise Exception('Could not find configuration template file at %s. ' % os.path.abspath(
                os.path.join(MPB_BINARY_DIR, 'benchmark_template.json')) +
                            'Make sure you run the benchmark binary without CLI arguments to generate ' +
                            'benchmark_template.json.')
        with open(config_file, 'r') as f:
            config = json.load(f)["settings"]  # type: dict
        return config

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
                self._planners.append(p)
            else:
                self["benchmark.planning." + p] = False
        print("Selected planners:", self._planners)

    def set_steer_functions(self, steerings: [str]):
        self._steer_functions = list(set(itertools.chain.from_iterable(map(parse_steer_functions, steerings))))
        self["benchmark.steer_functions"] = self._steer_functions
        print("Selected steer functions:", [steer_functions[index] for index in self._steer_functions])

    def set_smoothers(self, smoothers: [str]):
        self._smoothers = []
        for p in self["benchmark.smoothing"].keys():
            if p in smoothers:
                self["benchmark.smoothing." + p] = True
                self._smoothers.append(p)
            else:
                self["benchmark.smoothing." + p] = False
        print("Selected smoothers:", self._smoothers)

    def save_settings(self, filename: str):
        with open(filename, 'w') as f:
            json.dump({
                "settings": self.config
            }, f, indent=2)

    def set_id(self, id: str):
        self.id = id

    def set_subfolder(self, subfolder: str = ''):
        self.config_filename = os.path.join(subfolder, self.id) + "_config.json"
        self.results_filename = os.path.join(subfolder, self.id) + "_results.json"
        self["benchmark.log_file"] = os.path.abspath(self.results_filename)

    @staticmethod
    def get_memory():
        # thanks to https://stackoverflow.com/a/41125461
        with open('/proc/meminfo', 'r') as mem:
            free_memory = 0
            for i in mem:
                sline = i.split()
                if str(sline[0]) in ('MemFree:', 'Buffers:', 'Cached:'):
                    free_memory += int(sline[1])
        return free_memory

    def run(self, id: str = None, runs: Optional[int] = 1, subfolder: str = '', show_progress_bar: bool = True) -> int:
        if runs:
            self["benchmark.runs"] = runs
        else:
            runs = self["benchmark.runs"]
        ts = time.time()
        if not id:
            id = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H-%M-%S')
        self.set_id(id)
        self.set_subfolder(subfolder)
        log_filename = os.path.join(subfolder, self.id + ".log")
        logfile = open(log_filename, 'w')
        self.save_settings(self.config_filename)
        print("Running MPB with ID %s (log file at %s)..." % (self.id, log_filename))
        total_iterations = len(self._planners) * len(self._steer_functions) * runs
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
                # some planner (and its smoothers) has finished
                if show_progress_bar:
                    pbar.update(1)
                # print('%s: %i / %i' % (id, iteration, total_iterations))
                iteration += 1
            logfile.write(line)
        if show_progress_bar:
            pbar.close()
        logfile.close()
        code = tsk.poll()
        tsk.terminate()
        return code

    def print_info(self):
        with open(self.results_filename, "r") as f:
            data = json.load(f)
            run_ids = list(range(len(data["runs"])))
            for run_id in run_ids:
                print_run_info(data, run_id, run_ids)

    def visualize_trajectories(self, **kwargs):
        from trajectory import visualize
        visualize(self.results_filename, **kwargs)

    def plot_planner_stats(self, **kwargs):
        from plot_stats import plot_planner_stats
        plot_planner_stats(self.results_filename, **kwargs)

    def plot_smoother_stats(self, **kwargs):
        from plot_stats import plot_smoother_stats
        plot_smoother_stats(self.results_filename, **kwargs)
        
    @staticmethod
    def merge(mpbs, target_filename: str, make_separate_runs: bool = False):
        """
        Merges results of the given MPB instances into one file.
        """
        target = None
        for i, m in enumerate(mpbs):
            with open(m.results_filename) as res_file:
                res = json.load(res_file)
                if i == 0:
                    target = res
                else:
                    # TODO check settings, environments are the same for each run before merging
                    for run_id, run in enumerate(res["runs"]):
                        if make_separate_runs:
                            target["runs"].append(run)
                            continue
                        if run_id >= len(target["runs"]):
                            print("Run #%i does not exist in %s but in %s. Skipping."
                                  % (run_id, mpbs[i-1].results_filename, m.results_filename), file=sys.stderr)
                        else:
                            for planner, plan in run["plans"].items():
                                if planner in target["runs"][run_id]["plans"]:
                                    print("Planner %s already exists in %s and in %s. Skipping."
                                          % (planner, mpbs[i-1].results_filename, m.results_filename), file=sys.stderr)
                                else:
                                    target["runs"][run_id]["plans"][planner] = plan
        
        with open(target_filename, "w") as target_file:
            json.dump(target, target_file, indent=2)
            print("Successfully merged [%s] into %s." % (", ".join(m.results_filename for m in mpbs), target_filename))


class MultipleMPB:
    def __init__(self):
        self.id = None  # type: Optional[str]
        self.benchmarks = []  # type: [MPB]
        self.subfolder = ''

    def __getitem__(self, item: str, index: int = 0) -> dict:
        return self.benchmarks[index][item]

    def __setitem__(self, item: str, value):
        for i in range(len(self.benchmarks)):
            self.benchmarks[i][item] = value

    def update(self, config: dict):
        for m in self.benchmarks:
            m.update(config)

    @staticmethod
    def run_(arg) -> int:
        config_filename, index, mpb_id, subfolder, memory_limit = arg
        if memory_limit != 0:
            resource.setrlimit(resource.RLIMIT_AS, memory_limit)
        mpb = MPB(config_file=config_filename)
        code = mpb.run(id=mpb_id,
                       runs=None,
                       subfolder=subfolder,
                       show_progress_bar=True)
        if code == 0:
            print("Benchmark %i (%s) finished successfully." % (index, mpb_id))
        else:
            print("Benchmark %i (%s) failed. Return code: %i." % (index, mpb_id, code), file=sys.stderr)
        return code

    def run_parallel(self,
                     id: str = None,
                     use_subfolder: bool = True,
                     runs: int = 1,
                     processes: int = os.cpu_count(),
                     limit_memory: bool = True) -> bool:
        memory_limit = 0
        if limit_memory:
            print("Available memory: %.2f GB, limiting each MPB process to %.1f%% usage." %
                  (MPB.get_memory() / 1e6, MEMORY_LIMIT_FRACTION * 100))
            soft, hard = resource.getrlimit(resource.RLIMIT_AS)
            memory_limit = (MPB.get_memory() * 1024 * MEMORY_LIMIT_FRACTION, hard)

        self["benchmark.runs"] = runs
        ts = time.time()
        if id:
            self.id = id
        else:
            self.id = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H-%M-%S')
        if use_subfolder:
            if not os.path.exists(self.id):
                os.mkdir(self.id)
            self.subfolder = self.id

        ids = []
        config_files = []
        log_files = []
        subfolder_id = os.path.join(self.subfolder, self.id)
        for i, mpb in enumerate(self.benchmarks):
            if mpb.id is None:
                filename = "%s_%i_config.json" % (subfolder_id, i)
                mpb.set_id("%s_%i" % (self.id, i))
                log_files.append("%s_%i.log" % (subfolder_id, i))
            else:
                filename = os.path.join(self.subfolder, mpb.id + "_config.json")
                log_files.append(os.path.join(self.subfolder, mpb.id + ".log"))
            mpb.save_settings(filename)
            config_files.append(filename)
            ids.append(mpb.id)
            mpb.set_subfolder(self.id if use_subfolder else '')
        print("Creating pool of %i processes." % processes)
        sys.stdout.flush()
        with Pool(processes) as pool:
            results = pool.map(MultipleMPB.run_,
                               zip(config_files,
                                   range(len(self.benchmarks)),
                                   ids,
                                   [self.subfolder] * len(self.benchmarks),
                                   [memory_limit] * len(self.benchmarks)))
            if all([r == 0 for r in results]):
                print("All benchmarks succeeded.")
            else:
                print("Error(s) occurred, not all benchmarks succeeded.", file=sys.stderr)
                for i, code in enumerate(results):
                    if code == 0:
                        continue
                    print("Benchmark %i failed with return code %i. See log file %s."
                          % (i, code, log_files[i]), file=sys.stderr)
                return False
        return True

    def visualize_trajectories(self, **kwargs):
        import matplotlib.pyplot as plt
        plt.figure(figsize=(6 * len(self.benchmarks), 6))
        for i, m in enumerate(self.benchmarks):
            plt.subplot(1, len(self.benchmarks), i + 1)
            if "headless" not in kwargs:
                kwargs["headless"] = True
            if "combine_views" not in kwargs:
                kwargs["combine_views"] = False
            if "use_existing_subplot" not in kwargs:
                kwargs["use_existing_subplot"] = True
            if "show_legend" not in kwargs:
                kwargs["show_legend"] = False
            if "silence" not in kwargs:
                kwargs["silence"] = True
            m.visualize_trajectories(**kwargs)
            plt.title("%s" % m.id)
        plt.tight_layout()
        
    def plot_planner_stats(self, **kwargs):
        import matplotlib.pyplot as plt
        for i, m in enumerate(self.benchmarks):            
            m.plot_planner_stats(**kwargs)
            plt.suptitle(m.id, fontsize=24, y=1.05)
            plt.tight_layout()

    def plot_smoother_stats(self, **kwargs):
        import matplotlib.pyplot as plt
        for i, m in enumerate(self.benchmarks):            
            m.plot_smoother_stats(**kwargs)
            plt.suptitle(m.id, fontsize=24, y=1.05)
            plt.tight_layout()

---
layout: default
title:  "Getting Started"
date:   2021-01-04 13:20:59 +0100
has_children: true
nav_order: 2
---

# Getting Started

This project contains several build targets in the `experiments/` folder.
The main application for benchmarking is the `benchmark` executable that gets built
in the `bin/` folder in the project directory.

## Running a benchmark

### Python Frontend (Jupyter Notebooks)

Run `jupyter lab` from the project folder and navigate to the `plotting/` directory where you can find several notebooks that can execute experiments and allow you to plot and analyze the benchmark results.

![gif]({{ site.baseurl }}/assets/mpb_parallel.gif)

[Python Frontend Tutorial](tutorial){: .btn .btn-blue }

### C++ Backend

{:.label .label-yellow .mt-4  }
⚠ **It is recommended to run the benchmarks from the Jupyter frontend.**

Alternatively, you have the option to manually run benchmarks via JSON configuration files that define which planners to execute, and many other settings concerning environments, steer functions, etc.

In the `bin/` folder, start a benchmark via
```bash
./benchmark configuration.json
```
where `configuration.json` is any of the `json` files in the `benchmarks/` folder.

Optionally, if multiple CPUs are available, multiple benchmarks can be run in parallel
using [GNU Parallel](https://www.gnu.org/software/parallel/), e.g., via
```bash
parallel -k ./benchmark ::: ../benchmarks/corridor_radius_*
```
This command will execute the experiments with varying corridor sizes in parallel.
For more information, consult the GNU Parallel [tutorial](https://www.gnu.org/software/parallel/parallel_tutorial.html).


This will eventually output a line similar to
```
Info:    Saved path statistics log file <...>
```

The resulting JSON log file can be used for visualizing the planning results and plotting
the statistics. To get started, check out the Jupyter notebooks inside the `plotting/` folder 
where all the plotting tools are provided.

[![CircleCI](https://circleci.com/gh/eric-heiden/mpb.svg?style=svg&circle-token=331e9015b5539b432e663cb7591ba92a38a338d9)](https://circleci.com/gh/eric-heiden/mpb)

# Motion Planning Benchmark
Benchmarking motion planners for wheeled mobile robots in cluttered environments on scenarios close to real-world autonomous driving scenarios.

## Dependencies
* [libccd-1.4](https://github.com/danfis/libccd/releases/tag/v1.4) (because of the `chomp` implementation used here), included as submodule and automatically built
* [OMPL](https://github.com/ompl/ompl) - included as submodule, needs to be installed first
* [Jupyter Lab](https://github.com/jupyterlab/jupyterlab) with [Python 3 kernel](https://ipython.readthedocs.io/en/latest/install/kernel_install.html#kernels-for-python-2-and-3) and [ipywidgets extension](https://ipywidgets.readthedocs.io/en/stable/user_install.html#installing-the-jupyterlab-extension) for plotting and evaluation (see `Benchmark.ipynb`)

# Getting started

1.  Check out the submodules
    ```bash
    git submodule init && git submodule update
    ```
    
2.  Create build and log folders
    ```bash
    mkdir build && mkdir build/log
    ```

3.  Build project
    ```bash
    cd build
    cmake ..
    cmake --build . -- -j4
    ```

4.  Run demo
    ```bash
    ./showcase
    ```
    This will output a line similar to
    ```
    Info:    Saved path statistics log file /home/$USER/...
    ```
    
5.  Visualize trajectories and statistics by navigating to the project base folder and running
    ```
    jupyter lab
    ```
    Open `Benchmark.ipynb` and run all notebook cells (`Run` > `Run all cells`). If you see some `Import` errors you're missing some Python dependencies.
    Make sure you use a **Python 3** Jupyter kernel with the `ipywidgets` extension for Jupyter Lab and install the missing dependencies via `pip install ...`.    
    
    Now copy the absolute path of the `json` file from step 4 into the "Logs" textarea and hit `Plot`.
    You should see the trajectories and violin plots of various path statistics.

## Third-party libraries
This project uses the following the packages:

* [nlohmann/json](https://github.com/nlohmann/json)
* [swatbotics/chomp-multigrid](https://github.com/swatbotics/chomp-multigrid) (make sure to have libccd-1.4 installed since newer versions are incompatible)

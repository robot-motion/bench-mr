[![CircleCI](https://circleci.com/gh/eric-heiden/mpb.svg?style=svg&circle-token=331e9015b5539b432e663cb7591ba92a38a338d9)](https://circleci.com/gh/eric-heiden/mpb)

# Motion Planning Benchmark
Benchmarking motion planners for wheeled mobile robots in cluttered environments on scenarios close to real-world autonomous driving settings.

## Dependencies
* [libccd-1.4+](https://github.com/danfis/libccd/releases/tag/v1.4) (because of the `chomp` implementation used here), included as submodule and automatically built
* [OMPL](https://github.com/ompl/ompl) - included as submodule, needs to be installed first
* [nlohmann/json](https://github.com/nlohmann/json) - not provided, needs to be installed first
* [SBPL](https://github.com/sbpl/sbpl) - not provided, needs to be installed
* [Jupyter Lab](https://github.com/jupyterlab/jupyterlab) with [Python 3 kernel](https://ipython.readthedocs.io/en/latest/install/kernel_install.html#kernels-for-python-2-and-3) for plotting and evaluation (see [plotting/README.md](plotting/README.md))

The following boost libraries (version 1.58+) need to be installed:
* `boost_serialization`
* `boost_filesystem`
* `boost_system`
* `boost_program_options`

The provided CHOMP implementation requires, GLUT and other OpenGL libraries to be present, which can be installed through the `freeglut3-dev` package. PNG via `libpng-dev`, expat via `libexpat1-dev`.

To support visual debugging, Qt5 with the `Charts` and `Svg` modules needs to be installed (optional dependency).

## Using Docker

1. Build the Docker image
    ```bash
    docker build -t mpb .
    ```

2. Run the image
    ```bash
    docker run -p 8888:8888 -it mpb
    ```
   Optionally, you can mount your local `mpb` copy to it respective folder inside the docker via
   ```bash
   docker run -p 8888:8888 -v $(pwd):/root/code/mpb -it mpb
   ```
   Now you can edit files from outside the docker and use docker to build and run the experiments.

   You can connect multiple times to this same running docker, for example if you want to access it from multiple shell instances via
   ```bash
   docker exec -it $(docker ps -qf "ancestor=mpb") bash
   ```
   Alternatively, run the provided script `./docker_connect.sh` that executes this command.

## Build instructions
1.  Check out the submodules
    ```bash
    git submodule init && git submodule update
    ```
    
2.  Create build and log folders
    ```bash
    mkdir build
    ```

3.  Build project
    ```bash
    cd build
    cmake ..
    cmake --build . -- -j4
    ```
    If you see an error during the `cmake ..` command that Qt or one of the Qt modules could
    not be found, you can ignore this message as this dependency is optional.

## Getting started
This project contains several build targets under the `experiments/` folder.
The main application for benchmarking is the `benchmark` executable that gets built
to the `bin/` folder in the project directory.

### Running a benchmark
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
the statistics. To get started, see the Jupyter notebooks inside the `plotting/` folder 
where all the provided plotting tools are showcased.

## Third-party libraries
This project uses forks from some of the following repositories which are integrated into this project as GIT submodules

* [The Open Motion Planning Library (OMPL)](http://ompl.kavrakilab.org/)
* [Search-Based Planning Library (SBPL)](https://github.com/sbpl/sbpl/)
* [swatbotics/chomp-multigrid](https://github.com/swatbotics/chomp-multigrid)
* [hbanzhaf/steering_functions](https://github.com/hbanzhaf/steering_functions)

Besides the above contributions, the authors thank Nathan Sturtevant's Moving AI Lab
for providing the [`2D Pathfinding "MovingAI" Datasets`](https://www.movingai.com/benchmarks/grids.html).

## License
TODO

## Developers
* Eric Heiden

## Contributors
* Luigi Palmieri
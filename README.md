# Smooth Theta*
Combining any-angle path planning and trajectory optimization

## Dependencies
* [libccd-1.4](https://github.com/danfis/libccd/releases/tag/v1.4) (because of the `chomp` implementation used here), included as submodule and automatically built
* [OMPL}(https://github.com/ompl/ompl) - included as submodule, needs to be installed first

## Third-party libraries
This project uses the following the packages:

* [nlohmann/json](https://github.com/nlohmann/json)
* [swatbotics/chomp-multigrid](https://github.com/swatbotics/chomp-multigrid) (make sure to have libccd-1.4 installed since newer versions are incompatible)

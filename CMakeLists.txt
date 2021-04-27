cmake_minimum_required(VERSION 3.0)

project(mpb)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG")

remove_definitions(-Wignored-attributes)

add_subdirectory(third_party/chomp)
include_directories(third_party/chomp)

add_subdirectory(third_party/steering_functions)
include_directories(third_party/steering_functions/include)

add_subdirectory(third_party/params)
include_directories(third_party/params/include)

# add the binary tree to the search path for include files
# so that we will find GripsConfig.h
include_directories("${PROJECT_BINARY_DIR}")


set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

find_package(Qt5 COMPONENTS Core Widgets Charts Svg)
include_directories(${CMAKE_BINARY_DIR})
set(CMAKE_AUTOMOC ON)

find_package(ompl 1.5 REQUIRED)
include_directories(${OMPL_INCLUDE_DIR})

find_package(Boost 1.58 REQUIRED COMPONENTS serialization filesystem system program_options)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

set(EXTRA_LIB ${EXTRA_LIB} ${SBPL_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES} chomp mzcommon steering_functions)

if (Qt5_FOUND)
    set(EXTRA_LIB ${EXTRA_LIB} Qt5::Core Qt5::Widgets Qt5::Charts Qt5::Svg)
    add_definitions(-DQT_SUPPORT=1)
endif (Qt5_FOUND)

option(USE_G1 "Use G1 clothoid steering (not Open Source)" OFF)
if (USE_G1)
    file(GLOB_RECURSE G1_SRC src/steer_functions/G1Clothoid/*.?pp)
    set(EXTRA_SRC ${EXTRA_SRC} ${G1_SRC})
    add_definitions(-DG1_AVAILABLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DG1_AVAILABLE")
    MESSAGE(STATUS "G1 clothoid steering is available.")
endif (USE_G1)

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/third_party)

file(GLOB_RECURSE Base src/base/*)
file(GLOB Utils src/utils/*)
file(GLOB_RECURSE Steer src/steer_functions/*)
file(GLOB_RECURSE ForwardPropagation src/fp_models/*)
file(GLOB Metrics src/metrics/*)
file(GLOB_RECURSE Planners src/planners/*)
file(GLOB_RECURSE Smoothers src/smoothers/*)

set(EXTRA_SRC ${EXTRA_SRC} third_party/chomp/chomp/Map2D.cpp)
if (Qt5_FOUND)
    file(GLOB GUI_SRC gui/*.?pp)
    set(EXTRA_SRC ${EXTRA_SRC} ${GUI_SRC})
else (Qt5_FOUND)
    set(EXTRA_SRC ${EXTRA_SRC})
endif (Qt5_FOUND)

add_library(mpb_core ${Base} ${Utils} ${Metrics} ${EXTRA_SRC})
target_link_libraries(mpb_core ${EXTRA_LIB})

add_library(mpb_steering ${Steer})
target_link_libraries(mpb_steering mpb_core ${EXTRA_LIB})

add_library(mpb_planners ${Planners})
target_link_libraries(mpb_planners mpb_core ${EXTRA_LIB})

add_library(mpb_smoothers ${Smoothers})
target_link_libraries(mpb_smoothers mpb_core ${EXTRA_LIB})

set(EXTRA_LIB ${EXTRA_LIB} mpb_core mpb_steering mpb_planners mpb_smoothers)

add_executable(benchmark experiments/benchmark.cpp)
target_link_libraries(benchmark ${EXTRA_LIB})

add_executable(benchmark_corridors experiments/benchmark_corridors.cpp)
target_link_libraries(benchmark_corridors ${EXTRA_LIB})

add_executable(parking experiments/parking.cpp)
target_link_libraries(parking ${EXTRA_LIB})

add_executable(smoothing experiments/smoothing.cpp)
target_link_libraries(smoothing ${EXTRA_LIB})

add_executable(anytime experiments/anytime.cpp)
target_link_libraries(anytime ${EXTRA_LIB})

add_executable(movingAi experiments/movingai_cities_scenarios.cpp)
target_link_libraries(movingAi ${EXTRA_LIB})

add_executable(compute_distances experiments/compute_distances.cpp)
target_link_libraries(compute_distances ${EXTRA_LIB})

add_executable(posq_testing experiments/posq_testing.cpp)
target_link_libraries(posq_testing ${EXTRA_LIB})

add_executable(sbpl_testing experiments/sbpl_testing.cpp)
target_link_libraries(sbpl_testing ${EXTRA_LIB})

add_executable(thetastar_testing experiments/thetastar_testing.cpp)
target_link_libraries(thetastar_testing ${EXTRA_LIB})

add_executable(show_polygon_mazes experiments/show_polygon_mazes.cpp)
target_link_libraries(show_polygon_mazes ${EXTRA_LIB})

add_executable(generate_corridor_radii_benchmarks experiments/generate_corridor_radii_benchmarks.cpp)
target_link_libraries(generate_corridor_radii_benchmarks ${EXTRA_LIB})

add_executable(generate_obstacle_ratio_benchmarks experiments/generate_obstacle_ratio_benchmarks.cpp)
target_link_libraries(generate_obstacle_ratio_benchmarks ${EXTRA_LIB})

add_executable(generate_turning_radii_benchmarks experiments/generate_turning_radii_benchmarks.cpp)
target_link_libraries(generate_turning_radii_benchmarks ${EXTRA_LIB})

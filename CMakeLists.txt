cmake_minimum_required(VERSION 3.0)

project(mpb)

set(CMAKE_CXX_STANDARD 11)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

#set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

add_subdirectory(chomp)
include_directories(chomp)

add_subdirectory(steering_functions)
include_directories(steering_functions/include)

add_subdirectory(params)
include_directories(params/include)

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

# set(Qt5Core_DIR "/home/eric/Qt/5.11.1/gcc_64/lib")
# set(Qt5Widgets_DIR "/home/eric/Qt/5.11.1/gcc_64/lib")
# set(Qt5Charts_DIR "/home/eric/Qt/5.11.1/gcc_64/lib")
# set(Qt5_DIR "/home/eric/Qt/5.11.1/gcc_64/lib")
# set(QT_QMAKE_EXECUTABLE "/home/eric/Qt/5.11.1/gcc_64/bin/qmake")
# set(CMAKE_PREFIX_PATH /home/eric/Qt/5.11.1/gcc_64;${CMAKE_PREFIX_PATH})

# find_package(Qt5 COMPONENTS Core Widgets Charts Svg)
# include_directories( ${CMAKE_BINARY_DIR} )
# set(CMAKE_AUTOMOC ON)

find_package(Boost 1.58 REQUIRED COMPONENTS serialization filesystem system program_options)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

set(EXTRA_LIB ${EXTRA_LIB} ${SBPL_LIBRARIES} ${Boost_LIBRARIES} chomp mzcommon -lompl steering_functions)

if (Qt5_FOUND)
    set(EXTRA_LIB ${EXTRA_LIB} Qt5::Core Qt5::Widgets Qt5::Charts Qt5::Svg)
    add_definitions(-DQT_SUPPORT=1)
endif (Qt5_FOUND)

option(USE_G1 "Use G1 clothoid steering (not Open Source)" OFF)
if (USE_G1)
    file(GLOB_RECURSE G1_SRC steer_functions/G1Clothoid/*.?pp)
    set(EXTRA_SRC ${EXTRA_SRC} ${G1_SRC})
    add_definitions(-DG1_AVAILABLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DG1_AVAILABLE")
    MESSAGE(STATUS "G1 clothoid steering is available.")
endif (USE_G1)

include_directories(${PROJECT_SOURCE_DIR})

file(GLOB Base base/*)
file(GLOB Utils utils/*)
file(GLOB_RECURSE Steer steer_functions/*)
file(GLOB Metrics metrics/*)
file(GLOB_RECURSE Planners planners/*)
file(GLOB_RECURSE Smoothers smoothers/*)

set(EXTRA_SRC ${EXTRA_SRC} chomp/chomp/Map2D.cpp)
if (Qt5_FOUND)
    file(GLOB GUI_SRC gui/*.?pp)
    set(EXTRA_SRC ${EXTRA_SRC} ${GUI_SRC})
else (Qt5_FOUND)
    set(EXTRA_SRC ${EXTRA_SRC})
endif (Qt5_FOUND)

#file(GLOB_RECURSE CHOMP_SRC chomp/*)
set(ALL_SRC ${Base} ${Utils} ${Steer} ${Metrics} ${Planners} ${Smoothers} ${EXTRA_SRC})

link_directories(base/)
link_directories(steer_functions/)

add_executable(benchmark experiments/benchmark.cpp ${ALL_SRC})
target_link_libraries(benchmark ${EXTRA_LIB})

add_executable(benchmark_corridors experiments/benchmark_corridors.cpp ${ALL_SRC})
target_link_libraries(benchmark_corridors ${EXTRA_LIB})

add_executable(parking experiments/parking.cpp ${ALL_SRC})
target_link_libraries(parking ${EXTRA_LIB})

add_executable(smoothing experiments/smoothing.cpp ${ALL_SRC})
target_link_libraries(smoothing ${EXTRA_LIB})

add_executable(movingAi experiments/movingai_cities_scenarios.cpp ${ALL_SRC})
target_link_libraries(movingAi ${EXTRA_LIB})

add_executable(compute_distances experiments/compute_distances.cpp ${ALL_SRC})
target_link_libraries(compute_distances ${EXTRA_LIB})

add_executable(posq_testing experiments/posq_testing.cpp ${ALL_SRC})
target_link_libraries(posq_testing ${EXTRA_LIB})

add_executable(show_polygon_mazes experiments/show_polygon_mazes.cpp ${ALL_SRC})
target_link_libraries(show_polygon_mazes ${EXTRA_LIB})

add_executable(generate_corridor_radii_benchmarks experiments/generate_corridor_radii_benchmarks.cpp ${ALL_SRC})
target_link_libraries(generate_corridor_radii_benchmarks ${EXTRA_LIB})

add_executable(generate_obstacle_ratio_benchmarks experiments/generate_obstacle_ratio_benchmarks.cpp ${ALL_SRC})
target_link_libraries(generate_obstacle_ratio_benchmarks ${EXTRA_LIB})

add_executable(generate_turning_radii_benchmarks experiments/generate_turning_radii_benchmarks.cpp ${ALL_SRC})
target_link_libraries(generate_turning_radii_benchmarks ${EXTRA_LIB})

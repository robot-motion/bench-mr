cmake_minimum_required(VERSION 3.0)

project(smooth_star)

set(CMAKE_CXX_STANDARD 11)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

#set(CMAKE_CXX_FLAGS "-Wall -Wextra")
#set(CMAKE_CXX_FLAGS_DEBUG "-g")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
#        steering_functions
        )

add_subdirectory(chomp)
include_directories(chomp)

# add the binary tree to the search path for include files
# so that we will find GripsConfig.h
include_directories("${PROJECT_BINARY_DIR}")

catkin_package()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

find_package(Qt5 REQUIRED COMPONENTS Core Widgets Charts Svg)
include_directories( ${CMAKE_BINARY_DIR} )
set(CMAKE_AUTOMOC ON)
file(GLOB GUI_SRC gui/*.?pp)
set(EXTRA_SRC ${EXTRA_SRC} ${GUI_SRC})
set(EXTRA_LIB ${EXTRA_LIB} Qt5::Core Qt5::Widgets Qt5::Charts Qt5::Svg ${SBPL_LIBRARIES} ${catkin_LIBRARIES} chomp mzcommon)

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
file(GLOB Planners planners/*)

#file(GLOB_RECURSE CHOMP_SRC chomp/*)
set(ALL_SRC ${Base} ${Utils} ${Steer} ${Metrics} ${Planners} ${EXTRA_SRC} PostSmoothing.cpp chomp/chomp/Map2D.cpp)

link_directories(base/)
link_directories(steer_functions/)

add_executable(homotopy_test mainHomotopy.cpp ${ALL_SRC})
add_executable(benchmark mainBenchmark.cpp ${ALL_SRC})
add_executable(shortening_test mainShortening.cpp ${ALL_SRC})
add_executable(showcase mainShowcase.cpp ${ALL_SRC})
add_executable(visual_compare mainVisualCompare.cpp ${ALL_SRC})
add_executable(grips_params mainGripsParameters.cpp ${ALL_SRC})

add_executable(sbpl_test mainSBPL.cpp)

target_link_libraries(benchmark -lompl ${EXTRA_LIB})
target_link_libraries(homotopy_test -lompl ${EXTRA_LIB})
target_link_libraries(shortening_test -lompl ${EXTRA_LIB})
target_link_libraries(showcase -lompl ${EXTRA_LIB})
target_link_libraries(visual_compare -lompl ${EXTRA_LIB})
target_link_libraries(grips_params -lompl ${EXTRA_LIB})

target_link_libraries(sbpl_test ${SBPL_LIBRARIES})
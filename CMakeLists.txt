cmake_minimum_required(VERSION 3.0)

project(mpb)

set(CMAKE_CXX_STANDARD 11)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

#set(CMAKE_CXX_FLAGS "-Wall -Wextra")
#set(CMAKE_CXX_FLAGS_DEBUG "-g")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_subdirectory(chomp)
include_directories(chomp)

add_subdirectory(steering_functions)
include_directories(steering_functions)

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

set(EXTRA_LIB ${EXTRA_LIB} ${SBPL_LIBRARIES} chomp mzcommon -lompl steering_functions)

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
file(GLOB Planners planners/*)

if (Qt5_FOUND)
    file(GLOB GUI_SRC gui/*.?pp)
    set(EXTRA_SRC ${EXTRA_SRC} ${GUI_SRC})
else (Qt5_FOUND)
    set(GUI_SRC gui/PathEvaluation.cpp)
    set(EXTRA_SRC ${EXTRA_SRC} ${GUI_SRC})
endif (Qt5_FOUND)

#file(GLOB_RECURSE CHOMP_SRC chomp/*)
set(ALL_SRC ${Base} ${Utils} ${Steer} ${Metrics} ${Planners} ${EXTRA_SRC} PostSmoothing.cpp chomp/chomp/Map2D.cpp)

link_directories(base/)
link_directories(steer_functions/)

if (Qt5_FOUND)
#    add_executable(homotopy_test mainHomotopy.cpp ${ALL_SRC})
#    target_link_libraries(homotopy_test ${EXTRA_LIB})

#    add_executable(benchmark mainBenchmark.cpp ${ALL_SRC})
#    target_link_libraries(benchmark ${EXTRA_LIB})

    add_executable(grips_params mainGripsParameters.cpp ${ALL_SRC})
    target_link_libraries(grips_params ${EXTRA_LIB})

    add_executable(shortening_test mainShortening.cpp ${ALL_SRC})
    target_link_libraries(shortening_test ${EXTRA_LIB})

    add_executable(visual_compare mainVisualCompare.cpp ${ALL_SRC})
    target_link_libraries(visual_compare ${EXTRA_LIB})
endif (Qt5_FOUND)

add_executable(showcase mainShowcase.cpp ${ALL_SRC})
target_link_libraries(showcase ${EXTRA_LIB})

# test compiling moving AI test
add_executable(movingAi mainMovingAiTest.cpp ${ALL_SRC})
target_link_libraries(movingAi ${EXTRA_LIB})
cmake_minimum_required(VERSION 2.8)
project(solver_demo)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -pthread")

# Eigen
set(EIGEN3_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/thirdparty/eigen)
include_directories(${EIGEN3_INCLUDE_DIRS})

# ceres
set(CERES_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/thirdparty/ceres-linux/include)
set(CERES_LIBRARIES ${CMAKE_SOURCE_DIR}/thirdparty/ceres-linux/lib/libceres.a)
include_directories(${CERES_INCLUDE_DIRS})

# g2o
set(G2O_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/thirdparty/g2o-linux/g2o)
set(G2O_CORE_LIBRARY ${CMAKE_SOURCE_DIR}/thirdparty/g2o-linux/lib/libg2o_core.so)
set(G2O_STUFF_LIBRARY ${CMAKE_SOURCE_DIR}/thirdparty/g2o-linux/lib/libg2o_stuff.a)
include_directories(${CERES_INCLUDE_DIRS})

# OpenCV (RNG)
find_package(OpenCV REQUIRED)

add_subdirectory(src)
add_subdirectory(test)

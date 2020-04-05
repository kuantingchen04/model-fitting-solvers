cmake_minimum_required(VERSION 2.8)
project(solver_demo)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -pthread")

# Eigen
#set(EIGEN_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/thirdparty/eigen)
#include_directories(${EIGEN_INCLUDE_DIR})

# ceres
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/thirdparty/ceres-linux/lib/cmake/Ceres)
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
MESSAGE(STATUS "CERES_FOUND: ${CERES_FOUND}")

# g2o
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/thirdparty/g2o-linux/cmake_modules)
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
MESSAGE(STATUS "G2O_FOUND: ${G2O_FOUND}")

# OpenCV (RNG)
#find_package(OpenCV REQUIRED)

add_subdirectory(src)
add_subdirectory(test)

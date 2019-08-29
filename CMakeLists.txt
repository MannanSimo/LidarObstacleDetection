cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

project(LidarObstacleDetection)

find_package(PCL 1.9.1 REQUIRED)

set(CMAKE_CXX_STANDARD 14)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable(environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp src/Node.cpp src/Node.h src/KDTree.cpp src/KDTree.h src/ClusterUtils.cpp src/ClusterUtils.h)
target_link_libraries (environment ${PCL_LIBRARIES})
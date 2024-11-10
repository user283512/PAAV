cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(cluster_extraction)

find_package(PCL 1.14 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cluster_extraction 
  src/cluster_extraction.cpp 
  src/Renderer.cpp 
  src/tree_utilities.cpp
)
target_link_libraries (cluster_extraction ${PCL_LIBRARIES})

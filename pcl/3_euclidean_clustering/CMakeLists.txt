cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(cluster_extraction)

# Imposta il linguaggio standard a C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.12 REQUIRED)

# Includi le directory di PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Aggiungi l'eseguibile e collega le librerie PCL
add_executable (cluster_extraction cluster_extraction.cpp)
target_link_libraries (cluster_extraction ${PCL_LIBRARIES})

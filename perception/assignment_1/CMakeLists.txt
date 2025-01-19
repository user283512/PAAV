cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

# Imposta il linguaggio standard a C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(cluster_extraction)

find_package(PCL 1.12 REQUIRED)

# Includi le directory di PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB SOURCES "src/*.cpp")
add_executable(cluster_extraction ${SOURCES})

# Aggiungi l'eseguibile e collega le librerie PCL
target_link_libraries (cluster_extraction ${PCL_LIBRARIES})
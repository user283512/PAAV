cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

# Imposta il linguaggio standard a C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tracker)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

#-------------------------------------------------------------------------------
# External Libraries
#-------------------------------------------------------------------------------
find_package(PCL 1.12 REQUIRED)
find_package(Eigen3 REQUIRED)

#-------------------------------------------------------------------------------
# Build libraries
#-------------------------------------------------------------------------------
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

file(GLOB_RECURSE tracker_SRC
  "src/*.cpp"
  "src/**/*.cpp"
)

set(tracker_LIBS ${PCL_LIBRARIES} pthread)
add_library(tracker SHARED ${tracker_SRC})
target_link_libraries(tracker ${tracker_LIBS})

#-------------------------------------------------------------------------------
# Build executables
#-------------------------------------------------------------------------------
add_executable(main demo/main.cpp)
target_link_libraries(main tracker)

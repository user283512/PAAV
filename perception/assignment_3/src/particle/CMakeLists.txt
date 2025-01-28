<<<<<<< HEAD
cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(particle)

## Compile as C++17, supported in ROS Kinetic and newer
=======
cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(particle)

>>>>>>> 8456d541ac3f9881ed8d31fdab3f931a2c14e038
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
)

find_package(PCL 1.8 REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS nav_msgs roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

file(GLOB_RECURSE tracker_SRC
  "src/*.cpp"
  "src/**/*.cpp"
)
add_executable(${PROJECT_NAME}_node ${tracker_SRC})

target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ceres gflags
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
 install(TARGETS ${PROJECT_NAME}_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
cmake_minimum_required(VERSION 2.8.3)
project(trab2)

find_package(
  catkin REQUIRED COMPONENTS 
  roscpp
  laser_geometry
  geometry_msgs 
  tf
#  sensor_msgs
#  std_msgs
#  robot_state_publisher
#  urdf
)

add_compile_options(-std=c++11)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
#    CATKIN DEPENDS
)

add_executable(mapeamento
  src/Perception.cpp 
  src/Utils.cpp 
  src/main.cpp
)

target_link_libraries(mapeamento
  ${catkin_LIBRARIES}
)


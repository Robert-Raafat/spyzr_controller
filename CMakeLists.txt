cmake_minimum_required(VERSION 3.0.2)
project(spyzr_controller)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  DEPENDS Eigen3
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}  
  ${EIGEN3_INCLUDE_DIR}
)

add_library(simple_controller src/simple_controller.cpp)
add_dependencies(simple_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_controller ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(simple_controller_node nodes/simple_controller_node.cpp)
add_dependencies(simple_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_controller_node ${catkin_LIBRARIES} simple_controller)

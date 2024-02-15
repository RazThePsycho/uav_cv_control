cmake_minimum_required(VERSION 2.8.3)
project(profi2022_bachelor_solution)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
  hector_uav_msgs
)

find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories (
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(simple_move_node src/simple_move.cpp)
add_dependencies(simple_move_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_move_node ${OpenCV_LIBS} ${catkin_LIBRARIES})

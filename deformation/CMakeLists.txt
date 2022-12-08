cmake_minimum_required(VERSION 3.0.2)
project(deformation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES deformation
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(opti_flow
    src/opti_flow.cpp
)

target_link_libraries(opti_flow ${catkin_LIBRARIES} ${OpenCV_LIBS})

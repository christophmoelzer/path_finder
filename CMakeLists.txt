cmake_minimum_required(VERSION 3.0.2)
project(path_finder)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  OpenCV REQUIRED
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_finder
#  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


 add_executable(${PROJECT_NAME} src/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


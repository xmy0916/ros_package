cmake_minimum_required(VERSION 2.8.3)
project(buptsmartcar_cpp)

set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev/)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  OpenCV
  roscpp
  cv_bridge
  image_transport
  trajectory_msgs
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(findLine src/findLine.cpp)
target_link_libraries(findLine 
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
)
add_dependencies(findLine findLine_tutorials_generate_messages_cpp)


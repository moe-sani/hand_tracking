# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(hft_state_publisher)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  /opt/ros/kinetic/include/tf2_geometry_msgs/
  include /opt/ros/kinetic/include/tf2/
  include ${PROJECT_SOURCE_DIR}/util/libkin/
  include ${PROJECT_SOURCE_DIR}/util/libmt/
include ${PROJECT_SOURCE_DIR}/util/
)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(hft_state_publisher_node src/hft_state_publisher_node.cpp)
target_link_libraries(hft_state_publisher_node ${catkin_LIBRARIES})
add_dependencies(hft_state_publisher_node hft_state_publisher_generate_messages_cpp)


# %EndTag(FULLTEXT)%

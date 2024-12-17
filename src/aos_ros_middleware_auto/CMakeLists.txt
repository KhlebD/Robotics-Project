cmake_minimum_required(VERSION 3.0.2)
project(aos_ros_middleware_auto)
 
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  genmsg 
)


  

 
 generate_messages(
   DEPENDENCIES
   std_msgs 
 )

 
catkin_package(
CATKIN_DEPENDS message_runtime
)

include_directories( 
# include
  ${catkin_INCLUDE_DIRS}
)
 
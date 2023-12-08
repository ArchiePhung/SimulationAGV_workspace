cmake_minimum_required(VERSION 2.8.3)
project(sti_module)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  # fiducial_msgs
  #apriltag_ros
  std_msgs
  roscpp
  rospy 
  tf
  tf2_ros
  dynamic_reconfigure
  sti_msgs
)

catkin_package(

)

# add_service_files(FILES AddTwoInts.srv)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)




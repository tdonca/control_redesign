cmake_minimum_required(VERSION 2.8.3)
project(control_redesign)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  
  osrf_gear
  roscpp
  roslaunch
  actionlib
  message_generation
  std_msgs
  std_srvs
  sensor_msgs
  trajectory_msgs
  geometry_msgs
  geometric_shapes
  actionlib_msgs
    
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Find Launch Files
roslaunch_add_file_check(launch)



################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  #~ Location.msg
  #~ State.msg
  #~ Part.msg
  #~ Shipment.msg
  #~ TransitionAction.msg
  #~ GripperAction.msg
)


add_service_files(
  FILES
  #~ Belt.srv
  #~ CurrentRobotState.srv
  #~ UpdateRobotState.srv
  #~ PathBetweenStates.srv
  #~ FindParts.srv
  #~ EEState.srv
  #~ UpdatePartsAvailability.srv
  #~ UpdatePart.srv
  #~ CurrentPartsStates.srv
  #~ HeldPartPose.srv
  #~ ActiveBoxState.srv
)


add_action_files(
  DIRECTORY action
  FILES
  #~ FillOrder.action
  #~ ExecuteTransition.action
  #~ ExecuteGripper.action
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################


catkin_package(
   CATKIN_DEPENDS moveit_core moveit_ros_planning_interface message_runtime actionlib_msgs
   DEPENDS EIGEN3
)



###########
## Build ##
###########

include_directories(
 include
 ${CMAKE_SOURCE_DIR}/include
 ${catkin_INCLUDE_DIRS} 
 ${Eigen_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})



######################
## Node Executables ##
######################

### Test Node ###

# Add executable file
add_executable( test_server src/world/test_server.cpp )
add_dependencies( test_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( test_server ${catkin_LIBRARIES} )
# Link other class files
target_sources( test_server PUBLIC src/world/Part.cpp  )










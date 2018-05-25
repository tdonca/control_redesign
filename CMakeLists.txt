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
	Part.msg
	  #~ Location.msg
	  #~ State.msg
	  #~ Part.msg
	  #~ Shipment.msg
	  #~ TransitionAction.msg
	  #~ GripperAction.msg
)


add_service_files(
  FILES
	FindPartType.srv
	MarkPartUsed.srv
	ReleasePart.srv
	GetGripperPart.srv
	GetBoxParts.srv
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
	FillShipment.action
	  #~ ExecuteTransition.action
	  #~ ExecuteGripper.action
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
  osrf_gear
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

### World State Node ###

# Add executable file
add_executable( world_server src/world/world_server.cpp )
add_dependencies( world_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( world_server ${catkin_LIBRARIES} )
# Link other class files
target_sources( world_server 
	PUBLIC 
		src/world/WorldState.cpp 
		src/world/Part.cpp 
		src/world/Bin.cpp 
		src/world/Box.cpp 
		src/world/Gripper.cpp 
		src/world/LogicalCameraSensor.cpp 
		src/world/QualityControlSensor.cpp
		src/world/IIWA14Robot.cpp 
		src/world/StateGraph.cpp	
)



### Client Test Node ###

# Add executable file
add_executable( client_test src/client/client_test.cpp )
add_dependencies( client_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( client_test ${catkin_LIBRARIES} )
# Link other class files
target_sources( client_test 
	PUBLIC 
		src/client/WorldStateClient.cpp	
)




### Order Scheduler Node ###

# Add executable file
add_executable( order_server src/order/order_server.cpp )
add_dependencies( order_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( order_server ${catkin_LIBRARIES} )
# Link other class files
target_sources( order_server 
	PUBLIC 
		src/order/OrderScheduler.cpp
)



### Task Planner Node ###

# Add executable file
add_executable( task_server src/task/task_server.cpp )
add_dependencies( task_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( task_server ${catkin_LIBRARIES} )
# Link other class files
target_sources( task_server 
	PUBLIC 
		src/client/WorldStateClient.cpp
		
		src/task/TaskPlanner.cpp
		src/task/TaskManager.cpp
)



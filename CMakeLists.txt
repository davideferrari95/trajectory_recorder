cmake_minimum_required(VERSION 3.0.2)
project(trajectory_recorder)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  trajectory_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  parameter_msg.msg
)

add_service_files(
   FILES
   String.srv
   Load_Trajectory.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  ${PROJECT_NAME}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Executables 

add_executable(trajectory_recorder_Node 
	src/trajectory_recorder/trajectory_recorder_Node.cpp 
	src/trajectory_recorder/trajectory_recorder.cpp)
target_link_libraries(trajectory_recorder_Node ${catkin_LIBRARIES})
add_dependencies(trajectory_recorder_Node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

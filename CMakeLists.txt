cmake_minimum_required(VERSION 3.0.2)
project(multimap_nav)

add_compile_options(-std=c++17)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib_msgs
  actionlib
  message_generation
  std_msgs
  geometry_msgs
  move_base_msgs
  nav_msgs
  std_srvs
  genmsg
  roslib
)
find_package(SQLite3 REQUIRED)

# Generate actions 
add_action_files(
  DIRECTORY action 
  FILES SendGoal.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
  nav_msgs
)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib_msgs std_msgs message_runtime geometry_msgs std_srvs nav_msgs roslib
)

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)


install(
  DIRECTORY data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Add executables for the server and client
add_executable(multi_map_goal_server src/MultiMapGoalActionServer.cpp)
target_link_libraries(multi_map_goal_server ${catkin_LIBRARIES})
target_link_libraries(multi_map_goal_server ${SQLite3_LIBRARIES})

add_executable(multi_map_goal_client src/MultiMapGoalActionClient.cpp)
target_link_libraries(multi_map_goal_client ${catkin_LIBRARIES})

add_dependencies(multi_map_goal_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(multi_map_goal_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

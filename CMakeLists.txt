cmake_minimum_required(VERSION 2.8.3)
project(ur5_allegro_moveit)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    geometry_msgs
    kdl_control_tools
    tf2_geometry_msgs
    tf2_ros
    allegro_hand_kdl
)

# For YouCompleteMe
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    geometry_msgs
    kdl_control_tools
    tf2_ros
    tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(merged_joint_states
  src/js_merger_node.cpp
)

target_link_libraries(merged_joint_states
  ${catkin_LIBRARIES}
)


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


include_directories( ${catkin_INCLUDE_DIRS} )

add_library(${PROJECT_NAME}
  src/goal.cpp
)

add_executable(goal src/goal.cpp)
add_executable(closegrip src/closegrip.cpp)
add_executable(dummyforce src/dummyforce.cpp)

add_dependencies(goal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(closegrip ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(dummyforce ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(goal ${catkin_LIBRARIES} )
target_link_libraries(closegrip ${catkin_LIBRARIES} )
target_link_libraries(dummyforce ${catkin_LIBRARIES} )

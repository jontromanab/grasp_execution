cmake_minimum_required(VERSION 2.8.3)
project(grasp_execution)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  message_generation
  roscpp
  std_msgs
  actionlib_msgs
  visualization_msgs
  geometry_msgs
  rospy
  moveit_ros_planning_interface
  moveit_core
  pluginlib
  cmake_modules
  interactive_markers
  tf2
)

find_package (actionlib_msgs)

find_package(Boost REQUIRED COMPONENTS system)


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   grasp.msg

 )

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES ExecuteGrasp.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   actionlib_msgs
 )




catkin_package(
INCLUDE_DIRS
   include
LIBRARIES
   grasp_execution
   create_gripper_marker

CATKIN_DEPENDS  message_runtime actionlib_msgs roscpp
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ library

add_library(create_gripper_marker src/create_gripper_marker.cpp)
target_link_libraries(create_gripper_marker  ${catkin_LIBRARIES})

add_library(grasp_execution src/grasp_execution.cpp)
target_link_libraries(grasp_execution create_gripper_marker ${catkin_LIBRARIES})





#install(DIRECTORY include/${PROJECT_NAME}/
 #DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS grasp_execution create_gripper_marker
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_grasp_execution.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


add_executable(grasp_execution_server src/grasp_execution_server.cpp)
target_link_libraries(grasp_execution_server  ${catkin_LIBRARIES} grasp_execution)
add_dependencies( grasp_execution_server  grasp_execution ${grasp_execution_EXPORTED_TARGETS})

add_executable(grasp_execution_client src/grasp_execution_client.cpp)
target_link_libraries(grasp_execution_client ${catkin_LIBRARIES})
add_dependencies(grasp_execution_client ${grasp_execution_EXPORTED_TARGETS})



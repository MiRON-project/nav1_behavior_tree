cmake_minimum_required(VERSION 3.10)
project(nav1_behavior_tree)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib  
  actionlib_msgs
  behaviortree_cpp_v3
  message_generation
  move_base
  pluginlib
  roscpp
  std_msgs
  tf
  xmlrpcpp
)

find_package(Boost REQUIRED)

add_message_files(
  FILES
    BtState.msg
)

add_service_files(
  FILES
    LoadTree.srv
)

generate_messages(
  DEPENDENCIES 
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    actionlib  
    actionlib_msgs
    behaviortree_cpp_v3
    message_generation
    move_base
    pluginlib
    roscpp
    std_msgs
    tf
    xmlrpcpp
  DEPENDS
    Boost
)

#############
##  Build  ##
#############
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
  src/actions/move_base_planner.cc
  src/executor_bt.cc
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
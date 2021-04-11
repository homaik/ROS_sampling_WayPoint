cmake_minimum_required(VERSION 3.0.2)
project(HM_Rviz_path_planning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  tf
  visualization_msgs
  std_msgs
  actionlib
  move_base_msgs
  std_srvs
  nav_msgs
  actionlib_msgs
  #my_robot_msgs
)

add_action_files(
  FILES
  sampling.action
)




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES HM_Rviz_path_planning
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp tf
#  DEPENDS system_lib
#  CATKIN_DEPENDS my_robot_msgs
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


ADD_EXECUTABLE(test4_node 
src/test4.cpp

#include/HM_Rviz_path_planning/HM_visualization.h
include/HM_Rviz_path_planning/HM_visualization.h

#include/HM_Rviz_path_planning/HM_MapRviz.h
include/HM_Rviz_path_planning/HM_MapRviz.h

)


ADD_EXECUTABLE(received_plan 
src/received_plan.cpp

#include/HM_Rviz_path_planning/HM_visualization.h
include/HM_Rviz_path_planning/HM_visualization.h

#include/HM_WayPoint/HM_waypoint_server.h
include/HM_WayPoint/HM_waypoint_server.h


)

ADD_EXECUTABLE(sampling_server 
src/samplingService.cpp

#include/HM_Rviz_path_planning/HM_visualization.h
include/HM_Rviz_path_planning/HM_visualization.h

#include/HM_WayPoint/HM_waypoint_server.h
include/HM_WayPoint/HM_waypoint_server.h

)

ADD_EXECUTABLE(samplingServiceClient 
src/samplingServiceClient.cpp

#include/HM_Rviz_path_planning/HM_visualization.h
include/HM_Rviz_path_planning/HM_visualization.h

#include/HM_WayPoint/HM_waypoint_server.h
include/HM_WayPoint/HM_waypoint_server.h

)

ADD_EXECUTABLE(samplingAction 
src/samplingAction.cpp

#include/HM_Rviz_path_planning/HM_visualization.h
include/HM_Rviz_path_planning/HM_visualization.h

#include/HM_WayPoint/HM_waypoint_server.h
include/HM_WayPoint/HM_waypoint_server.h

)


ADD_EXECUTABLE(samplingAction_client 
src/samplingAction_client.cpp

#include/HM_Rviz_path_planning/HM_visualization.h
include/HM_Rviz_path_planning/HM_visualization.h

#include/HM_WayPoint/HM_waypoint_server.h
include/HM_WayPoint/HM_waypoint_server.h

)


TARGET_LINK_LIBRARIES(test4_node
    ${catkin_LIBRARIES}
)

TARGET_LINK_LIBRARIES(received_plan
    ${catkin_LIBRARIES}
)

TARGET_LINK_LIBRARIES(sampling_server
    ${catkin_LIBRARIES}
)

TARGET_LINK_LIBRARIES(samplingServiceClient
    ${catkin_LIBRARIES}
)


TARGET_LINK_LIBRARIES(samplingAction
    ${catkin_LIBRARIES}
)

TARGET_LINK_LIBRARIES(samplingAction_client
    ${catkin_LIBRARIES}
)
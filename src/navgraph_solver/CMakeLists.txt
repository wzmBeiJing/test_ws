cmake_minimum_required(VERSION 2.8.3)
project(navgraph_solver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  costmap_2d
  geometry_msgs
  actionlib_msgs
  nav_core
  nav_msgs
  pluginlib
  tf
  global_planner
  visualization_msgs
  interactive_markers
  message_generation
  move_base
  rviz
)

add_service_files(
  FILES
  waypoint.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES navgraph_solver
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    costmap_2d
    geometry_msgs
    actionlib_msgs
    nav_core
    nav_msgs
    pluginlib
    tf
    #global_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

set(SRC_FILES
  src/waypoint_panel.cpp
)
set(HEADER_FILES
  include/navgraph_solver/waypoint_panel.h
)
add_library(navgraph_rviz_panel ${SRC_FILES} ${HEADER_FILES})
# add_dependencies(navgraph_rviz_panel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navgraph_rviz_panel ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(station_navigator src/station_navigator.cpp)
add_dependencies(station_navigator navgraph_solver_generate_messages_cpp)
target_link_libraries(station_navigator ${catkin_LIBRARIES})

#add_executable(nav_move_base src/nav_move_base.cpp)
#add_dependencies(nav_move_base navgraph_solver_generate_messages_cpp)
#target_link_libraries(nav_move_base ${catkin_LIBRARIES} move_base)

add_library(navgraph_global_planner src/navgraph_global_planner.cpp)
add_dependencies(navgraph_global_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navgraph_global_planner ${catkin_LIBRARIES} global_planner my_astar)

add_library(my_astar src/my_astar.cpp)
add_dependencies(my_astar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(my_astar ${catkin_LIBRARIES})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS navgraph_global_planner my_astar navgraph_rviz_panel
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES bgp_plugin.xml rviz_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

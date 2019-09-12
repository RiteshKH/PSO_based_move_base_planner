cmake_minimum_required(VERSION 2.8.3)
project(pso_ros)


find_package(catkin REQUIRED COMPONENTS
  nav_core
  roscpp
  rospy
  std_msgs
  angles
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  navfn
  nav_msgs
  pluginlib
  tf2_geometry_msgs
  tf2_ros
)

generate_dynamic_reconfigure_options(
  src/cfg/PsoPotPlannerROS.cfg
)


#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pso_ros
#  CATKIN_DEPENDS nav_core roscpp rospy std_msgs
#  DEPENDS system_lib

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf2_ros
    rospy
    std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(
# include
#  ${catkin_INCLUDE_DIRS}
#)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_library(pso_ros_lib 
#           src/particle_so_ros.cpp 
#            include/pso_ros_include/PSO_depend.cpp 
#            include/pso_ros_include/pso_set.cpp)

add_library(${PROJECT_NAME} 
            src/particle_potential.cpp
            src/global_planner/astar.cpp
            src/global_planner/grid_path.cpp
            src/global_planner/orientation_filter.cpp
            src/global_planner/dijkstra.cpp
            include/pso_ros_include/PSO_depend.cpp 
            include/pso_ros_include/pso_set.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(pso_planner
  src/global_planner/plan_node.cpp
)

add_dependencies(pso_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pso_planner
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# target_link_libraries(plan_node
#  ${catkin_LIBRARIES}
# )
# add_dependencies(plan_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME} pso_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

install(FILES pso_pot_ros_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
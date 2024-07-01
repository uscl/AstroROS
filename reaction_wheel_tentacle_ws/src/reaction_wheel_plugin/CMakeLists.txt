cmake_minimum_required(VERSION 3.0.2)
project(reaction_wheel_plugin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
  geometry_msgs
  gazebo_plugins
)

find_package(Boost REQUIRED COMPONENTS system)
find_package (Eigen3 3.3 REQUIRED)
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
##++++++++++++++++++++++++++++++##
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )
##++++++++++++++++++++++++++++++##
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

catkin_package(
  DEPENDS
  roscpp
  gazebo_ros
  std_msgs
  geometry_msgs
)
##+++++++++++++++++++++++++++++##
add_library(reaction_wheel_plugin SHARED src/reaction_wheel_plugin.cpp)
##+++++++++++++++++++++++++++++##

add_dependencies(reaction_wheel_plugin ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(TARGETS reaction_wheel_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES
reaction_wheel_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
##+++++++++++++++++++++++++++++##
target_link_libraries(reaction_wheel_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
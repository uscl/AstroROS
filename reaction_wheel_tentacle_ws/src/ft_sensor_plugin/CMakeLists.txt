cmake_minimum_required(VERSION 2.8.3)
project(ft_sensor_plugin)

# Set policy for CMake 3.1+. Use OLD policy to let FindBoost.cmake, dependency
# of gazebo, use quoted variables in if()

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  gazebo_dev 
  gazebo_plugins
  geometry_msgs 
  std_msgs
  xacro)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp
    gazebo_dev 
    gazebo_plugins 
    geometry_msgs
    std_msgs
)

# Plugins require c++11
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)
link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_library(ft_sensor_plugin
  src/ft_sensor_plugin.cpp
)
target_link_libraries(ft_sensor_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
install(TARGETS ft_sensor_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  ft_sensor_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
cmake_minimum_required(VERSION 2.8.3)
project(monstertruck_gazebo_plugins)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  monstertruck_msgs
  roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

# Find gazebo 
include (FindPkgConfig) 
if (PKG_CONFIG_FOUND) 
  pkg_check_modules(GAZEBO gazebo) 
endif() 

include_directories(${GAZEBO_INCLUDE_DIRS}) 

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES monstertruck_gazebo_plugins
  CATKIN_DEPENDS gazebo_plugins gazebo_ros monstertruck_msgs roscpp
  DEPENDS gazebo
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

#rosbuild_link_boost(all_wheel_steering_plugin thread)

add_library(all_wheel_steering_plugin src/all_wheel_steering_plugin.cpp)
add_dependencies(all_wheel_steering_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(all_wheel_steering_plugin ${Boost_LIBRARIES} ${GAZEBO_LIBRARY_DIRS} ${catkin_LIBRARIES})



install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS
  all_wheel_steering_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

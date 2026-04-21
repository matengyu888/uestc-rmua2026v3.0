cmake_minimum_required(VERSION 3.0.2)
project(planner)
set(CMAKE_CXX_STANDARD 17)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)
set(pcl_conversions_DIR /usr/local/share/pcl_conversions/cmake)
set(pcl_ros_DIR /usr/local/share/pcl_ros/cmake)
find_package(catkin REQUIRED COMPONENTS
roslib
airsim_ros
message_generation
nav_msgs
# nodelet
roscpp
rospy
sensor_msgs
std_msgs
geographic_msgs
geometry_msgs
std_srvs
tf2
tf2_ros
tf2_sensor_msgs
tf2_geometry_msgs
image_transport
cv_bridge
pcl_conversions
pcl_ros
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(path_follower src/path_follower.cpp)
add_dependencies(path_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_follower
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(hybrid_route_manager_node src/hybrid_route_manager_node.cpp)
add_dependencies(hybrid_route_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hybrid_route_manager_node
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(stereo_depth_node src/stereo_depth_node.cpp)
add_dependencies(stereo_depth_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stereo_depth_node
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(DIRECTORY launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/dp_planner_runner.sh
  scripts/dp_planner_node.py
  scripts/local_goal_publisher.py
  scripts/model.py
  scripts/planner_switcher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

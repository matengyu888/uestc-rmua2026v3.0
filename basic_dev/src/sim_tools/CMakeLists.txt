cmake_minimum_required(VERSION 3.0.2)
project(sim_tools)
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

catkin_install_python(PROGRAMS
  scripts/path_loader.py
  scripts/vel_control.py
  scripts/recode_path.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# add_executable(path_follower src/path_follower.cpp)
# add_dependencies(path_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(path_follower
#   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


# install(DIRECTORY config launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
# add_executable(pos_controller src/pos_controller.cpp)
# add_dependencies(pos_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(pos_controller
#   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

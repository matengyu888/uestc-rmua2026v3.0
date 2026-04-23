cmake_minimum_required(VERSION 3.0.2)
project(basic_dev)

# 设置 OpenCV 路径（如果系统安装位置非标准）
set(OpenCV_DIR /usr/local/lib/cmake/opencv4)

# 查找 catkin 包（不要包含 OpenCV）
find_package(catkin REQUIRED COMPONENTS
  airsim_ros
  message_generation
  nav_msgs
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
  pcl_ros
  pcl_conversions
)

# 单独查找 OpenCV
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}   # 注意变量名正确
)

# add_executable(basic_dev src/basic_dev.cpp)
# target_link_libraries(basic_dev
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}           # 使用 OpenCV_LIBS（注意大小写）
# )

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

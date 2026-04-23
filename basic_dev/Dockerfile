FROM osrf/ros:noetic-desktop-full-focal

RUN rm -rf /basic_dev/src /basic_dev/build /basic_dev/devel

ADD src /basic_dev/src/
ADD setup.bash /

RUN chmod +x /setup.bash

USER root

RUN apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    python3-catkin-tools \
    python3-numpy \
    python3-pip \
    ros-noetic-geographic-msgs \
    ros-noetic-image-transport \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-tf2-sensor-msgs \
    net-tools \
 && python3 -m pip install --no-cache-dir \
    'https://mirrors.aliyun.com/pytorch-wheels/cu113/torch-1.12.1%2Bcu113-cp38-cp38-linux_x86_64.whl' \
 && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=noetic
ENV PYTHONUNBUFFERED=1
ENV DP_PLANNER_PYTHON=/usr/bin/python3

WORKDIR /basic_dev/
RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash \
    && catkin_make --only-pkg-with-deps airsim_ros \
    && source devel/setup.sh \
    && catkin_make --only-pkg-with-deps quadrotor_msgs \
    && source devel/setup.sh \
    && catkin_make --only-pkg-with-deps sim_tools \
    && source devel/setup.sh \
    && catkin_make --only-pkg-with-deps controller \
    && source devel/setup.sh \
    && catkin_make --only-pkg-with-deps planner"

ENTRYPOINT [ "/setup.bash" ]

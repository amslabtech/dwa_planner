FROM ros:melodic-ros-base

RUN apt update

RUN apt install -y ros-melodic-tf* \
                   python-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root

# ROS setting
RUN /bin/bash -c "mkdir -p catkin_ws/src"

ENV ROS_PACKAGE_PATH=/root/catkin_ws:$ROS_PACKAGE_PATH

ENV ROS_WORKSPACE=/root/catkin_ws

WORKDIR /root

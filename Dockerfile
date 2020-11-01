ARG ros_distro=melodic
FROM ros:${ros_distro}-ros-base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

# ROS setting
ARG ros_distro
ENV ROS_DISTRO=${ros_distro}
RUN mkdir -p catkin_ws/src

RUN cd catkin_ws/src && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_init_workspace

COPY . /root/catkin_ws/src/repo

RUN cd /root/catkin_ws && \
    apt-get update && \
    rosdep update && \
    rosdep install -i -r -y --from-paths src && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release

RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash && source /root/catkin_ws/devel/setup.bash && exec "$@"' \
    > /root/ros_entrypoint.sh

WORKDIR /root/catkin_ws

ENTRYPOINT ["bash", "/root/ros_entrypoint.sh"]

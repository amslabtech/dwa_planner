ARG ROS_DISTRO=noetic
FROM ros:${ROS_DISTRO}
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN ln -sf /usr/share/zoneinfo/Asia/Tokyo /etc/localtime

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && apt-get update \
    && apt-get install -y sudo git vim \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}
RUN apt-get update && apt-get upgrade -y
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    python3-catkin-tools \
    python3-vcstool \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs
RUN rm -rf /var/lib/apt/lists/*
RUN rm /etc/apt/apt.conf.d/docker-clean

# install and build
USER ${USERNAME}
RUN mkdir -p ~/ws/src
WORKDIR /home/${USERNAME}/ws/src
RUN git clone https://github.com/amslabtech/dwa_planner.git \
    && git clone -b ${ROS_DISTRO}-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git \
    && git clone -b ${ROS_DISTRO}-devel https://github.com/ROBOTIS-GIT/turtlebot3.git \
    && git clone -b ${ROS_DISTRO}-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
WORKDIR /home/${USERNAME}/ws
RUN sudo apt-get update && rosdep update && rosdep install -riy --from-paths src --rosdistro ${ROS_DISTRO}
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin build -DCMAKE_BUILD_TYPE=Release"

CMD ["/bin/bash"]

FROM osrf/ros:humble-desktop AS base

RUN apt-get update && apt-get install -y \
    locales \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    x11-apps \
    ros-humble-rclcpp \
    ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

RUN rosdep init || true
RUN rosdep update

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
    

FROM base AS dev

RUN apt-get update && apt-get install -y \
    ros-humble-std-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /workspace

CMD ["/bin/bash"]

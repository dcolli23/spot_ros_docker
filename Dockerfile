FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND noninteractive
ENV TERM xterm

# Going off of https://wilselby.com/2019/05/os-1-ros-package-deployment-with-docker/
# Seems to be pretty good.

# Dylan added qttools5-dev to get past build issue with the spot driver.
# Dylan added python3-pip to install boston dynamics libraries.
RUN apt-get update \
  && apt-get dist-upgrade -y \
  && apt-get install -y --no-install-recommends \
    git \
    build-essential \
    dialog \
    make \
    gcc \
    g++ \
    locales \
    wget \
    software-properties-common \
    sudo \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libxau6 \
    libxdmcp6 \
    libxcb1 \
    libxext6 \
    libx11-6 \
    tmux \
    xdg-utils \
    eog \
    qttools5-dev \
    python3-pip \
 && rm -rf /var/lib/apt/lists/*

ADD /scripts /scripts

RUN bash /scripts/install_scripts_for_docker.bash

# Trying to separate install scripts to take advantage of caching.
# NOTE: This unfortunately did not help with caching the image between bash script runs.
# RUN bash /scripts/create_catkin_workspace.bash
# RUN bash /scripts/install_ros_dependencies.bash
# RUN bash /scripts/install_spot_ros_driver.bash


# CMD ["roslaunch", "spot_driver", "driver.launch"]
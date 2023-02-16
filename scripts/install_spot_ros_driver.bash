#!/bin/bash
set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# get UBUNTU_CODENAME, ROS_DISTRO, REPO_DIR, CATKIN_DIR
source $SCRIPT_DIR/identify_environment.bash

# Install the boston dynamics required packages.
python3 -m pip install --upgrade bosdyn-client==3.2.0 bosdyn-mission==3.2.0 bosdyn-choreography-client==3.2.0

if [ ! -d "$HOME/catkin_ws/src/spot_ros" ]; then
    echo "spot_ros repository not detected"
    cd "$HOME/catkin_ws/src"
    git clone https://github.com/heuristicus/spot_ros.git
    cd "$HOME/catkin_ws"
    rosdep install --from-paths src --ignore-src -r -y
    catkin build --no-status
    echo "Package built successfully"
else
    echo "ouster_example already installed"
fi


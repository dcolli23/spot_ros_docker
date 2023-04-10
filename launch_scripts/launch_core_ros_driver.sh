#!/bin/bash

# Starts the ROS driver on the CORE.
# NOTE: This should only be executed after SSHing into the CORE.
# NOTE: For security reasons, SPOT_USER_PASSWORD environment variable should be set BEFORE running
#       this script!! (Don't want to store plain-text password in this script.)

# Specify some constants. These should almost certainly not change.
ROS_MASTER_URI_STR='http://10.1.0.1:11311'
ROS_IP_STR='10.1.0.1'

# This will change to the lab's Docker image once this has been transferred over.
DOCKER_CONTAINER_NAME='ghcr.io/dcolli23/spot_ros_driver'

# Launch the Docker container built for running the ROS driver.
# -e passes in environment variables which is required to run the ROS driver.
# The /bin/bash -c is to execute multiple commands. This is probably best handled via the Docker
# image's Dockerfile at build time but leaving the command specification in this form was more
# conducive to rapid prototyping.
sudo docker run --rm --net=host \
  -e ROS_MASTER_URI="$ROS_MASTER_URI_STR" \
  -e ROS_IP="$ROS_IP_STR" \
  -e SPOT_USER_PASSWORD="$SPOT_USER_PASSWORD" \
  $DOCKER_CONTAINER_NAME \
  /bin/bash -c 'cd ~/catkin_ws && \
                source devel/setup.bash && \
                roslaunch spot_driver driver.launch username:=user password:="$SPOT_USER_PASSWORD"'
#!/bin/bash

# Launches all necessary components to enable ROS-based control of Spot. This is accomplished by
# launching 3 persistent terminals (meaning they stick around after this script is done executing)
# that do the following:
# 1. Starts OpenVPN on Spot CORE
# 2. Starts the ROS driver on Spot via launching docker container
# 3. Starts OpenVPN on operator PC
#
# NOTE: This becomes vastly more convenient if you have SSH keys setup between the operator PC and
# the Spot CORE.

# Set up constants to make the script a bit more readable.
SPOT_CORE_PORT='20022'

# Setting up variables so that the option of passing in non-default arguments is easier down the
# road.
ROBOT_IP='192.168.80.3'

# Read the password from the command line arguments passed into this script.
SPOT_USER_PASSWORD=$1

# 1. Start OpenVPN on the Spot CORE
gnome-terminal -- ssh -tp $SPOT_CORE_PORT spot@$ROBOT_IP "./launch_openvpn.sh"
sleep 5

# 2. Start ROS driver on Spot via launching the docker container.
gnome-terminal -- ssh -tp $SPOT_CORE_PORT spot@$ROBOT_IP \
    "export SPOT_USER_PASSWORD=$SPOT_USER_PASSWORD && ./launch_core_ros_driver.sh"
sleep 5

# 3. Run OpenVPN on the operator PC (the PC you're on right now)
gnome-terminal -- ./launch_openvpn.sh
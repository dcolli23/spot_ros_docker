#!/bin/bash

# I think this causes the script to exit if any of the scripts return non-zero exit status.
set -e

# To add another install script, just source it using:
#     `source /scripts/<SCRIPT_NAME>`
echo "Running /scripts/create_catkin_workspace.bash"
source /scripts/create_catkin_workspace.bash
echo "Running /scripts/install_ros_dependencies.bash"
source /scripts/install_ros_dependencies.bash
echo "Running /scripts/install_spot_ros_driver.bash"
source /scripts/install_spot_ros_driver.bash
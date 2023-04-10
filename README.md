# Spot ROS Docker <!-- omit in toc -->

The University of Michigan ARM Lab's workflow for controlling Boston Dynamics Spot robots via the `spot_ros` ROS1 driver **Add link to repo**.
This is necessary as the ROS1 driver requires ROS Noetic but the earlier generation Spot COREs are locked into using Ubuntu 18.04 (and thus run ROS Melodic).


## Table of Contents <!-- omit in toc -->

- [High-level Overview](#high-level-overview)
- [Installation](#installation)
  - [Docker Image For Running ROS Driver On A Spot CORE](#docker-image-for-running-ros-driver-on-a-spot-core)
  - [Scripts For Easy ROS Driver Launch](#scripts-for-easy-ros-driver-launch)
  - [OpenVPN](#openvpn)
- [Usage](#usage)
- [External Resources Used In The Project](#external-resources-used-in-the-project)

## High-level Overview

This repository houses both:
- The Dockerfile and support scripts necessary for building the `spot_ros` ROS1 driver on a Spot CORE.
- Bash scripts to make the process of launching the ROS driver on the CORE more user-friendly.

## Installation

As this repo houses files to accomlish two goals, the installation (or lack thereof) will differ depending on what you'd like to get out of this package.
If you only need the Docker image, read only [this section](#docker-image-for-running-ros-driver-on-a-spot-core).
If you'd like to use both the Docker image *and* the bash scripts we wrote to make life a little easier, then continue reading the entirety of this section.

### Docker Image For Running ROS Driver On A Spot CORE

This repository is set up to automatically build the Docker image necessary to run the `spot_ros` ROS1 driver **add link to repo**.
As such, you shouldn't have to manually build the Docker image.
Instead, you should pull the most up-to-date Docker image by executing: `docker image pull ghcr.io/dcolli23/spot_ros_driver`.

**DYLAN UPDATE PULL COMMAND TO LAB'S USERNAME**
**ALSO UPDATE LAUNCH_CORE_ROS_DRIVER.SH DOCKER CONTAINER NAME TO LAB'S USERNAME**

However, if you'd like to build the Docker image locally for development reasons, the `build_docker_image.sh` script will build the image.

### Scripts For Easy ROS Driver Launch

This repository also contains some bash scripts that make launching the ROS driver a much simpler process.
Before installing these scripts, you'll need to [set up and configure OpenVPN](#openvpn) on the computer you wish to command Spot from (via ROS) as well as the CORE.
After OpenVPN is installed and configured on both devices, follow these steps to configure and "install" the launch scripts:

1. Copy the `launch_core_ros_driver.sh` and `launch_openvpn.sh` scripts to the home directory of the Spot CORE.
   1. Make sure they're executable! You can do this with `chmod +x <SCRIPT_NAME>`
2. Configure the `launch_openvpn.sh` script you just copied to the CORE so that the variables in the script point to the directory where OpenVPN was configured on the CORE.
3. Copy the `launch_openvpn.sh` script to the home directory of the computer that will control Spot via ROS.
4. Again, configure the `launch_openvpn.sh` script to point to the directory where OpenVPN was configured on the operator computer.
5. (Optional but highly recommended) Set up SSH keys between the operator computer and the Spot CORE.


### OpenVPN

**Waiting on Adam Li's approval**

## Usage

This section documents usage of this package with the assumption that you followed all of the steps in the [Installation](#installation) section, including the script setup.

To launch the ROS driver:

1. Turn on Spot.
2. Connect to Spot's WiFi hotspot with the operator PC.
3. On the operator PC, execute `./launch_spot.sh <SPOT_USER_PASSWORD>`
   1. This will launch 3 separate terminals that must be authenticated **in order**. Assuming you setup SSH keys between the operator computer and the CORE/Spot, the expected passwords and responsiblities of the 3 terminals are:
      1. The Spot CORE's `sudo` password.
         1. Launches OpenVPN on the CORE.
      2. Same as 1.
         1. Launches the docker container and ROS driver on the CORE.
      3. The operator computer's `sudo` password.
         1. Launches OpenVPN on the operator computer and connects to the CORE's OpenVPN network.
4. Finally, in any terminal you intend to interact with the ROS driver with, execute the following command to indicate where the ROS core is running:
    ```
    export ROS_MASTER_URI=http://10.1.0.1:11311 && export ROS_IP=10.1.0.2
    ```
    You can define your own command for this to save you from excessive typing.

## External Resources Used In The Project

[This page](https://wilselby.com/2019/05/os-1-ros-package-deployment-with-docker/) provided a great jumping off point for getting this to work.

Additionally, Adam Li of the University of Michigan Robotics Department was very helpful in setting up/configuring OpenVPN to painlessly allow the ROS driver to comunicate with operator computers.
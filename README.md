# Spot ROS Docker <!-- omit in toc -->

The [University of Michigan ARM Lab's](https://arm.eecs.umich.edu/) workflow for controlling Boston Dynamics Spot robots via the [`spot_ros` ROS1 driver](https://github.com/heuristicus/spot_ros). This is necessary as the ROS1 driver requires ROS Noetic but the earlier generation Spot COREs are locked into using Ubuntu 18.04 (and thus only ROS Melodic is supported on the hardware).


## Table of Contents <!-- omit in toc -->

- [High-level Overview](#high-level-overview)
- [Installation](#installation)
  - [Docker Image](#docker-image)
  - [Scripts For Easy ROS Driver Launch](#scripts-for-easy-ros-driver-launch)
  - [OpenVPN](#openvpn)
    - [On The Spot CORE](#on-the-spot-core)
    - [On The Operator Computer](#on-the-operator-computer)
- [Usage](#usage)
- [Contributing And Reporting Issues](#contributing-and-reporting-issues)
- [External Resources Used In The Project](#external-resources-used-in-the-project)

## High-level Overview

This repository houses both:
- The Dockerfile and support scripts necessary for building the `spot_ros` ROS1 driver on a Spot CORE.
- Bash scripts to make the process of launching the ROS driver on the CORE more user-friendly.

## Installation

As this repo houses files to accomlish two goals, the installation (or lack thereof) will differ depending on what you'd like to get out of this package. If you only need the Docker image, read only the [Docker Image](#docker-image-for-running-ros-driver-on-a-spot-core) section. If you'd like to use both the Docker image *and* the bash scripts we wrote to make life a little easier, continue reading the entirety of this section.

### Docker Image

This repository is set up to automatically build the Docker image necessary to run the `spot_ros` ROS1 driver. As such, you shouldn't have to manually build the Docker image. Instead, you should pull the most up-to-date Docker image by executing: `docker image pull ghcr.io/dcolli23/spot_ros_driver`.

**DYLAN UPDATE PULL COMMAND TO LAB'S USERNAME**
**ALSO UPDATE LAUNCH_CORE_ROS_DRIVER.SH DOCKER CONTAINER NAME TO LAB'S USERNAME**

However, if you'd like to build the Docker image locally for development reasons, the `build_docker_image.sh` script will build the image.

### Scripts For Easy ROS Driver Launch

This repository also contains some bash scripts that make launching the ROS driver a much simpler process. Before installing these scripts, you'll need to [set up and configure OpenVPN](#openvpn) on the computer you wish to command Spot from (via ROS) as well as the CORE. After OpenVPN is installed and configured on both devices, follow these steps to configure and "install" the launch scripts:

1. Copy the `launch_core_ros_driver.sh` and `launch_openvpn.sh` scripts to the home directory of the Spot CORE.
   1. Make sure they're executable! You can do this with `chmod +x <SCRIPT_NAME>`
2. Configure the `launch_openvpn.sh` script you just copied to the CORE so that the variables in the script point to the directory where OpenVPN was configured on the CORE.
3. Copy the `launch_openvpn.sh` script to the home directory of the computer that will control Spot via ROS.
4. Again, configure the `launch_openvpn.sh` script to point to the directory where OpenVPN was configured on the operator computer.
5. (Optional but highly recommended) Set up SSH keys between the operator computer and the Spot CORE.


### OpenVPN

[Adam Li](https://github.com/BuildingAtom) was gracious enough to provide us with the ROAHM Lab's documentation for using OpenVPN to circumvent headaches induced by configuring ROS to work with the CORE's network. In his own words, this is a "lazy" (read non-complicated) way to setup communication between the operator computer and the Spot CORE. This could also be accomplished with the use of custom port ranges but that requires more in-depth networking knowledge.

#### On The Spot CORE

1. Install OpenVPN.
   ```
   sudo apt install openvpn
   ```
2. Make a working directory for OpenVPN.
   ```
   mkdir ~/openvpn
   cd ~/openvpn
   ```
3. Generate a static key and copy the example static config over to the working directory.
   ```
   openvpn --genkey --secret SpotCoreVPN.key
   sudo cp /usr/share/doc/openvpn/examples/sample-config-files/static-office.conf spot-static.ovpn
   ```
4. Edit the example config:
   1. Comment out the line `up office.up` with `#` or `;`.
   2. Change `secret static.key` to `secret SpotCoreVPN.key`
   3. Uncomment and change the port to 21194.
   4. Change the cipher to 128 bit instead (to improve speed) `cipher AES-128-CBC`.
   5. Uncomment the section for more reliable detection of when the system loses its connection.

#### On The Operator Computer

1. Install OpenVPN.
   ```
   sudo apt install openvpn
   ```
2. Make a working directory for OpenVPN.
   ```
   mkdir ~/openvpn
   cd ~/openvpn
   ```
3. Download the static key from Spot to OpenVPN working directory.
   ```
   sudo scp -P 20022 spot@192.168.80.3:~/openvpn/SpotCoreVPN.key ~/openvpn/SpotCoreVPN.key
   ```
4. Copy the example static config over to the working directory.
   ```
   sudo cp /usr/share/doc/openvpn/examples/sample-config-files/static-home.conf spot-static.ovpn
   ```
5. Edit the example config:
   1. Update `remote 1.2.3.4` to `remote 192.168.80.3`.
   2. Comment out the line `up office.up` with `#` or `;`.
   3. Change `secret static.key` to `secret SpotCoreVPN.key`
   4. Uncomment and change the port to 21194.
   5. Change the cipher to 128 bit instead (to improve speed) `cipher AES-128-CBC`.
   6. Uncomment the section for more reliable detection of when the system loses its connection.

## Usage

This section documents usage of this package with the assumption that you followed *all* of the steps in the [Installation](#installation) section, including the script setup.

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
4. In any terminal you intend to interact with the ROS driver with, execute the following command to indicate where the ROS core is running:
    ```
    export ROS_MASTER_URI=http://10.1.0.1:11311 && export ROS_IP=10.1.0.2
    ```
    You can also define your own command for this or even add this to your `~/.bashrc` file to save you from excessive typing.
5. Test that everything is working properly by executing `rostopic list`. You should see many topics published by the ROS driver.


## Contributing And Reporting Issues

If you'd like to contribute to the project, feel free to create a pull request or contact Dylan Colli at dfcolli@umich.edu.

If you notice an error with the code or would like to request a feature, please submit a GitHub issue and we'll address the concern as soon as possible.

## External Resources Used In The Project

The following is a list of resources that made this project possible:
- [Adam Li](https://github.com/BuildingAtom) of the University of Michigan Robotics Department's ROAHM Lab was very helpful in setting up/configuring OpenVPN to painlessly allow the ROS driver to comunicate with operator computers.
- [This page](https://wilselby.com/2019/05/os-1-ros-package-deployment-with-docker/) provided an excellent jumping-off point for developing the Docker image.
  - This actually provides an extensible method to install packages in Docker images with bash scripts. This is almost certainly not best practices and becomes very unwieldy when installing multiple packages if you don't use Docker's multi-stage build system for checkpointing the build steps. However, it provides a very convenient way to build Docker images without getting into the weeds of Dockerfile syntax and gotchas.


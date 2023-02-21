# spot_ros_docker

Dockerfile for building an image that runs the spot_ros ROS1 driver with Noetic.

[This page](https://wilselby.com/2019/05/os-1-ros-package-deployment-with-docker/) provided a great jumping off point for getting this to work.

## Building

To build the image, execute `./build_docker_image.sh` from the repo's root directory.

## Running

To run a container (an image instance) and launch the ROS driver:

1. `sudo docker run -it --rm my/spot_ros:app`
2. `cd /root/catkin_ws`
3. `source devel/setup.bash`
4. `roslaunch spot_driver driver.launch username:=<OUR USERNAME> password:=<OUR PASSWORD>`

<font color=orange>TODO:</font> Make launching of the ROS driver automatic, pulling in the username, password, and potentially the robot's IP either from Docker command line arguments or environment variables.
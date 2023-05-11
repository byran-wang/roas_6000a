# This repo is adpoted from [TurtleBot3 Behavior Demos](https://github.com/sea-bass/turtlebot3_behavior_demos)
[Reading this](https://roboticseabass.com/2021/04/21/docker-and-ros/) tutorial is stongly recommended before using this repo.

## Setup
First, install Docker using [the official install guide](https://docs.docker.com/engine/install/ubuntu/).

To run Docker containers with graphics and GPU support, you will also need the [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-docker).

To use GUI based tools (e.g., RViz, Gazebo) inside Docker, there is additional setup required. The simplest way is to run the command below each time you log into your machine, but there is a more detailed walkthrough of options in the [ROS Wiki](http://wiki.ros.org/docker/Tutorials/GUI).

```
xhost + local:docker
make build
```

---

## Basic Usage
We use `make` to automate building, as shown above, but also for various useful entry points into the Docker container once it has been built. **All `make` commands below should be run from your host machine, and not from inside the container**.

To enter a Terminal in the overlay container:

```
make term
```

If you have an NVIDIA GPU and want to give your container access to the devices, add the following argument (this is true for all targets):

```
make term USE_GPU=true
```

To kill all running dockers
```
make kill
```

## For ROAS 6000A
We also install [V-REP](https://www.coppeliarobotics.com/) here. Along with the following packages.

```
ros-noetic-gmapping 
ros-noetic-navigation 
ros-noetic-py-trees 
ros-noetic-py-trees-ros 
ros-noetic-rqt-py-trees
```

### To use vrep:
```bash
cd /overlay_ws/vrep/
./coppeliaSim.sh 
```

### To use package:
```bash
cd /overlay_ws/6000a_ws/
catkin catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
rosun keyboard_control node
rosun image_detection detection.py
```

### Mounted 6000a_ws
The content of 6000a_ws is shared with your host machine and the container. Feel free to modify it in your host and build it in the container.

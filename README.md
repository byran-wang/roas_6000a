# This repo is adpoted from [TurtleBot3 Behavior Demos](https://github.com/sea-bass/turtlebot3_behavior_demos)

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
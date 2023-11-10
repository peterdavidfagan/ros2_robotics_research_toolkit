[![lite6_control](https://github.com/peterdavidfagan/ros2_robotics_research_toolkit/actions/workflows/control.yaml/badge.svg)](https://github.com/peterdavidfagan/ros2_robotics_research_toolkit/blob/ufactory_lite6/.github/workflows/control.yaml) [![motion_planning](https://github.com/peterdavidfagan/ros2_robotics_research_toolkit/actions/workflows/motion_planning.yaml/badge.svg)](https://github.com/peterdavidfagan/ros2_robotics_research_toolkit/blob/ufactory_lite6/.github/workflows/motion_planning.yaml) [![zed](https://github.com/peterdavidfagan/ros2_robotics_research_toolkit/actions/workflows/zed.yaml/badge.svg)](https://github.com/peterdavidfagan/ros2_robotics_research_toolkit/blob/ufactory_lite6/.github/workflows/zed.yaml)
[![foxglove_bridge](https://github.com/peterdavidfagan/ros2_robotics_research_toolkit/actions/workflows/foxglove_bridge.yaml/badge.svg)](https://github.com/peterdavidfagan/ros2_robotics_research_toolkit/blob/ufactory_lite6/.github/workflows/foxglove_bridge.yaml)

# ROS 2 Robot Learning Workspace 🚀 
This repository serves as a template for setting up a ROS 2 workspace for performing robot learning research. If you use ROS 2 and are interested in contributing to this codebase please reach out at peterdavidfagan@gmail.com. If you use these tools in your research please star this repository. 


# Supported Robots 🤖

| Robot | Image | Branch |
|----------|----------|----------|
| UFactory Lite6 | <img src="./assets/Lite-6.gif" width="200" /> | `ufactory_lite6` |
| Franka Emika Panda | <img src="./assets/franka-emika.gif" width="200" /> | `franka_emika_panda` |



# Lite6 Overview
The application code within this repository is intended to be run using Docker, with this being said, it is possible to build the ROS workspace on the host of a machine running Ubuntu 22.04. 

<img src="./assets/workspace.jpg" width="400" />

Tutorials for the following applications are included:

* [Camera Calibration](https://github.com/peterdavidfagan/ros2_robotics_research_toolkit/tree/ufactory_lite6/src/tutorials/camera_calibration) (intrinsic + extrinsic parameters)
* [Control](https://github.com/peterdavidfagan/ros2_robotics_research_toolkit/tree/ufactory_lite6/src/tutorials/control) (interfacing with ROS 2 Controllers)
* [Motion Planning](https://github.com/peterdavidfagan/ros2_robotics_research_toolkit/tree/ufactory_lite6/src/tutorials/motion_planning) (using MoveIt 2 for motion planning)
* [Policy Deployment](https://github.com/peterdavidfagan/ros2_robotics_research_toolkit/tree/ufactory_lite6/src/tutorials/policy_deployment) (deploying neural network policies on the robot)

The following tutorials are considered be added in future:

* Teleoperation
* Grasp Pose Estimation
* Data Collection + Converting ROS 2 MCAP data to RLDS 



# Hardware Setup 🔧
<img src="./assets/ufactory.png" width="600" />

### Components
* UFactory Lite6
* Intel NUC (Intel i5 as minimum CPU spec)
* Client machine (Nvidia GPU that supports CUDA 12+)
* Ethernet switch
* Zed camera


# Software Setup

### Software Prerequisites
This workspaces requires the following software to be installed:

* An installation of Docker ([instructions](https://docs.docker.com/engine/install/ubuntu/))
* An installation of Nvidia Container Toolkit (if you intend to use cameras + policy deployment) ([instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html))
* An installation of ROS rolling ([instructions](https://docs.ros.org/en/rolling/Installation.html))

### NUC Machine Setup
* Install Docker
* Perform Realtime Patch of Kernel
* CPU frequency scaling
* Set static IP
* Turns on SSH service
* Disables display manager

Run the following script to accomplish the above steps and setup your NUC:

```bash
./nuc_setup.sh
```

### Client Machine Setup
** Incomplete

* Set static IP
* Set XAUTH variables to enable X11 forwarding

Run the following script to accomplish the above steps and setup your NUC:
```bash
client_machine_setup.sh
```

### Running ROS Applications on Host
In order to install and build all workspace dependencies on your local machine all you need to run is:
```bash
./local_setup.sh
```

This is only necessary if you wish to run application code directly on the host machine, most applications within this repository are dockerised and hence don't require you to build the ROS 2 workspace on the host machine.

### Running ROS Applications with Docker
The following guides are recommended as background reading for this section:

* [Docker Guide](https://docs.docker.com/get-started/) 
* [Docker Compose](https://docs.docker.com/compose/)
* [Docker Swarm](https://docs.docker.com/engine/swarm/)

### Docker Application Deployment
The `.docker` folder of this repository contains multiple subfolders which docker compose files that can be used to deploy containers across machines within the LAN. 

### Docker Swarm Configuration
**Incomplete 

1. Initialize swarm on client machine
2. Make client machine a swarm manager
3. Add NUC and other devices as Nodes within the swarm

### Docker GUI Prerequisites
If you wish to run a container that contains GUI applications (e.g. rviz) you need to first manage X-server authentication. The most basic way to do so is through enabling access to all local applications by running: 

```
xhost +local:
```

This is in general bad practice as it disables security settings for local applications. In order to authenticate the docker container alone the following command needs to be run to populate a temporary file which our docker build will use:

```
export DOCKER_XAUTH=/tmp/.docker.xauth
touch $DOCKER_XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $DOCKER_XAUTH nmerge -
```
Please see [policy_deployment_tutorial](placeholder.com) for further instructions.

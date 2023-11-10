# Camera Calibration Tutorials
The camera calibration tutorial enables you to calibrate cameras with ROS 2 suport. This tutorial is split into two parts, the first section outlines camera intrisic parameters calibration, the second section outlines camera extrinsic parameters calibration.

**Important Note:** Docker swarm doesn't currently support GPU devices, as a result, the camera container must be deployed separately to the docker swarm deployment ([related article](https://blog.kronis.dev/everything%20is%20broken/docker-swarm-access-to-devices-is-nonexistent)).

## Intrinsic Parameter Calibration
We inherit directly from the following intrinsic parameter calibration [tutorial](https://navigation.ros.org/tutorials/docs/camera_calibration.html). In order to run this tutorial you need to ensure the camera you are calibration is deployed. To do this I recommend using a compose file in `.docker/cameras` for your camera, for the zed camera this would look like:

```bash
docker compose -f docker-compose-zed.yaml up -d
```

Once your camera is running you can load the intrisic camera calibration, using the outline in this [tutorial](https://navigation.ros.org/tutorials/docs/camera_calibration.html).

## Extrinsic Parameter Calibration

## Regular Deployment

If you have not already configured Docker swarm, you can deploy containers on machines through directly running docker compose commands on each machine. Further instructions are given below:

### Start the Control Server on the NUC
From a terminal session on the NUC, enter the following directory from this repository `.docker/tutorials/camera_calibration`. From this directory one can start the control server through running the following command

```bash
docker compose -f docker-compose-nuc-deployment.yaml up -d
```

### Start Motion Planning on the Client machine
From a terminal session on the client machine, enter the following directory from this repository `.docker/tutorials/camera_calibration`. From this directory one can start motion planning services through running the following command

```bash
docker compose -f docker-compose-client-deployment.yaml up -d
```

## Docker Swarm Deployment

To avoid having to manually run docker compose commands on both the NUC and the client machine, one can instead deploy a stack using docker swarm and a single docker compose file. If you have already configured a docker swarm you can deploy services across devices with the below command (where you replace the deploy constraints with the names of the hostname for nodes in your swarm):

```bash
docker stack deploy --compose-file docker-compose-motion-planning-notebook.yaml <name of deployed stack>
```

As docker swarm does not support deployment with access to GPU resources, the camera must be started separately, using compose files in `.docker/cameras` we can start a camera with:

```bash
docker compose -f docker-compose-zed.yaml up -d
```

To undeploy the stack of services run 

```bash
docker stack rm <name of the deployed stack>
```

## Camera Calibration Config Parameters

In order to configure the camera calibration procedure for your workspace, you must define a number of parameters, namely:

* charuco board parameters
* number of samples for calibration
* workspace from which to randomly sample end effector poses

A sample configuration file is provided below, importantly if you have multiple third person cameras, you will need to update this config for the calibration procedure for each camera. In future, it will be possible to simply set parameters for the calibration services through referencing the yaml configuration via the command line.

```yaml
camera_info_topic: /zed2i/zed_node/rgb_raw/camera_info
camera_topic: /zed2i/zed_node/rgb_raw/image_raw_color
eye_to_hand_calibration_service: /camera_calibration/run_eye_to_hand_calibration
eye_in_hand_calibration_service: /camera_calibration/run_eye_in_hand_calibration

charuco:
  squares_x: 14
  squares_y: 9
  square_length: 0.02
  marker_length: 0.015

eye_to_hand_calibration:
  num_samples: 25 
  sample_delay: 0.4

  workspace:
    x_min: 0.35
    x_max: 0.45
    y_min: -0.1
    y_max: 0.1
    z_min: 0.3
    z_max: 0.4 
    rot_x_min: 160.0
    rot_x_max: 200.0
    rot_y_min: -90.0
    rot_y_max: -135.0
    rot_z_min: -5.0
    rot_z_max: 5.0


eye_in_hand_calibration:
  num_samples: 25 
  sample_delay: 0.4

  workspace:
    x_min: 0.35
    x_max: 0.45
    y_min: -0.1
    y_max: 0.1
    z_min: 0.3
    z_max: 0.4 
    rot_x_min: 160.0
    rot_x_max: 200.0
    rot_y_min: -90.0
    rot_y_max: -135.0
    rot_z_min: -5.0
    rot_z_max: 5.0
```

## Running Camera Calibration Procedures
Once you have deployed all of the camera calibration prerequisites and appropriately updated your camera calibration config, you can run either the `/camera_calibration/run_eye_to_hand_calibration` or `/camera_calibration/run_eye_in_hand_calibration` calibration service via the CLI through calling:

```bash
ros2 service call <calibration service name> std_srvs/SetBool "{data: true}"
```

This will generate a file containing the extrinsic calibration parameters for you camera. Using this file you can update your camera pose, most cameras provide a ros service for doing so, in the case of the zed camera (and model) this service is called `/<camera model>/zed_node/set_pose`.

## Video Walkthrough

This is not added yet but I plan to include a video here in future.For now there is a qualitative example of running servo using the code in this repository: 

https://github.com/peterdavidfagan/ros2_robotics_research_toolkit/assets/42982057/338c0bb7-b29f-47e8-8d6e-c4c781ea112a


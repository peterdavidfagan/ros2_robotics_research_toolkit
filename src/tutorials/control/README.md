# Control Tutorials
The control tutorials provide base examples for interfacing with ROS 2 controllers via the `moveit_servo` package. This tutorial, first outlines how to deploy/start prerequisites before running scripts that interface with `moveit_servo`.

## Regular Deployment

If you have not already configured Docker swarm, you can deploy containers on machines through directly running docker compose commands on each machine. You may need to pull the containers used in these compose files if you haven't already, this can be accomplished for the control-server container with:

```
docker pull ghcr.io/peterdavidfagan/lite6_control:rolling
```

It is worth noting that the docker compose files when run will look to pull the container if it is not already on your host machine. Further instructions are given below:

### Start the Control Server on the NUC
From a terminal session on the NUC, enter the following directory from this repository `.docker/tutorials/control`. From this directory one can start the control server through running the following command

```
docker compose -f docker-compose-nuc-deployment.yaml up -d
```


### Start Motion Planning on the Client machine
From a terminal session on the client machine, enter the following directory from this repository `.docker/tutorials/control`. From this directory one can start motion planning services through running the following command

```
docker compose -f docker-compose-client-deployment.yaml up -d
```

## Docker Swarm Deployment

To avoid having to manually run docker compose commands on both the NUC and the client machine, one can instead deploy a stack using docker swarm and a single docker compose file. If you have already configured a docker swarm you can deploy services across devices with the below command (where you replace the deploy constraints with the names of the hostname for nodes in your swarm):

```
docker stack deploy --compose-file docker-compose-servo-application.yaml <name of deployed stack>
```

To undeploy the stack of services run 

```
docker stack rm <name of the deployed stack>
```

## Executing Scripts/Notebooks
At the time of writing this `README.md`, there appears to be latency issues when receiving joint information within an interactive session in a docker container. This needs to be investigated further, as a result in order to execute the python tutorials and/or start a notebook session that performs motion planning, I recommend doing so on your host machine directly (outside of Docker container). To accomplish this you need to install ros rolling and build the workspace locally through running the `build.sh` script located in the root of the repository. Once you have done this, running the python scripts and starting notebooks is the same as one would do regularly on the host machine. 

## Video Walkthrough

This is not added yet but I plan to include a video here in future. For now there is a qualitative example of running servo using the code in this repository: 

[<img src="https://github.com/peterdavidfagan/lite6_ws/blob/rolling/assets/workspace.jpg?raw=true" width="400">](https://www.youtube.com/shorts/DTBcyli4Wsw)

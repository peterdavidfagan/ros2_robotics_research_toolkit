# Motion Planning Tutorials
The motion planning tutorials provide base examples for interfacing with the MoveIt python library. There are two ways to interface with the library, via python scripts or a jupyter notebook. This tutorial, first outlines how to deploy/start applications before delving into the details of the available tutorials.

## Regular Deployment

If you have not already configured Docker swarm, you can deploy containers on machines through directly running docker compose commands on each machine. Further instructions are given below:

### Start the Control Server on the NUC
From a terminal session on the NUC, enter the following directory from this repository `.docker/tutorials/motion_planning`. From this directory one can start the control server through running the following command

```
docker compose -f docker-compose-nuc-deployment.yaml up -d
```


### Start Motion Planning on the Client machine
From a terminal session on the client machine, enter the following directory from this repository `.docker/tutorials/motion_planning`. From this directory one can start motion planning services through running the following command

```
docker compose -f docker-compose-client-deployment.yaml up -d
```

## Docker Swarm Deployment

To avoid having to manually run docker compose commands on both the NUC and the client machine, one can instead deploy a stack using docker swarm and a single docker compose file. If you have already configured a docker swarm you can deploy services across devices with the below command (where you replace the deploy constraints with the names of the hostname for nodes in your swarm):

```
docker stack deploy --compose-file docker-compose-motion-planning-notebook.yaml <name of deployed stack>
```

To undeploy the stack of services run 

```
docker stack rm <name of the deployed stack>
```

## Notebook Tutorial
[Official Tutorial](https://moveit.picknik.ai/main/doc/examples/jupyter_notebook_prototyping/jupyter_notebook_prototyping_tutorial.html)

## Motion Planning Script Tutorial
[Official Tutorial](https://moveit.picknik.ai/main/doc/examples/motion_planning_python_api/motion_planning_python_api_tutorial.html)

## Video Walkthrough

This is not added yet but I plan to include a video here in future.

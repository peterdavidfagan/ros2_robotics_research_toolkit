---
layout: default
title: Starting ROS 2 Controllers
parent: Tutorials
has_children: false
nav_order: 1
permalink: /docs/toolkit_guides
---

<details open markdown="block">
  <summary>
    Table of contents
  </summary>
  {: .text-delta }
1. TOC
{:toc}
</details>

# Background

Lets review ROS 2 control to understand the core concepts behind how the library manages controllers.

Please consult the official [ROS 2 control documentation]() for further information.

# Configuring Controllers

There are two files we care to configure to control which controllers we launch as part of our application: 

1. The launch file which launches the controller manager and controllers.
2. ROS 2 controller configuration files which control the configuration of controllers (e.g. the controller update frequency).


# Running Controllers via Docker (Recommended)

A built docker container is shipped as part of this repository, feel free to pull the latest version of the container with:

```bash
docker pull ghcr.io/peterdavidfagan/panda_control:rolling
```

Once you have pulled the built container you can run the container via the docker compose script present under `.docker/control` from the root directory of the repository.

```bash
docker compose -f docker-compose-control-server.launch.py up
```

You can expect to see output similar to the following:

```
```

# Running Controllers on the Host


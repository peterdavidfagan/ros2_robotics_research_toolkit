#!/bin/bash

ROOT_DIR="$(git rev-parse --show-toplevel)"
DOCKER_COMPOSE_DIR="$ROOT_DIR/.docker/motion_planning"
DOCKER_COMPOSE_FILE="$DOCKER_COMPOSE_DIR/docker-compose-motion_planning.yaml"
LAPTOP_IP="192.168.1.11"

echo "Welcome to the lite6_ws setup process."

# ensure GUI window is accessible from container
echo -e "Set Docker Xauth for x11 forwarding \n"

export DOCKER_XAUTH=/tmp/.docker.xauth
rm $DOCKER_XAUTH
touch $DOCKER_XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $DOCKER_XAUTH nmerge -

# build client server container
read -p "Do you want to rebuild the container image? (yes/no): " first_time

if [ "$first_time" = "yes" ]; then
	echo -e "build control server container \n"
	cd $DOCKER_COMPOSE_DIR && docker compose -f $DOCKER_COMPOSE_FILE build
fi

# find ethernet interface on device
echo -e "\n set static ip \n"

echo "Select an Ethernet interface to set a static IP for:"

interfaces=$(ip -o link show | grep -Eo '^[0-9]+: (en|eth|ens|eno|enp)[a-z0-9]*' | awk -F' ' '{print $2}')

# Display available interfaces for the user to choose from
select interface_name in $interfaces; do
    if [ -n "$interface_name" ]; then
        break
    else
        echo "Invalid selection. Please choose a valid interface."
    fi
done

echo "You've selected: $interface_name"

# Add and configure the static IP connection
nmcli connection delete "laptop_static"
nmcli connection add con-name "laptop_static" ifname "$interface_name" type ethernet
nmcli connection modify "laptop_static" ipv4.method manual ipv4.address $LAPTOP_IP/24
nmcli connection up "laptop_static"

echo "Static IP configuration complete for interface $interface_name."

# run docker container
echo -e "run client application \n"
docker compose -f $DOCKER_COMPOSE_FILE up -d

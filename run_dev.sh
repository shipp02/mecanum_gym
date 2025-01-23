#!/usr/bin/env bash

set -e

DOCKER_ARGS=()
BASE_NAME=mecanum_gym
CONTAINER_NAME="$BASE_NAME-env"
PWD=$(pwd)
XAUTH_FILE=$(xauth info | head -n 1 | gawk -e '{ print $3 }')
# All files in workspace and the .Xauthority file must be owned by aashay:aashay
# chown aashay:aashay ./* # in /sim_ws
# chown aashay:aashay /sim_ws # in /
# chown aashay:aashay ./.Xauthority # in /home/aashay
# TODO: there is a permission issue between host and docker due to the bind mount
# When the permsission were changed, it identified as GID 1001 on host and refused
# to let me edit them as aashays


# Remove any exited containers.
if [ "$(docker ps -a --quiet --filter status=exited --filter name=$CONTAINER_NAME)" ]; then
    docker rm $CONTAINER_NAME > /dev/null
fi

# Re-use existing container.
if [ "$(docker ps -a --quiet --filter status=running --filter name=$CONTAINER_NAME)" ]; then
    print_info "Attaching to running container: $CONTAINER_NAME"
    ISAAC_ROS_WS=$(docker exec $CONTAINER_NAME printenv ISAAC_ROS_WS)
    print_info "Docker workspace: $ISAAC_ROS_WS"
    docker exec -i -t -u admin --workdir $ISAAC_ROS_WS $CONTAINER_NAME /bin/bash $@
    exit 0
fi

DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
DOCKER_ARGS+=("-v $XAUTH_FILE:/home/$USER/.Xauthority:rw")
DOCKER_ARGS+=("-e DISPLAY")
DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")
DOCKER_ARGS+=("-e ROS_DOMAIN_ID")
DOCKER_ARGS+=("-e USER")
DOCKER_ARGS+=("-e USERNAME")
DOCKER_ARGS+=("-e ISAAC_ROS_WS=/workspaces/isaac_ros-dev")
DOCKER_ARGS+=("-e HOST_USER_UID=`id -u`")
DOCKER_ARGS+=("-e HOST_USER_GID=`id -g`")

docker run -it \
    --privileged \
    --network host \
    --ipc=host \
    --hostname mecanum-gym \
    ${DOCKER_ARGS[@]} \
    -v $PWD:/sim_ws \
    -v $PWD/ws_entrypoint.sh:/ws_entrypoint.sh \
    -v /etc/localtime:/etc/localtime:ro \
    --name "$CONTAINER_NAME" \
    --workdir /sim_ws \
    --entrypoint /ws_entrypoint.sh \
    $BASE_NAME \
    /bin/bash




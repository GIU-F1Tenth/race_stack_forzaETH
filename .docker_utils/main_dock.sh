#!/bin/bash
# filepath: /home/ubuntu/sim/racing_playground/.docker_utils/main_dock.sh

# Script to launch the main docker instance for the pblf110 car
#IMAGE=sim_x86_forzaeth_racestack_ros2 # for x86
IMAGE=jet_forzaeth_racestack_ros2 # for ARM

# Set proper paths for your system
FORZETH_DIR=/home/ubuntu/sim/racing_playground
XAUTH_LOC=/home/$USER/.Xauthority

# Create cache directories if they don't exist
mkdir -p $FORZETH_DIR/../cache/humble/{build,install,log}

# Ensure X11 auth file exists
if [ ! -f "$XAUTH_LOC" ]; then
    touch "$XAUTH_LOC"
fi

echo "Starting container with:"
echo "  Image: ${IMAGE}:humble"
echo "  Workspace: $FORZETH_DIR"
echo "  X11 Auth: $XAUTH_LOC"

docker run --tty \
    --interactive \
    --network=host \
    --env DISPLAY=$DISPLAY \
    --env USER=$USER \
    --env ROS_DOMAIN_ID=6 \
    --env ROS_LOCALHOST_ONLY=0 \
    --env RCUTILS_COLORIZED_OUTPUT=1 \
    --env XAUTHORITY=/home/$USER/.Xauthority \
    --volume $XAUTH_LOC:/home/$USER/.Xauthority \
    --volume /dev:/dev \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume $FORZETH_DIR/../cache/humble/build:/home/$USER/ws/build \
    --volume $FORZETH_DIR/../cache/humble/install:/home/$USER/ws/install \
    --volume $FORZETH_DIR/../cache/humble/log:/home/$USER/ws/log \
    --volume $FORZETH_DIR:/home/$USER/ws/src/racing_playground \
    --privileged \
    --name forzaeth_racestack_ros2_humble \
    --entrypoint /bin/bash \
    ${IMAGE}:humble
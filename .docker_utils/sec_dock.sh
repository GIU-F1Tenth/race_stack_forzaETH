#!/bin/bash
# filepath: /home/mohammedazab/ws/src/race_stack/.docker_utils/sec_dock.sh

# Script to open additional terminal in the running container
CONTAINER_NAME=forzaeth_racestack_ros2_humble

if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Opening new terminal in container: $CONTAINER_NAME"
    
    docker exec -it $CONTAINER_NAME bash -c "
        cd /home/$USER/ws/src/race_stack
        source /opt/ros/humble/setup.bash
        source ~/ws/install/setup.bash 2>/dev/null || true
        export ROS_DOMAIN_ID=6
        export ROS_LOCALHOST_ONLY=0
        export RCUTILS_COLORIZED_OUTPUT=1
        echo 'Secondary terminal ready! ROS2 environment loaded.'
        bash
    "
else
    echo "Error: Container '$CONTAINER_NAME' is not running"
    echo "Available containers:"
    docker ps --format "table {{.Names}}\t{{.Status}}"
    echo ""
    echo "Please start the main container first using: ./main_dock.sh"
    exit 1
fi
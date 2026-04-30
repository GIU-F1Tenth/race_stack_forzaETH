#!/usr/bin/env bash
# sim.sh – launch the base system in simulation mode
# Usage: ./scripts/sim.sh --map <MAP_NAME> [--racecar-version SIM]
#
# Can also be called via:  just sim base --map <MAP_NAME>
set -eo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/ws/src/racing_playground}"
MAPS_DIR="$WORKSPACE/stack_master/maps"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_INSTALL="/home/ubuntu/ws/install/setup.bash"

map=""
racecar_version="SIM"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --map)             map="$2";             shift 2 ;;
        --racecar-version) racecar_version="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $0 --map <MAP_NAME> [--racecar-version SIM]"
            exit 0
            ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

[[ -z "$map" ]] && { echo "Error: --map is required"; echo "Usage: $0 --map <MAP_NAME>"; exit 1; }

[[ ! -d "$MAPS_DIR/$map" ]] && {
    echo "Error: map '$map' not found in $MAPS_DIR"
    echo "Available maps:"
    ls "$MAPS_DIR"
    exit 1
}

set +u
source "$ROS_SETUP"
[[ -f "$WS_INSTALL" ]] && source "$WS_INSTALL"
set -u

echo "Launching base system – map: $map  racecar_version: $racecar_version  sim: True"
ros2 launch stack_master base_system_launch.xml \
    map_name:="$map" \
    sim:=True \
    racecar_version:="$racecar_version"

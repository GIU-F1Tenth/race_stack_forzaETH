#! /bin/bash

# Script to create a Xauthority file for the docker container
XAUTH=$HOME/.Xauthority
export XAUTH_LOC=$XAUTH

# Check if X11 socket directory exists, create if needed (without sudo)
if [ ! -d "/tmp/.X11-unix" ]; then
    echo "Warning: /tmp/.X11-unix does not exist. X11 forwarding may not work."
fi

# Allow local connections
xhost +local:$USER 2>/dev/null || echo "Warning: xhost command failed (probably running on Wayland)"

touch $XAUTH

# For Wayland compatibility, copy xauth info if available
if [ -n "$DISPLAY" ] && command -v xauth >/dev/null 2>&1; then
    xauth list $DISPLAY 2>/dev/null | head -1 | xauth -f $XAUTH nmerge - 2>/dev/null || true
fi

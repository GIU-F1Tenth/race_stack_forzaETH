#!/usr/bin/env bash
# build.sh – build the ROS 2 workspace
# Usage: ./scripts/build.sh
#
# Can also be called via:  just build
#
# NOTE: 'source install/setup.bash' runs inside this subshell and will NOT
# affect your calling terminal.  After the build completes, run:
#   source ~/ws/install/setup.bash
set -eo pipefail

ROS_SETUP="/opt/ros/humble/setup.bash"
WS="$HOME/ws"

set +u; source "$ROS_SETUP"; set -u
cd "$WS"

echo "Building workspace in $WS ..."
colcon build
# colcon build --symlink-install

echo ""
echo "Build complete."
echo "To apply the workspace in your current shell:"
echo "  source ~/ws/install/setup.bash"

#!/usr/bin/env bash
# Copy this workspace's sources into ~/slam_ws inside WSL with Unix line endings, then build.
# Run from the WSL terminal: bash "/mnt/c/.../slam_ws/sync_to_wsl.sh"
SRC="$(dirname "$(readlink -f "$0")")/src/"
rsync -a --exclude 'AutoDRIVE Simulator*' --exclude '*Zone.Identifier' --exclude __pycache__ "$SRC" ~/slam_ws/src/
find ~/slam_ws/src -type f \( -name '*.py' -o -name '*.xml' -o -name '*.yaml' -o -name '*.cfg' -o -name '*.txt' -o -name '*.md' -o -name '*.rviz' \) -exec sed -i 's/\r$//' {} +
source /opt/ros/humble/setup.bash
cd ~/slam_ws && colcon build --symlink-install

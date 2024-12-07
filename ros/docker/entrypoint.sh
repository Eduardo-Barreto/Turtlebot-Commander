#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "Sourced ROS 2 ${ROS_DISTRO}"

# Source the workspace
if [ -f ./install/setup.bash ]
then
  source ./install/setup.bash
  echo "Sourced workspace"
fi

# Execute the command passed into this entrypoint
exec "$@"

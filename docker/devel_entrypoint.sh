#!/bin/bash

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# Source the base workspace, if built
if [ -f /base_ws/install/setup.bash ]
then
    source /base_ws/install/setup.bash
fi

cd /overlay_ws
colcon build

# Source the overlay workspace, if built
if [ -f /overlay_ws/install/setup.bash ]
then
    source /overlay_ws/install/setup.bash
fi

exec "$@"
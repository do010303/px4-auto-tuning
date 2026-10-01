# Source this before running the ROS 2 tools:  source env.sh
source /opt/ros/humble/setup.bash
source "${PX4_ROS_WS:-$HOME/px4_ws}/install/setup.bash"

# The snap Micro XRCE-DDS agent announces non-loopback locators, which a
# localhost-only ROS participant silently drops: no /fmu topics show up.
export ROS_LOCALHOST_ONLY=0

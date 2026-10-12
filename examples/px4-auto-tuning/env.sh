# Source this before running the ROS 2 tools:  source env.sh
source /opt/ros/humble/setup.bash
source "${PX4_ROS_WS:-$HOME/px4_ws}/install/setup.bash"

# The snap Micro XRCE-DDS agent announces non-loopback locators, which a
# localhost-only ROS participant silently drops: no /fmu topics show up.
export ROS_LOCALHOST_ONLY=0

# run_sitl.sh keeps Gazebo transport on loopback; `gz topic` / `gz service`
# and gz_viz.py started from this shell have to use the same interface, or
# they lose the simulator whenever the network is down.
export GZ_IP="${GZ_IP:-127.0.0.1}"

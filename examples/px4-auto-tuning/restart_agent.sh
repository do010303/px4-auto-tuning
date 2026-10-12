#!/bin/bash
# Restart only the uXRCE-DDS agent; PX4 and Gazebo keep running and PX4
# reconnects by itself within a few seconds.
#
# Needed when ROS no longer sees the /fmu topics although PX4 is fine. The
# agent talks DDS over the network interface that was up when it started, so
# a Wi-Fi reconnect or a change of network cuts ROS off from it.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

pkill -f 'MicroXRCEAgent udp4 -p 8888$'
sleep 1
nohup micro-xrce-dds-agent udp4 -p 8888 > "$HERE/run/agent.log" 2>&1 < /dev/null &
disown

echo "agent restarted, waiting for PX4 to reconnect..."

for _ in $(seq 30); do
	sleep 1
	if grep -q "datawriter created" "$HERE/run/agent.log"; then
		echo "PX4 is connected again"
		exit 0
	fi
done

echo "PX4 did not reconnect within 30 s: is ./run_sitl.sh still running?" >&2
exit 1

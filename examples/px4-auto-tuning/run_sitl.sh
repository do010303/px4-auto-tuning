#!/bin/bash
# Start PX4 SITL (Gazebo, x500) and the uXRCE-DDS agent.
#
#   ./run_sitl.sh                  # default world, GUI
#   HEADLESS=1 ./run_sitl.sh       # no Gazebo GUI
#   PX4_GZ_WORLD=forest ./run_sitl.sh
#
# From a terminal PX4 runs in the foreground with its `pxh>` console.
# Without a terminal (or with PX4_DAEMON=1) it runs as a daemon and logs to
# run/px4.log; talk to it with `$PX4_DIR/build/px4_sitl_default/bin/px4-param` etc.

set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-$(cd "$HERE/../.." && pwd)}"
BUILD="$PX4_DIR/build/px4_sitl_default"
RUN_DIR="${RUN_DIR:-$HERE/run}"

mkdir -p "$RUN_DIR"

if [ ! -x "$BUILD/bin/px4" ]; then
	echo "PX4 SITL is not built: run 'make px4_sitl' in $PX4_DIR first" >&2
	exit 1
fi

micro-xrce-dds-agent udp4 -p 8888 > "$RUN_DIR/agent.log" 2>&1 &
AGENT_PID=$!
PX4_PID=

cleanup() {
	kill $PX4_PID $AGENT_PID 2>/dev/null
	pkill -f '^gz sim .* -s ' 2>/dev/null
}
trap cleanup EXIT

export PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-4001}"
export PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_x500}"
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-default}"

# Own rootfs: parameters start from firmware defaults and logs land in
# $RUN_DIR/rootfs/log, separate from the regular `make px4_sitl` rootfs.
ROOTFS="$RUN_DIR/rootfs"
mkdir -p "$ROOTFS"
cp "$BUILD/rootfs/gz_env.sh" "$ROOTFS/gz_env.sh"

apply_setup_params() {
	# px4-param only answers once the PX4 server is up
	until "$BUILD/bin/px4-param" show -q NAV_DLL_ACT > /dev/null 2>&1; do sleep 1; done

	grep -v '^#' "$HERE/sitl_setup.params" | while read -r name value; do
		[ -n "$name" ] && "$BUILD/bin/px4-param" set "$name" "$value" > /dev/null 2>&1
	done
}

cd "$ROOTFS"

if [ -t 0 ] && [ -t 1 ] && [ -z "$PX4_DAEMON" ]; then
	# Manual use: interactive console, output on the terminal
	apply_setup_params &
	"$BUILD/bin/px4" "$BUILD/etc" -w "$ROOTFS"
	exit $?
fi

"$BUILD/bin/px4" -d "$BUILD/etc" -w "$ROOTFS" > "$RUN_DIR/px4.log" 2>&1 &
PX4_PID=$!

until grep -q "Startup script returned successfully" "$RUN_DIR/px4.log"; do
	kill -0 $PX4_PID 2>/dev/null || { cat "$RUN_DIR/px4.log"; exit 1; }
	sleep 1
done

apply_setup_params

echo "PX4 SITL ready (world: $PX4_GZ_WORLD, log: $RUN_DIR/px4.log)"
tail -n +1 -f "$RUN_DIR/px4.log" &
wait $PX4_PID

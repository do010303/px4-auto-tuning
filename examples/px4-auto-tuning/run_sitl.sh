#!/bin/bash
# Start PX4 SITL (Gazebo, x500) and the uXRCE-DDS agent.
#
#   ./run_sitl.sh                  # default world, GUI
#   HEADLESS=1 ./run_sitl.sh       # no Gazebo GUI
#   PX4_GZ_WORLD=forest ./run_sitl.sh
#   PX4_SIM_SPEED_FACTOR=0.5 ./run_sitl.sh   # half real time
#   PX4_SIM_MODEL=gz_x500_heavy ./run_sitl.sh  # variant from ./models
#   KEEP_PARAMS=1 ./run_sitl.sh    # keep the parameters of the last session
#   PX4_GZ_NO_FOLLOW=1 ./run_sitl.sh   # Gazebo camera does not follow the vehicle
#   NO_VIZ=1 ./run_sitl.sh         # do not draw setpoint and flown path in Gazebo
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

# One SITL at a time. A second one would fight over the ports, and when it
# gave up its exit handler would stop the Gazebo of the first.
RUNNING=$(pgrep -f "^$BUILD/bin/px4 |^gz sim .* -s " || true)

if [ -n "$RUNNING" ]; then
	echo "A PX4 SITL or Gazebo server is already running (pid $(echo $RUNNING))." >&2
	echo "Use that one (tools take --attach), or stop it first: Ctrl-C or 'shutdown' at its pxh> prompt." >&2
	exit 1
fi

micro-xrce-dds-agent udp4 -p 8888 > "$RUN_DIR/agent.log" 2>&1 &
AGENT_PID=$!
PX4_PID=
TAIL_PID=
VIZ_PID=

cleanup() {
	kill $PX4_PID $AGENT_PID $TAIL_PID $VIZ_PID 2>/dev/null
	# Only one SITL runs at a time (checked above), so these are ours; the
	# agent may have been replaced by restart_agent.sh
	pkill -f '^gz sim .* -s ' 2>/dev/null
	pkill -f '^gz sim -g' 2>/dev/null
	pkill -f 'MicroXRCEAgent udp4 -p 8888$' 2>/dev/null
	pkill -f '^python3 -m px4_auto_tuning.gz_viz ' 2>/dev/null
}
trap cleanup EXIT

export PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-4001}"
export PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_x500}"
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-default}"

# Keep Gazebo transport on loopback. By default it binds to the primary network
# interface; over Wi-Fi that delivers sensor messages in bursts with stalls of
# up to 0.3 s, PX4 then drops IMU samples and the vehicle can crash.
export GZ_IP="${GZ_IP:-127.0.0.1}"

# The simulated GPS has no latency, but PX4 assumes 110 ms by default and
# compensates for it: the position estimate then runs ~125 ms ahead of the
# vehicle, which shows up as ~0.3 m of estimation error at 2.5 m/s.
export PX4_PARAM_SENS_GPS0_DELAY="${PX4_PARAM_SENS_GPS0_DELAY:-0}"

# Keep DDS timestamps in PX4 boot time instead of converting them to the
# agent's clock: they then match the ulog exactly, at any simulation speed.
export PX4_PARAM_UXRCE_DDS_SYNCT=0

# Own rootfs: parameters start from firmware defaults and logs land in
# $RUN_DIR/rootfs/log, separate from the regular `make px4_sitl` rootfs.
ROOTFS="$RUN_DIR/rootfs"
mkdir -p "$ROOTFS"

# PX4 saves every `param set` into the rootfs, so without this a session would
# inherit whatever the last one (or the last automatic run) left behind, e.g.
# a GPS noise profile. The previous values are kept as *.bson.prev;
# KEEP_PARAMS=1 keeps using them instead.
if [ -z "$KEEP_PARAMS" ]; then
	for f in "$ROOTFS/parameters.bson" "$ROOTFS/parameters_backup.bson"; do
		[ -f "$f" ] && mv -f "$f" "$f.prev"
	done
fi
cp "$BUILD/rootfs/gz_env.sh" "$ROOTFS/gz_env.sh"

# One model directory holding PX4's models plus the variants in ./models
# (see make_models.py), because PX4 spawns from a single PX4_GZ_MODELS path.
PX4_MODELS="$PX4_DIR/Tools/simulation/gz/models"
rm -rf "$RUN_DIR/models" && mkdir -p "$RUN_DIR/models"
ln -s "$PX4_MODELS"/* "$RUN_DIR/models/"
[ -d "$HERE/models" ] && ln -sf "$HERE"/models/* "$RUN_DIR/models/"
cat >> "$ROOTFS/gz_env.sh" <<EOT
export PX4_GZ_MODELS="$RUN_DIR/models"
export GZ_SIM_RESOURCE_PATH="\$GZ_SIM_RESOURCE_PATH:$RUN_DIR/models"
EOT

# PX4 parameters that belong to the selected model variant
MODEL_PARAMS="$HERE/models/${PX4_SIM_MODEL#gz_}/px4.params"

if [ -f "$MODEL_PARAMS" ]; then
	while read -r name value; do
		case "$name" in ''|'#'*) continue ;; esac
		export "PX4_PARAM_$name=$value"
	done < "$MODEL_PARAMS"
fi

apply_setup_params() {
	# px4-param only answers once the PX4 server is up
	until "$BUILD/bin/px4-param" show -q NAV_DLL_ACT > /dev/null 2>&1; do sleep 1; done

	grep -v '^#' "$HERE/sitl_setup.params" | while read -r name value; do
		[ -n "$name" ] && "$BUILD/bin/px4-param" set "$name" "$value" > /dev/null 2>&1
	done
}

cd "$ROOTFS"

start_viz() {
	# Setpoint and flown path drawn into the Gazebo GUI, see px4_auto_tuning/gz_viz.py
	[ -n "$HEADLESS" ] || [ -n "$NO_VIZ" ] && return 0
	(
		set +e
		cd "$HERE" && . "$HERE/env.sh"
		exec python3 -m px4_auto_tuning.gz_viz --world "$PX4_GZ_WORLD" --model "${PX4_SIM_MODEL#gz_}_0"
	) > "$RUN_DIR/viz.log" 2>&1 &
	VIZ_PID=$!
}

if [ -t 0 ] && [ -t 1 ] && [ -z "$PX4_DAEMON" ]; then
	# Manual use: interactive console on the terminal, also recorded in
	# run/px4_console.log so that an unexpected exit leaves a trace
	apply_setup_params &
	start_viz
	set +e
	script -q -f -e -c "'$BUILD/bin/px4' '$BUILD/etc' -w '$ROOTFS'" "$RUN_DIR/px4_console.log"
	CODE=$?
	echo "PX4 exited with code $CODE at $(date +%T) (139: segfault, 134: abort, 0: shutdown)" | tee -a "$RUN_DIR/px4_console.log"
	exit $CODE
fi

"$BUILD/bin/px4" -d "$BUILD/etc" -w "$ROOTFS" > "$RUN_DIR/px4.log" 2>&1 &
PX4_PID=$!

until grep -q "Startup script returned successfully" "$RUN_DIR/px4.log"; do
	kill -0 $PX4_PID 2>/dev/null || { cat "$RUN_DIR/px4.log"; exit 1; }
	sleep 1
done

apply_setup_params

start_viz
echo "PX4 SITL ready (world: $PX4_GZ_WORLD, log: $RUN_DIR/px4.log)"
tail -n +1 -f "$RUN_DIR/px4.log" &
TAIL_PID=$!
wait $PX4_PID

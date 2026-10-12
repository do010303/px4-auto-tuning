#!/bin/bash
# Put the parameters the experiments change back to their defaults, without
# restarting the SITL: controller gains (MPC_*, MC_*), the simulated GPS
# (SIM_GZ_GPS_*, SIM_GPS_USED) and the EKF2 GPS settings (EKF2_GPS_*, EKF2_REQ_*).
# Boot settings (SENS_GPS0_DELAY, NAV_DLL_ACT, COM_RCL_EXCEPT, model rotor
# geometry) are left alone. Use between scenarios of one long session.
#
#   ./reset_params.sh                 # the default set
#   ./reset_params.sh 'MPC_XY_*'      # only these (wildcard at the end allowed)

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARAM="$HERE/../../build/px4_sitl_default/bin/px4-param"

if [ $# -eq 0 ]; then
	set -- 'MPC_*' 'MC_*' 'SIM_GZ_GPS_*' SIM_GPS_USED 'EKF2_GPS_*' 'EKF2_REQ_*'
fi

"$PARAM" reset "$@" || { echo "no PX4 answered: is ./run_sitl.sh running?" >&2; exit 1; }
echo "reset to defaults: $*"
"$PARAM" show -c

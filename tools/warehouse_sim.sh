#!/bin/bash
# Isolated simulation; never starts micro-ROS or sends commands to real cars.
set -e
WS=$(cd "$(dirname "$0")/.." && pwd)
source "$WS/tools/setup_real_env.bash"
export ROS_DOMAIN_ID=88 GZ_PARTITION=warehouse_queue_sim
# LOCALHOST discovery in rmw_fastrtps adds SHM even when the XML disables
# builtin transports. Stale SHM ports can then block create_node forever.
# Keep the explicit XML's UDP-only 127.0.0.1 transport; SUBNET here does not
# expose traffic to the LAN because the interface whitelist is loopback only.
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
export QT_LOGGING_RULES='*.debug=false;qt.qpa.*=false'
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec python3 "$WS/tools/warehouse_launch.py" "$@"

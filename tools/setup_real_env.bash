# Source this file from each terminal used for real-robot experiments.
# It configures this terminal only and does not start nodes or move robots.
_agv_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$HOME/microros_ws/install/setup.bash"
source "$_agv_workspace/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export FASTRTPS_DEFAULT_PROFILES_FILE="$_agv_workspace/tools/fastdds_localhost.xml"
unset _agv_workspace

#!/bin/bash
# Mo phong HAI XE tren sa ban (xe ao + Nav2 that cho tung xe + dieu phoi).
# Chay tren ROS_DOMAIN_ID rieng nen khong dung toi xe that.
#
#   tools/fleet_sim.sh up      # bat map_server, 2 xe ao, 2 Nav2, 2 fleet_agent
#   ROS_DOMAIN_ID=87 python3 traffic_manager.py agv1 agv2
#   tools/fleet_sim.sh down    # tat het
#
# Vi tri dau: agv1 o (hang 2, cot 3), agv2 o (hang 3, cot 3), deu quay Dong.
# Doi bang bien moi truong: AGV1="2 3 0"  AGV2="3 3 0"  (hang cot huong_do).
WS=$(cd "$(dirname "$0")/.." && pwd)
L=${LOGDIR:-/tmp/fleet_sim}
CFG=$WS/src/my_bot_bringup/config
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-87} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# DDS chi dung UDP, bo SHM (cong /dev/shm bi khoa lam Nav2 mot xe vo hinh - xem fleet_real.sh)
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
# Goi DDS <= 1400 byte, chi unicast 127.0.0.1: WSL mirrored lam roi goi UDP > 1472 B tren
# loopback -> node khong thay nhau (tools/fastdds_localhost.xml)
export FASTRTPS_DEFAULT_PROFILES_FILE=$WS/tools/fastdds_localhost.xml
NAMES=(agv1 agv2)

down() {
  if [ -f $L/pids ]; then
    local stopped=()
    for p in $(cat "$L/pids"); do
      # PID cu co the da duoc dung lai. Chi dung nhom tien trinh cua domain nay.
      if [[ "$p" =~ ^[0-9]+$ ]] && [ -r "/proc/$p/environ" ] &&
          tr '\0' '\n' < "/proc/$p/environ" | grep -Fxq "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"; then
        kill -INT -"$p" 2>/dev/null
        stopped+=("$p")
      fi
    done
    sleep 4
    for p in "${stopped[@]}"; do
      if [ -r "/proc/$p/environ" ] &&
          tr '\0' '\n' < "/proc/$p/environ" | grep -Fxq "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"; then
        kill -9 -"$p" 2>/dev/null
      fi
    done
    rm -f $L/pids
  fi
}

if [ "${1:-}" = down ]; then
  down
  # Cua so Gazebo (tools/fleet_gazebo.py) - chi tat khi "down", "up" lai thi giu
  pkill -INT -f "^python3 (\S*/)?tools/fleet_gazebo\.py" 2>/dev/null && sleep 3
  pkill -9 -f "gz sim.*fleet_world|gz topic -e -t /world/fleet/|parameter_bridge /world/fleet/" 2>/dev/null
  echo "da tat"; exit 0
fi
down
rm -rf $L; mkdir -p $L

# Tham so Nav2 RIENG tung xe: /scan -> /agv1/scan... (ten DAY DU: lop vat can
# cua costmap tao subscription tu node /agv1/local_costmap, ten tuong doi
# "scan" thanh /agv1/local_costmap/scan - khong ai phat, costmap mu). Costmap
# tinh doc /map chung.
# inf_is_valid: LiDAR ao khong co tuong - tia khong cham gi la inf. Mac dinh
# costmap bo tia inf, khong dung no de XOA -> vet xe kia di qua nam lai mai
# tren costmap ("bong ma") chan ca dich lan duong. Xe that tia cham tuong nen
# khong can.
for n in "${NAMES[@]}"; do
  sed -e "s#odom_topic: /odom#odom_topic: /$n/odom#" \
      -e "s#topic: /scan#topic: /$n/scan#" -e "s#topic: \"/scan\"#topic: \"/$n/scan\"#" \
      -e "s#\"/local_costmap/published_footprint\"#\"/$n/local_costmap/published_footprint\"#" \
      -e 's#^\( *\)plugin: "nav2_costmap_2d::StaticLayer"#&\n\1map_topic: /map#' \
      -e 's#^\( *\)data_type: "LaserScan"#&\n\1inf_is_valid: True#' \
      "$CFG/navigation.yaml" > $L/navigation_$n.yaml
done

# Cho phep hieu chinh diem cat lenh xoay RIENG cua bo chap hanh ao khi co
# FAKE_DELAY. Nguong hoan thanh done_deg van giu 2.5 do nhu xe that.
SIM_ROT_STOP_DEG=${SIM_ROT_STOP_DEG:-0.6}
sed -e "s/^\( *stop_deg:\).*/\1 $SIM_ROT_STOP_DEG/" \
    "$CFG/grid_real.yaml" > $L/grid_fleet.yaml

pose() {   # "hang cot huong_do" -> "x y huong_do"
  python3 - "$@" <<EOF
import sys; sys.path.insert(0, "$WS")
from grid_board import Board, config_path
b = Board.load(config_path(True))
r, c, h = int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])
x, y = b.xy(r - 1, c - 1)
import math
print(f"{x:.3f} {y:.3f} {h + math.degrees(b.yaw):.1f}")
EOF
}
START=("${AGV1:-2 3 0}" "${AGV2:-3 3 0}")

PIDS=()
setsid ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=$CFG/real_room_map.yaml > $L/map.log 2>&1 & PIDS+=($!)
setsid ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lifecycle_manager_map \
  -p node_names:="['map_server']" -p autostart:=true > $L/lm_map.log 2>&1 & PIDS+=($!)

for i in 0 1; do
  n=${NAMES[$i]}; o=${NAMES[$((1 - i))]}
  read -r X Y H <<< "$(pose ${START[$i]})"
  TFR="-r /tf:=/$n/tf -r /tf_static:=/$n/tf_static"
  setsid python3 $WS/tools/fleet_fake_robot.py $X $Y $H --other /$o/odom \
    --ros-args -r __ns:=/$n $TFR > $L/fake_$n.log 2>&1 & PIDS+=($!)
done
echo "${PIDS[@]}" > $L/pids

# Nav2 tung xe MOT, khong bat song song: hai bo cung khoi dong thi co luc mot
# tra loi dich vu lifecycle bi mat ("failed to send response ... timeout") va
# lifecycle_manager cho mai. Khong len sau 120 s (goi DDS 1400 B: Nav2 len ~40 s) thi tat, bat lai (toi da 3 lan).
for n in "${NAMES[@]}"; do
  for try in 1 2 3; do
    echo -n "Nav2 $n (lan $try)"
    setsid ros2 launch $WS/tools/fleet_nav.launch.py namespace:=$n \
      params_file:=$L/navigation_$n.yaml > $L/nav_$n.log 2>&1 &
    NP=$!
    for k in $(seq 1 120); do
      grep -q "Managed nodes are active" $L/nav_$n.log && break
      echo -n "."; sleep 1
    done
    if grep -q "Managed nodes are active" $L/nav_$n.log; then
      echo " san sang"; PIDS+=($NP); break
    fi
    echo " KHONG LEN - bat lai"
    kill -INT -$NP 2>/dev/null; sleep 4; kill -9 -$NP 2>/dev/null
  done
  echo "${PIDS[@]}" > $L/pids
done

for n in "${NAMES[@]}"; do
  setsid python3 $WS/fleet_agent.py $n --grid $L/grid_fleet.yaml --ros-args -r __ns:=/$n \
    -r /tf:=/$n/tf -r /tf_static:=/$n/tf_static > $L/agent_$n.log 2>&1 & PIDS+=($!)
done
echo "${PIDS[@]}" > $L/pids
# Gop TF cac xe vao /tf chung cho RViz (tools/fleet.rviz): frame agvN/..., scan /fleet_viz/agvN/scan
setsid python3 $WS/tools/fleet_viz.py ${NAMES[@]} > $L/fleet_viz.log 2>&1 & PIDS+=($!)
echo "${PIDS[@]}" > $L/pids
echo "Da bat. Log: $L. Dieu phoi: ROS_DOMAIN_ID=$ROS_DOMAIN_ID python3 traffic_manager.py agv1 agv2"

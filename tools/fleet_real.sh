#!/bin/bash
# Chay HAI XE THAT tren sa ban (giong tools/fleet_sim.sh nhung xe that).
#
#   tools/fleet_real.sh up      # agent, ban do, moi xe: TF/odom/AMCL/Nav2/fleet_agent
#   python3 traffic_manager.py agv1 agv2
#   tools/fleet_real.sh down    # tat het
#
# Moi xe nap firmware voi #define XE 1 / XE 2 -> micro-ROS dat topic cua xe
# vao /agv1/... va /agv2/... (node /agvN/esp32_agv_base_N), dung chung agent.
#
# Mac dinh xe THAT tu dinh vi tu LiDAR + /map khi dung yen, khong can AGV1/AGV2.
# Neu khong khop chac chan, script dung truoc khi khoi dong Nav2/fleet_agent.
# AUTO_LOCALIZE=0 chi dung khi thu co chu y voi vi tri AMCL da biet:
#   AUTO_LOCALIZE=0 AGV1="2 3 0" AGV2="3 3 0" tools/fleet_real.sh up
# Hieu chinh RIENG tung xe (mac dinh = so do tren xe 1):
#   WHEEL_R_agv2=0.0330 tools/fleet_real.sh up
#   WHEEL_R_*: ban kinh lan banh (m, wheel_odometry).
# CAM=0: khong bat tools/esp32_cam_node.py (anh ESP32-CAM cho cargo_task.py).
# Da co agent chay rieng (vd bringup_real.launch.py) thi tu dung chung; AGENT=0
# thi khong bat agent (thu voi ESP32 gia: tools/fleet_fake_robot.py --esp32,
# them EMULATE=1: LiDAR gia khong co tuong -> inf_is_valid).
WS=$(cd "$(dirname "$0")/.." && pwd)
L=${LOGDIR:-/tmp/fleet_real}   # tools/agvctl dat LOGDIR vao ~/agv_logs/run-*/fleet
CFG=$WS/src/my_bot_bringup/config
source /opt/ros/jazzy/setup.bash
[ -f ~/microros_ws/install/setup.bash ] && source ~/microros_ws/install/setup.bash
source "$WS/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# DDS chi dung UDP (localhost), BO bo nho chung SHM: cong /dev/shm/fastrtps_port*
# bi khoa ("RTPS_TRANSPORT_SHM ... open_and_lock_file failed") lam ca container Nav2
# cua mot xe VO HINH (2026-10-10: xe 2 khong co costmap, khong follow_path) ma tien
# trinh van song. Luu luong o day nho, UDP thua suc.
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
# Goi DDS <= 1400 byte, chi unicast 127.0.0.1: WSL mirrored lam roi goi UDP > 1472 B tren
# loopback -> node khong thay nhau (tools/fastdds_localhost.xml)
export FASTRTPS_DEFAULT_PROFILES_FILE=$WS/tools/fastdds_localhost.xml
NAMES=(${ROBOTS:-agv1 agv2})

down() {
  if [ -f $L/pids ]; then
    for p in $(cat $L/pids); do kill -INT -"$p" 2>/dev/null; done
    sleep 4
    for p in $(cat $L/pids); do kill -9 -"$p" 2>/dev/null; done
    rm -f $L/pids
  fi
  pkill -9 -f "^python3 $WS/(fleet_agent|tools/fleet_viz|tools/esp32_cam_node)\.py" 2>/dev/null
  pkill -9 -f "fleet_(nav|real_robot)\.launch\.py" 2>/dev/null
  # Tien trinh bi kill -9 de lai cong bo nho chung (/dev/shm/fastrtps_*) bi khoa.
  # Container Nav2 bat sau ma vuong cong do thi KHONG ai do thay no (da gap:
  # "RTPS_TRANSPORT_SHM ... open_and_lock_file failed", fleet_agent bao "khong
  # thay Nav2"). Lenh nay chi xoa vung khong con tien trinh nao dung.
  fastdds shm clean >/dev/null 2>&1 || true
}

if [ "${1:-}" = down ]; then
  down
  # Lenh cuoi cho ca hai xe: dung han (ESP32 tu dung sau 0.5 s neu mat lenh)
  for n in "${NAMES[@]}"; do
    timeout 3 ros2 topic pub --once /$n/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 &
  done
  wait
  echo "da tat"; exit 0
fi
if [ "${1:-}" != up ]; then
  sed -n '2,22p' "$0"; exit 1
fi
AUTO_LOCALIZE=${AUTO_LOCALIZE:-1}
if [ "$AUTO_LOCALIZE" != 1 ] && [ "$AUTO_LOCALIZE" != 0 ]; then
  echo "AUTO_LOCALIZE phai la 1 (tu dinh vi) hoac 0 (nhap vi tri thu cong)." >&2
  exit 2
fi
down
rm -rf $L; mkdir -p $L

# Tham so Nav2 RIENG tung xe: topic cua xe nam trong /agvN (ten DAY DU - xem
# giai thich trong fleet_sim.sh). Costmap tinh doc /map chung. Khac mo phong:
# KHONG inf_is_valid (xe that tia cham tuong, tia khong ve la tia hong).
for n in "${NAMES[@]}"; do
  sed -e "s#odom_topic: /odom#odom_topic: /$n/odom#" \
      -e "s#topic: /scan#topic: /$n/scan_filtered#" -e "s#topic: \"/scan\"#topic: \"/$n/scan_filtered\"#" \
      -e "s#\"/local_costmap/published_footprint\"#\"/$n/local_costmap/published_footprint\"#" \
      -e 's#^\( *\)plugin: "nav2_costmap_2d::StaticLayer"#&\n\1map_topic: /map#' \
      "$CFG/navigation.yaml" > $L/navigation_$n.yaml
  # EMULATE=1: thu voi ESP32 GIA (LiDAR gia khong co tuong) - giong mo phong
  [ "${EMULATE:-0}" = 1 ] && sed -i 's#^\( *\)data_type: "LaserScan"#&\n\1inf_is_valid: True#' $L/navigation_$n.yaml
done
# Xoay mot luot khong con dung mo hinh coast cu. Dung chung cau hinh
# rotate trong grid_real.yaml; giu hieu chinh ban kinh banh rieng tung xe.
declare -A GRID WHEEL_R
for n in "${NAMES[@]}"; do
  w="WHEEL_R_$n"; c="COAST_$n"
  WHEEL_R[$n]=${!w:-0.03281}
  GRID[$n]=$CFG/grid_real.yaml
  if [ -n "${!c:-}" ]; then
    echo "Chu y: $c khong con duoc dung trong bo xoay mot luot."
  fi
  echo "Xe $n: wheel_radius ${WHEEL_R[$n]} m, xoay mot luot (grid_real.yaml)"
done

pose() {   # "hang cot huong_do" -> "x y yaw_rad"
  python3 - "$@" <<EOF
import sys, math; sys.path.insert(0, "$WS")
from grid_board import Board, config_path
b = Board.load(config_path(True))
r, c, h = int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])
x, y = b.xy(r - 1, c - 1)
print(f"{x:.3f} {y:.3f} {math.radians(h) + b.yaw:.4f}")
EOF
}
declare -A START=([agv1]="${AGV1:-2 3 0}" [agv2]="${AGV2:-3 3 0}")

PIDS=()
if [ "${AGENT:-1}" = 0 ]; then
  echo "AGENT=0: khong bat micro_ros_agent"
elif pgrep -f "micro_ros_agent udp4" >/dev/null; then
  echo "micro_ros_agent da chay san - dung chung"
else
  setsid ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 > $L/agent.log 2>&1 & PIDS+=($!)
fi
setsid ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=$CFG/real_room_map.yaml > $L/map.log 2>&1 & PIDS+=($!)
setsid ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lifecycle_manager_map \
  -p node_names:="['map_server']" -p autostart:=true > $L/lm_map.log 2>&1 & PIDS+=($!)
echo "${PIDS[@]}" > $L/pids

# Cho ESP32 cua tung xe len (topic /agvN/scan co nguoi phat)
for n in "${NAMES[@]}"; do
  echo -n "Cho xe $n (/$n/scan)"
  for k in $(seq 1 60); do
    ros2 topic info /$n/scan 2>/dev/null | grep -q "Publisher count: [1-9]" && break
    echo -n "."; sleep 1
  done
  if ros2 topic info /$n/scan 2>/dev/null | grep -q "Publisher count: [1-9]"; then
    echo " co"
  else
    echo " KHONG THAY - xe $n da bat nguon chua? da nap #define XE ${n#agv} chua?"
  fi
done

# Phan laptop cua tung xe: TF, odometry, AMCL. Tu dinh vi khong nap vi tri doan san.
for n in "${NAMES[@]}"; do
  if [ "$AUTO_LOCALIZE" = 1 ]; then
    echo "Xe $n: cho tu dinh vi tu LiDAR + ban do"
    setsid ros2 launch $WS/tools/fleet_real_robot.launch.py ns:=$n auto_localize:=true \
      wheel_radius:=${WHEEL_R[$n]} > $L/robot_$n.log 2>&1 & PIDS+=($!)
  else
    read -r X Y YAW <<< "$(pose ${START[$n]})"
    echo "Xe $n dat tai (${START[$n]}) -> map ($X, $Y, $YAW rad)"
    setsid ros2 launch $WS/tools/fleet_real_robot.launch.py ns:=$n x:=$X y:=$Y yaw:=$YAW \
      wheel_radius:=${WHEEL_R[$n]} > $L/robot_$n.log 2>&1 & PIDS+=($!)
  fi
done
echo "${PIDS[@]}" > $L/pids

if [ "$AUTO_LOCALIZE" = 1 ]; then
  for n in "${NAMES[@]}"; do
    echo "Dang tu dinh vi $n (xe dung yen)..."
    if ! python3 -u $WS/tools/auto_initial_pose.py --ns "$n" > "$L/localize_$n.log" 2>&1; then
      cat "$L/localize_$n.log"
      echo "KHONG XAC DINH DUOC VI TRI $n; khong bat Nav2 hoac giao nhiem vu."
      down
      exit 1
    fi
    cat "$L/localize_$n.log"
  done
fi

# Nav2 tung xe MOT (bat song song hay mat tra loi lifecycle - xem fleet_sim.sh)
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
  setsid python3 $WS/fleet_agent.py $n --grid ${GRID[$n]} --ros-args -r __ns:=/$n \
    -r /tf:=/$n/tf -r /tf_static:=/$n/tf_static > $L/agent_$n.log 2>&1 & PIDS+=($!)
done
echo "${PIDS[@]}" > $L/pids
# Gop TF cac xe vao /tf chung cho RViz (tools/fleet.rviz): frame agvN/..., scan /fleet_viz/agvN/scan
SCAN_REMAPS=()
for n in "${NAMES[@]}"; do SCAN_REMAPS+=(-r /$n/scan:=/$n/scan_filtered); done
setsid python3 $WS/tools/fleet_viz.py ${NAMES[@]} --ros-args "${SCAN_REMAPS[@]}" > $L/fleet_viz.log 2>&1 & PIDS+=($!)
echo "${PIDS[@]}" > $L/pids
# ESP32-CAM tung xe -> /agvN/camera/image_raw (cargo.yaml cameras:). CAM=0: khong bat.
# Camera chua bat nguon thi node tu thu lai moi 2 s - khong can bat lai he.
if [ "${CAM:-1}" = 1 ] && [ "${EMULATE:-0}" != 1 ]; then
  setsid python3 $WS/tools/esp32_cam_node.py ${NAMES[@]} > $L/camera.log 2>&1 & PIDS+=($!)
  echo "${PIDS[@]}" > $L/pids
  echo "Camera: log $L/camera.log"
fi
echo "Da bat. Log: $L. Dieu phoi: python3 traffic_manager.py ${NAMES[*]}"

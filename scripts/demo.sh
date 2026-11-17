#!/bin/bash
# One-command demo: sim (+ Gazebo window if GUI=true) + MoveIt + the full MASTER loop, then shuts everything down.
#   GUI=true scripts/demo.sh          # with the Gazebo window (run from a system terminal)
#   scripts/demo.sh                   # headless; picture of the end state in $OUT (default /tmp/demo)
source /opt/ros/jazzy/setup.bash; source ~/kinova-dev/ros_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-78} GZ_IP=127.0.0.1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=demo_$$
OUT=${OUT:-/tmp/demo}; mkdir -p "$OUT"
if pgrep -x gz >/dev/null; then echo "A Gazebo is already running: close it first (pkill -x gz) and rerun."; exit 1; fi
cleanup() { kill -INT -$P2 -$P1 2>/dev/null; sleep 5; kill -9 -$P2 -$P1 2>/dev/null; }
trap cleanup EXIT
setsid ros2 launch gr_blocks_ros gen3_2f140_sim.launch.py rigid_grasp:=true gui:=${GUI:-false} > "$OUT/sim.log" 2>&1 &
P1=$!
echo "Starting simulation (logs in $OUT)..."; sleep 35
setsid ros2 launch gr_blocks_ros gen3_2f140_sim_moveit.launch.py > "$OUT/moveit.log" 2>&1 &
P2=$!
echo "Waiting for MoveIt (/move_action)..."
for i in $(seq 1 90); do ros2 action list 2>/dev/null | grep -q '^/move_action$' && break; sleep 2; done
ros2 action list 2>/dev/null | grep -q '^/move_action$' || { echo "MoveIt never came up; see $OUT/moveit.log and $OUT/sim.log"; exit 1; }
sleep 15
echo "Running the full loop..."
ros2 run gr_blocks_ros full_loop 1 2>&1 | tee "$OUT/full_loop.txt"
ros2 run gr_blocks_ros save_image /view_cam/image "$OUT/final.png" >/dev/null 2>&1
echo "Done. Final picture: $OUT/final.png"
[ "$GUI" = "true" ] && { echo "Window stays open 60 s for viewing..."; sleep 60; }

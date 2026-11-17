#!/bin/bash
# usage: scripts/run_exp.sh OUTDIR TAG "SIM_LAUNCH_ARGS" <ros2 run gr_blocks_ros executable> [args...]
# Fresh headless sim + MoveIt, run one command, shut everything down. Logs and the end image go to OUTDIR.
# example: scripts/run_exp.sh /tmp/out full "rigid_grasp:=true" full_loop 1
source /opt/ros/jazzy/setup.bash; source ~/kinova-dev/ros_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-78} GZ_IP=127.0.0.1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
D=$1; T=$2; SIMARGS=$3; shift 3
mkdir -p "$D"
export GZ_PARTITION=grblocks_${T}_$$   # unique per run: no crosstalk with a lingering server
for i in $(seq 1 30); do pgrep -x gz >/dev/null || break; sleep 1; done
setsid ros2 launch gr_blocks_ros gen3_2f140_sim.launch.py $SIMARGS > "$D/${T}_sim.log" 2>&1 &
P1=$!
sleep 35
setsid ros2 launch gr_blocks_ros gen3_2f140_sim_moveit.launch.py > "$D/${T}_mg.log" 2>&1 &
P2=$!
sleep 20
timeout "${EXP_TIMEOUT:-600}" ros2 run gr_blocks_ros "$@" > "$D/${T}_out.txt" 2>&1
ros2 run gr_blocks_ros save_image /view_cam/image "$D/${T}_after.png" >> "$D/${T}_out.txt" 2>&1
kill -INT -$P2 -$P1; sleep 6; kill -9 -$P2 -$P1 2>/dev/null
echo done > "$D/${T}_done.txt"

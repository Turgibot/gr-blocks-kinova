# Demo guide: what works so far (simulation only)

Everything below runs in Gazebo with the Gen3 + Robotiq 2F-140. Nothing here touches a real arm.
Total time for the full demo: about 15 minutes. Each scenario needs a **freshly launched sim**.

## Quickest way: one command

```
GUI=true scripts/demo.sh      # with the Gazebo window, from a system terminal (GUI + weld not tried yet)
scripts/demo.sh               # headless; picture in /tmp/demo/final.png
```
It starts the sim and MoveIt, waits for `/move_action`, runs `full_loop`, then shuts everything down (headless run tested: FULL_LOOP PASS, 104 s). `action server missing` from `full_loop` means MoveIt/sim were not running in that terminal's ROS_DOMAIN_ID; `full_loop` alone only works while both launch files are up.

## 0. One-time setup

```
cd ~/kinova-dev/ros_ws && colcon build --packages-select gr_blocks_ros --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```
In every terminal:
```
source /opt/ros/jazzy/setup.bash; source ~/kinova-dev/ros_ws/install/setup.bash
export ROS_DOMAIN_ID=78 GZ_PARTITION=demo GZ_IP=127.0.0.1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```
Open the terminals from a **system terminal** (not VS Code's snap terminal: it crashes the Gazebo GUI).

## 1. Symbolic layer (no sim, 5 s)

```
python3 -m gr_blocks_ros.blocksworld
```
Shows R_ed's 16 cached MASTER actions, the joint helper plan from t_join = 6, every state matching the cache, and an illegal
action being refused. Ends with `SELFTEST PASS`.

## 2. The simulated lab (with the Gazebo window)

Terminal A (sim, GUI):
```
ros2 launch gr_blocks_ros gen3_2f140_sim.launch.py rigid_grasp:=true gui:=true
```
Terminal B (MoveIt + cube tracking):
```
ros2 launch gr_blocks_ros gen3_2f140_sim_moveit.launch.py          # add launch_rviz:=true to also show RViz
```
Wait ~1 minute until the arm sits at Home above three towers of lettered cubes.
Point out: the table extended under the arm, 12 cubes (letters on the faces), the 2F-140 gripper.
Note: **the GUI together with `rigid_grasp:=true` has not been tried yet**; if it misbehaves use the headless route in section 6
and look at the saved images.

## 3. Scenario: pick-and-place and the table (about 1.5 min)

Terminal C:
```
ros2 run gr_blocks_ros table_test
```
The arm puts cubes down on table slots, picks one up from the table and stacks it, with a physical-vs-symbolic check after each
action. Expected: ends with a pass message and no stack scattered. (Passed once so far.)

Restart the sim (Ctrl+C in A and B, wait for Gazebo to exit, relaunch both) before the next scenario.

## 4. Scenario: R_ing helps from the t_join world (about 1 min)

Launch the sim with the matching world:
```
ros2 launch gr_blocks_ros gen3_2f140_sim.launch.py rigid_grasp:=true gui:=true world:=blocks_world_tjoin.sdf
ros2 launch gr_blocks_ros gen3_2f140_sim_moveit.launch.py
ros2 run gr_blocks_ros replay_helper
```
R_ing performs `unstack W B, stack W L, pick-up A, stack A S`. The final towers equal the cached world. (2/2 passes.)

## 5. The full MASTER scenario (about 90 s) - the main demo

Fresh sim as in section 2, then:
```
ros2 run gr_blocks_ros full_loop 1
```
What you will see and what to say:
1. The scripted human (R_ed) plays its first 6 actions; each cube it "holds" is parked at the table corner. The printed belief over
   goal words updates after each action.
2. At the cached join time t_join = 6 the arm starts: it performs exactly the helper plan `pi_ring`.
3. R_ed finishes (`unstack M O`, `stack M A`) and the tower **R-E-T-S-A-M** stands. The script prints success.
(2/2 passes.)

## 6. Headless route with saved evidence

```
scripts/run_exp.sh /tmp/demo full "rigid_grasp:=true" full_loop 1
```
Logs, the printed output (`full_out.txt`) and a picture of the final scene (`full_after.png`) land in `/tmp/demo`.
Repeatability data: `docs/results/*.csv`; pictures: `docs/img/`.

## 7. Be honest about these limits when presenting

- The grasp is a **rigid weld** in the sim (the physical 2F-140 grasp does not hold in Gazebo). Real-arm grasping is not rehearsed.
- Recognition is **read from a cache** and join time is the cached value, not computed live. Only the word MASTER was run.
- The human only teleports cubes; there is no visible hand.
- Results: 2 of 2 passes for the replay and the full loop, 1 of 1 for `table_test`; placement error mean 1.7 mm, max 6 mm over 12
  cycles. Flaky spots are listed in `docs/PHASE3_NOTES.md`.

## If something goes wrong

- Arm does not move: MoveIt started before the sim finished; restart B after A shows the arm.
- Cubes toppled at the start: relaunch the sim (scenarios reset the layout, but a lingering old Gazebo breaks it).
- Old Gazebo still running: wait for `pgrep -x gz` to print nothing, or use a different `GZ_PARTITION`/`ROS_DOMAIN_ID`.

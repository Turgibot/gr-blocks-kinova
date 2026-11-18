# Handoff for the next session (written 2026-11-18, end of session 3)

Read in this order: `CLAUDE.md` (standing rules), this file, then `docs/HANDOFF_SESSION3.md` (the old pipeline and its lessons).
Branch: `feature/grasp-and-blocksworld` (pushed to GitHub `origin`). Nothing has run on the real arm.

## The goal (unchanged)

Freedman & Zilberstein (AAAI-17) on a Kinova Gen3 7-DoF + Robotiq 2F-140, first in Gazebo. The Gen3 is **R_ing**
(recognizes and helps); a **human is R_ed** and builds a word (first: **MASTER**) from 12 lettered cubes. The symbolic side
(recognition, plans) is reused from `Turgibot/GR-Blocks`, not re-derived.

## What happened this session

1. Ran the old demo (`scripts/demo.sh`, GUI). Two problems:
   - **Stale gz server**: an orphaned `gz sim` from a previous run (process name `ruby`, so `pgrep -x gz` misses it) answered
     as a second `/controller_manager` and caused "no TF between end_effector_link and finger pads". Kill leftovers before
     starting (`ps aux | grep "gz sim"`). Proposed demo.sh fixes (wait-for-ready instead of fixed sleeps, better pgrep,
     cleanup) were **not** made.
   - **Gripper is open while lifting cubes.** Diagnosed with `scripts/dev/pick_debug.py`: the cause is the Gazebo
     DetachableJoint **weld**, which physically pushes the fingers open (fully open after a stalled close; e.g. 17.5 s stall,
     finger went to 0.077). The ease-off in `skills.py:347` is not the cause. The grasp is also too low
     (`executor.py` `PICK_DZ = 0.015`). Not fixed.
2. The user decided: **start over with new files and new scripts**, small steps, watching the GUI (do NOT run headless).
   `skills.py` has the user's own uncommitted debug prints (now committed as-is); the old pipeline is left untouched.

## The new start (new files, all in `gr_blocks_ros/`)

| Step | Done | File |
|---|---|---|
| 1. Arm at Home when the sim starts | yes, seen in GUI | `launch/sim_home.launch.py` (SRDF Home -> xacro `initial_positions_file`) |
| 2. Table longer in +x (gripper direction) | yes | `worlds/blocks_world_long.sdf` (table x -0.25..1.25, 1.0 wide) |
| 3. Cubes 1 m from the base | yes | same world: 3 towers at x = 1.00, y = 0.12 / 0 / -0.12 |
| 4. Gazebo GUI start view | yes | launch arg `view` -> copied gui.config with `<camera_pose>` |
| 5. Wrist camera on | yes (`vision:=true`) | launch bridges `/wrist_mounted_camera/image` and `/camera_info` |
| 6. Observe pose (camera sees all cubes) | pose found by the user in the GUI | `config/poses.yaml` `observe` |
| 7. Move to a named pose | written, built, **not yet run in sim** | `gr_blocks_ros/go_pose.py` |

`sim_home.launch.py`: Gazebo (bullet-featherstone, needed for the 2F-140 mimic joints; DART fails with "mimic constraint"),
spawn at z = 0.30, controllers chained (joint_state_broadcaster -> joint_trajectory_controller -> robotiq_gripper_controller).
No MoveIt, no weld plugins, no scripts.

**Observe pose** `[0.0, 0.45, 3.14, -2.50, 0.0, 0.96, 1.57]`: camera at about (0.43, 0, 0.33) m in base_link, 24 deg down;
the 12 cubes project to u 183-464, v 147-312 of the 640x480 image (~40 px per cube), lowest link 16 cm above the table.
Room around the towers is left for cubes put on the table during word assembly.

## How to run (lab machine; GUI, from a system terminal, not the VS Code snap terminal)

```
cd ~/kinova-dev/ros_ws && colcon build --packages-select gr_blocks_ros && source install/setup.bash
export ROS_DOMAIN_ID=78 GZ_PARTITION=home GZ_IP=127.0.0.1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

ros2 launch gr_blocks_ros sim_home.launch.py                 # [view:="x y z r p y"] [world:=...]
python3 -m gr_blocks_ros.go_pose observe 3                    # or: home 5
ros2 run rqt_image_view rqt_image_view /wrist_mounted_camera/image
# set joints directly (no collision check):
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
 "{joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7], points: [{positions: [0.0, 0.45, 3.14, -2.5, 0.0, 0.96, 1.57], time_from_start: {sec: 3}}]}"
# read the current GUI view (to set a new default `view`):
gz topic -e -n 1 -t /gui/camera/pose          # needs the same GZ_PARTITION as the sim
```

## Next steps (to be decided with the user, one at a time)

1. Run `go_pose observe` in the sim and check the wrist image in rqt (expected: all 12 cubes, central region).
2. Continue the loop from `observe`: likely perceive cubes (letters/positions) from the wrist image, then act.
3. Open questions, not decided:
   - **Placement area** for cubes put on the table during assembly. `blocksworld.py` SLOTS (P1-P5 at x 0.45, Q1-Q3 at
     x 0.65) are stale for the new layout. Options offered: two rows next to the towers, one long row, or custom.
   - The camera must also see that placement area from `observe`.
   - **Grasp redesign**: weld only at contact / no weld fight, higher grasp point than `PICK_DZ = 0.015`.
   - `go_pose` has no `setup.py` entry point (would need an edit to an existing file).

## Dev tools (copied from the session scratchpad into `scripts/dev/`)

- `pick_debug.py`: step-by-step replica of `Arm.pick` with pauses and finger/weld logging (`--ease --no-weld --touch --dz --no-pause`).
  Needs the old launch (`rigid_grasp:=true`) + MoveIt.
- `finger_log.py`: logs `finger_joint` and `/held_block`.
- `find_view.py URDF`: offline FK + IK + pinhole projection search for camera poses that see all cubes
  (generate the URDF with xacro, `vision:=true`; `TOWER_X` in the script is 0.80, set it to 1.00 for the current world).

## Rules that still hold

Small verified steps; additive changes (new files, ask before editing existing ones); never touch `install.sh` or
`~/kinova-dev/ros_ws/src/ros2_*`; commit/push only when asked; report outcomes faithfully. The user wants to **watch the GUI**.

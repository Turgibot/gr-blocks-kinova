# Phase 1-2 notes: Gazebo sim, MoveIt, first pick and place (Gen3 + Robotiq 2F-140)

> Continued and partly superseded by `docs/PHASE3_NOTES.md` (rigid grasp in the sim, block world, replay, full loop).

Written 2026-11-16 at the end of the first working session. Everything below was observed in headless Gazebo
(Jazzy, gz-sim 8, bullet-featherstone); nothing has run on the real arm.

## What runs

```
ros2 launch gr_blocks_ros gen3_2f140_sim.launch.py [gui:=true]      # Gazebo + controllers, blocks world
ros2 launch gr_blocks_ros gen3_2f140_sim_moveit.launch.py           # move_group + world_state (cube TF + planning scene)
ros2 run gr_blocks_ros move_named Home                              # SRDF pose through MoveGroup
ros2 run gr_blocks_ros pick_test L                                  # first pick
ros2 run gr_blocks_ros place_test L H                               # pick L, put it on H
```

Use an isolated `ROS_DOMAIN_ID` and `GZ_PARTITION` in every terminal (HANDOFF.md section 5). `gui:=true` must be started from
a system terminal, not VS Code's snap terminal.

## Findings that cost time (so nobody pays twice)

- **Kinova's sim launch cannot start the 2F-140 gripper controller:** its `ros2_controllers.yaml` names the 2F-85 joint.
  `config/ros2_controllers_2f140_sim.yaml` is that file with `joint: finger_joint`.
- **Two `Sensors` systems crash the Gazebo server** (duplicate Ogre2 materials). The Kortex xacro already loads one and
  defaults `gazebo_renderer` to `ogre` (Ogre 1); we pass `gazebo_renderer:=ogre2` and the generated world has no `Sensors` plugin.
- **The snap GUI crash kills the server** when the GUI runs from the VS Code terminal: use `gui:=false` (default) there.
- **ros_gz_bridge drops entity names from `Pose_V`.** `world_state` reads the pose stream with the `gz.transport13` Python
  bindings instead.
- **Frames:** Gazebo places the robot base at z = 0.30 (table top); MoveIt's `base_link` is its own origin, so `world_state`
  subtracts 0.30 from z. A planning-scene diff replaces the whole allowed-collision matrix, so `world_state` fetches it and
  extends it (table <-> base_link) instead of sending a new one.
- **`move_group` rejects some planned paths** in `ValidateSolution` (a pad would graze a cube); skills retry planning.
- **The gripper keeps holding only while the process that commanded it is alive.** Pick and place must run in one process.
- Pads are 30 x 70 x 7.5 mm with the 70 mm side vertical: a pad centred on a 45 mm cube also clamps the cube below.
  The grasp point is therefore 12.5 mm above the cube centre.
- Pad-frame distance is not the cube-contact distance: a 45 mm cube stalls the fingers at about 0.45 rad (pad frames
  about 63 mm apart), not at the 0.53 rad where the frames are 45 mm apart.

## Measured

Pad-frame gap vs `finger_joint` (`gripper_gap`): 0.00 rad 134.2 mm, 0.20 105.4, 0.40 69.1, 0.50 50.0, 0.60 30.4, 0.70 10.6.

Pick of the top cube L (close command 0.46, grasp point +12.5 mm): two runs, cube rose 97.9 and 98.5 mm, <1 mm sideways,
tilt 3.8-4.0 degrees, no other cube moved.

Place of L on H (`place_test`): final error (x, y, z) in mm: run 1 (2, 9, 0), run 2 (-3.3, 3.2, 0), tilt 0.

## Open problem: the grasp is too weak, and squeezing harder is worse

`release_test` (12 planned cycles, stopped after 6): the final horizontal error grew from 3.6 to 11.3 mm. Logging the
cube against the gripper showed the cube slid in y *while held*, before release (tcp-minus-cube offset +2.9 mm at the
target in cycle 0, +6.5 mm in cycle 4), so release (variants A/B/C: drop 2 mm / gradual open / drop 0.5 mm) is not the
main error and the three variants were not separated.

`creep_test`: holding a picked cube still for 15 s, the gripper stayed vertical (0.01 degrees) but the cube slid down
in the grip, 6.5 mm in 15 s and accelerating. The squeeze at command 0.46 (0.014 rad past contact) is weak.

`grip_sweep` (one cycle per value, so only indicative): close 0.46 -> slip 3 mm in 10 s, placement error 5-7 mm.
Close 0.50 -> slip 18 mm, placement error 67 mm. Close 0.54 -> the cube was pushed out of the fingers (190 mm).
Harder squeezing makes it worse, which suggests the pad contact behaves like a wedge (pads not parallel, or contact
parameters too soft) instead of a clamp. Not investigated further. Candidates: pad friction and contact stiffness
(`mu`, `kp`, `kd`, `min_depth` on the pad collisions), a different physics engine, a lower close speed, checking the
pad faces stay parallel while closing, or the fallback of attaching the cube with a Gazebo detachable joint (not used).

A fourth sweep value (0.58) failed on the first place with `START_STATE_INVALID` (a gripper joint was out of bounds after
the cube had been squeezed out); `move_group` also segfaulted at shutdown in one run. Neither was investigated.

## Not done yet

Placing on the table, picking cubes lower in a tower or from the table (pads would hit the table at the cube-centre
height), the table slot layout for loose blocks, the operator mapping and state tracker, the helper-plan replay.

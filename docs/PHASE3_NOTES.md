# Phases 2-3 notes: grasp decision, block world, replay and the full MASTER scenario (simulation)

Written 2026-11-17 after the second working session. Everything here was observed in headless Gazebo (Jazzy, gz-sim 8,
bullet-featherstone); nothing has run on the real arm. Continues `docs/PHASE2_NOTES.md`.

## Summary

| Step | Result |
|---|---|
| 1 Grasp | The physical 2F-140 grasp in Gazebo does not hold (tried: friction, contact stiffness, joint gain, DART). The sim now uses a **rigid grasp** (Gazebo `DetachableJoint` welds the cube to the gripper on grasp): zero slip, pick-and-place errors <= 2 mm. Sim-only; the real arm will use a real grasp. |
| 2 Repeatability | 12 pick-and-place cycles with the rigid grasp: mean horizontal error 1.7 mm, max 6.0 mm (below). |
| 3 Low picks / table | `table_test` passes: put-down on two table slots, pick-up from the table, stack on a table cube (1 run). Also `pick-up A` in the replay. |
| 4 Table slots | 8 slots (`blocksworld.SLOTS`): P1-P5 at x = 0.45, Q1-Q3 at x = 0.65. The MASTER run needs all 8 at once at t = 12. |
| 5 Operators + state tracker | `blocksworld.py`: operators of `data/actions.json` with preconditions, per-slot stacks, and a self-test that replays the cached MASTER trace and joint plan and compares every state (passes). |
| 6 Replay of the helper plan | `replay_helper` passes (2 runs): R_ing performs `unstack W B, stack W L, pick-up A, stack A S` from the t_join world, the physical stacks equal the symbolic ones after each action, final towers equal the cached world. |
| 7 Full loop | `full_loop` passes (2 runs): scripted R_ed (teleport) plays 6 actions with the cached belief printed live, R_ing joins at the cached t_join and performs exactly `pi_ring`, R_ed finishes, tower R-E-T-S-A-M stands, ~90 s. **Recognition is read from the cache**, and the join time is the cached t_join: neither is computed live. |

## Running it

```
ros2 launch gr_blocks_ros gen3_2f140_sim.launch.py rigid_grasp:=true [world:=blocks_world_tjoin.sdf] [gui:=true]
ros2 launch gr_blocks_ros gen3_2f140_sim_moveit.launch.py
ros2 run gr_blocks_ros full_loop 1        # whole scenario (tempo of R_ed's early actions in s)
ros2 run gr_blocks_ros replay_helper      # needs the t_join world above
ros2 run gr_blocks_ros table_test
ros2 run gr_blocks_ros release_test 12 L out.csv
python3 -m gr_blocks_ros.blocksworld      # symbolic self-test, no sim
```
Use an isolated `ROS_DOMAIN_ID` and `GZ_PARTITION`, start each scenario in a freshly launched sim (they begin by resetting the
layout and expect nobody to hold a cube), and let the previous Gazebo exit completely first.

## Step 1 in detail: what was tried for the physical grasp

All with the creep test (pick the top cube L, hold still 15 s, log how the cube slides in the grip); the baseline slides
about 0.5 mm/s downwards and was already known to fail repeatability (`PHASE2_NOTES.md`).

| Variant | Outcome |
|---|---|
| Finger/cube friction mu 3 | same creep (z 5.8 mm at 13 s), cube fell out at 14 s |
| mu 3 + contact kp 1e6, kd 100 | different pattern, 10 mm sideways slip |
| mu 10 | same creep (z 5.0 mm at 15 s): friction is not the limit |
| `position_proportional_gain` 1.0 (default 0.1) | cube wobbles, up to 15 mm in all directions |
| gain 5.0 | pick fails (cube rises 0 mm) |
| DART physics engine | fingers close to the commanded angle with nothing stopping them, nothing is lifted; Gazebo logs that the engine "does not support mimic constraints" |
| **Rigid weld** (DetachableJoint) | zero slip over 15 s; place errors below 2 mm; adopted |

Reading: the 2F-140's four-bar linkage is driven in Gazebo as independently position-controlled joints (the "mimic" joints), so the
pads do not stay parallel under load and the contact acts like a wedge. A physical fix would mean modelling the gripper
differently (a simplified parallel gripper for the sim, or a closed-loop linkage in an engine that supports it); not attempted.

How the rigid grasp works (`sim_patches.add_attach_plugins`, `skills.py`): one DetachableJoint per cube, parent link
`bracelet_link` (the gripper base is lumped into it). After the fingers close on the cube the arm sends the attach message, eases the
fingers off by 0.08 rad so the pads do not fight the weld, and lifts; before opening it sends the detach message.
The launch argument `rigid_grasp:=true` adds the plugins; the arm detects them from the attach topics (`GR_RIGID_GRASP=0/1` overrides).

**Quirk that needs a workaround:** the plugin starts attached. When the robot spawns, every cube is welded to the gripper and a
few mm of jolt (3-10 mm on 9 of 12 cubes within 60 ms; a control run without the plugins moves none) can topple stacks.
`release_cubes.py` detaches all cubes right after the spawn, and every scenario script starts with
`Executor.reset_layout()` (wait for stillness, stage all cubes in a free corner, rebuild the stacks bottom-up by teleport, verify).
Without the reset, 3 of the 5 scenario starts I ran had collapsed or dragged towers (the full loop and the replay got lucky once each).

## Step 2: repeatability (rigid grasp)

`release_test`: L moved back and forth between the left and the middle tower, 12 cycles, variants interleaved. Horizontal error of the
final cube position against the cube below (mm); raw data in `docs/results/release_rigid_grasp.csv`.

| Variant | n | mean | max |
|---|---|---|---|
| A: 2 mm drop, fast open | 4 | 1.5 | 4.5 |
| B: 2 mm drop, gradual open (steps of 0.03 rad) | 4 | 0.7 | 1.3 |
| C: 0.5 mm drop, fast open | 4 | 2.8 | 6.0 |

Small samples (4 each), so B being best is indicative, not established. Height is right (the cube rests on its support, tilt <= 0.1
degrees). A placement bumps neighbouring cubes by up to 6 mm. The physical grasp for comparison (6 cycles before it failed,
`docs/results/release_physical_grasp_partial.csv`): mean 6.7 mm, max 11.3 mm, worsening each cycle.
Default placement still uses variant A; `place_at(slow_open=True)` is variant B.

## Skills added or changed in this session

- `pick` verifies that the cube rose with the gripper (else the pick fails) and restarts a failed approach from the SRDF Home pose.
- `place_at` descends in two segments (collision-checked down to 12 mm above the support, then unchecked), restarts from Home when
  the approach fails, and retries the retreat without collision checking. The attached cube sits only 2 mm above its support, which
  made the last millimetres collision-flagged before.
- `settle` waits for sim time and wall time: after a blocking pause the backlog of /clock messages used a sim-time wait up at once,
  before the newest TF had arrived (a teleported cube looked unmoved). Scripts must not `time.sleep` without spinning.
- Data (`data/`) is installed with the package and found by `paths.data_dir()` (share dir, source tree, or `GR_BLOCKS_DATA`).

## Known problems and limits

- The simulated human only teleports cubes (parked at a table corner while "held"), so the recognizer sees symbolic actions, not a hand.
- The belief comes from `data/cached_plans/trace_MASTER.json` and only for prefixes of the cached plan (`LookupError` otherwise).
  After the join the cached joint plan, not R_ed's own 16-step plan, is what runs; R_ed's actions there are the joint plan's.
- Only MASTER was run. The other 11 words, the safe-exposure policy and the adversarial scenarios are untested.
- Flakiness: a few arm configurations cannot finish a straight descent (handled by the Home retry). One `table_test` run, before the
  place fix, ended with every stack on the table scattered after the first put-down; the cause was not isolated, the run after the
  fix passed. Treat a single passing run as indicative.
- Planning around the held cube works because the cube is an attached object; neighbouring towers 120 mm away are close to the
  gripper body, and the planner has no margin beyond the geometry.
- Not tried: the Gazebo GUI with `rigid_grasp:=true`, RViz, the real arm, other cube sizes, runs longer than a few minutes.

## Next

1. A physical grasp that holds (simplified parallel gripper for the sim, or a better model of the linkage), so the real-arm
   behaviour can be rehearsed.
2. Live recognition: call the GR-Blocks recognizer per observed action (measure the latency; the cached version avoids Fast Downward).
3. Other words and the safe-exposure policy; check each cached helper plan for actions a single arm cannot do.
4. A visible simulated human (a moving hand) instead of teleporting, and a pause protocol when the human reaches for a tower.
5. Phase 5: perception from the wrist camera (poses and letters) feeding the same `World`.

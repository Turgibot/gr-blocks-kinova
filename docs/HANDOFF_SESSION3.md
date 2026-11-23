> **Removed 2026-11-21:** the old pipeline code described here (and the `scripts/` it mentions) was deleted from the working tree in the cleanup; it is still in git history. Paths below use the old names.

# Handoff for the next session (written 2026-11-17, end of session 2)

Read in this order: `CLAUDE.md` (standing rules), this file, `docs/PHASE3_NOTES.md` (results and details), `docs/HANDOFF.md`
and `docs/PLAN.md` (original). Everything here was observed in headless Gazebo only; **nothing has run on the real arm.**

## Repo state

- Path `~/kinova-dev/ros_ws/src/gr-blocks-kinova`. Remotes: `github` = `git@github.com:Turgibot/gr-blocks-kinova.git` (public,
  Apache-2.0), `origin` = GitLab (empty; push of `main` rejected: protected default branch / role. Not worked around).
- Branches: `main` (scaffolding + LICENSE only, **does not contain `gr_blocks_ros`**), `feature/gazebo-sim-2f140` (package, first
  stage), `feature/grasp-and-blocksworld` (everything, latest). Work from the latest feature branch or branch off it.
- Git identity is set locally (Guy Tordjman). Commit trailer: `Co-Authored-By: Claude Sonnet 5.5 <noreply@anthropic.com>`.
- Rules that still hold (CLAUDE.md): small verified steps, additive changes, ask before editing existing files, never touch
  `install.sh` or `~/kinova-dev/ros_ws/src/ros2_*`, no push/PR/GitLab repo without the user saying so, report outcomes faithfully.
- Build: `cd ~/kinova-dev/ros_ws && colcon build --packages-select gr_blocks_ros --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`
  then `source install/setup.bash`.

## What works (sim)

Gazebo + MoveIt for Gen3 + 2F-140, blocks world on an extended table, ground-truth cube TF and planning scene, pick/place skills,
symbolic `World` (operators, slots) validated against the cached MASTER trace, helper-plan replay (2/2 passes), full MASTER loop
(2/2 passes, ~90 s), `table_test` (1 pass), 12-cycle repeatability (mean 1.7 mm, max 6.0 mm). Table of results: `PHASE3_NOTES.md`.

## What does NOT work / is not real

1. **The physical 2F-140 grasp does not hold in Gazebo** (tried friction, contact params, joint gain, DART). The sim uses a
   rigid weld (`rigid_grasp:=true`, DetachableJoint). The real arm needs a real grasp: not rehearsed anywhere yet.
2. Recognition is **read from the cache** and only answers for prefixes of the cached MASTER observed plan (`LookupError`
   otherwise). `t_join` = 6 is the cached value, not computed live.
3. The human is simulated by teleporting cubes (held cube parked at table corner). Only the word MASTER was run.
4. Flaky spots: some arm configs fail the straight descent (Home retry handles it); one `table_test` run scrambled all stacks and the
   cause was not isolated; the weld plugin starts attached and jolts cubes at spawn, so every scenario starts with
   `Executor.reset_layout()` and needs a freshly launched sim.
5. Not tried: GUI with `rigid_grasp:=true`, RViz, other cube sizes, long runs, perception.

## How to run (headless, isolated)

```
scripts/run_exp.sh /tmp/out full "rigid_grasp:=true" full_loop 1
scripts/run_exp.sh /tmp/out rep  "rigid_grasp:=true world:=blocks_world_tjoin.sdf" replay_helper
scripts/run_exp.sh /tmp/out tab  "rigid_grasp:=true" table_test
python3 -m gr_blocks_ros.blocksworld        # symbolic self-test, no sim
```
`scripts/run_exp.sh` starts a fresh sim and MoveIt with a unique `GZ_PARTITION`, runs one executable, saves `view_cam` image and
logs, and shuts down (it was a scratch helper, now committed; `pgrep -x gz` replaces the earlier pattern, untested in that form).
Manual launch: see `PHASE3_NOTES.md` "Running it". Use your own `ROS_DOMAIN_ID`. `gui:=true` only from a system terminal (snap/VS
Code terminal crashes the server).

## Hard-won lessons (do not relearn)

- Wait until the previous `gz sim` has fully exited before starting another; use a unique `GZ_PARTITION` per run.
- A grip lasts only while the commanding process is alive: pick and place run in one process.
- Never block with `time.sleep` in a script without spinning; use `Arm.settle` (sim time + wall floor), otherwise a /clock/TF
  backlog makes teleported cubes look unmoved.
- A planning-scene diff replaces the whole ACM: fetch and extend it. Removing a non-existent object fails the whole diff.
- z_moveit = z_gazebo - 0.30 (robot base spawns on the table top).
- Pads are 70 mm tall: grasp point is 15 mm above cube centre; a 45 mm cube stalls the fingers at ~0.45 rad, close cmd 0.46.
- Do not `pkill -f` patterns that match your own shell; kill by pid/process group.

## Code map (`gr_blocks/gr_blocks_ros/`)

`skills.py` (Arm: pick, place_at, place_on, rigid-grasp helpers) - `blocksworld.py` (World, SLOTS) - `executor.py` (Executor,
HumanSim, reset_layout) - `recognizer.py` (CachedRecognizer) - `make_world.py` (SDF generator) - `sim_patches.py` (pads, weld
plugins, gain) - `release_cubes.py` - `paths.py` - scenario scripts `full_loop.py`, `replay_helper.py`, `table_test.py`,
`release_test.py`, etc. Launch files in `gr_blocks/launch/`. Data from the sibling repo is in `data/` (cached plans).

## Suggested next steps (user decides)

1. Decide with the user about the lab session: what is available (real arm? network, safety setup?). Real-arm work needs a plan
   first (grasp without the weld, speed limits, workspace limits, e-stop procedure) before any motion.
2. Physical grasp that holds: simplified parallel-gripper model for the sim, or tune on the real gripper (on the real arm the
   2F-140 linkage is a real closed loop, so the Gazebo problem may not exist there).
3. Live recognition (call the GR-Blocks recognizer per action, measure latency).
4. Other words, safe-exposure policy, adversarial scenarios.
5. Visible simulated human + pause protocol; wrist-camera perception feeding `World`.
6. Housekeeping the user may want: merge the feature branch into `main` (open a PR), redact internal paths from the public repo,
   resolve the GitLab permission.

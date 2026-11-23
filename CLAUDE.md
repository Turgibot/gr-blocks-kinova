# CLAUDE.md

Guidance for Claude Code in this repo. Read `docs/HANDOFF.md` (facts) and `docs/PLAN.md` (the plan) first.

## What this is

A physical/simulated implementation of the experiment in Freedman & Zilberstein, *Integration of Planning with
Recognition for Responsive Interaction Using Classical Planners* (AAAI-17; PDF in `reference/`, gitignored), on a
**Kinova Gen3 7-DoF arm with a Robotiq 2F-140 gripper**, in Gazebo (first) and on the real arm (later).

- The Gen3 plays **R_ing** (the recognizing, helping robot). A **human plays R_ed** and builds a word from lettered blocks.
- The symbolic side (recognition, plans) already exists in a sibling repo: `/home/guy/Projects/PhD/GR-Blocks`
  (GitHub `Turgibot/GR-Blocks`). This repo is only the robot side + glue. Do not re-derive the maths; reuse its outputs.
- First milestone: Gazebo, goal word **MASTER**, replaying the cached plans.

## User preferences (carried over from earlier sessions; the user's memory is keyed to other folders, so it is repeated here)

- Small verified steps. Show formulas, plans and costs. Do not add unrequested features.
- **Additive changes.** New files, not edits to existing ones. Ask before editing any existing file.
  Never edit `install.sh` or vendored code in `~/kinova-dev/ros_ws/src/ros2_*`.
- **No `git push`, no MR/PR, no GitLab repo creation until the user says so.** Commit only when asked.
  The GitLab remote is not set yet (see `docs/HANDOFF.md`); ask the user for the URL.
- Separate feature branches from `main`, reviewable, with a "no regression" check where it applies.
- Report outcomes faithfully; if something is not verified, say so.

## Conventions

- ROS 2 Jazzy, Ubuntu 24.04, Python. Package name `gr_blocks_ros` (ament_python), placed at the repo root as
  `gr_blocks/` (the ROS package folder; the Python module is `gr_blocks/gr_blocks_ros/`).
- This repo is cloned at `~/kinova-dev/ros_ws/src/gr-blocks-kinova` and built with kinova-dev's single workspace
  (`cd ~/kinova-dev/ros_ws && colcon build --packages-select gr_blocks_ros`). It is hidden from kinova-dev's git via
  `kinova-dev/.git/info/exclude`. Remote: `git@gitlab.com:birc2/kinova_gen3/gr-blocks-kinova.git` (empty, nothing pushed).
- Block-world facts are `on(x,y)`, `ontable(x)`, `clear(x)`, `holding(x)`, `handempty`; goal = "X on Y" pairs plus
  "last letter on the table".

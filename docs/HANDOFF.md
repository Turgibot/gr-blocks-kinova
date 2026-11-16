# Handoff facts (written 2026-11-16 from the planning session)

Everything here was true when written. Verify anything you depend on (branch/MR state especially).

## 1. Decisions already made with the user

| Question | Decision |
|---|---|
| Role of the arm | **Gen3 = R_ing** (recognizes R_ed's goal and helps). **A human is R_ed**, building the word with real blocks. |
| Perception | **Full vision on the real arm too, letters read** (wrist camera). In sim, a simulated wrist camera with letter textures; ground truth is allowed for early milestones. |
| First milestone | **Sim, single goal word MASTER**, 2F-140 gripper, replaying cached plans, then widen to other words, then the real arm. |
| Repo | `git@gitlab.com:birc2/kinova_gen3/gr-blocks-kinova.git` (created, empty), cloned into `~/kinova-dev/ros_ws/src/gr-blocks-kinova`. Do not push until told. |

Still open (ask the user when they matter): whether to vendor the recognition scripts into this repo or call
`/home/guy/Projects/PhD/GR-Blocks`; block size and table layout; whether single-arm filtering of helper plans or
a re-plan with R_ing constrained is wanted (see risks in PLAN.md).

## 2. The paper's experiment (what the symbolic repo already does)

- Domain "Block-words": 12 lettered blocks in 3 towers, listed top to bottom, left to right as in Fig. 1:
  `H-T-E-R`, `L-S-M-O`, `F-W-B-A`. Four operators: pick-up, put-down, stack, unstack, every action cost 1.
- 12 goal words: HER, OTHER, BOTHER, MOTHER, FATHER, LATHER, MASTER, FASTER, FOSTER, HATER, LATER, WATER.
  R_ed's hidden goal in the paper's example is MASTER (16 steps alone; 8 with two agents).
- Recognition: observed actions are compiled to planning problems G+O and G+notO (Fast Downward, A* + LM-cut) ->
  likelihood and dynamic prior (parameters beta=1, lambda=1, tau=0.3, foresight eps in {0,1,3,6}) -> posterior over words.
- R_ing joins once the recognized goals no longer conflict (`t_join`; 6 for MASTER with eps=3) and executes the
  paper's helper plan. For MASTER the helper plan `pi_ring` is: `unstack W B`, `stack W L`, `pick-up A`, `stack A S`.
- Extension (user's own idea, branch `improvements` in GR-Blocks): **safe exposure**: before the join, R_ing does moves
  harmless for every goal still alive. Steps until built, R_ed alone / paper / safe exposure: MASTER 16/12/10,
  FOSTER 12/18/10, BOTHER 22/20/14, MOTHER 18/16/16.
- The two-arm planner (`scripts/twoarm.py` there) includes handovers, parallel moves and stacked-pair lifts that a
  single arm cannot do. Helper plans for MASTER happen to be single-arm-looking, but check every word.
- Known limit: our recognition does not reproduce the paper's Fig. 2 curves exactly (documented in the GR-Blocks README).
  Do not spend time on that.

## 3. Files in this folder

| Path | What |
|---|---|
| `data/blockworld.json`, `data/actions.json` | World (towers, goal words, paper costs) and the four operators. Copied from GR-Blocks `data/`. |
| `data/cached_plans/trace_MASTER.json` | `meta` (observed_plan = R_ed's 16 actions, params, goals + conditions) and `steps[t]` (world, per-goal costs, likelihood, prior, posterior). 1.1 MB. |
| `data/cached_plans/helper_MASTER.json` | R_ing's plan `pi_ring`, `t_join`, `G_hat`, `goal`, and joint/alone plans as step lists with `world` after each step. |
| `data/cached_plans/scenarios_MASTER.json` | independent (HOWL, WOLF) and adversarial (hoard, S on M) scenarios. |
| `data/cached_plans/exposure_MASTER.json` | safe-exposure run: per step posterior, candidates, safety test. |
| `reference/replay_actions.py` | Small symbolic state machine (apply operator, check preconditions). A good seed for the state tracker. |
| `reference/make_letter_textures.py` | Draws the letter PNGs for the (abandoned) Isaac scene; reusable for Gazebo textures. Needs matplotlib + Pillow. |
| `reference/paper_freedman_zilberstein_aaai17.pdf` | The paper (gitignored). |

Other helper/trace JSON for the other 8 words, `docs/safe_exposure.md`, and the HTML workbench are in
`/home/guy/Projects/PhD/GR-Blocks` (`out/`, `docs/`, `recognition-calculations.html`). Copy more `out/*_WORD.json` as needed.
Fast Downward is built at `/home/guy/Projects/PhD/GR-Blocks/third_party/downward` (657 MB, untracked), Python env
`/home/guy/Projects/PhD/GR-Blocks/.venv`. Run its scripts from that repo root as `python3 scripts/<name>.py WORD`.
Cost of recompute: all traces ~10 min in parallel; two-arm helper searches 30 min to ~6 h (BOTHER). Use the cache.

## 4. The Kinova workspace (`/home/guy/kinova-dev`, GitLab `birc2/kinova_gen3/kinova-basics`, SSH push works)

- Ubuntu 24.04, ROS 2 Jazzy, lab PC "lab1". `ros_ws/` is built (`ros_ws/install` exists). Source
  `~/kinova-dev/ros_ws/install/setup.bash` (plus `/opt/ros/jazzy/setup.bash`) as the underlay.
- Package `kinova_example` has launch files: `gen3_moveit.launch.py`, `gen3_ik.launch.py`,
  `gen3_2f85_moveit.launch.py`, `gen3_2f85_sim_moveit.launch.py` (MoveIt gripper working in Gazebo, 2F-85).
- **2F-140:** branch `gripper-2f140` (MR !3 as recorded in the user's notes; state not re-verified) adds
  `launch/gen3_2f140_moveit.launch.py` and `config/2f140/{gen3_2f140.srdf, joint_limits_2f140.yaml,
  moveit_controllers_2f140.yaml, ros2_controllers_2f140.yaml}`. It is not on `main`; it covers MoveIt/real-hardware
  style launch. **A Gazebo simulation launch for the 2F-140 does not exist yet** (the sim launch is 2F-85 only), and
  that is the first thing to build or verify here.
- 2F-85 gripper fix: branch `fix-2f85-gripper-parallel-command` (MR !4, closes ticket #1).
  Kortex's `moveit_controllers.yaml` says `GripperCommand` but the controller serves `ParallelGripperCommand`;
  MoveIt 2.12.4 supports both.
- `ros2_kortex_vision` is in the workspace (`kinova_vision.launch.py`, camera streams) for the real wrist camera.
- Sim launch: `kortex_sim_control.launch.py` (in `ros2_kortex`) defaults to NO gripper; pass `gripper:=robotiq_2f_85`.

## 5. Machine gotchas (must read before running anything)

- **Shared network:** default `ROS_DOMAIN_ID` 0 is shared with other lab machines. Use an isolated value in 0-101 in
  EVERY terminal (values above 101 are invalid). Gazebo needs its own `GZ_PARTITION` (and `GZ_IP=127.0.0.1` for
  headless tests), otherwise the robot can spawn into someone else's world.
- VS Code's snap environment (`GTK_PATH`, `GIO_MODULE_DIR`, `LOCPATH`, ...) crashes the Gazebo GUI. Unset them or use a
  system terminal.
- A uv-installed `~/.local/bin/python3.8` breaks ament; build with `-DPython3_EXECUTABLE=/usr/bin/python3`.
  ros2_control, ros2_controllers, generate_parameter_library, control_msgs, control_toolbox, ros2_control_cmake are
  excluded from the kinova-dev build with untracked `COLCON_IGNORE` files (apt copies are used; mixing source builds
  with apt MoveIt fails).
- `ros2 action send_goal` is flaky through pipes; use a small rclpy client in test scripts.
- Python 3 environment for Python-side work: the GR-Blocks `.venv`; ROS needs the system `/usr/bin/python3`.

## 6. Physical setup facts to settle early (nothing decided yet)

- 2F-140: stroke 140 mm. Suggest cubes of 40 to 50 mm; check payload and finger depth.
- Gen3 reach about 900 mm: three tower slots, a human zone, and a robot zone must fit; stack of up to 6 letters
  (the longest word has 6 letters, BOTHER) means a tower height of about 6 cubes.
- Letters on at least the faces the wrist camera can see (top and one or more sides).

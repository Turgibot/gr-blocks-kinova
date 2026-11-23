# Plan: GR-Blocks on a Kinova Gen3 + Robotiq 2F-140

Choices: Gen3 = R_ing, human = R_ed; letters read by vision (sim and real); first milestone = sim, word MASTER.

```
 human (R_ed) builds a word  ──►  perception: block poses + letters
                                          │ symbolic state, observed actions
                                          ▼
                    recognition (existing GR-Blocks scripts: G+O / G+notO, Fast Downward)
                                          │ belief over 12 goal words
                                          ▼
              R_ing policy (paper's helper plan, or safe exposure)
                                          │ stack(x,y), unstack(x,y), put-down(x), pick-up(x)
                                          ▼
     action executor: Blocksworld op -> MoveIt pick/place with the 2F-140
                                          │
                          Gazebo (sim)  /  real Gen3
```

## Phases

**0. Scaffolding and decisions**
- ament_python package `gr_blocks_ros` in `gr_blocks/` at the repo root (Python module `gr_blocks/gr_blocks_ros/`); this repo lives in `~/kinova-dev/ros_ws/src/` and builds with that workspace.
- README, pin down block size, tower slots, human zone vs robot zone.
- Decide: vendor recognition scripts vs call the GR-Blocks repo.

**1. Sim world (Gazebo + 2F-140)**
- Table + 12 lettered blocks + 3 start towers (H-T-E-R, L-S-M-O, F-W-B-A) as SDF generated from `data/blockworld.json`.
- Simulated wrist camera; letter textures (see `reference/make_letter_textures.py`).
- A Gazebo launch for the Gen3 + 2F-140 (does not exist yet; base on `gen3_2f85_sim_moveit.launch.py` and the
  `gripper-2f140` branch configs). Follow the machine gotchas in HANDOFF.md section 5.

**2. Primitive skills (sim)**
- `pick(block)` / `place(on=block|table)` MoveIt skill: grasp poses, approach/retreat, stack-height offsets.
  Gazebo grasp physics (friction, contact settings) is the usual trouble: settle it before building on top.
- Operator mapping: unstack = pick, stack = place on a block, put-down = place on table, pick-up = pick from table.
- Symbolic state tracker that checks preconditions and effects (seed: `reference/replay_actions.py`).
- Test: replay cached R_ing plans (`helper_MASTER.json`) in Gazebo, no human.

**3. Simulated human and full loop**
- Scripted R_ed plays `meta.observed_plan` from `trace_MASTER.json` into the same sim at a configurable tempo.
- Observed actions go to the existing recognition code; R_ing starts at the paper's join condition (`t_join`)
  and runs the helper plan.
- Success: word MASTER built; step counts match Python (R_ed 16 alone; R_ing helper plan 4 actions; cost numbers as in
  `helper_MASTER.json`); live belief matches `trace_MASTER.json` posteriors.

**4. Widen**
- Other cached words (9 have helper plans), safe-exposure policy (compare with its step counts), adversarial scenario.

**5. Real perception**
- Block pose detection and letter reading (OCR/classifier), robust to stacking, occlusion by the human's hand, glare.
- Output a symbolic state (on / clear / ontable / holding) and infer R_ed's actions from state changes.
- Develop on recorded Kinova camera data first; `ros2_kortex_vision` is in the workspace.

**6. Real arm**
- Hand-eye calibration, 2F-140 grasp tuning on real blocks, speed/force limits, human safety (slow speed cap,
  pause when a hand is in the workspace). Run MASTER with a human, then other words.

## Risks / open questions
1. **Online recognition latency.** Cached traces were precomputed (minutes to hours). Plan: use cached plans for known
   words; measure one recognition update per step before promising live replanning.
2. **Shared tower access** between human and robot: define which towers/zones each may touch and a pause protocol.
3. **Handovers / pair lifts** in two-arm plans are impossible for one arm: filter helper plans to single-arm actions
   or re-run the planner with R_ing constrained (`forced_arm2` option in `twoarm.py`).
4. **Letter reading** is the largest perception effort. Fallback: read each letter once at layout time and track
   identity by position.
5. The 2F-140 Gazebo sim launch does not exist yet.

## Suggested start
Phases 0 and 1, then 2: one Gazebo world with the 2F-140, a working pick/place, and the cached MASTER R_ing plan
replayed in sim. That tests the riskiest physical part first; everything later reuses the symbolic code.

"""Run Blocksworld operators in the simulation: R_ing with the Gen3, R_ed (the simulated human) by teleporting cubes.

R_ing: unstack/pick-up -> Arm.pick, stack -> Arm.place_on, put-down -> Arm.place_at(slot).
R_ed:  a cube that R_ed 'holds' is parked at HAND (a spot on the table corner, away from the arm); stack / put-down
       teleport it to its destination with Gazebo's set_pose service. R_ed's pace is the caller's business.
After every action the physical stacks (from the cube poses) are compared with the symbolic World.
"""
import numpy as np

from gr_blocks_ros.blocksworld import SLOTS, World
from gr_blocks_ros.skills import CUBE, DROP

TABLE_Z = 0.30                 # Gazebo world z of the table top (robot base height)
HAND = (0.30, 0.43)            # where cubes held by R_ed wait (Gazebo world x, y)
PICK_DZ = 0.015                # grasp point above the cube centre: pads (70 mm) clear the cube below / the table
LETTERS = "ABEFHLMORSTW"
# Parking spots used by reset_layout: two rows of six at the far end of the table, clear of every slot
STAGING = [(0.72 + 0.08 * (i % 2), -0.19 + 0.076 * (i // 2)) for i in range(12)]


class HumanSim:
    """Teleports cubes with the world's set_pose service (gz transport Python bindings)."""

    def __init__(self, world_name="blocks_world"):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.pose_pb2 import Pose
        from gz.transport13 import Node as GzNode
        self._node, self._Pose, self._Boolean = GzNode(), Pose, Boolean
        self._srv = f"/world/{world_name}/set_pose"

    def teleport(self, letter, x, y, z):
        req = self._Pose()
        req.name = f"block_{letter}"
        req.position.x, req.position.y, req.position.z = x, y, z
        req.orientation.w = 1.0
        ok, rep = self._node.request(self._srv, req, self._Pose, self._Boolean, 3000)
        if not (ok and rep.data):
            raise RuntimeError(f"set_pose failed for block_{letter}")

    def place(self, letter, slot, level):
        x, y = SLOTS[slot]
        self.teleport(letter, x, y, TABLE_Z + CUBE / 2 + 0.0005 + level * (CUBE + 0.0005))

    def park(self, letter):
        self.teleport(letter, HAND[0], HAND[1], TABLE_Z + CUBE / 2 + 0.0005)


class Executor:
    def __init__(self, arm, world: World, human: HumanSim):
        self.arm, self.world, self.human = arm, world, human
        self.log = []

    # ---- physical state ---------------------------------------------------------------------------------------
    def poses(self):
        """Cube centres in the Gazebo world frame (TF is in base_link, which sits TABLE_Z below)."""
        return {l: tuple(np.array(self.arm.block(l)) + [0, 0, TABLE_Z]) for l in LETTERS}

    def physical_stacks(self):
        held = {l for l in self.world.held.values() if l}
        return World.from_poses({l: p for l, p in self.poses().items() if l not in held}).stacks

    def consistent(self):
        phys = self.physical_stacks()
        want = {s: st for s, st in self.world.stacks.items()}
        return phys == want, phys, want

    # ---- test harness: set the table --------------------------------------------------------------------------
    def wait_still(self, timeout=15.0, tol=0.0005):
        """Wait until no cube moves (poses compared 0.6 s apart)."""
        import time
        end = time.monotonic() + timeout
        prev = self.poses()
        while time.monotonic() < end:
            self.arm.settle(0.6)
            cur = self.poses()
            if max(np.linalg.norm(np.array(cur[l]) - np.array(prev[l])) for l in LETTERS) < tol:
                return True
            prev = cur
        return False

    def reset_layout(self):
        """Put every cube where the symbolic world says it is, upright (teleport; no robot involved).

        Needed because the sim start can disturb the stacks (the rigid-grasp plugin welds the cubes to the gripper when the
        robot spawns and jolts them by a few mm). All cubes first go to a free staging area, so nothing lies in the way of
        the slots, then the stacks are rebuilt bottom-up. Returns True when the physical stacks then equal the symbolic ones."""
        assert not self.world.holds(), "reset_layout needs a world where nobody holds a cube"
        self.wait_still()
        letters = [l for st in self.world.stacks.values() for l in st]
        for l, (x, y) in zip(letters, STAGING):
            self.human.teleport(l, x, y, TABLE_Z + CUBE / 2 + 0.002)
            self.arm.settle(0.15)
        self.arm.settle(1.0)
        for slot, st in self.world.stacks.items():
            for level, l in enumerate(st):
                self.human.place(l, slot, level)
                self.arm.settle(0.25)
        self.arm.settle(1.5)
        self.wait_still()
        return self.consistent()[0]

    # ---- actions -----------------------------------------------------------------------------------------------
    def do(self, text, agent):
        """Run one operator (symbolic check first, then physical). Returns True when the physical state agrees."""
        before = self.poses()
        info = self.world.apply_text(text, agent)  # raises IllegalAction before anything moves
        ok = self._ring(info) if agent == "R_ing" else self._human(info)
        self.arm.settle(1.0)
        same, phys, want = self.consistent()
        self.log.append((agent, text, ok, same))
        after = self.poses()
        bumped = {l: round(1000 * float(np.linalg.norm(np.array(after[l]) - np.array(before[l]))), 1)
                  for l in LETTERS if l != info["block"] and np.linalg.norm(np.array(after[l]) - np.array(before[l])) > 0.005}
        if bumped:
            print(f"  note: other cubes moved (mm) by {agent} {text}: {bumped}")
        if not (ok and same):
            print(f"  !! {agent} {text}: motion ok={ok}, physical==symbolic: {same}")
            if not same:
                print("     physical:", {s: ''.join(v) for s, v in phys.items()})
                print("     symbolic:", {s: ''.join(v) for s, v in want.items()})
        return ok and same

    def _ring(self, info):
        op, x = info["op"], info["block"]
        if op in ("unstack", "pick-up"):
            return self.arm.pick(x, dz=PICK_DZ)
        if op == "stack":
            return self.arm.place_on(info["onto"])
        if op == "put-down":
            sx, sy = SLOTS[info["slot"]]
            return self.arm.place_at((sx, sy), CUBE / 2 + DROP)  # base-frame z: the table top is z = 0
        raise ValueError(op)

    def _human(self, info):
        op, x = info["op"], info["block"]
        if op in ("unstack", "pick-up"):
            self.human.park(x)
        else:
            self.human.place(x, info["slot"], info["level"])
        return True

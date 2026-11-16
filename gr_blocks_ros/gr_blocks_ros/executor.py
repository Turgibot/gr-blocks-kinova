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

    # ---- actions -----------------------------------------------------------------------------------------------
    def do(self, text, agent):
        """Run one operator (symbolic check first, then physical). Returns True when the physical state agrees."""
        info = self.world.apply_text(text, agent)  # raises IllegalAction before anything moves
        ok = self._ring(info) if agent == "R_ing" else self._human(info)
        self.arm.settle(1.0)
        same, phys, want = self.consistent()
        self.log.append((agent, text, ok, same))
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

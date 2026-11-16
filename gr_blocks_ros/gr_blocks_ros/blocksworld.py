"""Symbolic block world with table slots: the bridge between Blocksworld plans and positions in the simulation.

A `World` keeps, for every table slot, the stack standing on it (bottom first) and what each agent holds. Agents are
"R_ing" (the Gen3) and "R_ed" (the human). Operators and their preconditions are the ones of data/actions.json, checked
on the equivalent facts (on / ontable / clear / holding / handempty), so a plan that is illegal symbolically is refused
before the arm moves.

    python3 -m gr_blocks_ros.blocksworld          # self-test against the cached MASTER trace and joint helper plan
"""
import json
from pathlib import Path

from gr_blocks_ros.paths import data_dir

# Table slots (x, y) in the Gazebo world frame (the robot base is at the origin, +x forward, +y left), metres.
# P2, P3, P4 hold the three start towers of blockworld.json (left, middle, right).
SLOTS = {
    "P1": (0.45, 0.24), "P2": (0.45, 0.12), "P3": (0.45, 0.00), "P4": (0.45, -0.12), "P5": (0.45, -0.24),
    "Q1": (0.65, 0.18), "Q2": (0.65, 0.00), "Q3": (0.65, -0.18),
}
START_SLOTS = ["P2", "P3", "P4"]  # tower i of blockworld.json -> slot
PUT_DOWN_ORDER = ["P1", "P5", "Q2", "Q1", "Q3", "P2", "P3", "P4"]  # preference for a new base on the table

DATA = data_dir()


def load_operators(path=None):
    path = Path(path) if path else DATA / "actions.json"
    return json.loads(path.read_text())["operators"]


class IllegalAction(ValueError):
    pass


class World:
    def __init__(self, stacks, held=None, operators=None):
        self.stacks = {s: list(v) for s, v in stacks.items() if v}  # slot -> bottom..top
        self.held = {"R_ing": None, "R_ed": None}
        self.held.update(held or {})
        self.ops = operators or load_operators()

    # ---- construction -----------------------------------------------------------------------------------------
    @classmethod
    def from_towers(cls, towers, slots=None, held=None):
        """towers: lists top-first (as in the JSON files); slots: one slot name per tower (default P2, P3, P4, then free ones)."""
        slots = list(slots or [])
        free = [s for s in START_SLOTS + PUT_DOWN_ORDER if s not in slots]
        stacks = {}
        for i, t in enumerate(towers):
            slot = slots[i] if i < len(slots) else next(s for s in free if s not in stacks)
            stacks[slot] = list(reversed(t))
        return cls(stacks, held)

    def copy(self):
        return World(self.stacks, dict(self.held), self.ops)

    # ---- queries ---------------------------------------------------------------------------------------------
    def slot_of(self, letter):
        for s, st in self.stacks.items():
            if letter in st:
                return s
        return None

    def top(self, slot):
        return self.stacks[slot][-1] if self.stacks.get(slot) else None

    def holds(self):
        return {a for a, h in self.held.items() if h}

    def towers(self):
        """Stacks as top-first lists, sorted (for comparison with the cached JSON worlds)."""
        return sorted(tuple(reversed(st)) for st in self.stacks.values())

    def facts(self, agent="R_ing"):
        f = set()
        for st in self.stacks.values():
            f |= {("ontable", st[0]), ("clear", st[-1])}
            f |= {("on", up, lo) for lo, up in zip(st, st[1:])}
        h = self.held[agent]
        f.add(("holding", h) if h else ("handempty",))
        return f

    def free_slot(self, prefer=None):
        for s in ([prefer] if prefer else []) + PUT_DOWN_ORDER:
            if s and s not in self.stacks:
                return s
        raise IllegalAction("no free table slot")

    # ---- operators ------------------------------------------------------------------------------------------
    def _check(self, name, args, agent):
        op = self.ops[name]
        binding = dict(zip(op["params"], args))
        need = {tuple([f[0]] + [binding.get(a, a) for a in f[1:]]) for f in op["pre"]}
        missing = need - self.facts(agent)
        if missing:
            raise IllegalAction(f"{name}({','.join(args)}) by {agent} needs {sorted(missing)}")

    def apply(self, name, *args, agent="R_ing"):
        """Apply an operator; returns a dict describing where things moved (used by the executors)."""
        self._check(name, args, agent)
        if name in ("unstack", "pick-up"):
            x = args[0]
            slot = self.slot_of(x)
            self.stacks[slot].pop()
            if not self.stacks[slot]:
                del self.stacks[slot]
            self.held[agent] = x
            return {"op": name, "block": x, "from_slot": slot}
        x = args[0]
        if name == "stack":
            slot = self.slot_of(args[1])
            self.stacks[slot].append(x)
            self.held[agent] = None
            return {"op": name, "block": x, "onto": args[1], "slot": slot, "level": len(self.stacks[slot]) - 1}
        if name == "put-down":
            slot = self.free_slot()
            self.stacks[slot] = [x]
            self.held[agent] = None
            return {"op": name, "block": x, "slot": slot, "level": 0}
        raise IllegalAction(f"unknown operator {name}")

    def apply_text(self, text, agent="R_ing"):
        name, *args = text.split()
        return self.apply(name, *args, agent=agent)

    # ---- comparison with measurements -----------------------------------------------------------------------
    @classmethod
    def from_poses(cls, poses, cube=0.045, table_z=0.30, xy_tol=0.04):
        """Rebuild the stacks from measured cube centres {letter: (x, y, z)} (Gazebo world frame).

        A cube belongs to the nearest slot if it is within xy_tol of it; cubes are ordered by height."""
        stacks = {}
        for letter, p in poses.items():
            slot = min(SLOTS, key=lambda s: (SLOTS[s][0] - p[0]) ** 2 + (SLOTS[s][1] - p[1]) ** 2)
            if ((SLOTS[slot][0] - p[0]) ** 2 + (SLOTS[slot][1] - p[1]) ** 2) ** 0.5 <= xy_tol:
                stacks.setdefault(slot, []).append((p[2], letter))
        return cls({s: [l for _, l in sorted(v)] for s, v in stacks.items()})

    def expected_xyz(self, slot, level, cube=0.045, table_z=0.30):
        x, y = SLOTS[slot]
        return (x, y, table_z + cube / 2 + level * cube)


def _selftest():
    """Replay the cached MASTER traces with the operators and compare every state with the cached one."""
    plans = DATA / "cached_plans"
    trace = json.loads((plans / "trace_MASTER.json").read_text())
    helper = json.loads((plans / "helper_MASTER.json").read_text())
    start = trace["meta"]["start_towers"]
    w = World.from_towers(start, START_SLOTS)
    ok = True

    def same(a, b):
        return sorted(tuple(t) for t in a) == sorted(tuple(t) for t in b)

    print("== R_ed's 16 observed actions vs cached worlds")
    assert same(w.towers(), trace["steps"][0]["world"]["towers"])
    for t, text in enumerate(trace["meta"]["observed_plan"], 1):
        w.apply_text(text, "R_ed")
        cached = trace["steps"][t]["world"]
        good = same(w.towers(), cached["towers"]) and (w.held["R_ed"] == cached["hand"])
        ok &= good
        print(f"  t={t:2d} {text:12s} -> {'ok' if good else 'MISMATCH'}  slots {dict(sorted((s, ''.join(v)) for s, v in w.stacks.items()))}")

    print("== joint helper plan from t_join (arm 1 = R_ed first, then arm 2 = R_ing)")
    tj = helper["t_join"]
    w = World.from_towers(start, START_SLOTS)
    for text in trace["meta"]["observed_plan"][:tj]:
        w.apply_text(text, "R_ed")
    assert same(w.towers(), helper["start"]["towers"]), "t_join world differs from the helper plan's start"
    ring = []
    for step in helper["joint_Ghat"]["steps"]:
        for arm, agent in (("1", "R_ed"), ("2", "R_ing")):
            comp = step["comps"][arm]
            if comp:
                w.apply_text(comp, agent)
                if agent == "R_ing":
                    ring.append(comp)
        cached = step["world"]
        good = (same(w.towers(), cached["towers"]) and w.held["R_ed"] == cached["hands"]["1"]
                and w.held["R_ing"] == cached["hands"]["2"])
        ok &= good
        print(f"  {step['name']:18s} {step['comps']} -> {'ok' if good else 'MISMATCH'}")
    same_ring = ring == helper["pi_ring"]
    ok &= same_ring
    print(f"  R_ing's actions {ring} equal pi_ring: {same_ring}")

    print("== illegal action is refused")
    w2 = World.from_towers(start, START_SLOTS)
    try:
        w2.apply("stack", "H", "E")
        ok = False
        print("  NOT refused")
    except IllegalAction as e:
        print("  refused:", e)
    print("SELFTEST", "PASS" if ok else "FAIL")
    return ok


if __name__ == "__main__":
    raise SystemExit(0 if _selftest() else 1)

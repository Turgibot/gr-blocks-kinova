"""Belief over the goal words, read from the cached trace (data/cached_plans/trace_*.json).

This is a stand-in for the live recognizer of the GR-Blocks repo (Fast Downward, minutes per update): it answers only for
action sequences that are prefixes of the cached observed plan, which is exactly what the scripted R_ed plays. Anything
else raises LookupError, so a run never silently uses a belief that was not computed for what was observed.
"""
import json

from gr_blocks_ros.paths import data_dir

DATA = data_dir() / "cached_plans"


class CachedRecognizer:
    def __init__(self, word="MASTER", eps="eps3"):
        self.trace = json.loads((DATA / f"trace_{word}.json").read_text())
        self.plan = self.trace["meta"]["observed_plan"]
        self.eps = eps

    def belief(self, observed):
        """{word: probability} after the observed actions (list of strings like 'unstack H T')."""
        t = len(observed)
        if list(observed) != self.plan[:t]:
            raise LookupError(f"observed actions {list(observed)} are not a prefix of the cached plan")
        return dict(self.trace["steps"][t]["posterior"][self.eps])

    def top(self, observed, n=3):
        b = self.belief(observed)
        return sorted(b.items(), key=lambda kv: -kv[1])[:n]

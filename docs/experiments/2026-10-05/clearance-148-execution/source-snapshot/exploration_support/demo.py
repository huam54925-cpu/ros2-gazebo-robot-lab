"""Synthetic OFFLINE demonstration. No ROS, API key, network or motion."""
from dataclasses import asdict
import json
import tempfile
from pathlib import Path
import numpy as np
from .grid import Grid, capture_patch, visible_unknown_boundary
from .memory import Event, EventStore
from .policy import Candidate, filter_for_recheck, low_gain_cycle, Config


def main() -> None:
    a = np.full((40, 60), -1, dtype=np.int16)
    a[1:39, 1:30] = 0
    a[0, :] = a[-1, :] = a[:, 0] = 100
    a[1:39, 29] = 100   # wall blocks unknown behind it
    wall_map = Grid(a, 0.2)
    blocked = visible_unknown_boundary(wall_map, (3.0, 4.0, 0.0), 8.0)
    b = a.copy()
    b[15:25, 29] = 0    # known-free opening exposes genuine unknown
    open_map = Grid(b, 0.2)
    opening = visible_unknown_boundary(open_map, (3.0, 4.0, 0.0), 8.0)
    events = []
    for i, pose in enumerate([(2., 3., 0.), (3., 3., 0.)]*2, 1):
        events.append(Event(f"task-{i}", "demo-mission", "epoch-1", i, pose,
                            "succeeded", True, gain_m2=0.02,
                            patch=capture_patch(open_map, pose[:2])))
    candidates = [Candidate("A-new-id", 2., 3., 0., 2., 2.),
                  Candidate("B-new-id", 3., 3., 0., 2., 2.),
                  Candidate("C-other-view", 4.8, 4., 0., 3., 4.)]
    with tempfile.TemporaryDirectory() as tmp:
        store = EventStore(Path(tmp)/"memory.sqlite3")
        for e in events:
            store.append(e)
        replay_inserted = store.append(events[-1])
        history = store.events("demo-mission", "epoch-1")
        advice = filter_for_recheck(open_map, candidates, history, "demo-mission", "epoch-1", 5)
    print(json.dumps({
        "scope": "synthetic offline logic, not robot acceptance",
        "wall_blocked_proxy_m2": blocked.proxy_m2,
        "door_open_proxy_m2": opening.proxy_m2,
        "low_gain_cycle_detected": bool(low_gain_cycle(events, Config())),
        "replay_created_new_memory_event": replay_inserted,
        "candidate_advice": [asdict(v) for v in advice],
        "motion_authorized": False,
    }, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

"""Persistent advisory memory. Separate from the existing authoritative task DB.

Only the LOCAL supervisor records events. Model text, polls and request replays
must not create extra observations. Every connection is explicitly closed.
"""
from __future__ import annotations
from contextlib import contextmanager
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import sqlite3
from typing import Any, Iterator


@dataclass(frozen=True)
class Event:
    event_id: str          # e.g. terminal:<authoritative-task-id>
    mission_id: str
    map_epoch: str         # stable frame/SLAM map epoch, NOT every /map hash
    step: int             # monotonic authoritative decision sequence
    target: tuple[float, float, float]
    status: str           # succeeded / rejected / aborted / ...
    stopped: bool
    purpose: str = "observe_goal"  # or transit / return / wait
    gain_m2: float | None = None   # measured comparable unknown->observed, not estimate
    reason: str = ""
    failure_scope: str = "none"  # none / goal_pose / approach / system
    approach_key: str = ""       # stable corridor/witness ID, NOT frontier batch ID
    patch: dict[str, Any] | None = None
    frontier_cluster_id: str = ""
    frontier_lineage: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if not self.event_id or not self.mission_id or not self.map_epoch or self.step < 0:
            raise ValueError("Event identity and nonnegative sequence required")
        if len(self.target) != 3 or not all(math.isfinite(v) for v in self.target):
            raise ValueError("Invalid event target")
        if self.gain_m2 is not None and not math.isfinite(self.gain_m2):
            raise ValueError("Measured gain must be finite or None")
        if self.failure_scope not in {"none", "goal_pose", "approach", "system"}:
            raise ValueError("Invalid failure scope")
        if self.purpose not in {"observe_goal", "transit", "return", "wait"}:
            raise ValueError("Invalid task purpose")
        if self.failure_scope == "approach" and not self.approach_key:
            raise ValueError("Approach-scoped failure requires a stable approach key")


class EventStore:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        if str(path) == ":memory:":
            raise ValueError("Use a file: this store closes each connection explicitly")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.connection() as con:
            con.execute("""CREATE TABLE IF NOT EXISTS exploration_events (
                event_id TEXT PRIMARY KEY, mission_id TEXT NOT NULL,
                map_epoch TEXT NOT NULL, step INTEGER NOT NULL,
                payload TEXT NOT NULL)""")
            con.execute("""CREATE INDEX IF NOT EXISTS exploration_events_mission
                ON exploration_events(mission_id, map_epoch, step)""")

    @contextmanager
    def connection(self) -> Iterator[sqlite3.Connection]:
        con = sqlite3.connect(self.path, timeout=5.0)
        try:
            con.execute("PRAGMA busy_timeout=5000")
            with con:
                yield con
        finally:
            con.close()  # `with con` alone manages transactions, NOT connection life.

    def append(self, event: Event) -> bool:
        text = json.dumps(asdict(event), sort_keys=True, separators=(",", ":"), allow_nan=False)
        with self.connection() as con:
            con.execute("BEGIN IMMEDIATE")
            row = con.execute("SELECT payload FROM exploration_events WHERE event_id=?",
                              (event.event_id,)).fetchone()
            if row:
                # Old records predate optional lineage fields. Normalize their
                # defaults without rewriting historical evidence.
                existing=json.dumps(asdict(Event(**json.loads(row[0]))),sort_keys=True,
                                    separators=(",", ":"),allow_nan=False)
                if existing != text:
                    raise ValueError("Conflicting content for existing event ID")
                return False
            con.execute("INSERT INTO exploration_events VALUES(?,?,?,?,?)",
                        (event.event_id, event.mission_id, event.map_epoch, event.step, text))
            return True

    def events(self, mission_id: str, map_epoch: str, limit: int = 500) -> list[Event]:
        if not 1 <= limit <= 100_000:
            raise ValueError("Invalid read limit")
        with self.connection() as con:
            rows = con.execute("""SELECT payload FROM exploration_events
                WHERE mission_id=? AND map_epoch=? ORDER BY step DESC, rowid DESC LIMIT ?""",
                (mission_id, map_epoch, limit)).fetchall()
        result = []
        for (text,) in reversed(rows):
            item = json.loads(text)
            item["target"] = tuple(item["target"])
            item["frontier_lineage"] = tuple(item.get("frontier_lineage",()))
            result.append(Event(**item))
        return result

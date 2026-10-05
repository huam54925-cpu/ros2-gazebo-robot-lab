"""Advisory candidate filtering. RETAIN means 'send to existing safety checks'.

This module never calls Nav2, never says that a pose/path is safe, never changes
occupancy, guard thresholds, the task ledger, or the persistent stop latch.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Sequence
from .grid import Grid, patch_change
from .memory import Event


@dataclass(frozen=True)
class Candidate:
    frontier_id: str
    x: float
    y: float
    yaw: float
    estimated_gain_m2: float
    path_length_m: float
    approach_key: str = ""
    purpose: str = "observe_goal"

    def __post_init__(self) -> None:
        if not self.frontier_id or not all(math.isfinite(v) for v in
            (self.x, self.y, self.yaw, self.estimated_gain_m2, self.path_length_m)):
            raise ValueError("Invalid candidate")
        if self.estimated_gain_m2 < 0 or self.path_length_m < 0:
            raise ValueError("Negative candidate cost/gain")
        if self.purpose not in {"observe_goal", "transit", "return"}:
            raise ValueError("Invalid candidate purpose")


@dataclass(frozen=True)
class Config:
    # Heuristic starting points; not safety constants and not calibrated.
    same_place_m: float = 0.50
    same_heading_rad: float = math.pi/4
    recent_steps: int = 5
    low_gain_m2: float = 0.50
    loop_gain_m2: float = 0.50
    patch_min_cells: int = 3
    patch_min_fraction: float = 0.05

    def __post_init__(self) -> None:
        numbers = (self.same_place_m, self.same_heading_rad, self.low_gain_m2,
                   self.loop_gain_m2, self.patch_min_fraction)
        if not all(math.isfinite(v) and v >= 0 for v in numbers):
            raise ValueError("Invalid policy thresholds")
        if self.same_place_m <= 0 or self.recent_steps < 1 or self.patch_min_cells < 1:
            raise ValueError("Invalid policy window")
        if self.patch_min_fraction > 1:
            raise ValueError("Invalid patch fraction")


@dataclass(frozen=True)
class Advice:
    frontier_id: str
    retain_for_safety_recheck: bool
    reason: str
    evidence_event: str | None = None


def same_pose(a: tuple[float, float, float], b: tuple[float, float, float], cfg: Config) -> bool:
    return (math.hypot(a[0]-b[0], a[1]-b[1]) <= cfg.same_place_m and
            abs(math.remainder(a[2]-b[2], 2*math.pi)) <= cfg.same_heading_rad)


def low_gain_cycle(events: Sequence[Event], cfg: Config) -> list[Event]:
    """ABAB at completed observation goals, low MEASURED gain. ABA is not enough.

    Uses authoritative records, not the high-rate robot trajectory. Any transit,
    failure, missing comparable gain, or unconfirmed stop prevents this match.
    """
    if len(events) < 4:
        return []
    e = list(events[-4:])
    if len({(v.mission_id, v.map_epoch) for v in e}) != 1:
        return []
    if any(v.status != "succeeded" or not v.stopped or v.purpose != "observe_goal"
           or v.gain_m2 is None for v in e):
        return []
    if not (same_pose(e[0].target, e[2].target, cfg) and same_pose(e[1].target, e[3].target, cfg)):
        return []
    if math.hypot(e[0].target[0]-e[1].target[0], e[0].target[1]-e[1].target[1]) <= cfg.same_place_m:
        return []
    if sum(max(0.0, v.gain_m2 or 0.0) for v in e) > cfg.loop_gain_m2:
        return []
    return e


def filter_for_recheck(grid: Grid, candidates: Sequence[Candidate], events: Sequence[Event],
                       mission_id: str, map_epoch: str, step: int,
                       cfg: Config = Config()) -> list[Advice]:
    """Filter repeated proposals without converting rejected poses into obstacles.

    No automatic release on a global map-version change or elapsed wall time.
    Relevant local semantic changes permit replanning. If all proposals are
    removed, expand the candidate SEARCH or report exhaustion; do not force a
    suppressed/unsafe goal. Planner/executor limits remain unchanged.
    """
    history = sorted((e for e in events if e.mission_id == mission_id and
                      e.map_epoch == map_epoch and e.step <= step), key=lambda e: e.step)
    cycle = low_gain_cycle(history, cfg)
    changed_cache: dict[str, bool] = {}

    def changed(e: Event) -> bool:
        if e.event_id not in changed_cache:
            # Without a patch there is insufficient evidence for a spatial ban.
            changed_cache[e.event_id] = (e.patch is None or patch_change(
                grid, e.patch, cfg.patch_min_cells, cfg.patch_min_fraction)[0])
        return changed_cache[e.event_id]

    answers = []
    for c in candidates:
        target = (c.x, c.y, c.yaw)
        advice = Advice(c.frontier_id, True, "RETAIN_FOR_EXISTING_SAFETY_CHECKS")
        for e in reversed(history):
            if e.status == "succeeded" or e.failure_scope in {"none", "system"}:
                continue
            matches = (same_pose(target, e.target, cfg) if e.failure_scope == "goal_pose"
                       else bool(c.approach_key and c.approach_key == e.approach_key))
            if matches and not changed(e):
                reason = "SAME_FAILED_GOAL_POSE" if e.failure_scope == "goal_pose" else "SAME_FAILED_APPROACH"
                advice = Advice(c.frontier_id, False, reason, e.event_id)
                break
        if advice.retain_for_safety_recheck and c.purpose == "observe_goal":
            for e in reversed(history):
                if step-e.step > cfg.recent_steps:
                    break
                if (e.status == "succeeded" and e.stopped and e.purpose == "observe_goal"
                    and e.gain_m2 is not None and e.gain_m2 <= cfg.low_gain_m2
                    and same_pose(target, e.target, cfg) and not changed(e)):
                    advice = Advice(c.frontier_id, False, "RECENT_LOW_GAIN_OBSERVATION", e.event_id)
                    break
        if advice.retain_for_safety_recheck and c.purpose == "observe_goal":
            for e in cycle:
                if same_pose(target, e.target, cfg) and not changed(e):
                    advice = Advice(c.frontier_id, False, "LOW_GAIN_ABAB_REPEAT", e.event_id)
                    break
        answers.append(advice)
    return answers


def parse_choice(payload: dict, allowed_ids: set[str]) -> str | None:
    """Optional client-side selection contract; NOT the existing MCP schema.

    Demonstrates membership/extra-field checks. Existing execute_frontier keeps
    frontier_id + request_id and performs its own server-side checks again.
    """
    if not isinstance(payload, dict) or set(payload) != {"action", "frontier_id"}:
        raise ValueError("Expected only action and frontier_id")
    action, frontier = payload["action"], payload["frontier_id"]
    if action == "stop" and frontier is None:
        return None
    if action != "move" or not isinstance(frontier, str) or frontier not in allowed_ids:
        raise ValueError("Choice is not in current executable candidate set")
    return frontier

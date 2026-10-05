"""Raw /map semantics, visibility proxies and comparable map changes.

Not a collision checker or a SLAM implementation. Inputs are signed occupancy
values (-1 and 0..100), never an internal Nav2 uint8 costmap. Thresholds below
match the inspected repository's exploration convention, not a ROS-wide rule.
"""
from __future__ import annotations
from dataclasses import dataclass
from enum import IntEnum
import hashlib
import math
from typing import Any, Iterator
import numpy as np


def _snap_grid_coordinate(value: float) -> float:
    # Consistent half-open cells after integer-cell map-origin shifts.
    # This tolerance is 1e-10 of ONE CELL, not a motion safety margin.
    nearest = round(value)
    return float(nearest) if abs(value-nearest) <= 1e-10 else value


class Cell(IntEnum):
    UNKNOWN = 0
    FREE = 1
    OCCUPIED = 2
    UNCERTAIN = 3
    OUTSIDE = 4


@dataclass(frozen=True)
class Grid:
    data: np.ndarray
    resolution: float
    origin: tuple[float, float, float] = (0.0, 0.0, 0.0)
    frame: str = "map"
    free_below: int = 25
    occupied_at: int = 65

    def __post_init__(self) -> None:
        a = np.asarray(self.data)
        if a.ndim != 2 or not a.size or a.dtype.kind not in "iu":
            raise ValueError("Expected a nonempty 2D signed occupancy integer array")
        if np.any((a < -1) | (a > 100)):
            raise ValueError("Expected -1 or 0..100; do not pass a Nav2 byte costmap")
        if not math.isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError("resolution must be positive and finite")
        if len(self.origin) != 3 or not all(math.isfinite(v) for v in self.origin):
            raise ValueError("origin must be finite (x, y, yaw)")
        if not self.frame or not 0 < self.free_below <= self.occupied_at <= 100:
            raise ValueError("Invalid frame or occupancy thresholds")
        a = np.array(a, dtype=np.int16, copy=True)
        a.flags.writeable = False
        object.__setattr__(self, "data", a)

    def labels(self) -> np.ndarray:
        a = self.data
        result = np.full(a.shape, Cell.UNCERTAIN, dtype=np.uint8)
        result[a == -1] = Cell.UNKNOWN
        result[(a >= 0) & (a < self.free_below)] = Cell.FREE
        result[a >= self.occupied_at] = Cell.OCCUPIED
        return result

    def local_xy(self, x: float, y: float) -> tuple[float, float]:
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("Non-finite coordinates")
        ox, oy, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        return c*(x-ox)+s*(y-oy), -s*(x-ox)+c*(y-oy)

    def cell(self, x: float, y: float) -> tuple[int, int]:
        u, v = self.local_xy(x, y)
        return (math.floor(_snap_grid_coordinate(v/self.resolution)),
                math.floor(_snap_grid_coordinate(u/self.resolution)))

    def world(self, row: int, col: int) -> tuple[float, float]:
        u, v = (col+0.5)*self.resolution, (row+0.5)*self.resolution
        ox, oy, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        return ox+c*u-s*v, oy+s*u+c*v

    def inside(self, row: int, col: int) -> bool:
        return 0 <= row < self.data.shape[0] and 0 <= col < self.data.shape[1]

    def value(self, x: float, y: float) -> int | None:
        r, c = self.cell(x, y)
        return int(self.data[r, c]) if self.inside(r, c) else None

    def label_at(self, x: float, y: float) -> Cell:
        value = self.value(x, y)
        if value is None:
            return Cell.OUTSIDE
        if value == -1:
            return Cell.UNKNOWN
        if value < self.free_below:
            return Cell.FREE
        if value >= self.occupied_at:
            return Cell.OCCUPIED
        return Cell.UNCERTAIN

    def frontier_mask(self, include_map_edge: bool = False) -> np.ndarray:
        """FREE cells with a four-connected genuine UNKNOWN neighbor.

        Map-edge seeds may be enabled separately, but outside cells are not
        counted as known-size information gain or evidence of free space.
        """
        labels = self.labels()
        u = np.pad(labels == Cell.UNKNOWN, 1, constant_values=include_map_edge)
        adjacent = u[:-2, 1:-1] | u[2:, 1:-1] | u[1:-1, :-2] | u[1:-1, 2:]
        return (labels == Cell.FREE) & adjacent

    def counts(self) -> dict[str, int]:
        a = self.labels()
        return {k.name.lower(): int(np.count_nonzero(a == k)) for k in Cell if k != Cell.OUTSIDE}


def sensor_pose(base_pose: tuple[float, float, float],
                offset: tuple[float, float, float]) -> tuple[float, float, float]:
    """Compose base->sensor 2D TF. Supply an actual, current calibration."""
    if len(base_pose) != 3 or len(offset) != 3 or not all(
        math.isfinite(v) for v in (*base_pose, *offset)
    ):
        raise ValueError("Expected two finite (x,y,yaw) poses")
    x, y, a = base_pose
    dx, dy, da = offset
    return x+math.cos(a)*dx-math.sin(a)*dy, y+math.sin(a)*dx+math.cos(a)*dy, a+da


def ray_cells(grid: Grid, x: float, y: float, angle: float,
              range_m: float) -> Iterator[tuple[int, int]]:
    """Grid DDA, including side cells at corner crossings (conservative).

    Used only for information scoring, NEVER for motion clearance.
    """
    if not math.isfinite(angle) or not math.isfinite(range_m) or range_m <= 0:
        raise ValueError("Invalid ray parameters")
    u, v = grid.local_xy(x, y)
    gx = _snap_grid_coordinate(u/grid.resolution)
    gy = _snap_grid_coordinate(v/grid.resolution)
    col, row = math.floor(gx), math.floor(gy)
    local_angle = angle-grid.origin[2]
    dx, dy = math.cos(local_angle), math.sin(local_angle)
    sx = 1 if dx > 1e-14 else (-1 if dx < -1e-14 else 0)
    sy = 1 if dy > 1e-14 else (-1 if dy < -1e-14 else 0)
    tx = ((col+1-gx) if sx > 0 else (gx-col))*grid.resolution/abs(dx) if sx else math.inf
    ty = ((row+1-gy) if sy > 0 else (gy-row))*grid.resolution/abs(dy) if sy else math.inf
    dtx = grid.resolution/abs(dx) if sx else math.inf
    dty = grid.resolution/abs(dy) if sy else math.inf
    yield row, col
    while min(tx, ty) <= range_m:
        if abs(tx-ty) < 1e-10:
            yield row, col+sx
            yield row+sy, col
            col += sx
            row += sy
            tx += dtx
            ty += dty
        elif tx < ty:
            col += sx
            tx += dtx
        else:
            row += sy
            ty += dty
        yield row, col


@dataclass(frozen=True)
class Visibility:
    first_unknown_cells: frozenset[tuple[int, int]]
    proxy_m2: float
    rays_hit_occupied: int
    rays_hit_uncertain: int
    rays_leave_map: int
    rays_reach_range: int


def visible_unknown_boundary(grid: Grid, sensor: tuple[float, float, float],
                             range_m: float = 12.0, rays: int = 180,
                             fov_rad: float = 2*math.pi) -> Visibility:
    """First-unknown-cell proxy, not predicted true area or information entropy.

    A ray stops at the first occupied, uncertain, or unknown cell. It does not
    assume that hidden unknown cells behind the first unknown are transparent.
    Full-circle 2D sensing does not create new geometry merely by changing yaw
    when the sensor center is unchanged. Finite ray discretization can differ.
    """
    if rays < 4 or not math.isfinite(fov_rad) or not 0 < fov_rad <= 2*math.pi:
        raise ValueError("Invalid ray count or field of view")
    x, y, yaw = sensor
    if grid.label_at(x, y) != Cell.FREE:
        raise ValueError("Information scoring requires a known-free sensor origin")
    labels = grid.labels()
    seen: set[tuple[int, int]] = set()
    occupied = uncertain = outside = reached = 0
    for a in np.linspace(yaw-fov_rad/2, yaw+fov_rad/2, rays, endpoint=False):
        for r, c in ray_cells(grid, x, y, float(a), range_m):
            if not grid.inside(r, c):
                outside += 1
                break
            label = labels[r, c]
            if label == Cell.OCCUPIED:
                occupied += 1
                break
            if label == Cell.UNCERTAIN:
                uncertain += 1
                break
            if label == Cell.UNKNOWN:
                seen.add((r, c))
                break
        else:
            reached += 1
    return Visibility(frozenset(seen), len(seen)*grid.resolution**2,
                      occupied, uncertain, outside, reached)


def _evidence_label(grid: Grid, x: float, y: float) -> int:
    # Both outside the allocated array and explicit -1 mean unobserved for
    # memory comparison. Merely allocating unknown cells is not new evidence.
    label = grid.label_at(x, y)
    return int(Cell.UNKNOWN if label == Cell.OUTSIDE else label)


def capture_patch(grid: Grid, center: tuple[float, float], radius_m: float = 1.5,
                  spacing_m: float = 0.25) -> dict[str, Any]:
    """Fixed WORLD sample points; does not depend on current array indices."""
    if not math.isfinite(radius_m) or radius_m <= 0 or not math.isfinite(spacing_m) or spacing_m <= 0:
        raise ValueError("Invalid patch dimensions")
    n = math.ceil(radius_m/spacing_m)
    if (2*n+1)**2 > 100_000:
        raise ValueError("Patch too large")
    points = [(center[0]+i*spacing_m, center[1]+j*spacing_m)
              for j in range(-n, n+1) for i in range(-n, n+1)]
    values = [_evidence_label(grid, x, y) for x, y in points]
    return {"frame": grid.frame, "points": points, "labels": values,
            "digest": hashlib.sha256(bytes(values)).hexdigest()}


def patch_change(grid: Grid, patch: dict[str, Any], min_cells: int = 3,
                 min_fraction: float = 0.05) -> tuple[bool, int, float]:
    """A change permits a NEW safety check; it never grants motion permission."""
    if patch["frame"] != grid.frame:
        raise ValueError("Frame changed; invalidate/remap exploration memory explicitly")
    points, before = patch["points"], patch["labels"]
    if not points or len(points) != len(before):
        raise ValueError("Malformed evidence patch")
    changes = sum(_evidence_label(grid, x, y) != old for (x, y), old in zip(points, before))
    fraction = changes/len(points)
    return changes >= min_cells and fraction >= min_fraction, changes, fraction


@dataclass(frozen=True)
class MapDelta:
    newly_observed_m2: float
    lost_observed_m2: float
    net_known_area_m2: float


def observed_delta(before: Grid, after: Grid, same_map_epoch: bool) -> MapDelta:
    """Count unknown/outside -> observed without crediting mere map resizing.

    Supports equal resolution, equal yaw and integer-cell origin shifts only.
    Nonrigid SLAM map revisions must NOT be compared using this function.
    Nonnegative raw occupancy means observed, including intermediate values.
    """
    if not same_map_epoch or before.frame != after.frame:
        raise ValueError("Different map epoch/frame: gain is not comparable")
    if abs(before.resolution-after.resolution) > 1e-9 or abs(
        math.remainder(before.origin[2]-after.origin[2], 2*math.pi)
    ) > 1e-9:
        raise ValueError("Reprojection/alignment required")
    x, y = before.local_xy(after.origin[0], after.origin[1])
    shifts = np.array([y, x])/before.resolution
    rounded = np.rint(shifts).astype(int)
    if not np.allclose(shifts, rounded, atol=1e-5, rtol=0):
        raise ValueError("Non-cell-aligned origin shift: gain is unknown")
    dy, dx = map(int, rounded)
    old_known = before.data >= 0
    new_known = after.data >= 0
    overlap = 0
    r0, c0 = max(0, -dy), max(0, -dx)
    r1 = min(after.data.shape[0], before.data.shape[0]-dy)
    c1 = min(after.data.shape[1], before.data.shape[1]-dx)
    if r1 > r0 and c1 > c0:
        overlap = int(np.count_nonzero(new_known[r0:r1, c0:c1] &
                            old_known[r0+dy:r1+dy, c0+dx:c1+dx]))
    area = before.resolution**2
    added = (int(new_known.sum())-overlap)*area
    lost = (int(old_known.sum())-overlap)*area
    return MapDelta(added, lost, added-lost)

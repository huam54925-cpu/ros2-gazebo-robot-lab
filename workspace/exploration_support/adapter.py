"""Read-only enrichment of existing Frontier dictionaries. No ROS dependency."""
from __future__ import annotations
from typing import Any, Sequence
from .grid import Grid, sensor_pose, visible_unknown_boundary


def enrich_existing_candidates(
    grid: Grid,
    robot_base_pose: tuple[float, float, float],
    candidates: Sequence[dict[str, Any]],
    base_to_lidar: tuple[float, float, float],
    range_m: float = 12.0,
    rays: int = 180,
) -> list[dict[str, Any]]:
    """Keep existing IDs and scores; add a separately named visibility proxy.

    Supply actual current base->lidar TF, and use the SAME raw map snapshot for
    the baseline and every candidate. Any exception should yield diagnosis/no
    new command, never a fall-through that bypasses existing execution checks.
    A zero proxy is a limited estimator result, NOT proof of no useful view.
    """
    current = visible_unknown_boundary(
        grid, sensor_pose(robot_base_pose, base_to_lidar), range_m, rays
    )
    output = []
    for item in candidates:
        row = dict(item)
        sensor = sensor_pose((float(row["x"]), float(row["y"]), float(row["yaw"])), base_to_lidar)
        view = visible_unknown_boundary(grid, sensor, range_m, rays)
        new = view.first_unknown_cells-current.first_unknown_cells
        row.update({
            "visible_unknown_boundary_proxy_m2": view.proxy_m2,
            "incremental_boundary_proxy_m2": len(new)*grid.resolution**2,
            "visibility_method": "first_unknown_supercover_v1",
            "visibility_range_m": range_m,
            "visibility_rays": rays,
            "ray_stops": {
                "occupied": view.rays_hit_occupied,
                "uncertain": view.rays_hit_uncertain,
                "outside_map": view.rays_leave_map,
                "range_limit": view.rays_reach_range,
            },
            "unknown_cause": "not_identified_from_map_alone",
            "visibility_is_safety_certificate": False,
        })
        output.append(row)
    return output

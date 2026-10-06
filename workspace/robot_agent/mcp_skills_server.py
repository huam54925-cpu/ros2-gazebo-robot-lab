"""Legacy tools plus mission-scoped persistent investigation capabilities."""
import asyncio
import os
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_skills.api import RobotSkills
from mcp.server.mcpserver import MCPServer
from mcp.types import ToolAnnotations
from mcp.server.mcpserver.tools import Tool

skills = RobotSkills(mission_id=os.environ.get('ROBOT_MISSION_ID'),
                     observations=os.environ.get('ROBOT_OBSERVATIONS_ENABLED') == '1')
registered_tools = []
READ = ToolAnnotations(read_only_hint=True, destructive_hint=False, open_world_hint=False)
WRITE = ToolAnnotations(read_only_hint=False, destructive_hint=False, idempotent_hint=True, open_world_hint=False)


def strict_tool(annotations):
    def decorate(function):
        tool = Tool.from_function(function, annotations=annotations)
        model = tool.fn_metadata.arg_model
        model.model_config['extra'] = 'forbid'
        model.model_rebuild(force=True)
        tool.parameters = model.model_json_schema(by_alias=True)
        registered_tools.append(tool)
        return function
    return decorate


@strict_tool(annotations=READ)
def get_robot_state() -> dict:
    """Read live pose, map, scan, navigation/guard observations and freshness."""
    return skills.get_robot_state()


@strict_tool(annotations=READ)
def get_decision_context() -> dict:
    """Read cached candidates, robot state and recent/active tasks. No planning or motion."""
    return skills.get_decision_context()


@strict_tool(annotations=READ)
async def get_safe_frontiers() -> dict:
    """Get locally validated reachable Frontier IDs; may compute paths, never move."""
    return await asyncio.to_thread(skills.get_safe_frontiers)


@strict_tool(annotations=WRITE)
def execute_frontier(frontier_id: str, request_id: str) -> dict:
    """Submit one candidate ID with a UUID; accepted is not success. Poll task_id.
    Local executor replans, rechecks clearance and rejects expired/unsafe candidates.
    """
    return skills.execute_frontier(frontier_id, request_id)


@strict_tool(annotations=READ)
def get_task_status(task_id: str) -> dict:
    """Read authoritative local task status including confirmed stopped and sensor freshness."""
    return skills.get_task_status(task_id)


@strict_tool(annotations=WRITE)
def stop_robot(request_id: str) -> dict:
    """Latch out new movement, cancel active goals and pause Nav2; poll task_id for confirmed stop.
    Movement stays latched until an explicit local operator resume (not an MCP tool).
    """
    return skills.stop_robot(request_id)


if skills.observations:
    @strict_tool(annotations=READ)
    async def get_observation_options() -> dict:
        """Generate finite safe observation IDs; planning only, no movement."""
        return await asyncio.to_thread(skills.get_observation_options)

    @strict_tool(annotations=WRITE)
    def perform_observation(option_id: str, request_id: str) -> dict:
        """Execute one local observation ID; no arbitrary angles, coordinates or speeds."""
        return skills.perform_observation(option_id, request_id)


if skills.investigations_enabled():
    @strict_tool(annotations=WRITE)
    def view_map(request_id: str) -> dict:
        """Start an online SLAM snapshot; poll task_id for PNG, metric transform and unknown regions."""
        return skills.view_map(request_id)

    @strict_tool(annotations=WRITE)
    def start_candidate_search(request_id: str) -> dict:
        """Start a read-only planning search, returning task_id immediately; never moves."""
        return skills.start_candidate_search(request_id)

    @strict_tool(annotations=WRITE)
    def plan_navigation(poses: list[dict[str, float]], map_epoch: str, request_id: str) -> dict:
        """Query alternatives from current pose. Poses have x,y,yaw in map; no motion or permission grant."""
        return skills.plan_navigation(poses,map_epoch,request_id)

    @strict_tool(annotations=WRITE)
    def start_investigation(subject: dict[str, float], hypothesis: str, task_type: str,
                            request_id: str, passage: dict | None = None) -> dict:
        """Persist a question: observe_structure or verify_passage. Unknowns remain hypotheses.
        passage requires a,b endpoint XY arrays and destination_side (-1 or 1).
        """
        return skills.start_investigation(subject,hypothesis,task_type,request_id,passage)

    @strict_tool(annotations=WRITE)
    def navigate_to_pose(investigation_id: str, pose: dict[str, float], map_epoch: str, request_id: str) -> dict:
        """Submit a checked map goal within an active investigation. Poll the returned task_id."""
        return skills.navigate_to_pose(investigation_id,pose,map_epoch,request_id)

    @strict_tool(annotations=WRITE)
    def navigate_through_poses(investigation_id: str, poses: list[dict[str, float]], map_epoch: str, request_id: str) -> dict:
        """Execute ordered poses with Nav2; each leg replans and confirms stop. Maximum 32 per request."""
        return skills.navigate_through_poses(investigation_id,poses,map_epoch,request_id)

    @strict_tool(annotations=WRITE)
    def probe_forward(investigation_id: str, map_epoch: str, request_id: str) -> dict:
        """Low-speed short step along current heading, only through confirmed free swept body space.
        Local policy chooses 5-20 cm at 0.04 m/s. Poll task_id; no speed or safety overrides.
        """
        return skills.probe_forward(investigation_id,map_epoch,request_id)

    @strict_tool(annotations=WRITE)
    def recover_short_reverse(investigation_id: str, map_epoch: str, source_task_id: str, request_id: str) -> dict:
        """One protected reverse along recent straight odometry after an owned stopped eligible failure.
        Local policy caps distance, speed and attempts. Automatic recovery may have already consumed it.
        Never unlocks operator stop or resumes paused navigation. Poll task_id.
        """
        return skills.recover_short_reverse(investigation_id,map_epoch,source_task_id,request_id)

    @strict_tool(annotations=WRITE)
    def finish_investigation(investigation_id: str, outcome: str, assessment: str, evidence_task_ids: list[str]) -> dict:
        """Record observations_collected, passage_verified, blocked, or unresolved with local evidence IDs.
        Assessment is interpretation, not map truth. Navigation success alone is not structure confirmation.
        """
        return skills.finish_investigation(investigation_id,outcome,assessment,evidence_task_ids)

    @strict_tool(annotations=WRITE)
    def cancel_task(task_id: str) -> dict:
        """Request cancellation of own task; poll until terminal and stopped. Does not release stop latch."""
        return skills.cancel_task(task_id)


server = MCPServer('robot-skills', log_level='WARNING', tools=registered_tools)

if __name__ == '__main__':
    server.run(transport='stdio')

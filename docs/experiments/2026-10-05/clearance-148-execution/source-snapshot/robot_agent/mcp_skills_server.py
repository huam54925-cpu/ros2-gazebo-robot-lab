"""Exactly six tools; coordinates and control parameters are not accepted."""
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


server = MCPServer('robot-skills', log_level='WARNING', tools=registered_tools)

if __name__ == '__main__':
    server.run(transport='stdio')

"""Explicit opt-in MCP server for one bounded simulation movement."""
import asyncio
from mcp.server.mcpserver import MCPServer
from mcp.types import ToolAnnotations
from motion_trial import move_forward_trial as move, cancel_motion_trial as cancel
from robot_status import get_robot_status as read

server = MCPServer('robot-motion-trial', log_level='WARNING')
consumed_id = None


@server.tool(annotations=ToolAnnotations(read_only_hint=True))
def get_robot_status() -> dict:
    """Read robot state; no control action."""
    result = read()
    result['motion_tools_enabled'] = True
    result['motion_scope'] = {'simulation_only': True, 'maximum_trials_per_session': 1,
                               'requested_distance_m': 0.4, 'goal_timeout_wall_s': 45}
    return result


@server.tool(annotations=ToolAnnotations(read_only_hint=False, destructive_hint=False,
                                         idempotent_hint=True, open_world_hint=False))
async def move_forward_trial(request_id: str) -> dict:
    """Try one 0.4 m forward Nav2 goal in simulation. Reuse the UUID to prevent replay.
    Local full-path checks and runtime guard may reject or abort. Returns confirmed stop status.
    """
    global consumed_id
    if consumed_id is not None and consumed_id != request_id:
        return {'status': 'rejected', 'reason': 'one_trial_per_session'}
    consumed_id = request_id
    return await asyncio.to_thread(move, request_id)


@server.tool(annotations=ToolAnnotations(read_only_hint=False, destructive_hint=False,
                                         idempotent_hint=True, open_world_hint=False))
def cancel_motion_trial(request_id: str) -> dict:
    """Request cancellation for this trial. An acknowledgement is not a confirmed stop."""
    return cancel(request_id)


if __name__ == '__main__':
    server.run(transport='stdio')

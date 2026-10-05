"""Local stdio MCP server exposing only the robot status tool."""
from mcp.server.mcpserver import MCPServer
from mcp.types import ToolAnnotations

from robot_status import get_robot_status as read_status

server = MCPServer('robot-readonly', instructions='Read-only robot observations. No motion tools.',
                   log_level='WARNING')


@server.tool(annotations=ToolAnnotations(read_only_hint=True, destructive_hint=False,
                                         idempotent_hint=True, open_world_hint=False))
def get_robot_status() -> dict:
    """Get live robot pose, map summary, scan, guard observations and navigation status with freshness."""
    return read_status()


if __name__ == '__main__':
    server.run(transport='stdio')

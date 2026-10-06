"""Small direct-motion MCP interface for the explicitly unprotected Gazebo run."""
import json
import os
from pathlib import Path
import subprocess
import time
import uuid
from mcp.server.mcpserver import MCPServer
from mcp.server.mcpserver.tools import Tool

ROOT=Path(__file__).resolve().parents[2]
RUN=Path(os.environ['SCAN_DRIVE_RUN_DIRECTORY'])
CONFIG=json.loads((RUN/'config.json').read_text())
CONTAINER_RUN='/work/robot_ws/'+str(RUN.relative_to(ROOT/'workspace'))
registered=[]


def action(kind, **arguments):
    remaining=CONFIG['deadline_unix_s']-time.time()
    if remaining<=0 and kind!='stop':return {'status':'failed','reason':'run_wall_budget'}
    identifier=uuid.uuid4().hex
    request={'action':kind,'action_id':identifier,'directory':CONTAINER_RUN,
             'wall_remaining_s':max(8.,remaining) if kind=='stop' else remaining,**arguments}
    path=RUN/(identifier+'.request.json');path.write_text(json.dumps(request))
    command=['docker','exec','robot-sim-gui','bash','-lc',
        'source /opt/ros/lyrical/setup.bash; exec python3 /work/robot_ws/robot_agent/scan_drive_worker.py "$@"',
        'bash','--request',CONTAINER_RUN+'/'+path.name]
    result=subprocess.run(command,capture_output=True,text=True,timeout=max(20.,remaining+20))
    (RUN/(identifier+'.worker.log')).write_text(result.stdout+'\n'+result.stderr)
    output=RUN/(identifier+'.result.json')
    if not output.exists():return {'status':'failed','reason':'motion_worker_failed','returncode':result.returncode}
    return json.loads(output.read_text())


def tool(function):
    entry=Tool.from_function(function)
    model=entry.fn_metadata.arg_model;model.model_config['extra']='forbid';model.model_rebuild(force=True)
    entry.parameters=model.model_json_schema(by_alias=True);registered.append(entry);return function


@tool
def scan_surroundings() -> dict:
    """Rotate a full 360 degrees, stop, and return online SLAM plus directional laser ranges."""
    return action('scan')


@tool
def drive_half_visible_range(heading_map_rad: float, rationale: str) -> dict:
    """Choose any map-frame heading in radians. Turn to that direction and drive straight
    for half its current laser range (infinity uses sensor range_max). No Nav2 or collision
    check is active in this Gazebo experiment. Completion is followed by another full scan.
    rationale briefly explains the next area to observe; it is not a safety approval.
    """
    return action('drive',heading_map_rad=heading_map_rad,rationale=rationale)


@tool
def stop_exploration(reason: str) -> dict:
    """Finish this experiment and send zero velocity. Do not stop merely for low immediate map gain."""
    return {**action('stop'),'finish_reason':reason}


server=MCPServer('gazebo-scan-drive',tools=registered,log_level='WARNING')
if __name__=='__main__':server.run(transport='stdio')

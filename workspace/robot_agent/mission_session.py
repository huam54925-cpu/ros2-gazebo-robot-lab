"""Lifecycle, MCP session and watchdog for the single two-stage runtime."""
import argparse
import asyncio
import contextlib
import json
import math
import os
import signal
from pathlib import Path
import subprocess
import sys
import time
import uuid

import httpx2
import openai
from mcp import ClientSession
from mcp.client.stdio import StdioServerParameters, stdio_client
from check_environment import configuration
from run_readonly import safe_error

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from robot_skills.api import RobotSkills
from robot_skills import mission
from robot_skills.provenance import source_manifest
BASE_TOOLS={'get_robot_state','get_decision_context','get_task_status','stop_robot'}


def unpack(result):
    if result.is_error:raise RuntimeError('mcp_tool_failed')
    if result.structured_content is not None:return result.structured_content
    return json.loads(''.join(c.text for c in result.content if c.type=='text'))


def install_termination_handler():
    """SIGTERM must reach the coroutine's stop/final-log block, like Ctrl-C.

    The detached watchdog remains the fallback for SIGKILL or a frozen client.
    Ignore repeated SIGTERM while orderly stop confirmation is in progress.
    """
    loop = asyncio.get_running_loop()
    task = asyncio.current_task()
    requested = False
    def cancel_once():
        nonlocal requested
        if not requested:
            requested = True
            task.cancel()
    loop.add_signal_handler(signal.SIGTERM, cancel_once)
    return lambda: loop.remove_signal_handler(signal.SIGTERM)


async def run(args, report, output):
    skills=RobotSkills(); store=skills.store
    budget=mission.limits(None,args.max_distance,args.max_sim_time,args.max_wall_time,args.max_failures,profile='mission')
    m=mission.start(store,budget,skills.get_robot_state(),two_stage=True,ai_reserve_fraction=args.ai_reserve_fraction)
    mid=m['mission_id'];skills=RobotSkills(store=store,mission_id=mid)
    report.update(mission_id=mid,limits=budget,rounds=[],policy='two_stage',
                  ai_reserve=m['ai_reserve'],source_sha256=source_manifest())
    def checkpoint():
        current=store.meta('mission:'+mid)
        if current and current['limits']!=report['limits']:
            report.setdefault('initial_limits',report['limits'])
            report['limits']=current['limits']
            report['budget_changes']=current.get('budget_changes',[])
        temporary=output.with_suffix(output.suffix+'.tmp')
        temporary.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n'); temporary.replace(output)
    def end(why): mission.update(store,mid,status='stopping',stop_reason=why)
    def session_state():
        current=store.meta('mission:'+mid); tasks=store.tasks()
        state=skills.get_robot_state(); sim=state.get('sources',{}).get('clock',{}).get('sim_time_s')
        why=mission.reason(current,tasks,sim)
        if why and current['status']=='running': current=mission.update(store,mid,status='stopping',stop_reason=why)
        return {**current,'progress':mission.progress(current,tasks)}
    checkpoint()
    with (store.directory/(mid+'.watchdog.log')).open('w') as log:
        try:
            subprocess.Popen([sys.executable,str(HERE.parent/'robot_skills/mission_watchdog.py'),mid],
                             stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True,close_fds=True)
        except OSError:
            end('watchdog_start_failed'); skills.stop_robot(m['stop_request_id']); raise
    async def heartbeat():
        while True:
            mission.update(store,mid,heartbeat_monotonic_s=time.monotonic())
            await asyncio.sleep(1)
    beat=asyncio.create_task(heartbeat())
    remove_termination_handler=install_termination_handler()
    try:
        env={**os.environ,'ROBOT_MISSION_ID':mid}
        params=StdioServerParameters(command=sys.executable,args=[str(HERE/'mcp_skills_server.py')],env=env)
        tool_timeout=260
        async with stdio_client(params) as streams:
            async with ClientSession(*streams,read_timeout_seconds=tool_timeout) as session:
                await session.initialize()
                from investigation_loop import TOOLS as INVESTIGATION_TOOLS
                allowed=BASE_TOOLS | INVESTIGATION_TOOLS | {'execute_frontier'}
                definitions=(await session.list_tools()).tools
                if {t.name for t in definitions} != allowed: raise RuntimeError('unexpected_tools')
                async def call(name, arguments=None):
                    if name not in allowed: raise ValueError('tool_not_allowed')
                    response=await session.call_tool(name,arguments or {},read_timeout_seconds=tool_timeout)
                    if response.is_error:
                        return {'status':'request_rejected','reason':''.join(
                            c.text for c in response.content if c.type=='text')[:1500]}
                    return unpack(response)
                key,base_url,model=configuration();report['model']=model
                if not key or not model:raise ValueError('missing_api_key_or_model')
                async with httpx2.AsyncClient(follow_redirects=False) as http:
                    async with openai.AsyncOpenAI(api_key=key,base_url=base_url,timeout=60,max_retries=0,http_client=http) as client:
                        from investigation_loop import run as run_investigations
                        await run_investigations(call,definitions,skills,session_state,end,checkpoint,report,client,model)

    except (Exception, asyncio.CancelledError) as error:
        report['error']=({'status':'client_interrupted'} if isinstance(error,asyncio.CancelledError) else safe_error(error))
        end('client_interrupted_or_error')
    finally:
        end('loop_completed')
        # Explicit stop also covers cancellation of this coroutine; watchdog owns completion.
        skills.stop_robot(m['stop_request_id'])
        deadline=time.monotonic()+85
        while time.monotonic()<deadline:
            current=store.meta('mission:'+mid)
            if current['status']=='finished': break
            await asyncio.sleep(.5)
        report['mission']=store.meta('mission:'+mid); report['final_state']=skills.get_robot_state()
        beat.cancel()
        with contextlib.suppress(asyncio.CancelledError): await beat
        checkpoint()
        remove_termination_handler()
    return report

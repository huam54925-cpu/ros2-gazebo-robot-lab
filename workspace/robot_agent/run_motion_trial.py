"""Explicit one-shot model movement trial; no background model loop."""
import argparse
import asyncio
import json
from pathlib import Path
import sys
import uuid

import httpx2
import openai
from mcp import ClientSession
from mcp.client.stdio import StdioServerParameters, stdio_client

from check_environment import configuration
from motion_trial import cancel_motion_trial, identifier
from run_readonly import safe_error

HERE = Path(__file__).resolve().parent
TOOL = {'type': 'function', 'name': 'move_forward_trial',
        'description': 'Try exactly one locally checked 0.4 m forward Nav2 movement in simulation and stop. May reject or abort.',
        'parameters': {'type': 'object', 'properties': {'request_id': {'type': 'string'}},
                       'required': ['request_id'], 'additionalProperties': False}, 'strict': True}
INSTRUCTIONS = ('你是受限的仿真机器人助手。用户授权一次0.4米前进测试。只调用一次指定工具，'
                '必须使用给定的request_id。工具可拒绝；不得绕过安全检查或重试其他目标。'
                '最终按工具结果报告实际距离、成功/取消/拒绝和停车是否确认，不能把请求成功当成运动成功。')


async def run(request_id, use_model):
    parameters = StdioServerParameters(command=sys.executable, args=[str(HERE / 'mcp_motion_server.py')])
    async with stdio_client(parameters) as streams:
        async with ClientSession(*streams, read_timeout_seconds=200) as session:
            await session.initialize()
            catalog = await session.list_tools()
            if set(tool.name for tool in catalog.tools) != {'get_robot_status', 'move_forward_trial', 'cancel_motion_trial'}:
                raise RuntimeError('unexpected_mcp_tools')
            async def invoke(name, arguments):
                if name != 'move_forward_trial' or arguments != {'request_id': request_id}:
                    raise ValueError('tool_not_allowed')
                result = await session.call_tool(name, arguments, read_timeout_seconds=200)
                if result.is_error:
                    raise RuntimeError('mcp_tool_failed')
                if result.structured_content is not None:
                    return result.structured_content
                return json.loads(''.join(item.text for item in result.content if item.type == 'text'))
            report = {'request_id': request_id, 'model_invoked': use_model}
            if not use_model:
                report['motion'] = await invoke('move_forward_trial', {'request_id': request_id})
                return report
            key, base_url, model = configuration()
            if not key or not model:
                raise ValueError('missing_api_key_or_model')
            report['model'] = model
            status = await session.call_tool('get_robot_status', {}, read_timeout_seconds=15)
            prompt = [{'role': 'user', 'content': '尝试前进0.4米然后停车。request_id=' + request_id +
                       '。只读状态：' + json.dumps(status.structured_content, ensure_ascii=False)}]
            async with httpx2.AsyncClient(follow_redirects=False) as http_client:
                async with openai.AsyncOpenAI(api_key=key, base_url=base_url, timeout=90,
                                              max_retries=0, http_client=http_client) as client:
                    first = await client.responses.create(model=model, input=prompt, instructions=INSTRUCTIONS,
                        tools=[TOOL], tool_choice={'type': 'function', 'name': 'move_forward_trial'},
                        parallel_tool_calls=False, store=False, max_output_tokens=1200)
                    calls = [item for item in first.output if item.type == 'function_call']
                    if first.status != 'completed' or len(calls) != 1:
                        raise RuntimeError('model_did_not_complete_one_tool_call')
                    call = calls[0]
                    motion = await invoke(call.name, json.loads(call.arguments))
                    report['motion'] = motion
                    prompt.extend(first.output)
                    prompt.append({'type': 'function_call_output', 'call_id': call.call_id,
                                   'output': json.dumps(motion, ensure_ascii=False)})
                    # Preserve the authoritative movement result if model explanation fails.
                    try:
                        second = await client.responses.create(model=model, input=prompt, instructions=INSTRUCTIONS,
                            tools=[TOOL], tool_choice='none', store=False, max_output_tokens=1000)
                        report.update(summary=second.output_text, response_status=second.status)
                    except Exception as error:
                        report['explanation_error'] = safe_error(error)
            return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', action='store_true')
    parser.add_argument('--request-id', default=None)
    parser.add_argument('--cancel', metavar='REQUEST_ID')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    request_id = identifier(args.cancel or args.request_id or str(uuid.uuid4()))
    print('Motion trial request_id: ' + request_id, file=sys.stderr, flush=True)
    try:
        report = cancel_motion_trial(request_id) if args.cancel else asyncio.run(run(request_id, args.model))
    except (Exception, KeyboardInterrupt) as error:
        cancel_motion_trial(request_id)
        report = {**safe_error(error), 'request_id': request_id, 'cancel_requested': True}
    encoded = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    output = args.output or HERE.parents[1] / 'logs' / ('motion-trial-' + request_id + '.json')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(encoded)
    print(encoded)
    motion = report.get('motion') or {}
    return 0 if args.cancel or (motion.get('stopped') and motion.get('status') == 'succeeded') else 1


if __name__ == '__main__':
    raise SystemExit(main())

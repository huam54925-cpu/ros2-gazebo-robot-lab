"""Read status or explicitly stop/resume; exploration uses run_investigation.py."""
import argparse
import json
from pathlib import Path
import sys
import uuid
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=['state','context','status','stop','resume','recover'])
    parser.add_argument('id',nargs='?')
    parser.add_argument('--request-id')
    args=parser.parse_args()
    if args.command in ('status','recover') and not args.id:parser.error('task_id required')
    from robot_skills.api import RobotSkills
    skills=RobotSkills()
    actions={'state':skills.get_robot_state,'context':skills.get_decision_context,
             'status':lambda:skills.get_task_status(args.id),
             'stop':lambda:skills.stop_robot(args.request_id or str(uuid.uuid4())),
             'resume':skills.resume_operator_only,'recover':lambda:skills.recover_operator_only(args.id)}
    result=actions[args.command]()
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()

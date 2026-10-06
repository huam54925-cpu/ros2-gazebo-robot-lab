"""Explicit local entry point for two-stage exploration. Never auto-started by MCP."""
import argparse
import asyncio
import json
from pathlib import Path
import uuid


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--max-distance',type=float,default=80.)
    parser.add_argument('--max-sim-time',type=float,default=900.)
    parser.add_argument('--max-wall-time',type=float,default=3600.)
    parser.add_argument('--max-failures',type=int,default=12)
    parser.add_argument('--ai-reserve-fraction',type=float,default=.3,help='Reserve this fraction of time, distance and failure budget for AI (0 < fraction < 1).')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    # Importing this file or reviewing the CLI does not create missions/workers.
    from mission_session import run
    output=args.output or Path(__file__).resolve().parents[2]/'logs'/('investigation-'+str(uuid.uuid4())+'.json')
    output.parent.mkdir(parents=True,exist_ok=True)
    report={'mode':'two_stage','runtime_validation':'not_previously_verified'}
    asyncio.run(run(args,report,output))
    print(json.dumps({'output':str(output),'mission':report.get('mission')},ensure_ascii=False))
    return 0 if report.get('mission',{}).get('stopped') and not report.get('error') else 1


if __name__=='__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""Record Gazebo ground truth for later evaluation only; never controls the robot."""
import argparse
import json
import math
from pathlib import Path
import time
from gz.transport import Node
from gz.msgs.pose_v_pb2 import Pose_V


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
    output=Path(a.output);output.parent.mkdir(parents=True,exist_ok=True);last=[-math.inf]
    def callback(message):
        stamp=message.header.stamp.sec+message.header.stamp.nsec*1e-9
        if stamp-last[0]<.2:return
        model=next((pose for pose in message.pose if pose.name=='vehicle'),None)
        if model is None:return
        q=model.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        axle=[.5542825,0.]
        row={'wall_unix_s':time.time(),'simulation_seconds':stamp,
             'model_world_pose':[model.position.x,model.position.y,yaw],
             'axle_world_pose':[model.position.x+math.cos(yaw)*axle[0],model.position.y+math.sin(yaw)*axle[0],yaw],
             'source':'Gazebo dynamic_pose; evaluation only; not fed to AI or motion'}
        with output.open('a') as stream:stream.write(json.dumps(row)+'\n')
        last[0]=stamp
    node=Node()
    if not node.subscribe(Pose_V,'/world/demo/dynamic_pose/info',callback):raise RuntimeError('truth_subscription_failed')
    while True:time.sleep(1)


if __name__=='__main__':main()

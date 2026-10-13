"""Persistent request-only Gazebo transport process (no subscriber callbacks)."""
import json
import sys
from google.protobuf import text_format
from gz.transport import Node
from gz.msgs.stringmsg_pb2 import StringMsg
from gz.msgs.boolean_pb2 import Boolean
from gz.msgs.empty_pb2 import Empty
from gz.msgs.world_control_pb2 import WorldControl

def main():
    node=Node()
    classes={'gz.msgs.Empty':Empty,'gz.msgs.StringMsg':StringMsg,'gz.msgs.WorldControl':WorldControl}
    for line in sys.stdin:
        try:
            q=json.loads(line);kind=classes[q['reqtype']];reply=StringMsg if q['reptype']=='StringMsg' else Boolean
            message=text_format.Parse(q['request'],kind())
            ok,response=node.request(q['name'],message,kind,reply,3000)
            result={'ok':ok,'data':response.data}
        except Exception as exc:result={'ok':False,'error':str(exc)}
        print(json.dumps(result),flush=True)
if __name__=='__main__':main()

"""One map identity for planning and status; includes geometry, not just cells."""
from array import array
import hashlib
import json


def map_version(message):
    info = message.info
    p, q = info.origin.position, info.origin.orientation
    geometry = [message.header.frame_id, info.width, info.height, info.resolution,
                p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    return hashlib.sha256(json.dumps(geometry, separators=(',', ':')).encode()
                          + array('b', message.data).tobytes()).hexdigest()[:20]

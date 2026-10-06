"""Connected unknown-boundary labels for visualization; no candidate planning."""
import numpy as np

def components(mask):
    seen=np.zeros_like(mask,dtype=bool);out=[];h,w=mask.shape
    for y,x in np.argwhere(mask):
        if seen[y,x]:continue
        todo=[(int(y),int(x))];seen[y,x]=True;points=[]
        while todo:
            yy,xx=todo.pop();points.append((yy,xx))
            for dy in [-1,0,1]:
                for dx in [-1,0,1]:
                    y2,x2=yy+dy,xx+dx
                    if 0<=y2<h and 0<=x2<w and mask[y2,x2] and not seen[y2,x2]:
                        seen[y2,x2]=True;todo.append((y2,x2))
        out.append(np.asarray(points))
    return out

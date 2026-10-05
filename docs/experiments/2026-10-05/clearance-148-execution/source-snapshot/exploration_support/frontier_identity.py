"""World-coordinate frontier lineage, independent of disposable candidate IDs.

Bounded spatial association is evidence of likely continuity, not a claim of
room identity/reachability. Split/merge lineage is diagnostic, never a ban.
"""
import math
import numpy as np


def associate(groups, previous=None, tolerance=.6):
    previous=previous or {}
    old=previous.get('clusters',[])
    points=[np.asarray(g,dtype=float).reshape(-1,2) for g in groups]
    links=[]
    for group in points:
        matches=[]
        for index,parent in enumerate(old):
            other=np.asarray(parent['points'])
            if not len(group) or not len(other):continue
            # Chunked distances bound temporary memory on large boundaries.
            a=np.full(len(group),np.inf);b=np.full(len(other),np.inf)
            for start in range(0,len(group),128):
                distances=np.linalg.norm(group[start:start+128,None,:]-other[None,:,:],axis=2)
                a[start:start+128]=distances.min(axis=1);b=np.minimum(b,distances.min(axis=0))
            overlap=min(int((a<=tolerance).sum()),int((b<=tolerance).sum()))
            if overlap>=min(3,len(group),len(other)) and max(float((a<=tolerance).mean()),float((b<=tolerance).mean()))>=.5:
                matches.append((index,overlap))
        links.append(matches)
    # Largest child keeps its parent's ID; other children receive new IDs and
    # retain ancestry. Merges retain all parents even when only one ID survives.
    owner={j:max((i for i,m in enumerate(links) if any(k==j for k,_ in m)),
                 key=lambda i:(dict(links[i])[j],len(points[i]),-i))
           for j in range(len(old)) if any(j==k for m in links for k,_ in m)}
    serial=previous.get('next_id',1);clusters=[]
    for i,(group,matches) in enumerate(zip(points,links)):
        parents=[old[j] for j,_ in matches]
        keep=sorted(((n,old[j]['id']) for j,n in matches if owner[j]==i),reverse=True)
        if keep:identity=keep[0][1]
        else:identity=f'cluster-{serial}';serial+=1
        lineage=sorted({identity}|{p['id'] for p in parents}|{v for p in parents for v in p.get('lineage',[])})
        split=any(sum(any(k==j for k,_ in m) for m in links)>1 for j,_ in matches)
        relation='split_merge' if split and len(parents)>1 else 'merge' if len(parents)>1 else 'split' if split else 'continued' if parents else 'new'
        clusters.append({'id':identity,'lineage':lineage,'parents':[p['id'] for p in parents],
                         'relation':relation,'points':group.tolist(),'centroid':group.mean(axis=0).tolist()})
    return {'next_id':serial,'clusters':clusters,'association_tolerance_m':tolerance,
            'scope':'current_map_world_coordinates; diagnostic_not_spatial_exclusion'}


def candidate_identity(candidate, state):
    if not state['clusters']:return {}
    x,y=candidate['x'],candidate['y']
    cluster=min(state['clusters'],key=lambda c:min(math.hypot(p[0]-x,p[1]-y) for p in c['points']))
    return {'frontier_cluster_id':cluster['id'],'frontier_lineage':cluster['lineage'],
            'frontier_relation':cluster['relation'],'frontier_centroid':cluster['centroid']}

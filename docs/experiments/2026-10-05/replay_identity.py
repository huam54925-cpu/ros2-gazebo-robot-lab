import json,sys,os
from pathlib import Path
import numpy as np
os.chdir(Path(__file__).resolve().parents[3])
sys.path[:0]=['workspace','workspace/navigation_base/exploration']
from aggressive import AggressiveMap
from frontier import components
from exploration_support.frontier_identity import associate
p=json.load(open('docs/experiments/2026-10-04/evidence/memory-trial-live.json'))
state={};rows=[]
for cat in [*[r['candidate_set'] for r in p['rounds']],p['last_candidate_set']]:
 cid=cat['catalog_id']
 with np.load('workspace/log/robot-skills/exploration-upgrade/'+cid+'.map.npz') as z:
  model=AggressiveMap(z['data'],float(z['resolution']),tuple(z['origin']))
 groups=[model.world(g) for g in components(model.frontier) if len(g)*model.resolution>=model.settings.minimum_frontier_length]
 state=associate(groups,state)
 rows.append({'catalog_id':cid,'clusters':[{k:v for k,v in c.items() if k!='points'} for c in state['clusters']]})
Path('docs/experiments/2026-10-05/evidence/frontier-identity-replay.json').write_text(json.dumps({'scope':'world-coordinate association of three recorded catalogs; advisory only','rounds':rows},indent=2)+'\n')
print([{ 'catalog':r['catalog_id'],'relations':[c['relation'] for c in r['clusters']]} for r in rows])

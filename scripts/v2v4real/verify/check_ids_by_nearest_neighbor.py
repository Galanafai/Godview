"""V2V4Real format check used for the Findings in docs/PHASE_C_HANDOFF.md.

Usage: python -I check_ids_by_nearest_neighbor.py $V2V4REAL_DIR/test1
"""
import sys,os,glob,collections
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from v2v4real_io import load_yaml
import numpy as np
root=sys.argv[1]
def yaw_of(T): return np.degrees(np.arctan2(T[1,0],T[0,0]))
for sc in sorted(glob.glob(root+'/*/')):
    frames=sorted(os.path.basename(p)[:-5] for p in glob.glob(sc+'0/*.yaml'))
    nn=[]; same_id=0; nn_pairs=0; ass_same=0; ass_pairs=0; idmap=collections.defaultdict(collections.Counter); dyaw=[]
    egoother=[]
    for f in frames:
        d=[load_yaml(f'{sc}{c}/{f}.yaml') for c in (0,1)]
        T=[np.array(x['lidar_pose']) for x in d]
        V=[x['vehicles'] or {} for x in d]
        # where is car1 in car0 frame, and is it labeled in car0?
        rel=np.linalg.inv(T[0])@T[1]
        p1in0=rel[:3,3]
        if V[0]:
            dd=min(np.hypot(*(np.array(v['location'][:2])-p1in0[:2])) for v in V[0].values()); egoother.append(dd)
        if not V[0] or not V[1]: continue
        L1=np.array([v['location'] for v in V[1].values()]); ids1=list(V[1])
        inv=np.linalg.inv(T[1])@T[0]
        for vid,v in V[0].items():
            p=(inv@np.r_[v['location'],1])[:3]
            dist=np.linalg.norm(L1[:,:2]-p[:2],axis=1); j=int(np.argmin(dist))
            nn.append(dist[j])
            if dist[j]<2.0:
                nn_pairs+=1; vid1=ids1[j]; idmap[vid][vid1]+=1
                same_id+= (vid==vid1)
                a0=v['ass_id']; a1=V[1][vid1]['ass_id']
                if a0!=-1 or a1!=-1: ass_pairs+=1; ass_same+=(a0==a1)
                y0=yaw_of(T[0])+v['angle'][1]; y1=yaw_of(T[1])+V[1][vid1]['angle'][1]
                dyaw.append((y0-y1+180)%360-180)
    nn=np.array(nn); dyaw=np.array(dyaw)
    print(os.path.basename(sc[:-1]))
    print('  car0 labels -> car1 frame, NN dist: median %.2f, frac<1m %.2f, frac<2m %.2f'%(np.median(nn),(nn<1).mean(),(nn<2).mean()))
    print('  matched(<2m) pairs %d; same numeric id %d; pairs with ass_id set %d, ass_id equal %d'%(nn_pairs,same_id,ass_pairs,ass_same))
    print('  id map car0->car1 (top):',{k:dict(v.most_common(2)) for k,v in list(idmap.items())[:8]})
    if len(dyaw): print('  yaw diff of matched pairs deg: median %.2f, |.| p95 %.2f'%(np.median(dyaw),np.percentile(np.abs(dyaw),95)))
    print('  dist from car1 pose (in car0 frame) to nearest car0 label: median %.2f'%np.median(egoother))

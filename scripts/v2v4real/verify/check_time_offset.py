"""V2V4Real format check used for the Findings in docs/PHASE_C_HANDOFF.md.

Usage: python -I check_time_offset.py $V2V4REAL_DIR/test1
"""
import sys,os,glob
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from v2v4real_io import load_yaml
import numpy as np
root=sys.argv[1]
def yaw_of(T): return np.degrees(np.arctan2(T[1,0],T[0,0]))
for sc in sorted(glob.glob(root+'/*/')):
    frames=sorted(os.path.basename(p)[:-5] for p in glob.glob(sc+'0/*.yaml'))
    D=[[load_yaml(f'{sc}{c}/{f}.yaml') for c in (0,1)] for f in frames]
    W={}  # (frame, car1 id) -> world xy
    for i,d in enumerate(D):
        T1=np.array(d[1]['lidar_pose'])
        for vid,v in (d[1]['vehicles'] or {}).items(): W[(i,vid)]=(T1@np.r_[v['location'],1])[:2]
    rows=[]
    for i,d in enumerate(D):
        T0=np.array(d[0]['lidar_pose']); T1=np.array(d[1]['lidar_pose'])
        for vid,v in (d[1]['vehicles'] or {}).items():
            k=v['ass_id']
            if k==-1 or k not in (d[0]['vehicles'] or {}): continue
            if (i-1,vid) not in W or (i+1,vid) not in W: continue
            vel=(W[(i+1,vid)]-W[(i-1,vid)])/2.0   # metres per frame, world
            w0=(T0@np.r_[d[0]['vehicles'][k]['location'],1])[:2]
            r=w0-W[(i,vid)]
            sp=np.linalg.norm(vel)
            u=vel/sp if sp>1e-6 else np.zeros(2)
            ego0=np.linalg.norm(np.array(D[min(i+1,len(D)-1)][0]['lidar_pose'])[:2,3]-np.array(D[max(i-1,0)][0]['lidar_pose'])[:2,3])/2
            rows.append((sp, r@u, u[0]*r[1]-u[1]*r[0], ego0))
    R=np.array(rows)
    print('=====',os.path.basename(sc[:-1]),'n',len(R))
    if len(R)<3: continue
    sp,al,cr,eg=R.T
    A=np.c_[sp,np.ones_like(sp)]
    coef,*_=np.linalg.lstsq(A,al,rcond=None)
    print('  object speed m/frame: min %.2f median %.2f max %.2f'%(sp.min(),np.median(sp),sp.max()))
    print('  residual along object motion: mean %.2f std %.2f; across: mean %.2f std %.2f'%(al.mean(),al.std(),cr.mean(),cr.std()))
    print('  fit along = a*speed + b: a=%.3f frames, b=%.2f m; corr(speed,along)=%.2f'%(coef[0],coef[1],np.corrcoef(sp,al)[0,1] if sp.std()>0 else float('nan')))
    st=sp<0.2
    if st.any(): print('  near-stationary objects (<0.2 m/frame): n=%d, |r| mean %.2f'%(st.sum(), np.mean(np.hypot(al[st],cr[st]))))

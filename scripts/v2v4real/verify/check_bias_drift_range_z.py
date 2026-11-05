"""V2V4Real format check used for the Findings in docs/PHASE_C_HANDOFF.md.

Usage: python -I check_bias_drift_range_z.py $V2V4REAL_DIR/test1
"""
import sys,os,glob
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from v2v4real_io import load_yaml, load_pcd_xyz
import numpy as np
np.set_printoptions(precision=2, suppress=True)
root=sys.argv[1]
def yaw_of(T): return np.degrees(np.arctan2(T[1,0],T[0,0]))
for sc in sorted(glob.glob(root+'/*/')):
    frames=sorted(os.path.basename(p)[:-5] for p in glob.glob(sc+'0/*.yaml'))
    print('=====',os.path.basename(sc[:-1]))
    win={}
    lab_r=[];zrel=[];gh={0:[],1:[]}
    for i,f in enumerate(frames):
        d=[load_yaml(f'{sc}{c}/{f}.yaml') for c in (0,1)]
        T=[np.array(x['lidar_pose']) for x in d]
        for c in (0,1): gh[c].append(((np.degrees(d[c]['gps'][3])-yaw_of(T[c]))+180)%360-180)
        inv=np.linalg.inv(T[1])@T[0]
        for vid,v in (d[1]['vehicles'] or {}).items():
            k=v['ass_id']
            if k!=-1 and k in (d[0]['vehicles'] or {}):
                p=(inv@np.r_[d[0]['vehicles'][k]['location'],1])[:3]
                win.setdefault(i//25,[]).append((p-np.array(v['location']))[:2])
        for c in (0,1):
            for v in (d[c]['vehicles'] or {}).values(): lab_r.append(np.hypot(*v['location'][:2]))
        if i%50==0:
            for c in (0,1):
                P=load_pcd_xyz(f'{sc}{c}/{f}.pcd'); r=np.hypot(P[:,0],P[:,1])
                print('  frame %s car%d: points %d, range p99 %.1f max %.1f'%(f,c,len(P),np.percentile(r,99),r.max()))
                for v in (d[c]['vehicles'] or {}).values():
                    L,E,a=np.array(v['location']),np.array(v['extent']),np.radians(v['angle'][1])
                    q=P-L; x=np.cos(a)*q[:,0]+np.sin(a)*q[:,1]; y=-np.sin(a)*q[:,0]+np.cos(a)*q[:,1]
                    m=(np.abs(x)<E[0])&(np.abs(y)<E[1])&(np.abs(q[:,2])<2*E[2]+1)
                    if m.sum()>=30: zrel.append(((P[m,2].min()-L[2])/E[2],(P[m,2].max()-L[2])/E[2]))
    for w,r in sorted(win.items()):
        r=np.array(r); print('  frames %3d-%3d: n %3d mean residual (car1 frame) %s std %s'%(w*25,w*25+24,len(r),r.mean(0),r.std(0)))
    print('  label range: max %.1f, p95 %.1f'%(max(lab_r),np.percentile(lab_r,95)))
    for c in (0,1): print('  car%d gps[3] deg minus pose yaw: median %.2f std %.2f'%(c,np.median(gh[c]),np.std(gh[c])))
    if zrel: z=np.array(zrel); print('  points z relative to label z, in units of extent[2]: lowest median %.2f, highest median %.2f'%(np.median(z[:,0]),np.median(z[:,1])))

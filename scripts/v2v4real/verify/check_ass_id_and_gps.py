"""V2V4Real format check used for the Findings in docs/PHASE_C_HANDOFF.md.

Usage: python -I check_ass_id_and_gps.py $V2V4REAL_DIR/test1
"""
import sys,os,glob,collections
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from v2v4real_io import load_yaml
import numpy as np
np.set_printoptions(precision=2, suppress=True)
root=sys.argv[1]
def yaw_of(T): return np.degrees(np.arctan2(T[1,0],T[0,0]))
def wrap(a): return (a+180)%360-180
R_E=6378137.0
allres=[]
for sc in sorted(glob.glob(root+'/*/')):
    frames=sorted(os.path.basename(p)[:-5] for p in glob.glob(sc+'0/*.yaml'))
    res=[];dyaw=[];dyawneg=[];missing=0;n_ass=0;cav=[];geo=[];gps_disp=[];pose_disp=[];gh=[]
    first=None; prevg=None; prevT=None
    reuse=collections.Counter()
    for f in frames:
        d=[load_yaml(f'{sc}{c}/{f}.yaml') for c in (0,1)]
        T=[np.array(x['lidar_pose']) for x in d]
        V=[x['vehicles'] or {} for x in d]
        inv=np.linalg.inv(T[1])@T[0]
        for vid,v in V[1].items():
            k=v['ass_id']
            if k==-1: continue
            n_ass+=1
            if k not in V[0]: missing+=1; continue
            p=(inv@np.r_[V[0][k]['location'],1])[:3]
            res.append(p-np.array(v['location']))
            y0=yaw_of(T[0])+V[0][k]['angle'][1]; y1=yaw_of(T[1])+v['angle'][1]
            y0n=yaw_of(T[0])-V[0][k]['angle'][1]; y1n=yaw_of(T[1])-v['angle'][1]
            dyaw.append(wrap(y0-y1)); dyawneg.append(wrap(y0n-y1n))
        for c in (0,1):
            g=np.array(d[c]['gps'])
            gh.append(wrap(np.degrees(g[3])-yaw_of(T[c])))
        g=np.array(d[0]['gps'][:2])
        if prevg is not None:
            dn=np.radians(g[0]-prevg[0])*R_E; de=np.radians(g[1]-prevg[1])*R_E*np.cos(np.radians(g[0]))
            gps_disp.append(np.hypot(dn,de)); pose_disp.append(np.linalg.norm(T[0][:2,3]-prevT[:2,3]))
        prevg=g; prevT=T[0]
    res=np.array(res)
    print('=====',os.path.basename(sc[:-1]),'frames',len(frames))
    print('  car1 labels with ass_id: %d; target id absent in car0 same frame: %d'%(n_ass,missing))
    if len(res):
        print('  residual (car0 label mapped to car1 frame minus car1 label): mean xyz',res.mean(0),' std',res.std(0))
        print('  |dxy| median %.2f p95 %.2f max %.2f'%(np.median(np.linalg.norm(res[:,:2],axis=1)),np.percentile(np.linalg.norm(res[:,:2],axis=1),95),np.linalg.norm(res[:,:2],axis=1).max()))
        print('  yaw diff (yaw = pose yaw + angle[1]): median %.2f |.|p95 %.2f ; with angle negated: median %.2f |.|p95 %.2f'%(np.median(dyaw),np.percentile(np.abs(dyaw),95),np.median(dyawneg),np.percentile(np.abs(dyawneg),95)))
        allres.append(res)
    gps_disp=np.array(gps_disp); pose_disp=np.array(pose_disp)
    print('  per-frame displacement: pose median %.3f m, gps median %.3f m, ratio median %.3f'%(np.median(pose_disp),np.median(gps_disp),np.median(gps_disp[pose_disp>0.5]/pose_disp[pose_disp>0.5])))
    gh=np.array(gh); print('  gps[3] (deg) minus pose yaw: median %.2f, std %.2f'%(np.median(gh),gh.std()))

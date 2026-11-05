"""V2V4Real format check used for the Findings in docs/PHASE_C_HANDOFF.md.

Usage: python -I check_label_frame_and_extent.py $V2V4REAL_DIR/test1
"""
import sys,os,glob
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from v2v4real_io import load_yaml, load_pcd_xyz
import numpy as np
root=sys.argv[1]
def pts_in_box(P, loc, yaw_deg, half, scale=1.0, zoff=0.0):
    a=np.radians(yaw_deg); c,s=np.cos(a),np.sin(a)
    q=P-np.array(loc); q[:,2]-=zoff
    x= c*q[:,0]+s*q[:,1]; y=-s*q[:,0]+c*q[:,1]
    h=np.array(half)*scale
    return int(np.sum((np.abs(x)<=h[0])&(np.abs(y)<=h[1])&(np.abs(q[:,2])<=h[2])))
rows=[]
for sc in sorted(glob.glob(root+'/*/')):
    for c in (0,1):
        for f in ['000000','000040','000080']:
            y=load_yaml(f'{sc}{c}/{f}.yaml'); P=load_pcd_xyz(f'{sc}{c}/{f}.pcd')
            for vid,v in (y['vehicles'] or {}).items():
                L,A,E=v['location'],v['angle'],v['extent']
                r=np.hypot(L[0],L[1])
                n_half=pts_in_box(P,L,A[1],E)            # extent = half size
                n_full=pts_in_box(P,L,A[1],E,0.5)        # extent = full size
                n_big=pts_in_box(P,L,A[1],E,1.5)
                n_neg=pts_in_box(P,L,-A[1],E)             # yaw sign flipped
                n_rad=pts_in_box(P,L,np.degrees(A[1]),E) # yaw in radians
                rows.append((os.path.basename(sc[:-1])[-4:],c,f,vid,r,n_full,n_half,n_big,n_neg,n_rad,E))
import collections
R=np.array([[r[4],r[5],r[6],r[7],r[8],r[9]] for r in rows])
print('boxes',len(R),'pts in pcd example',P.shape, 'pcd z range', P[:,2].min(), P[:,2].max())
print('fraction of boxes with >=5 points: E as full %.2f, E as half %.2f, 1.5*half %.2f'%((R[:,1]>=5).mean(),(R[:,2]>=5).mean(),(R[:,3]>=5).mean()))
print('median points: full %.0f half %.0f 1.5half %.0f yaw-negated %.0f yaw-as-rad %.0f'%tuple(np.median(R[:,1:],axis=0)))
print('total points: half %d, 1.5*half %d (ratio %.3f)'%(R[:,2].sum(),R[:,3].sum(),R[:,3].sum()/R[:,2].sum()))
print('total yaw +: %d yaw -: %d'%(R[:,2].sum(),R[:,4].sum()))
for r in rows[:12]: print(r[:10])

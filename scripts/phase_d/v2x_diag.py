import json,glob,os,numpy as np
from collections import Counter
D='/workspace/data/v2xreal/'; O=D+'d2_val_runs/'
# 1. filter translations: the true offset is zero (labels are in the exact map frame)
for run in ('noself_filter','noself_anchor'):
    mags={'vehicle':[], 'rsu':[]}; n_est=Counter(); n=Counter()
    for f in sorted(glob.glob(O+run+'/*.dump.jsonl')):
        s=os.path.basename(f)[:-len('.dump.jsonl')]
        T=json.load(open(D+f'd2_in/val/{s}/truth.json')); kinds=[a['kind'] for a in T['agents']]
        for line in open(f):
            e=json.loads(line)
            for ag in e['agents']:
                a=ag['index']
                for x in ag['alignment']:
                    if x['src']!=a or run=='noself_anchor':
                        mags[kinds[a]].append(float(np.hypot(*x['t_odom'][:2])))
    print(run,{k:'n=%d median %.2f m p90 %.2f m'%(len(v),np.median(v),np.percentile(v,90)) for k,v in mags.items() if v})
# 2. wrong merges involving an agent: self objects or others?
for run in ('self_alignoff',):
    c=Counter(); extra=Counter()
    for f in sorted(glob.glob(O+run+'/*.dump.jsonl')):
        s=os.path.basename(f)[:-len('.dump.jsonl')]
        T=json.load(open(D+f'd2_in/val/{s}/truth.json'))
        for line in open(f):
            e=json.loads(line); tf=T['frames'][e['frame']]
            bodies={o['gid']:o for o in tf['objects'] if o['agent'] is not None}
            cls={o['gid']:o['class_id'] for o in tf['objects']}
            for ag in e['agents']:
                for o in ag['ego']:
                    L=set(o['labels'])
                    if len(L)>=2 and L&set(bodies):
                        c['self object' if o['self_agent'] is not None else 'not a self object']+=1
                        for g in L-set(bodies): extra[cls.get(g)]+=1
    print(run,'agent-involving wrong merges:',dict(c),'| class of the other merged label:',dict(extra))

import sys, math, random
from pathlib import Path
sys.path.insert(0, str(Path("manuals/_build")))
from workspace_probe import CHAIN
from under_cabinet_probe import joint_points
random.seed(17)
buckets={}; best=[]
for i in range(350000):
    q=[random.uniform(lo,hi) for _n,_x,_r,_a,lo,hi in CHAIN]
    pts,tool,z=joint_points(q)
    if z[2] > -0.90: continue
    if not (0.0 <= tool[2] <= 0.06): continue
    zs=[p[2] for p in pts]
    if max(zs) > 0.43 or min(zs) < 0.015: continue
    r=math.hypot(tool[0],tool[1])
    b=int(r*10)/10; buckets[b]=buckets.get(b,0)+1
    if r>0.45: best.append((r,max(zs),min(zs),tool,q))
print("valid fold-under poses by radius:")
for b in sorted(buckets): print(f"  {b:.1f}-{b+0.1:.1f}: {buckets[b]}")
best.sort(reverse=True)
print("max radius:", round(best[0][0],3) if best else None)
for r,mx,mn,t,q in best[:5]:
    print(" r=%.3f maxz=%.3f minz=%.3f tool=%s q=%s"%(r,mx,mn,[round(v,2) for v in t],[round(v,2) for v in q]))

#!/usr/bin/env python3
"""Compare saved MapGraph poses with dense at DB Node stamps; no terminal service claim."""
import argparse, csv, hashlib, json, math, pathlib, sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Path
from rtabmap_msgs.msg import MapGraph
ap=argparse.ArgumentParser();ap.add_argument('run',type=pathlib.Path);a=ap.parse_args();p=a.run
c=sqlite3.connect('file:'+str(p/'rtabmap.db')+'?mode=ro',uri=True)
stamps={identifier:round(t*1e9) for identifier,t in c.execute('select id,stamp from Node')};c.close()
g=deserialize_message((p/'final_mapgraph.cdr').read_bytes(),MapGraph)
d=deserialize_message((p/'final_dense.cdr').read_bytes(),Path)
def stamp(m):return m.header.stamp.sec*1000000000+m.header.stamp.nanosec
dense={stamp(x):x.pose for x in d.poses};errors=[];missing=[]
for identifier,x in zip(g.poses_id,g.poses):
    t=stamps.get(identifier)
    if t not in dense:missing.append(identifier);continue
    y=dense[t]
    pos=float(np.linalg.norm(np.array([x.position.x,x.position.y,x.position.z])-[y.position.x,y.position.y,y.position.z]))
    q=np.array([x.orientation.x,x.orientation.y,x.orientation.z,x.orientation.w]);r=np.array([y.orientation.x,y.orientation.y,y.orientation.z,y.orientation.w])
    rot=math.degrees(2*math.acos(float(np.clip(abs(np.dot(q,r)/(np.linalg.norm(q)*np.linalg.norm(r))),0,1))))
    errors.append(dict(id=identifier,stamp_ns=t,translation_m=pos,rotation_deg=rot))
odom=[int(x['stamp_ns']) for x in csv.DictReader((p/'odom.csv').open())]
result=dict(scope='Offline final subscribed MapGraph + saved dense + read-only DB Node.stamp; not terminal GetMap2, no epoch guarantee, no GT accuracy; runtime FAIL unchanged',graph_poses=len(g.poses),matched=len(errors),unmatched_ids=missing,max_translation_m=max((e['translation_m'] for e in errors),default=None),max_rotation_deg=max((e['rotation_deg'] for e in errors),default=None),all_finite=all(math.isfinite(e[k]) for e in errors for k in ['translation_m','rotation_deg']),dense_exact_observed_odom=[stamp(x) for x in d.poses]==odom,anchors=errors,sha256={n:hashlib.sha256((p/n).read_bytes()).hexdigest() for n in ['rtabmap.db','final_mapgraph.cdr','final_dense.cdr','odom.csv']})
(p/'offline_anchor_consistency.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='anchors'},indent=2))

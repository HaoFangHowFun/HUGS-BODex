"""Shadow-seeded 50-ball volume fit; five designated contact spheres stay exact."""
from pathlib import Path
import sys,copy,json,xml.etree.ElementTree as E
import numpy as np,trimesh,yaml
from scipy.optimize import least_squares
W=Path(__file__).resolve().parents[2];C=W/'HUGS-BODex/src/curobo/content/configs/robot';A=W/'HUGS-BODex/src/curobo/content/assets';O=W/'benchmarks/sharpa_50_volume'
sys.path.insert(0,str(W/'HUGS-Sharpa-Assets/scripts'))
from validate_models import transform,urdf_fk
rng=np.random.default_rng(123)
robot=yaml.safe_load((C/'right_sharpa_wave_sim.yml').read_text());kin=robot['robot_cfg']['kinematics'];up=(A/kin['urdf_path']).resolve();tree=E.parse(up).getroot();fk=urdf_fk(tree,dict.fromkeys(kin['cspace']['joint_names'],0.))
old=yaml.safe_load((C/kin['collision_spheres']).read_text())['collision_spheres'];ss=yaml.safe_load((C/'spheres/right_shadow_hand.yml').read_text())['collision_spheres'];st=E.parse(A/'robot/shadow_hand/right_sim.urdf').getroot()
def meshes(t,path,tag):
 out={}
 for l in t.findall('link'):
  parts=[]
  for v in l.findall(tag):
   m=v.find('geometry/mesh')
   if m is None:continue
   x=trimesh.load(path.parent/m.get('filename'),force='mesh');x.apply_scale(np.fromstring(m.get('scale','1 1 1'),sep=' '));origin=v.find('origin');x.apply_transform(transform(**({} if origin is None else origin.attrib)));parts.append(x)
  if parts:out[l.get('name')]=trimesh.util.concatenate(parts)
 return out
sm=meshes(st,A/'robot/shadow_hand/right_sim.urdf','visual');tm=meshes(tree,up,'collision')
groups=[('right_hand_C_MC','rh_palm',5,None),('right_thumb_MC','rh_thproximal',3,None),('right_thumb_PP','rh_thmiddle',2,None),('right_thumb_DP','rh_thdistal',2,'right_thumb_elastomer'),('right_pinky_MC','rh_lfmetacarpal',2,None)]
for finger,sh in zip(['index','middle','ring','pinky'],['ff','mf','rf','lf']):
 for part,suffix,n in [('MCP_VL','proximal',1),('PP','proximal',3),('MP','middle',2),('DP','distal',3)]:groups.append((f'right_{finger}_{part}',f'rh_{sh}{suffix}',n,f'right_{finger}_elastomer' if part=='DP' else None))
assert sum(x[2] for x in groups)==50
result={};initial={};report=[]
def basis(long,dorsal):
 z=np.array(long,dtype=float);z/=np.linalg.norm(z);y=np.array(dorsal,dtype=float);y-=z*np.dot(y,z);y/=np.linalg.norm(y);return np.column_stack([np.cross(y,z),y,z])
def assess(points,inside,balls):
 c=balls[:,:3];r=balls[:,3];d=np.linalg.norm(points[:,None]-c,axis=2)-r;v=d.min(axis=1);return {'coverage_fraction':float((v[inside]<=0).mean()),'outside_box_fraction':float(((v<=0)&~inside).mean()),'missed_volume_box_fraction':float(((v>0)&inside).mean())}
for name,shadow,n,pad in groups:
 mesh=tm[name].copy();fixed=[]
 if pad:
  T=np.linalg.inv(fk[name])@fk[pad];part=tm[pad].copy();part.apply_transform(T);mesh=trimesh.util.concatenate([mesh,part]);f=old[pad][0];fixed=[np.r_[(T@np.r_[f['center'],1])[:3],f['radius']]]
 # Use each component's closed mesh, or convex hull only for an open component.
 components=[tm[name].copy()]
 if pad:components.append(part)
 proxies=[m if m.is_watertight else m.convex_hull for m in components]
 def contains(points):
  mask=np.zeros(len(points),bool)
  for m in proxies:
   for start in range(0,len(points),256):mask[start:start+256]|=m.contains(points[start:start+256])
  return mask
 if name=='right_hand_C_MC':B=basis([0,0,1],[-1,0,0])
 elif name=='right_pinky_MC':B=basis([0,0,-1],[1,0,0])
 elif 'MCP_VL' in name:B=np.eye(3)
 else:
  finger=name.split('_')[1];p=f'right_{finger}_elastomer';d=(np.linalg.inv(fk[name])@fk[p])[:3,:3]@np.array([0,0,1]);B=basis([1,0,0],d)
 v=mesh.vertices@B;lo,hi=v.min(0),v.max(0);slo,shi=sm[shadow].bounds
 src=ss[shadow];free=n-len(fixed)
 # Reuse non-contact Shadow centers; 5 of 9 palm seeds selected spread-out.
 indices=list(range(len(src)-(1 if pad else 0)))
 if free<len(indices):indices=np.linspace(0,len(indices)-1,free).round().astype(int).tolist()
 if 'MCP_VL' in name:seed=np.array([np.r_[mesh.centroid,.0072]])
 else:
  seed=np.array([np.r_[((np.array(src[i]['center'])-slo)/(shi-slo)*(hi-lo)+lo)@B.T,src[i]['radius']*np.sqrt(np.prod((hi-lo)[:2]/(shi-slo)[:2]))] for i in indices[:free]])
 bounds=mesh.bounds.copy();ext=mesh.extents;bounds[0]-=.2*ext;bounds[1]+=.2*ext
 points=rng.uniform(bounds[0],bounds[1],(6500,3));inside=contains(points);assert inside.any(),name
 fixed=np.array(fixed).reshape(-1,4)
 # mm optimization units, equal volume-sample weights, no directional bias or collision constraints.
 P=points*1000;F=fixed*1000
 def residual(x):
  balls=np.vstack([x.reshape(-1,4),F]);d=(np.linalg.norm(P[:,None]-balls[None,:,:3],axis=2)-balls[None,:,3]).min(axis=1)
  return np.where(inside,np.maximum(d,0),np.minimum(d,0))
 lower=np.tile(np.r_[bounds[0]*1000,.2],free);upper=np.tile(np.r_[bounds[1]*1000,np.linalg.norm(ext)*1000],free)
 x=np.clip(seed.reshape(-1)*1000,lower+1e-6,upper-1e-6)
 fit=least_squares(residual,x,bounds=(lower,upper),max_nfev=100,ftol=1e-5,xtol=1e-5,gtol=1e-5)
 balls=np.vstack([fit.x.reshape(-1,4)/1000,fixed]);seeds=np.vstack([x.reshape(-1,4)/1000,fixed])
 result[name]=[dict(center=b[:3].tolist(),radius=float(b[3])) for b in balls[:free]];initial[name]=[dict(center=b[:3].tolist(),radius=float(b[3])) for b in seeds[:free]]
 if pad:result[pad]=[copy.deepcopy(old[pad][0])];initial[pad]=[copy.deepcopy(old[pad][0])]
 baseline=[]
 for link in [name]+([pad] if pad else []):
  T=np.linalg.inv(fk[name])@fk[link]
  baseline.extend([np.r_[(T@np.r_[b['center'],1])[:3],b['radius']] for b in old[link]])
 allballs=np.vstack([balls,seeds,np.array(baseline)])
 ebounds=np.array([np.minimum(bounds[0],(allballs[:,:3]-allballs[:,3,None]).min(0)),np.maximum(bounds[1],(allballs[:,:3]+allballs[:,3,None]).max(0))])
 test=rng.uniform(ebounds[0],ebounds[1],(20000,3));isinside=contains(test)
 row={'link':name,'shadow_source':shadow,'spheres':n,'proxy_convex_hull_components':sum(not m.is_watertight for m in components),'box_volume_m3':float(np.prod(ebounds[1]-ebounds[0])),'sampled_inside_fraction':float(isinside.mean()),'initial':assess(test,isinside,seeds),'fit':assess(test,isinside,balls),'baseline34':assess(test,isinside,np.array(baseline)),'initial_loss':float(np.mean(residual(x)**2)),'final_loss':float(np.mean(fit.fun**2)),'evaluations':fit.nfev}
 report.append(row);print(name,'coverage',round(row['baseline34']['coverage_fraction'],3),'->',round(row['fit']['coverage_fraction'],3),flush=True)
assert sum(map(len,result.values()))==50
for f in ['thumb','index','middle','ring','pinky']:assert result[f'right_{f}_elastomer'][0]==old[f'right_{f}_elastomer'][0]
O.mkdir(exist_ok=True)
for label,data in [('volume',result),('shadow_seed',initial)]:
 path=f'spheres/right_sharpa_wave_50_{label}.yml';(C/path).write_text(yaml.safe_dump({'collision_spheres':data},sort_keys=False));cfg=copy.deepcopy(robot);cfg['robot_cfg']['kinematics']['collision_spheres']=path;(C/f'right_sharpa_wave_50_{label}_preview.yml').write_text(yaml.safe_dump(cfg,sort_keys=False))
summary={}
volume=sum(r['box_volume_m3']*r['sampled_inside_fraction'] for r in report)
for key in ['baseline34','initial','fit']:
 summary[key]={'coverage_fraction':sum(r['box_volume_m3']*r['sampled_inside_fraction']*r[key]['coverage_fraction'] for r in report)/volume,'outside_volume_over_target_volume':sum(r['box_volume_m3']*r[key]['outside_box_fraction'] for r in report)/volume}
(O/'fit_report.json').write_text(json.dumps({'method':'Shadow-seeded per-rigid-group equal-weight volume classification squared distance; open meshes use per-component convex hull; 5 contact balls fixed; no self collision fitting constraints. Evaluation boxes contain full extents of fitted, initial and baseline balls; metrics sum rigid-group volumes, not whole-hand geometric union.','sphere_count':50,'fixed_contact_count':5,'summary':summary,'groups':report},indent=2));print(summary,flush=True)

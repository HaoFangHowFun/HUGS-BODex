"""Validate unmodified BODex poses and render a deterministic grasp contact sheet."""
import argparse
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT.parent/'HUGS-Sharpa-Assets'
sys.path.insert(0,str(ASSETS/'scripts'))
from validate_models import urdf_fk, transform


def pose(p):
    t=np.eye(4);t[:3,3]=p[:3];t[:3,:3]=Rotation.from_quat(np.roll(p[3:7],-1)).as_matrix();return t


def main():
    parser=argparse.ArgumentParser();parser.add_argument('input');parser.add_argument('output');args=parser.parse_args()
    output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
    raw=np.load(args.input,allow_pickle=True).item();q=raw['robot_pose'][0];names=list(raw['joint_names'])
    urdf=ASSETS/'urdf/right_sharpa_wave_with_flange.urdf';tree=ET.parse(urdf).getroot()
    moving={j.get('name'):j for j in tree.findall('joint') if j.get('type')!='fixed'}
    assert names==list(moving)
    limits=np.array([[float(moving[n].find('limit').get(k)) for k in ('lower','upper')] for n in names])
    violations=np.maximum(limits[:,0]-q[...,7:],q[...,7:]-limits[:,1]).clip(0)
    report=dict(candidates=len(q),canonical_joint_order=True,all_finite=bool(np.isfinite(q).all()),
                max_quaternion_norm_error=float(np.abs(np.linalg.norm(q[...,3:7],axis=-1)-1).max()),
                stages=['pregrasp','grasp','squeeze'],max_limit_violation_rad_by_stage=violations.max(axis=(0,2)).tolist(),
                per_candidate_max_limit_violation_rad=violations.max(axis=2).tolist(),
                valid_pregrasp_and_grasp=int(np.sum(violations[:,:2].max(axis=(1,2))<1e-5)),
                all_stages_within_limits=int(np.sum(violations.max(axis=(1,2))<1e-5)),
                squeeze_policy='Unchanged BODex 2*grasp-pregrasp control target; violations reported, never clamped.')
    (output/'candidate_validation.json').write_text(json.dumps(report,indent=2)+'\n')
    from curobo.util_file import load_scene_cfg
    scene_path=Path(raw['scene_path'][0])
    if not scene_path.is_absolute():scene_path=Path(os.environ['HUGS_DATASET_ROOT'])/scene_path
    scene=load_scene_cfg(str(scene_path));obj=scene['scene'][scene['task']['obj_name']]
    om=trimesh.load(obj['file_path']);om.apply_scale(obj['scale']);om.apply_transform(pose(obj['pose']))
    meshes={}
    for link in tree.findall('link'):
        visual=link.find('visual')
        if visual is None:continue
        mesh=trimesh.load(urdf.parent/visual.find('geometry/mesh').get('filename'))
        origin=visual.find('origin');mesh.apply_transform(transform(**(origin.attrib if origin is not None else {})))
        meshes[link.get('name')]=mesh
    fig=plt.figure(figsize=(16,9));selected=[0,5,15,18]
    for panel,i in enumerate(selected):
        ax=fig.add_subplot(2,2,panel+1,projection='3d');fk=urdf_fk(tree,dict(zip(names,q[i,1,7:])));root=pose(q[i,1])
        for name,mesh in meshes.items():
            m=mesh.copy();m.apply_transform(root@fk[name]);tri=m.triangles[::max(1,len(m.faces)//1600)]
            ax.add_collection3d(Poly3DCollection(tri,facecolor='#62a7cb' if 'elastomer' in name else '#cccccc',edgecolor='none',alpha=.85))
        ax.add_collection3d(Poly3DCollection(om.triangles,facecolor='#df974a',edgecolor='none',alpha=.9))
        center=om.bounds.mean(axis=0);radius=.17
        ax.set(xlim=(center[0]-radius,center[0]+radius),ylim=(center[1]-radius,center[1]+radius),zlim=(0,2*radius),xlabel='X (m)',ylabel='Y (m)',zlabel='Z (m)',title=f'Candidate {i}: optimized grasp pose')
        ax.set_box_aspect((1,1,1));ax.view_init(elev=23,azim=-50)
    fig.suptitle('Sharpa / BODex: real generated candidates before MuJoCo evaluation')
    fig.tight_layout();fig.savefig(output/'candidate_grasps.png',dpi=150)
    print(json.dumps({k:v for k,v in report.items() if not k.startswith('per_candidate')},indent=2))


if __name__=='__main__':main()

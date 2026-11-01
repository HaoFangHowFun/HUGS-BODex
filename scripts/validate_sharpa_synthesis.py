"""Exercise real CUDA FK and human-prior seed mapping against independent URDF FK."""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import torch
import yaml
from curobo.types.robot import RobotConfig
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.util.human_prior_seed import HumanPriorSeedBuilder, HumanPriorSceneJob

ROOT=Path(__file__).resolve().parents[1];CFG=ROOT/'src/curobo/content/configs'
sys.path.insert(0,str(ROOT.parent/'HUGS-Sharpa-Assets/scripts'))
from validate_models import urdf_fk


def main():
    raw=yaml.safe_load((CFG/'robot/right_sharpa_wave_sim.yml').read_text())
    names=raw['robot_cfg']['kinematics']['cspace']['joint_names']
    model=CudaRobotModel(RobotConfig.from_dict(raw).kinematics)
    assert model.joint_names==names
    tree=ET.parse((ROOT/'src/curobo/content/assets'/raw['robot_cfg']['kinematics']['urdf_path']).resolve()).getroot()
    joints={j.get('name'):j for j in tree.findall('joint')}
    limits=np.array([[float(joints[n].find('limit').get(k)) for k in ('lower','upper')] for n in names])
    rng=np.random.default_rng(123);q=rng.uniform(*limits.T,(12,22));q[0]=0
    base_r=Rotation.random(12,random_state=rng);base_t=rng.uniform(-.3,.3,(12,3))
    root=np.column_stack([base_t,np.roll(base_r.as_quat(),1,axis=1)])
    state=model.get_state(torch.tensor(np.column_stack([root,q]),device='cuda',dtype=torch.float32))
    error=0.
    for i in range(12):
        fk=urdf_fk(tree,dict(zip(names,q[i])))
        for j,n in enumerate(model.link_names):
            expected=base_r[i].as_matrix()@fk[n][:3,3]+base_t[i]
            error=max(error,float(np.abs(state.links_position[i,j].cpu().numpy()-expected).max()))
            expected_r=base_r[i].as_matrix()@fk[n][:3,:3]
            actual_r=Rotation.from_quat(np.roll(state.links_quaternion[i,j].cpu().numpy(),-1)).as_matrix()
            error=max(error,float(np.abs(actual_r-expected_r).max()))
    assert error<2e-5,error
    manip=yaml.safe_load((CFG/'manip/sim_sharpa/tabletop_full.yml').read_text())
    suite=yaml.safe_load((CFG/'manip/sim_sharpa.yml').read_text())
    builder=HumanPriorSeedBuilder(manip,{'human_prior':{'jitter':{'enabled':False}}},3,'3_right_full',suite['grasp_types']['3_right_full'])
    zero=urdf_fk(tree,dict.fromkeys(names,0.));offset=zero['right_index_MCP_VL'][:3,3]
    # Construct anatomically specified human frame from known world root transforms.
    human_axes=np.array([[0,-1,0],[0,0,1],[-1,0,0]])
    positions=np.zeros((5,12,1,3));quats=np.zeros((5,12,1,4));quats[...,0]=1
    positions[2,:,0]=base_t+base_r.apply(np.broadcast_to(offset,(12,3)))
    quats[2,:,0]=np.roll(Rotation.from_matrix(base_r.as_matrix()@human_axes).as_quat(),1,axis=1)
    record=dict(index_mcp_pos=positions,wrist_quat=quats,active_hand_mask=np.ones((5,12,1),bool))
    job=HumanPriorSceneJob('fixture','fixture',record,'synthetic_mapping_fixture',np.ones(5),np.array([0,0,12,0,0]),12,np.arange(12),np.zeros(12,bool))
    seed,_=builder.build_seed_tensor([job],torch.device('cuda'),torch.float32,expected_dof=29)
    seed=seed[0].cpu().numpy()
    position_error=float(np.abs(seed[:,:3]-base_t).max())
    rotation_error=float(np.abs(Rotation.from_quat(np.roll(seed[:,3:7],-1,axis=1)).as_matrix()-base_r.as_matrix()).max())
    assert position_error<2e-6 and rotation_error<2e-6
    assert np.all(seed[:,7:]==0)
    kin=raw['robot_cfg']['kinematics']
    local=yaml.safe_load((CFG/'robot'/kin['collision_spheres']).read_text())['collision_spheres']
    world={n:np.array([zero[n][:3,:3]@s['center']+zero[n][:3,3] for s in spheres]) for n,spheres in local.items()}
    collision_pairs=[]
    keys=list(local)
    for i,a in enumerate(keys):
        for b in keys[i+1:]:
            if b in kin['self_collision_ignore'].get(a,[]) or a in kin['self_collision_ignore'].get(b,[]):continue
            radii_a=np.array([s['radius'] for s in local[a]])
            radii_b=np.array([s['radius'] for s in local[b]])
            distance=np.linalg.norm(world[a][:,None]-world[b][None,:],axis=-1)-radii_a[:,None]-radii_b[None,:]
            if distance.min()<0:collision_pairs.append(dict(links=[a,b],sphere_penetration_m=float(-distance.min())))
    report=dict(cuda_fk_samples=12,canonical_joint_order=True,max_fk_matrix_error=error,
                human_prior_mapping_samples=12,human_prior_position_error_m=position_error,human_prior_rotation_matrix_error=rotation_error,
                human_prior_scope='Actual seed builder with synthetic world poses; no human dataset quality or finger-joint retargeting claim.',
                zero_pose_sphere_overlap_pairs=collision_pairs,
                zero_pose_overlap_note='Proxy overlaps, not physical mesh penetration; kept visible and not excluded.')
    (ROOT/'docs/sharpa_synthesis_validation.json').write_text(json.dumps(report,indent=2)+'\n');print(report)


if __name__=='__main__':main()

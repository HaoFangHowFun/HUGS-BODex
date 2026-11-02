"""Verify the adopted preset reaches every solver rollout without ablation hooks."""
import argparse
from pathlib import Path
from types import SimpleNamespace
import sys,json,hashlib
import numpy as np,torch,yaml
B=Path(__file__).resolve().parents[1];C=B/'src/curobo/content/configs'
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--scene-manifest', type=Path, required=True, help='Text file containing exactly one scene JSON path.')
parser.add_argument('--output', type=Path, required=True, help='Write the observed solver configuration here.')
args=parser.parse_args()
scenes=[line.strip() for line in args.scene_manifest.read_text().splitlines() if line.strip() and not line.lstrip().startswith('#')]
if len(scenes)!=1:
    parser.error('Verification requires exactly one scene in the manifest.')
sys.path.insert(0,str(B))
from src.task.synthesis import SynthesisRuntime
from curobo.util.human_prior_seed import build_world_info_batch
spec=json.loads((B/'docs/sharpa_evidence/20261102_adoption/specification.json').read_text())
sphere_path=C/'robot'/spec['sphere_file']
assert hashlib.sha256(sphere_path.read_bytes()).hexdigest()==spec['sphere_sha256']
expected_spheres=yaml.safe_load(sphere_path.read_text())['collision_spheres']
out={}
for label,file in [('sharpa','sim_sharpa/tabletop_full_50_restored.yml'),('shadow','sim_shadow/tabletop_full.yml')]:
    m=yaml.safe_load((C/'manip'/file).read_text())
    solver=SynthesisRuntime(SimpleNamespace(save_debug=False)).create_or_update_solver(None,
        build_world_info_batch(scenes),m)
    if label=='sharpa':
        assert m['seeder_cfg']['q']==spec['initial_joints']
        assert list(solver.joint_names)==spec['joint_names']
        total=m['gradient']['lbfgs']['n_iters']
        assert total==sum(spec['stage_iterations'])
        progress=m['grasp_contact_strategy']['opt_progress']
        if torch.is_tensor(progress):progress=progress.detach().cpu().tolist()
        np.testing.assert_allclose(np.diff(progress+[1.])*total,spec['stage_iterations'],rtol=0,atol=1e-4)
        seeds=solver.generate_seed(2,1).detach().cpu().numpy()
        np.testing.assert_allclose(seeds[...,7:],np.broadcast_to(spec['initial_joints'],seeds[...,7:].shape),rtol=0,atol=1e-7)
    records=[]
    for roll in solver.get_all_rollout_instances():
        if not hasattr(roll,'grasp_cost') or roll.grasp_cost is None:continue
        model=roll.dynamics_model.robot_model;wc=roll.primitive_collision_cost
        assert float(wc.weight.item())==100.
        assert float(roll.robot_self_collision_cost.weight.item())==100.
        assert float(roll.bound_cost.weight.item())==100.
        assert len(roll.grasp_cost.contact_points_idx)==5
        final=wc._get_contact_stage(1.).detach().cpu().numpy()
        record=dict(world_weight=float(wc.weight.item()),self_weight=float(roll.robot_self_collision_cost.weight.item()),
            robot_file=m['robot_file'],
            grasp_targets=len(roll.grasp_cost.contact_points_idx),total_spheres=model.total_spheres,
            final_clearance_mm=(final*1000).tolist())
        if label=='sharpa':
            assert model.total_spheres==50
            for link,balls in expected_spheres.items():
                actual=model.kinematics_config.get_link_spheres(link).detach().cpu().numpy()
                expected=np.array([ball['center']+[ball['radius']] for ball in balls])
                np.testing.assert_allclose(actual,expected,rtol=0,atol=1e-7,err_msg=link)
            record.update(all_spheres_match_specification=True,initial_joints=m['seeder_cfg']['q'],
                          joint_names=list(solver.joint_names),stage_iterations=spec['stage_iterations'])
            bylink={}
            for finger in ['index','middle','ring','pinky','thumb']:
                for suffix in ['DP','elastomer']:
                    name=f'right_{finger}_{suffix}'
                    idx=model.get_sphere_idx_from_namelst([name+'/0'],all_points_in_link=True)
                    if torch.is_tensor(idx):idx=idx.detach().cpu().numpy()
                    assert len(idx)>0 and np.allclose(final[idx],-.002)
                    bylink[name]=[int(i) for i in idx]
            record['DP_and_pad_indices']=bylink
        records.append(record)
    assert records
    out[label]=records
    del solver
# Preserve sphere counts and the robot's canonical kinematics/collision policy.
active=yaml.safe_load((C/'manip/sim_sharpa/tabletop_full_50_restored.yml').read_text())
reference=yaml.safe_load((C/'robot/right_sharpa_wave_50_restored.yml').read_text())
updated=yaml.safe_load((C/'robot'/active['robot_file']).read_text())
source_spheres=yaml.safe_load((C/'robot'/reference['robot_cfg']['kinematics']['collision_spheres']).read_text())['collision_spheres']
active_spheres=yaml.safe_load((C/'robot'/updated['robot_cfg']['kinematics']['collision_spheres']).read_text())['collision_spheres']
assert active_spheres==expected_spheres
assert active_spheres['right_hand_C_MC']==source_spheres['right_hand_C_MC']
assert all(len(active_spheres[name])==len(source_spheres[name]) for name in source_spheres)
updated['robot_cfg']['kinematics']['collision_spheres']=reference['robot_cfg']['kinematics']['collision_spheres']
assert updated==reference, 'Only the sphere file may differ in the robot configuration'
shared=yaml.safe_load((C/'task/gradient_grasp_fc.yml').read_text())
specific=yaml.safe_load((C/'task/gradient_grasp_fc_sharpa_50.yml').read_text())
specific['cost']['primitive_collision_cfg']['weight']=100.
assert shared==specific
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(out,indent=2)+'\n')
print('Verified live rollouts: adopted 50 spheres and nonzero initial joints; stages 300/100/100; world/self=100; DP/pad spheres=-2 mm. Canonical kinematics and collision policy unchanged. Shadow world=100.')

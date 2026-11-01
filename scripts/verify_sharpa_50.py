"""Verify the adopted preset reaches every solver rollout without ablation hooks."""
import argparse
from pathlib import Path
from types import SimpleNamespace
import sys,json
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
out={}
for label,file in [('sharpa','sim_sharpa/tabletop_full_50_restored.yml'),('shadow','sim_shadow/tabletop_full.yml')]:
    m=yaml.safe_load((C/'manip'/file).read_text())
    solver=SynthesisRuntime(SimpleNamespace(save_debug=False)).create_or_update_solver(None,
        build_world_info_batch(scenes),m)
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
# Historical geometry remains intact; only PP and designated contact balls changed.
active=yaml.safe_load((C/'manip/sim_sharpa/tabletop_full_50_restored.yml').read_text())
reference=yaml.safe_load((C/'robot/right_sharpa_wave_50_restored.yml').read_text())
updated=yaml.safe_load((C/'robot'/active['robot_file']).read_text())
source_spheres=yaml.safe_load((C/'robot'/reference['robot_cfg']['kinematics']['collision_spheres']).read_text())['collision_spheres']
active_spheres=yaml.safe_load((C/'robot'/updated['robot_cfg']['kinematics']['collision_spheres']).read_text())['collision_spheres']
changed=[name for name in source_spheres if source_spheres[name]!=active_spheres[name]]
assert set(changed)=={f'right_{f}_PP' for f in ['index','middle','ring','pinky']}|{f'right_{f}_elastomer' for f in ['index','middle','ring','pinky','thumb']}
pp_spheres=yaml.safe_load((C/'robot/spheres/right_sharpa_wave_50_pp_refined.yml').read_text())['collision_spheres']
for name in source_spheres:
    if not name.endswith('_elastomer'):assert active_spheres[name]==pp_spheres[name]
assert all(len(active_spheres[name])==len(source_spheres[name]) for name in source_spheres)
updated['robot_cfg']['kinematics']['collision_spheres']=reference['robot_cfg']['kinematics']['collision_spheres']
assert updated==reference, 'Only the sphere file may differ in the robot configuration'
shared=yaml.safe_load((C/'task/gradient_grasp_fc.yml').read_text())
specific=yaml.safe_load((C/'task/gradient_grasp_fc_sharpa_50.yml').read_text())
specific['cost']['primitive_collision_cfg']['weight']=100.
assert shared==specific
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(out,indent=2)+'\n')
print('Verified live rollouts: world/self=100; DP/pad spheres=-2 mm; 50 spheres; existing PP correction retained; only 5 contact balls changed versus PP preset. DP geometry and collision policy unchanged. Shadow world=100.')

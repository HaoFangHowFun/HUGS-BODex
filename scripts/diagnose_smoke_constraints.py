"""Re-score saved grasps with actual collision costs; no optimization or model edits."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
import torch
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from src.task.synthesis import SynthesisRuntime
from curobo.util.human_prior_seed import build_world_info_batch
from curobo.types.state import JointState


def main():
    scene=(ROOT.parent/'benchmarks/sharpa_phase2_final/scenes.txt').read_text().strip()
    reports={}
    for label,folder,suite in [('Sharpa','sharpa_phase2_final','sim_sharpa'),('Shadow','shadow_bottle_control','sim_shadow')]:
        run=ROOT.parent/'benchmarks'/folder
        raw=np.load(next((run/'output').rglob('*_grasp.npy')),allow_pickle=True).item()
        manip=yaml.safe_load((run/f'configs/manip/{suite}/tabletop_full.yml').read_text())
        runtime=SynthesisRuntime(SimpleNamespace(save_debug=False))
        solver=runtime.create_or_update_solver(None,build_world_info_batch([scene]),manip)
        rollout=solver.rollout_fn
        robot=rollout.dynamics_model.robot_model
        assert list(raw['joint_names'])==robot.joint_names
        q=torch.tensor(raw['robot_pose'][0,:,1,:],device='cuda',dtype=torch.float32)
        zeros=torch.zeros_like(q[:,None])
        bound_cost=rollout.bound_cost(JointState(position=q[:,None].contiguous(),velocity=zeros,
            acceleration=zeros,jerk=zeros)).detach().cpu().numpy().reshape(-1)
        spheres=robot.get_state(q).link_spheres_tensor[:,None].contiguous()
        env=torch.zeros(len(q),device='cuda',dtype=torch.int32)
        cost=rollout.primitive_collision_cost
        aggregate=cost(spheres,env,opt_progress=1.).detach().cpu().numpy().reshape(-1)
        per_sphere=cost.coll_check_fn(spheres,cost._collision_query_buffer,cost.weight,
            env_query_idx=env,contact_distance=cost._get_contact_stage(1.),activation_distance=cost.activation_distance,
            return_loss=cost.return_loss,sum_collisions=cost.sum_collisions,compute_esdf=cost.compute_esdf).detach().cpu().numpy()[:,0]
        self_cost=rollout.robot_self_collision_cost(spheres).detach().cpu().numpy().reshape(-1)
        kin=robot.kinematics_config
        cfg=yaml.safe_load((run/f'configs/robot/{manip["robot_file"]}').read_text())['robot_cfg']['kinematics']
        link_cost={}
        for name in cfg['collision_link_names']:
            idx=robot.get_sphere_idx_from_namelst([name+'/0'],all_points_in_link=True)
            idx=idx.detach().cpu().numpy() if torch.is_tensor(idx) else idx
            link_cost[name]=per_sphere[:,idx].sum(axis=-1).tolist()
        report=dict(spheres=robot.total_spheres,collision_weight=cost.weight.detach().cpu().tolist(),
                    primitive_cost_sum_over_spheres=bool(cost.sum_distance),
                    aggregate_world_cost=aggregate.tolist(),self_collision_cost=self_cost.tolist(),
                    joint_bound_cost=bound_cost.tolist(),
                    world_cost_per_link=link_cost,active_spheres_per_candidate=(per_sphere>1e-9).sum(axis=1).tolist(),
                    final_contact_distance_by_sphere_m=cost._get_contact_stage(1.).detach().cpu().tolist(),
                    saved_grasp_distance_error_m=np.asarray(raw['dist_error'])[0,:,1].tolist())
        reports[label]=report
        print(label,'spheres',robot.total_spheres,'median world cost',float(np.median(aggregate)),
              'median self cost',float(np.median(self_cost)),flush=True)
        del solver,runtime,rollout,robot
    output=ROOT.parent/'benchmarks/shadow_bottle_control/constraint_diagnostics.json'
    output.write_text(json.dumps(reports,indent=2)+'\n')


if __name__=='__main__':main()

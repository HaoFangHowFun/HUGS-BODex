"""Standard synthesis with diagnostics for Sharpa and its Shadow control run."""
import json
import os
from pathlib import Path
import sys
import shutil

import hydra
import numpy as np
import torch
import trimesh
import yaml
from omegaconf import OmegaConf

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.task.synthesis import SynthesisRuntime, task_synthesis


@hydra.main(config_path='../cfg', config_name='base', version_base=None)
def main(cfg):
    if cfg.task.suite_config not in ('sim_sharpa.yml', 'sim_shadow.yml', 'sim_sharpa_s34_distal.yml', 'sim_sharpa_50_manual.yml', 'sim_sharpa_50_restored.yml'):
        raise ValueError('Only the Sharpa/Shadow smoke suites are supported')
    out = Path(os.environ.get('SMOKE_DIAGNOSTIC_DIR') or os.environ['SHARPA_DIAGNOSTIC_DIR'])
    out.mkdir(parents=True, exist_ok=True)
    # Recent trimesh uses OS entropy when seed=None, independent of np.random.seed.
    # Restrict the compatibility shim to this smoke process; Shadow source is unchanged.
    sample_surface=trimesh.sample.sample_surface
    def seeded_surface(*args,**kwargs):
        if kwargs.get('seed') is None:kwargs['seed']=int(np.random.randint(0,2**31-1))
        return sample_surface(*args,**kwargs)
    trimesh.sample.sample_surface=seeded_surface
    configs=ROOT/'src/curobo/content/configs'
    manip_data = yaml.safe_load((configs / 'manip' / cfg.manip_cfg_file).read_text())
    robot_relative = 'robot/' + manip_data['robot_file']
    kin = yaml.safe_load((configs / robot_relative).read_text())['robot_cfg']['kinematics']
    suite_data = yaml.safe_load((configs / 'manip' / cfg.task.suite_config).read_text())
    relatives = [robot_relative, 'robot/' + kin['collision_spheres'],
                 'robot/' + kin['hand_pose_transfer_path'],
                 f'manip/{cfg.task.suite_config}', f'manip/{cfg.manip_cfg_file}']
    relatives += ['robot/' + entry['transfer_file'] for entry in suite_data['grasp_types'].values()]
    relatives += ['task/' + manip_data[key] for key in ('base_cfg_file', 'particle_file', 'gradient_file')]
    for relative in dict.fromkeys(relatives):
        target=out/'configs'/relative;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(configs/relative,target)
    urdf = (ROOT / 'src/curobo/content/assets' / kin['urdf_path']).resolve()
    shutil.copy2(urdf, out / 'configs' / 'source_robot.urdf')
    (out / 'configs' / 'pose_convention.json').write_text(json.dumps(
        {'base_link': kin['base_link'], 'urdf_source': str(urdf)}, indent=2) + '\n')
    OmegaConf.save(cfg,out/'resolved_synthesis.yaml',resolve=True)
    original = SynthesisRuntime.attach_result_to_world_info
    def capture(self, world, result, solver, manip):
        arrays = {}
        for key in ('solution','seed','success','grasp_error','dist_error','contact_point','contact_frame','contact_force'):
            value = getattr(result,key,None)
            if isinstance(value,torch.Tensor):
                arrays[key] = value.detach().cpu().numpy()
        np.savez(out / 'solver_result.npz',**arrays)
        (out/'solver_summary.json').write_text(json.dumps(dict(
            optimized_success=int(arrays['success'].sum()),success_shape=list(arrays['success'].shape),
            solve_time_s=float(result.solve_time),seed=123),indent=2)+'\n')
        return original(self,world,result,solver,manip)
    SynthesisRuntime.attach_result_to_world_info = capture
    task_synthesis(cfg)


if __name__ == '__main__':
    main()

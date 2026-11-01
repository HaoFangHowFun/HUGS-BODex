Current configuration and versioned result snapshots: [project state](sharpa_project_state.md).
Historical workspace `reviews/` and `benchmarks/` links below require the original workspace.

# Sharpa 50-ball: DP contact eligibility, world-collision weight 100

Adopted for the current 50-restored preset on 2026-10-31 at the user's request.
World-collision weight was subsequently restored from 10 to 100 at the user's
request, retaining DP eligibility. No sphere positions or radii were changed.
Subsequently, the user authorized the PP sphere correction described in
[the PP refinement report](../../reviews/sharpa_pp_refinement/README.md).
Only the twelve long-finger PP spheres changed; all five contact balls remain fixed.
Latest update: the user then authorized fingertip position/radius adjustment.
The active preset now retains that PP correction and changes only the five
contact balls relative to it; DP geometry remains as before. See the
[fingertip refinement report](../../reviews/sharpa_tip_refinement/README.md).
The generic `sim_sharpa.yml` / 34-ball default and the Shadow preset are unchanged.

## Active configuration

- `src/curobo/content/configs/manip/sim_sharpa/tabletop_full_50_restored.yml`
  adds all five DP links to `additional_collision_contact_points_name`.
- `src/curobo/content/configs/task/gradient_grasp_fc.yml` supplies the original
  world-collision weight 100. The dedicated `gradient_grasp_fc_sharpa_50.yml`
  weight-10 task is retained for the prior experiment but is no longer selected.
- Continue using suite `sim_sharpa_50_restored.yml` and robot
  `right_sharpa_wave_50_tip_contact.yml`. The PP-refined and original 50-restored robot/spheres
  remain frozen for the historical comparisons below.

DP spheres remain active for world collision and self collision. Final world
clearance for DP spheres changes from +2 mm to -2 mm, matching the existing
contact-link policy. This is the tested contact-eligibility option, **not** the
separately discussed 0 mm / no additional penetration-allowance option.
Negative clearance is a soft proxy allowance, not a guarantee of safe mesh
penetration. Earlier DP stage clearances are 8 mm, 8 mm, then -2 mm.

The world cost includes scene collision checks; its weight remains nonzero.
Self-collision weight stays 100. The five elastomer attraction target identities,
the 50-sphere count, canonical joints/URDF, initialization,
500 iterations and the Bench controller/evaluation thresholds are unchanged.

## Current preset validation

The current contact-ball refinement lifted 10/20 and 11/20, with 7/20 and 9/20
successful and valid at all stages. Normal solver construction confirms only
the five contact balls differ from the PP preset, world/self weights remain
100 and DP/pad clearances remain -2 mm. Worst static self penetration remains
6.846 mm; this is not a collision-free readiness claim. The following results
describe the preceding configurations.

For the current PP-refined geometry, repeated lift results are 9/20 and 8/20,
with 8/20 and 7/20 successful and valid at all stages. Zero-pose false PP
overlaps were removed; PP volume coverage decreased from 90.97% to 87.19%.
Normal solver construction confirms only the 12 PP spheres changed. See the
PP refinement report for penetration and contact-distance regressions; the
geometry correction is not a demonstrated overall grasp-quality improvement.
The following DP-only trials used the earlier sphere geometry.

The existing `distal_allowance` and `distal_allowance_repeat` tests used this
DP -2 mm / weight-100 setting: physical success 8/20 and 9/20, successful with
all-stage joint limits 6/20 and 7/20, maximum static hand/object penetration
2.426 mm and 2.097 mm, and no numerical warnings. These are the previously
completed trials; restoring the setting did not generate new grasps. Normal
solver construction was checked again to verify weight 100 and DP eligibility.

## Mild-flex initialization trial (weight 100)

The current weight-100 / DP-enabled preset was also tested with mild initial
flexion (MCP_FE/PIP 0.2 rad, DIP/thumb IP 0.1 rad). Two new zero-pose controls
lifted 8/20 and 8/20; the matched-wrist mild-flex trials lifted 7/20 and 7/20.
Late contact convergence did not consistently improve, although squeeze-limit
violations decreased. The active initial pose remains all zeros. See the
[preshape study](../../reviews/sharpa_optimizer_diagnosis/preshape_study/README.md)
for exact inputs, convergence plots and validation.

## Historical combined validation (weight 10, no longer active)

Same bottle, 20 captured initial seeds, two runs. Initial seed tensors match
the previous baseline exactly; GPU optimization outputs are not bitwise
reproducible. Results under
`../../reviews/sharpa_optimizer_diagnosis/runs/distal_weight10{,_repeat}/`:

| Metric | Run 1 | Run 2 |
|---|---:|---:|
| Physical lift success | 10/20 | 11/20 |
| Success and all-stage joint limits | 8/20 | 7/20 |
| All-stage joint limits | 12/20 | 11/20 |
| Five designated pads within ±2 mm | 6/20 | 10/20 |
| Maximum static hand/object proxy penetration | 5.740 mm | 3.312 mm |
| Maximum static self proxy penetration | 2.836 mm | 2.457 mm |
| MuJoCo numerical warnings | 0 | 0 |

Previous repeated baseline: 4/20 and 5/20 physical success. The combination
improves this small-scene baseline but is not shown to outperform lowering
world weight alone (13/20 and 11/20). The two changes are not additive.
All pregrasp/grasp commands are within limits; the original extrapolated
`2*grasp-pregrasp` squeeze exceeds limits in some candidates (maximum 0.1091
and 0.0971 rad). Commands were not clipped. Penetration is measured using
MuJoCo collision proxies. This is not a full-object-set readiness claim.

`../../reviews/sharpa_optimizer_diagnosis/active_configuration_verification.json`
records current normal solver construction without ablation hooks: world weight 100,
self weight 100, 50 spheres, 5 attraction targets and all DP sphere clearances
at -2 mm. Shadow world weight remains 100. The dedicated gradient task is
otherwise exactly equal to the shared task configuration.

The smoke runner now also snapshots the referenced base, particle and gradient
task files, so future output directories retain the actual weight source.
Existing benchmark outputs and the two running viewers were preserved.

## Run this preset

From `HUGS-BODex`, choose a fresh output directory:

```bash
export HUGS_DATASET_ROOT=/home/hao-fang/hugs-dataset
export HUGS_OBJECT_ROOT="$HUGS_DATASET_ROOT/object"
export SMOKE_DIAGNOSTIC_DIR=/home/hao-fang/hugs-workspace/benchmarks/sharpa_50_dp_weight100_next
export HUGS_OUTPUT_ROOT="$SMOKE_DIAGNOSTIC_DIR/output"
.venv/bin/python scripts/run_sharpa_smoke.py task=synthesis name=sharpa_50_dp_weight100 \
 manip_cfg_file=sim_sharpa/tabletop_full_50_restored.yml \
 task.suite_config=sim_sharpa_50_restored.yml 'task.grasp_types=[3_right_full]' \
 task.scene_source.manifest_path=/home/hao-fang/hugs-workspace/benchmarks/sharpa_50_restored/scenes.txt \
 task.parallel_world=1 n_worker=1
```

Raw poses remain palm-rooted. Use the verified palm-to-flange conversion before
Bench evaluation; no root or joint convention was changed by this preset.

Current configuration and versioned result snapshots: [project state](sharpa_project_state.md).
Historical workspace `reviews/` and `benchmarks/` links below require the original workspace.

# Sharpa Wave right-hand default

The current 50-restored experiment was subsequently updated with DP contact
eligibility; world-collision weight was restored to 100 at the user's request. See
[sharpa_50_contact.md](sharpa_50_contact.md). This does not switch the generic
34-ball `sim_sharpa.yml` default described below.

As of 2026-10-31, the approved default is the 34-sphere configuration tested in
`../../benchmarks/sharpa_s34/README.md`. This replaces the 998-sphere synthesis
preset and the 50-sphere preview experiments.

## Active files (under src/curobo/content)

- `configs/robot/right_sharpa_wave_sim.yml`: active robot definition, palm root,
  26 collision links and the imported self-collision exclusion policy.
- `configs/robot/spheres/right_sharpa_wave_s34.yml`: 34 collision spheres.
- `assets/robot/sharpa_wave_s34/right_sharpa_wave_canonical_limits.urdf`:
  palm-root derivative referencing shared meshes; original canonical joint
  geometry, ordering and limits retained.
- `configs/robot/hand_pose_transfer/right_sharpa_wave_s34.yml`: object
  initialization transform, keyed by `right_hand_C_MC`. Its translation was
  corrected to `[0, 0.01, 0.0989]` on 2026-11-01 after five-seed visual checks.
  The imported `Sharpa_content` translation had the opposite sign; BODex's
  current transfer computes `root_translation = sampled_translation - R @ t`.
  The corrected value places the sampled frame near the middle MCP rather than
  behind the wrist.
- `configs/robot/hand_pose_human_transfer/right_sharpa_wave_s34.yml`: existing
  human-prior calibration converted to the palm frame.
- `configs/manip/sim_sharpa.yml` and `configs/manip/sim_sharpa/tabletop_full.yml`:
  active suite/task. Five elastomer/0 contact targets; 22 initial joint angles
  remain zero. Shared optimization settings are unchanged.

The source supplier's narrowed joint limits were not adopted. Left/right
`*_kinematics.yml` fixtures are retained for canonical asset validation, not
alternative synthesis presets. Shadow settings are unchanged.

## Validation and pose convention

```bash
.venv/bin/python scripts/validate_sharpa_synthesis.py
```

This checks CUDA FK and actual human-prior initialization against independent
FK, and records proxy overlaps without suppressing them.

Generated roots are palm-relative. Existing Bench models expect flange roots:
`p_flange = p_palm - R * [0, 0, 0.0295]`, with unchanged rotation and joint angles.
Use the verified conversion in `../../benchmarks/sharpa_s34/prepare_bench_input.py`
for that run; its raw outputs are preserved. Old flange-root results must not
receive the conversion. The general Bench formatter does not auto-detect roots.

The one-scene 20-candidate test achieved 7/20 physical lifts, 20/20 static contact
cases, and 15/20 targets within limits at all three stages. All seven successful
candidates satisfy all-stage limits. There were no numerical warnings, but
millimeter-scale collision-proxy penetration and five out-of-range squeeze
targets remain. Default status does not imply full grasp or continuous arm-path
validation. See the benchmark report for details and reproduction commands.

## Cleanup

Old 998/50/dorsal runtime configs and unused flange transfer files were removed.
Old builder scripts were retired to
`../../reviews/sharpa_external_20261031/retired_builders/` as historical source;
they must not be used to regenerate this default. Historical benchmark results
and config snapshots remain for reproducibility.

For the historical 34-sphere view, reuse the Sharpa viewer port after stopping
its existing process; do not launch a duplicate service:

```bash
# From HUGS-DexGraspBench
.venv/bin/python scripts/preview_collision_spheres.py --port 8088
```

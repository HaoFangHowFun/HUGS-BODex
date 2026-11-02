# Sharpa project state — 2026-11-02

Concise comparison tables for the three-day investigation:
[Sharpa 三天測試與比較摘要](sharpa_three_day_test_summary.md).

## Adopted specification — 2026-11-02

At the user's request, the selected 50-sphere preset now adopts the latest
four-finger and thumb collision/contact geometry and the Shadow-matched initial
joint pose. **The production schedule remains 300/100/100 (500 total steps)**;
1200/100/100 is comparison evidence only. Controller remains **7.5/0/0.05**.
The adopted 300-step configuration lifted **162/200 (81.0%)**, with **146/200**
successful and all commanded targets legal. The generic 34-sphere preset and
Shadow are separate, unchanged configurations; select `sim_sharpa_50_restored.yml`.

Versioned [specification and evidence](sharpa_evidence/20261102_adoption/README.md)
include the exact 22-joint pose, sphere-file checksum, paired outcomes and
normal-solver validation. Workspace experiment artifacts remain local.

## Latest comparison: current thumb spheres and matched initial pose

The configuration formerly previewed as `All-finger contact fit`, including the
thumb, plus `Draft initial vs Shadow` completed a paired **200 candidates per budget**
study. Both runs use identical full initial tensors: the historical 20-scene
study's saved palm poses with all 22 joints replaced by the current preview
pose. Actual schedules were **300/100/100** and **1200/100/100**, with the retained
**kp=7.5, kv=0, passive damping=0.05** controller.

| Metric | Stage 1: 300 | Stage 1: 1200 |
|---|---:|---:|
| Lift success | 162/200 (81.0%) | 185/200 (92.5%) |
| Lift + all targets legal | 146/200 (73.0%) | 168/200 (84.0%) |
| Pre-lift six-dimensional closure | 182/200 | 192/200 |
| Squeeze limit violations | 19/200 | 17/200 |
| Synthesis wall time, including load/logging | 69.7 s | 178.5 s |

1200 steps gained 27 successful seeds and lost 4, for **+11.5 percentage points**
(20-scene bootstrap 95% interval **+6.5 to +17.5**). Neither run had pregrasp/grasp
limit violations, MuJoCo warnings, or pre-simulation rejections. Runtime checks
verified all 50 sphere tensors, joint order and pose, actual stage counts, finite
curves, unchanged source assets, and identical initial seeds across budgets.
Apart from output experiment names, only the iteration schedule differs between
the two effective manipulator configurations.

These selected failure-scene results do not estimate the full 600-case suite.
Relative to the preceding 119/200 study, thumb spheres and initial joints both
changed, so that difference cannot isolate either cause. Each budget ran once.
The 300-step configuration was subsequently adopted as specified above.
The 1200-step result remains a comparison, not the default. Report and exact snapshots:
[current configuration paired iteration study](../../reviews/sharpa_thumb_pose_iterations_200/README.md).

## Earlier recorded progress: contact/DP geometry at 300 steps

The preview `Contact + DP fit` was evaluated on the same saved 200 initial pose
tensors from the earlier 20-scene study, using **300/100/100**, zero initial
joints and the retained **7.5/0/0.05** controller. New spheres lifted
**119/200 (59.5%)**, with **100/200** lifting successfully and all commanded
targets within limits. Historical paired old-sphere 300-step runs lifted
111/200 and 116/200; old spheres at 1200 steps lifted 149/200.

The new-versus-first-old delta is +4.0 percentage points (20-scene bootstrap
95% interval −3.0 to +11.5), and +1.5 points versus the old repeat (−6.0 to +9.0).
This is not clear evidence of a stable improvement. Actual pre-lift closure was
159/200 versus 157/200 in the first old run. Squeeze targets exceeded limits in
43/200 cases; pregrasp/grasp targets stayed within limits. No MuJoCo warnings or
pre-simulation rejections occurred. The run changed the complete preview sphere
layout, not only the four contact balls, so it is not a contact-only ablation.

Runtime audits verified all 50 local spheres against the preview snapshot,
actual 300/100/100 stage counts, identical saved initial tensors, original
collision policy and unchanged physical assets/controller. The experiment used
isolated configuration files; at that time formal settings were unchanged and
no adoption or commit was made. Full report:
[new sphere 300-step paired study](../../reviews/sharpa_contact_dp_300_200/README.md).

## Earlier recorded progress: first-stage iteration validation

The 600-grasp paired experiment is complete. With **kp=7.5, kv=0, passive
joint damping=0.05**, all 22 initial joints zero, and later optimizer stages
fixed at 100/100, increasing stage 1 from 300 to 1200 steps produced:

| Metric | Stage 1: 300 | Stage 1: 1200 |
|---|---:|---:|
| Lift success | 348/600 (58.0%) | 451/600 (75.2%) |
| Lift success with all three targets within limits | 283/600 | 389/600 |
| Actual six-dimensional force closure before lift, table excluded | 461/600 | 521/600 |
| Lift success in the additional 400 candidates | 237/400 | 302/400 |

The full comparison improved 135 candidates and regressed 32. Nineteen object
categories improved; ToyFigure declined from 17/30 to 15/30. All 1,200 physical
evaluations completed without MuJoCo warnings. Generation took about 2.7 times
as long for the new 400 candidates. The prior 200 paired candidates were reused
and their physical outcomes reverified; the remaining 400 were evaluated after
selecting the iteration budget, on additional poses of the same 20 objects.

This comparison uses saved, identical initial pose tensors between budgets.
Historical Sharpa 384/600 and Shadow 476/600 used different generated poses and
are references, not the paired baseline. Full results, paired cases, and source
verification are in the workspace-local
[600-grasp report](../../reviews/sharpa_stage1_1200_600/README.md).

The older 600-case study used zero joints and previous spheres. Its 1200-step
recommendation did not change the default. The subsequently adopted specification
above uses the new geometry and initial pose, with the user-selected 300-step
first stage. Shadow configuration and evaluation remain intact.

## Selected configuration

All configuration paths below are relative to `src/curobo/content/configs/`.

| Role | File / value |
|---|---|
| Suite | `manip/sim_sharpa_50_restored.yml` |
| Manipulator | `manip/sim_sharpa/tabletop_full_50_restored.yml` |
| Robot | `robot/right_sharpa_wave_50_tip_contact.yml` |
| Spheres | `robot/spheres/right_sharpa_wave_50_tip_contact.yml` |
| Optimization task | `task/gradient_grasp_fc.yml` |
| World / self collision weight | 100 / 100 |
| Attraction targets | Five elastomer `/0` spheres |
| Additional contact-eligible links | All five DP links, final clearance −2 mm |
| Initial joints | Shadow-matched thumb; four fingers spread and MCP-flexed (exact values in specification) |
| Stage iterations | **300 / 100 / 100**, total 500 |
| Controller kp / kv / passive damping | **7.5 / 0 / 0.05** |
| Surface-pose transfer | `robot/hand_pose_transfer/right_sharpa_wave_s34.yml`, translation `[0, 0.01, 0.0989]` |
| Geometry | Adopted continuous four-finger chain, fitted DP/contact balls, and fitted thumb MC/PP/DP/contact balls |

The sphere total is **45 non-attraction spheres + 5 attraction spheres**.
Both sets participate in collision handling. DP eligibility changes the soft
clearance, not the five grasp-attraction targets or the self-collision exclusions.
Canonical URDF joint ordering, axes, limits and kinematics are preserved.

Contact centers/radii below are rounded local elastomer-frame values in mm;
the sphere YAML and versioned checksum define the exact values.

| Fingers | x | y | z | r |
|---|---:|---:|---:|---:|
| Index, middle, ring, pinky | −0.007 | −19.910 | 0.960 | 6.808 |
| Thumb | 0.021 | −21.763 | 2.504 | 7.923 |

The pad-facing direction is local −Z. Contact diameter is approximately the
local pad width; long-finger contact-volume overlap with DP balls is 18.8%, and
thumb overlap is 20.6%. The chain is continuous in sampled articulations, but
spheres do not fully enclose the mesh. In particular thumb MC/PP volume coverage
is about 78.4%/69.2%. The matched initial pose's minimum enabled sphere gap is
3.006 mm. Canonical joint limits and collision exclusions remain unchanged.

## Historical results and limitations

Two matched-initialization runs of 20 candidates on one bottle:

| Configuration | Lift A / B | Lift + all-stage limits A / B | Max static object penetration A / B | Max static self penetration A / B |
|---|---|---|---|---|
| Previous PP baseline | 9/20, 8/20 | 8/20, 7/20 | 3.055, 4.945 mm | 3.242, 5.296 mm |
| Retained contact refinement | 10/20, 11/20 | 7/20, 9/20 | 3.391, 2.834 mm | 6.846, 5.145 mm |

All 120 new fingertip-ablation replays completed 750 steps without numerical
warnings. These are historical results, not new runs during commit preparation.
Only one object was tested. Worst self penetration increased; successful lifting
does not establish collision-free full evaluation readiness. Penetration is from
MuJoCo collision proxies, not exact CAD geometry. Original squeeze extrapolation
`2*grasp-pregrasp` can exceed limits; it was not silently clipped. See
[versioned evidence](sharpa_evidence/README.md) for raw summary values and provenance.

The corrected surface-pose transfer was evaluated on one scene each of bottle,
mug, and camera with ten candidates per scene. It lifted 6/30, versus 3/30 for
the earlier Sharpa transfer on those same scenes and 27/30 for historical Shadow.
Five of the six new lifts also had all three commanded stages within joint
limits. Candidate seeds and optimization trajectories were not paired, so the
small comparison does not establish a causal lift-rate improvement. Detailed
results and reproduction commands are in the workspace-local
`benchmarks/sharpa_transfer_small/README.md`.

## Configuration history

Historical configs are retained because existing comparisons and viewers reference
them. They are not alternate definitions of the current retained geometry.

| Robot/sphere suffix | Status |
|---|---|
| `sim` / `s34` | Generic 34-sphere preset, separate baseline |
| `50_restored` | Frozen original 50-volume geometry |
| `50_pp_refined` | Intermediate PP-only correction |
| `50_tip_contact` | **Adopted: continuous finger/thumb geometry and exposed contact balls** |
| `50_tip_dp`, `50_tip_combined` | Evaluated candidates, not adopted |
| `50_manual` | Historical manual contact editing trial |
| `50_volume_preview`, `50_shadow_seed_preview` | Historical geometry previews |
| `*_kinematics` | Left/right flange-root validation fixtures, not synthesis |

`gradient_grasp_fc_sharpa_50.yml` is the historical weight-10 ablation; the selected
50-sphere preset uses the shared weight-100 task. `sim_sharpa_s34_distal.yml` is a
separate 34-sphere contact-definition trial. Existing filenames are retained to
avoid invalidating stored commands and experiment identities.

## Repository boundaries

Keep these four repositories as siblings:

- `HUGS-Sharpa-Assets`: canonical left/right URDFs, meshes, provenance, source
  notices and independent FK validation. Required by the other three repositories.
- `HUGS-BODex`: synthesis configs, collision/contact proxies, initialization,
  smoke diagnostics and this selected-preset evidence. Right-hand synthesis only;
  both hands have kinematics validation.
- `HUGS-DexGraspBench`: left/right MJCF, Bench-local palm decomposition and
  controllers, motion/contact validation, smoke evaluation and preview tools.
- `HUGS-DexLearn`: left/right FK/mesh configs and checks. No Sharpa training or
  complete learned-inference pipeline is claimed.

Workspace `reviews/`, `benchmarks/`, `Sharpa_content/`, `agent_tasks/` and
`third_party_build/` remain local. Preserve them separately if exact historical
experiments are needed. The supplied configuration source was adapted to preserve
canonical limits; the shared asset repository records original model provenance.

## Verification and use

From BODex with its environment and the shared assets installed:

```bash
.venv/bin/python scripts/check_sharpa_assets.py
.venv/bin/python scripts/validate_sharpa_synthesis.py
# Set HUGS_DATASET_ROOT / HUGS_OBJECT_ROOT for your dataset installation.
.venv/bin/python scripts/verify_sharpa_50.py \
  --scene-manifest /path/to/one_scene.txt \
  --output /tmp/sharpa_active_verification.json
```

The last command constructs the normal Sharpa and Shadow solvers and inspects
actual rollout weights/contact allowances; it does not generate grasps. The first
two check both-hand CPU FK and generic 34-sphere CUDA initialization respectively.
The selected 50-sphere suite can be passed to `scripts/run_sharpa_smoke.py` as shown
in [the contact configuration notes](sharpa_50_contact.md).

Generated root poses refer to the palm. Bench expects the flange:
`p_flange = p_palm - R @ [0, 0, 0.0295]`. Apply only to identified palm-root data
and verify world FK; do not convert old flange-root records again. The general
Bench formatter does not infer this distinction. Historical conversion drivers
are retained with their runs in the local workspace.

The previous initialization and grasp-replay services were stopped. The current
Shadow/Sharpa sphere comparison can be launched from `HUGS-DexGraspBench` with:

```bash
.venv/bin/python scripts/preview_collision_spheres.py --sharpa-spheres active --port 8089
```

The `active` mode reads the retained manipulator's robot file, contact targets,
and initial joints, including the current PP/contact-sphere refinements. Green
marks designated attraction/contact spheres, orange the other collision spheres;
both participate in collision handling. Historical CLI modes remain available
with explicit preset names. The display shows URDF meshes and BODex spheres,
not MuJoCo convex contact geometry or measured force contacts.

The active viewer now defaults to `BODex initial` and loads the adopted sphere
and manipulator files directly, without workspace draft overrides. `Joint zero`
remains available as an inspection pose. Optional sphere/pose draft arguments
can still compare experimental configurations without changing formal settings.
The current service is http://localhost:8089; workspace launch metadata is in
`reviews/sharpa_shadow_spheres_current/launch.json`.

The geometry/pose development history remains in workspace `reviews/`:
`sharpa_continuous_spheres`, `sharpa_contact_dp_fit`, `sharpa_thumb_spheres_fit`,
`sharpa_initial_pose_preview`, and `sharpa_thumb_shadow_pose`. Their notes describe
states at the time of fitting; the adopted files and versioned specification
above are authoritative. The original 119/200 study used zero joints and did
not include the latest thumb refit. The current paired 162/200 versus 185/200
study covers the adopted geometry and matched pose, at two iteration budgets.

> Historical 998-sphere baseline. Superseded by [the current default](sharpa_default.md) on 2026-10-31. Old builders/configurations described below are retired; benchmark snapshots remain available.

# Sharpa right: BODex smoke integration

This is an experimental synthesis preset, not a production grasp generator.
The measured end-to-end report is in
`../../HUGS-DexGraspBench/docs/sharpa_phase2_results.md`.

## Implementation

- `scripts/prepare_sharpa_synthesis.py` builds Sharpa-specific robot, sphere,
  surface-pose transfer, human-pose transfer, manipulation and suite YAMLs.
- `scripts/validate_sharpa_synthesis.py` checks actual CUDA FK against independent
  URDF FK at 12 random root/joint poses, canonical joint ordering, the actual human
  prior seed builder at 12 known world poses, and unfiltered zero-pose sphere overlaps.
- `scripts/run_sharpa_smoke.py` calls the existing synthesis runtime and captures
  its real success flags, errors, contact arrays, seeds, solutions, config snapshots
  and resolved Hydra configuration. It does not change solver success thresholds.
- `scripts/inspect_sharpa_smoke.py` validates generated poses and renders them.

The source URDF, canonical 22-joint order and limits are unchanged. cuRobo traverses
the configured link chains to derive its internal order, so thumb is explicitly
first in `link_names`. Contact groups remain index/middle/ring/pinky/thumb, with
independent named indexing. There are no Shadow tendon couplings.

## Collision policy and fitting

Spheres are derived from each original collision mesh. Candidate centers come
from a 3 mm voxel grid inside the source mesh. Interior radii are nearest-boundary
distance plus 1 mm; supplemental surface spheres have radius 1 mm. Greedy selection
targets a sampled 1.5 mm surface gap, subject to explicit per-link budgets. This
target is not universally met: independent coverage and outside-distance values
are reported per link in `sharpa_sphere_validation.json`. Signed distances for open
MP/DP source meshes are approximate; outside-distance certification is omitted
for those meshes. A shell-to-mesh unsigned distance can measure *interior* space
and must not be described as outside overshoot.

Only pairs on the same fixed rigid body or directly adjacent moving rigid bodies
are excluded. Fixed fingertip pads belong to their DP rigid body. Moving virtual
frames are not collapsed: palm/thumb_MC and thumb_MC/thumb_PP remain tested when
separated by two moving joints. Palm/thumb, cross-finger and other nonadjacent
contacts are not broadly disabled. The complete symmetric exclusion dictionary
and zero-pose remaining overlaps are in the JSON reports. Sphere approximations
are BODex-local; Bench continues to use its separately validated collision meshes.

An initial enclosing-cluster sphere prototype produced large false overlaps and
was rejected. The final method constrains radii using interior distance. It uses
more spheres and triggers cuRobo's slower self-collision kernel; this is a known
performance limitation, not a solver error.

## Frames and contacts

All roots use the URDF `right_hand_flange` frame; quaternions are wxyz. The palm
link is 29.5 mm above the flange. The surface-sampling frame has axes +X toward the
object, +Y toward the right thumb, +Z toward the fingers, with chosen sampling
origin `[0.025, 0, 0.085]` m in flange coordinates. This is an optimization anchor,
not a change to the palm link's physical origin.

Human prior uses the actual zero-pose index MCP position `[0.001, 0.0303, 0.1254]`
m in flange coordinates. Its right-handed axes in flange coordinates are
`[-Z, -X, +Y]`. The middle axis sign follows the cross product; using +X would
create a reflection. The mapping uses the existing equation
`R_base = R_human R_transfer.T`, `p_base = p_human - R_base t_transfer`.
Tests exercise the actual seed builder with synthetic records; no human dataset
quality, finger-joint retargeting or learned Sharpa prior is claimed.

Five elastomer meshes provide independent finger contacts, with pad-local normal
`[0,0,-1]`. Zero-pose index pad normal is approximately flange +X; the thumb's
direction follows its own link rotation. Contact sphere index 0 is the fitted
sphere nearest each pad centroid. Later optimization stages use the existing
mesh contact query. The provisional Shadow force/pressure objective is preserved
and still needs Sharpa-specific calibration.

The seeder's canonical hand joint pose is all zero. The historical Bench-only
thumb_CMC_AA=-10 degree hold workaround is not imported.

## Reproducibility

Use explicit trimesh sample seeds 123/124/125 for fit and independent diagnostics;
recent trimesh versions otherwise draw OS entropy even after `np.random.seed`.
The smoke wrapper seeds unseeded trimesh surface calls from the runtime's NumPy
stream. Synthesis's existing module seed is 123 (the top-level Hydra seed alone
is not sufficient). Final run config snapshots are stored before solving.

See the Bench report for exact commands, counts and limitations. Do not scale
up from this smoke preset until contact quality and all-stage joint validity pass.

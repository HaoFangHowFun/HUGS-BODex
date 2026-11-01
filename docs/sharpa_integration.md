Current configuration and versioned result snapshots: [project state](sharpa_project_state.md).
Historical workspace `reviews/` and `benchmarks/` links below require the original workspace.

Current right-hand synthesis default: [Sharpa 34-sphere configuration](sharpa_default.md). The flange-root configs below remain dedicated left/right kinematics validation fixtures.

# Sharpa kinematics validation

Place `HUGS-Sharpa-Assets` beside this repository. The two
`src/curobo/content/configs/robot/{right,left}_sharpa_wave_kinematics.yml` configs
use the existing cuRobo schema and asset path resolution. Root poses refer to the
hand flange. The 22 joints follow the source canonical order, with no tendons.

```bash
.venv/bin/python scripts/check_sharpa_assets.py
```

This checks the CPU cuRobo parser, config, limits, mesh resolution and independent
FK at 19 poses per hand; see `sharpa_validation.json`. It does not test CUDA FK.
These are **kinematics-only configs**, not synthesis presets: collision spheres,
self-collision policy, contact groups and hand-pose/human-prior transfer still need
Sharpa-specific calibration. Jerk/acceleration settings follow the existing config
schema defaults and are not hardware calibration. No generation was started.

Full audit and source provenance: `../HUGS-Sharpa-Assets/SHADOW_AUDIT.md` and
`../HUGS-Sharpa-Assets/provenance.json` relative to the repository root.

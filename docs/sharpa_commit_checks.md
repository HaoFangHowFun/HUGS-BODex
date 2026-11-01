# Commit preparation checks — 2026-11-01

- Shared assets: 57 source hashes verified; left/right 36-pose FK passed.
- BODex CPU parser: left/right 19-pose FK, canonical joint order and limits passed.
- DexLearn: left/right 19-pose FK and 30 visual-link mesh loading passed.
- Bench: both-hand FK, 22 actuator mappings and 5-second initial-pose holds passed.
- BODex generic preset: 12 CUDA FK and 12 human-prior mapping cases passed.
- Selected 50-sphere preset: normal Sharpa/Shadow solver rollout checks passed
  (world/self 100, five attraction targets, DP/pad final clearance −2 mm,
  retained PP refinement, unchanged DP geometry).
- Existing BODex public-contract tests: 18 passed. System pytest was appended to
  the existing virtual environment's import path; no packages were installed.
- Existing Bench single-hand formatter tests: 4 passed.

Expected parser-extension and dependency deprecation warnings remain. These are
integration/regression checks; full Phase 1 motion/contact scans and grasp
synthesis were not rerun during cleanup. Historical lift results are explicitly
identified in the [project state](sharpa_project_state.md).

Runtime configuration, meshes and viewers were not changed by this cleanup.
The shared assets are committed locally; their GitHub repository must exist before
that commit can be pushed.

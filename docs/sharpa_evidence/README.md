# Adopted Sharpa 50-sphere evidence

Snapshot of the completed experiments, preserved inside the BODex repository for
review. See [current project state](../sharpa_project_state.md) for the selected
configuration, limitations, commands and historical preset inventory.

- `tip_comparison.json`: PP baseline, DP-only, contact-only and combined trials,
  two matched-seed batches of 20 each. `tip_contact_a/b` is adopted.
- `tip_geometry_validation.json`: independent dense geometry checks, including
  coverage losses and residual protrusion; the DP candidate was not adopted.
- `pp_summary.json`: preceding twelve-PP-sphere correction and its comparisons.
- `active_configuration_verification.json`: saved normal solver rollout checks
  for Sharpa and Shadow. Recheck with `scripts/verify_sharpa_50.py`.
- `active_contact_adjustments.png`: adopted contact-only geometry comparison.
- `provenance.json`: original workspace paths and SHA-256 of these snapshots.

These are recorded results, not fresh synthesis runs performed during commit
preparation. Full optimizer tensors, trajectories, raw contact traces and research
scripts remain in the workspace's `reviews/` and `benchmarks/` directories. They
are not included here; a clone reproduces the configured pipeline but does not
contain the exact historical random inputs or all ablation drivers. Do not claim
bitwise reproducibility from these summaries. The historical GPU runs were not
bitwise deterministic even with matching initial tensors.

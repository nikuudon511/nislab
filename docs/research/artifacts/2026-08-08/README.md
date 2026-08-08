# Selected Research Artifacts: 2026-08-08

This directory contains a lightweight GitHub-friendly export of the most
important current evidence.  Raw `results/` directories remain the authoritative
source and should be backed up separately to an external disk.

## Contents

- `results/`: selected `summary.csv`, `observation.csv`, and `run_config.txt`
  files for the current main and prediction-control experiments.
- `figures/`: selected slide/resume figures copied from
  `results/slide_progress_figures/`.
- `SHA256SUMS.txt`: checksum manifest for this export.

## Verification

From the repository root:

```bash
cd docs/research/artifacts/2026-08-08
sha256sum -c SHA256SUMS.txt
```

## Notes

- This directory is intended to be committed to the private GitHub repository.
- It is not a replacement for external-disk backup of the full `results/`
  directory.
- If a future slide or resume uses a new result, copy its `summary.csv`,
  `observation.csv`, `run_config.txt`, and related figure into a new dated
  artifact directory.

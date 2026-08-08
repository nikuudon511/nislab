# Research Backup Plan

Last updated: 2026-08-08 JST

## Purpose

This file defines how to preserve the research state before travel or machine
replacement.  The goal is to avoid losing source code, research notes,
simulation evidence, and paper/slide artifacts even if the local PC fails.

## Backup Policy

Use a 3-2-1 style backup:

- Local working copy: this PC.
- Remote private Git repository: source code, research notes, scripts, paper
  files, and selected result artifacts.
- External disk or second PC: full raw `results/` directory and full project
  archive.

## GitHub Private Repository Scope

Commit these to the private repository:

- `docs/research/`
- `docs/research/artifacts/YYYY-MM-DD/` selected lightweight evidence exports.
- `AGENTS.md`
- `src/automotive/examples/v2v-hybrid-nr-v2n2v.cc`
- `src/automotive/examples/run_hybrid_nr_v2n2v_*.sh`
- `src/automotive/examples/FIT2026_RFurumai/`
- selected figure/CSV exports copied from `results/` when they are used in a
  slide, resume paper, or supervisor discussion.

Do not commit these as normal Git files:

- `results/` raw run directories.
- `build/`, `cmake-cache/`, and other generated build outputs.
- large packet captures (`*.pcap`) and transient logs.

Reason: raw simulation outputs grow quickly and are already ignored by
`.gitignore`.  They should be preserved as external archives instead of normal
Git history.

## Current Critical Raw Result Directories

Preserve these directories on an external disk because they support the current
main claims and prediction-control discussion:

- `results/ttl02_bg500_v2n2v_rmr_pred_dl_seed30_100s_20260806_005249`
- `results/ttl02_bg500_v2v_rmr_predicted_seed30_100s_20260805_012133`
- `results/ttl02_bg500_v2n2v_only_object_rmr_seed30_100s_20260726_001243`
- `results/ttl02_bg500_v2v_only_rmr_seed30_100s_20260726_014015`
- `results/ttl02_bg500_hybrid_rmr_no_high_dual_seed30_100s_20260730_003248`
- `results/ttl02_bg500_hybrid_no_rmr_seed30_100s_20260731_004816`
- `results/ttl02_bg500_v2v_no_rmr_seed30_100s_20260801_031910`
- `results/ttl02_bg500_v2n2v_only_no_rmr_seed30_100s_20260802_012824`
- `results/slide_progress_figures`

The authoritative experiment list remains `docs/research/experiment_index.md`.

## External Disk Backup Commands

Replace `/media/$USER/RESEARCH_BACKUP` with the mounted external disk path.

```bash
cd /home/ryose/VaN3Twin/ns-3-dev
BACKUP_ROOT="/media/$USER/RESEARCH_BACKUP/van3twin_$(date +%Y%m%d)"
mkdir -p "$BACKUP_ROOT"

rsync -aH --info=progress2 \
  --exclude build/ \
  --exclude cmake-cache/ \
  --exclude 'cmake-build-*/' \
  ./ "$BACKUP_ROOT/ns-3-dev/"
```

Create a checksum manifest after copying:

```bash
cd "$BACKUP_ROOT"
find ns-3-dev -type f -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS.txt
```

Verify later:

```bash
cd "$BACKUP_ROOT"
sha256sum -c SHA256SUMS.txt
```

## GitHub Private Backup Commands

Run these manually after reviewing `git status`.

```bash
cd /home/ryose/VaN3Twin/ns-3-dev
git status --short
git add AGENTS.md docs/research src/automotive/examples/FIT2026_RFurumai \
  src/automotive/examples/v2v-hybrid-nr-v2n2v.cc \
  src/automotive/examples/run_hybrid_nr_v2n2v_*.sh
git status --short
git commit -m "Preserve research state before travel"
git push origin HEAD
```

Do not use `git add results/` unless selected files have first been copied into
a dedicated tracked artifact directory.

Current selected artifact export:

- `docs/research/artifacts/2026-08-08/`

## Recovery Checklist

On a new machine:

1. Clone the private repository.
2. Read `docs/research/README.md`.
3. Restore raw `results/` from the external disk archive if full evidence is
   needed.
4. Run `git status --short` and verify the working tree before new edits.
5. Use `docs/research/experiment_index.md` to locate authoritative result
   directories and figure files.

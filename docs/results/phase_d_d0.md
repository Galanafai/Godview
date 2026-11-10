# Phase D0: environment gate (V2V4Real baseline reproduction)

Run on 2026-11-10 on a RunPod A40 (compute capability 8.6, driver 595.91.07).

## Result

The V2V4Real repository's own evaluation (`opencood/tools/inference.py`) of
its published PointPillars late-fusion checkpoint, on the full test split
(test1, test2, test3: 9 scenarios, 1,993 frames), sync mode:

| Method | AP@0.5 (ours) | AP@0.5 (README) | AP@0.7 (ours) | AP@0.7 (README) |
|---|---|---|---|---|
| No Fusion | 39.81 | 39.8 | 22.02 | 22.0 |
| Late Fusion | 55.07 | 55.0 | 26.63 | 26.7 |

All four numbers are within 0.1 AP point of the README. **Gate: passed.**

By range (AP@0.5 / AP@0.7), from the same runs:

| Method | 0 to 30 m | 30 to 50 m | 50 to 100 m |
|---|---|---|---|
| No Fusion | 69.1 / 42.7 | 29.3 / 14.4 | 4.8 / 1.6 |
| Late Fusion | 73.5 / 36.7 | 43.7 / 22.2 | 36.3 / 17.2 |

Raw outputs: `docs/results/phase_d/d0/*_eval.yaml`.

## What was run

* Code: https://github.com/ucla-mobility/V2V4Real at commit
  `5a821e13753bafc611f95c47bc1a306acdcb0f7c`.
* Checkpoint: Box file 1619899319493 (the README links it for both No Fusion
  and Late Fusion; it is one single-car PointPillars model, `net_epoch60.pth`,
  sha256 `9221830d15badc39196164575d49f08dea05e02cea59ef387ed506de919e3fc9`).
  Its pickle contains only `OrderedDict` and torch tensor rebuild calls
  (checked with `pickletools` before loading).
* The only change to the shipped `config.yaml` is `validate_dir`, pointed at
  a folder that links the 9 test scenarios. Both configs used are in
  `docs/results/phase_d/d0/`. `wild_setting.async` is `false` (sync).
* Data: `test_01.zip`, `test_02.zip`, `test_03.zip` from the Box share
  (633,582,640, 1,228,581,180 and 1,551,328,836 bytes), paths checked before
  unpacking.

## Environment

Original stack, no version substitutions: Python 3.7.12, PyTorch
1.12.0+cu113 (pip wheel, has sm_86 kernels), spconv-cu113 2.3.6, cumm-cu113
0.4.11, numpy 1.21.6. Full list: `docs/results/phase_d/d0/pip_freeze.txt`.
Rebuild script: `scripts/phase_d/setup_v2v4real_env.sh`. Everything is under
`/workspace/tools`.

Changes needed to make it run:

1. `requirements.txt` installs the generic `cumm` package (0.5.3) next to
   `cumm-cu113`, and `import spconv` then fails ("could not convert default
   argument 'workspace: tv::Tensor'"). Removing `cumm` and reinstalling
   `cumm-cu113` 0.4.11 fixed it. No code was changed.
2. `python setup.py develop` was replaced by putting the repository on
   `PYTHONPATH`. Same effect for running the tools.

## GPU time

No Fusion 4 min 15 s, Late Fusion 4 min 46 s, plus one 15 s run that failed at
import (fix 1). About 0.15 GPU hours in total.

## Not shown here

* Async mode (README 50.2 / 22.4 for Late Fusion) was not run; the gate is
  on sync only.
* The match is on the dataset's own evaluation code, which D1 will reuse; it
  says nothing yet about GodView.

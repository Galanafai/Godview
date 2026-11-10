# Phase D1: tracker ablation on val

Val only (3 scenarios, 748 frames, 1,496 car-frames), CPU only, nothing run on
test. Protocol and selection rule were committed first
(`docs/results/phase_d_d1_plan.md`, "Tracker ablation on val"). AP is car 0's
world against the dataset's ground truth, all frames, AP@0.5 / AP@0.7 in
percent, from `scripts/phase_d/ap_eval.py`. Runs: `scripts/phase_d/ablate_val.py`;
raw rows: `docs/results/phase_d/d1/ablation_*.json`. On val, No Fusion is
45.95 / 25.73 and Late Fusion 52.01 / 28.80.

## Read this first: what this AP measures

The dataset's AP code appends each frame's true and false positives to one
list in frame order and never sorts them by score across frames (checked in
`opencood/utils/eval_utils.py`); the score only decides which box takes a
ground-truth box first within a frame. So AP here depends almost entirely on
which boxes are in the world each frame, and little on their scores: changing
the score rule changed AP by at most 0.01 (below). Everything that removes
false boxes raises this AP; the metric does not reward a box for being
uncertain, and does not reward remembering an object that no sensor sees
this frame.

## 1. Where the no-sharing loss is

Baseline (committed tracker, no sharing): 41.95 / 24.08, against 45.95 /
25.73 for the detector alone, a loss of 4.0 / 1.65 on val (7 / 5 points on
test). The baseline world holds 14.4 objects per car-frame; 4,884 object-frames
are coasting tracks and there are 914 duplicates.

| scored as | AP@0.5 / AP@0.7 |
|---|---|
| world object as published (box, score) | 41.95 / 24.08 |
| passthrough: latest raw detection box, centre and score of each live track | 38.28 / 20.69 |
| filtered box, latest raw score | 41.95 / 24.09 |
| latest raw box and centre, filtered score | 38.28 / 20.69 |
| No Fusion | 45.95 / 25.73 |

The passthrough does not recover No Fusion's AP; it is worse. The loss is in
which tracks exist, not in boxes or scores. The passthrough is worse than the
published box because a coasting track keeps its last detection, which is
where the object was, not where it is. On tracks updated this frame (next
section), the raw box costs 1.2 AP@0.5 and 3.6 AP@0.7 relative to the filtered
one (49.28 / 27.45 against 50.49 / 31.05).

## 2. One change at a time (no sharing)

| change | AP@0.5 / AP@0.7 | duplicates | split label-frames | coasting object-frames | objects per car-frame | missed |
|---|---|---|---|---|---|---|
| baseline | 41.95 / 24.08 | 914 | 886 | 4,884 | 14.38 | 2,788 |
| (a) publish only tracks updated this frame | **50.49 / 31.05** | 13 | 0 | 0 | 9.51 | 3,761 |
| (b) overlap prune 0.3 IoU, keep older | 42.31 / 24.23 | 302 | 248 | 4,306 | 13.73 | 2,799 |
| (b2) overlap prune 0.3 IoU, keep more recently updated (added) | 42.96 / 24.87 | 176 | 123 | 3,742 | 13.17 | 3,088 |
| (c) score = latest | 41.95 / 24.09 | 914 | 886 | 4,884 | 14.38 | 2,788 |
| (c) score = max | 41.95 / 24.08 | 914 | 886 | 4,884 | 14.38 | 2,788 |
| (d) published box = latest detection's | 41.41 / 22.41 | 914 | 886 | 4,884 | 14.38 | 2,788 |
| (e) confirm after 1 hit | 30.89 / 17.29 | 1,645 | 1,612 | 8,966 | 20.37 | 2,647 |

* (a) is the whole story: 4,884 coasting object-frames disappear and AP rises
  8.5 points, past the detector alone (45.95), because tracks also remove
  unconfirmed single-frame detections. The price is more misses (3,761 against
  2,788): an object whose detection drops for a frame leaves the world. Whether
  that persistence is wanted is not measured by this AP.
* (b) as asked (keep the older) barely helps (+0.4) because the older track is
  often the stale coasting one; keeping the more recently updated track (b2,
  my addition) helps more (+1.0), but both leave coasting tracks in the world.
* (c) changes nothing, as the note above explains. (d) hurts, as does (e).

## 3. Rule applied (committed before the runs)

Adopted (at least +0.5 AP@0.5 without sharing): (a) and (b2). Stage S1, with
sharing, alignment off:

| tracker | AP@0.5 / AP@0.7 | duplicates | split label-frames | false objects from sharing per car-frame | gained per car-frame | missed |
|---|---|---|---|---|---|---|
| committed | 50.23 / 29.80 | 1,844 | 1,837 | 2.89 | 1.46 | 78 |
| (a) | 64.63 / 40.18 | 82 | 78 | 0.85 | 1.40 | 422 |
| (b2) | 55.76 / 33.63 | 435 | 356 | 2.63 | 1.40 | 309 |
| (a) + (b2) | **64.98 / 40.36** | 73 | 69 | 0.83 | 1.38 | 473 |

Best AP@0.5: (a) + (b2), 0.35 points above (a) alone (more than the 0.1
tie margin). Stage S2, publish threshold on the track score (running mean of
detection scores; detector threshold is 0.2):

| (a) + (b2) with threshold | AP@0.5 / AP@0.7 | duplicates | split label-frames | false from sharing per car-frame | gained per car-frame | missed | objects per car-frame |
|---|---|---|---|---|---|---|---|
| 0.2 (no change, check) | 64.98 / 40.36 | 73 | 69 | 0.83 | 1.38 | 473 | 13.71 |
| **0.3** | **67.17 / 42.06** | 51 | 52 | **0.44** | 1.36 | 669 | 12.54 |
| 0.4 | 60.97 / 42.13 | 20 | 43 | 0.08 | 1.00 | 2,762 | 10.10 |

**The rule selects `--fresh-only --overlap-iou 0.3 --overlap-keep recent
--pub-min-score 0.3`** (confirmation 3 hits, mean score, filtered box). On val
this gives, with sharing and alignment off, 67.17 / 42.06 against the
committed tracker's 50.23 / 29.80, Late Fusion 52.01 / 28.80 and No Fusion
45.95 / 25.73, and 0.44 false objects from sharing per car-frame against 2.89,
while objects gained through sharing stay about the same (1.36 per
car-frame against 1.46). Without sharing the same settings
score 52.03 / 33.38 (its passthrough, raw boxes: 51.01 / 29.52).

## 4. Alignment gauge

Alignment on, shared world unchanged, each car's view shifted back by that
car's own estimated translation:

| tracker | alignment off | alignment on | on, shifted back |
|---|---|---|---|
| committed | 50.23 / 29.80 | 39.58 / 9.13 | 39.00 / 20.12 |
| selected | 67.17 / 42.06 | 56.04 / 15.96 | 53.57 / 29.80 |

The shift back recovers about half of the AP@0.7 lost to alignment (9.1 to
20.1 of 20.7 points lost; 16.0 to 29.8 of 26.1) and nothing of the AP@0.5
loss, which gets slightly worse. So the frame offset is part of the loss, not
all of it. What else alignment changes (which reports fuse, car 1's reports
shifted into car 0's world) is not separated here.

## Not proven

* The selected tracker is chosen on val among 4 + 3 options, scored on the
  same frames, with one scenario dominating the duplicates. It is not shown to
  hold on test; the threshold 0.3 in particular is a val choice.
* The gain is largely from showing fewer boxes. Fresh-only publishing drops
  tracks with no detection this frame, including objects only briefly missed
  (misses with sharing 78 -> 669 of about 16,000 expected object-frames). The
  dataset AP cannot value keeping them.
* Overlap pruning uses IoU 0.3, fixed in advance, not tuned. Its effect on
  top of fresh-only is 0.35 AP@0.5.
* Why detections of the same object restart tracks (pose steps in one scenario,
  fast oncoming objects) is not addressed by these settings.
* Nothing here used test. A second test run with these settings would be
  labeled separately from the first.

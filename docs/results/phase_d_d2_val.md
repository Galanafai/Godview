# D2 steps 2 and 3: V2X-Real val, labels plus noise, first contact

2026-11-10. Four-agent converter and replay on V2X-Real val (6 scenarios, 717 frames,
two vehicles and two roadside units, 3 agents in one scenario), detections = labels plus
noise (no detector). Variants, scoring rules and expectations were committed before any run
(`docs/PHASE_D_HANDOFF.md`, "D2 steps 2 and 3", commit `6f67e5c`); the runs are from the
tree at `v2x_val.sh`'s commit; nothing was chosen from the results. CPU only: conversion
about 40 minutes (point-in-box tests), the 36 replays 1.5 minutes, the evaluation 2
minutes; no GPU. Raw output: `docs/results/phase_d/d2/val_eval.json`. Scripts:
`convert_v2xreal.py`, `v2x_val.sh`, `v2x_eval.py`, `v2x_diag.py`; replay binary `v2x-replay`.

## Result: all three expectations fail

Class-pooled dataset-style AP@0.5 (percent), all scenarios, ego views, boxes within 70 m,
the agent's own box excluded, truth the shared map-frame labels (the shared world scored
the same way for reference):

| | vehicle views | roadside-unit views |
|---|---|---|
| self off, alignment off | 33.0 | 23.7 |
| self off, filtered | 31.7 | 21.1 |
| self off, filtered, units as anchors | 21.7 | 20.7 |
| self on, alignment off | 33.0 | 23.9 |
| self on, filtered | 30.9 | 21.2 |
| self on, filtered, units as anchors | 21.2 | 20.8 |
| shared world, self off / on (reference) | 32.0 / 32.2 | 23.7 / 23.9 |

| expectation | comparison | result |
|---|---|---|
| **E1** filtered loses no AP against off (tolerance 0.5) | vehicle views, self off: 31.7 against 33.0 (-1.3); self on: 30.9 against 33.0 (-2.1); unit views, self off: 21.1 against 23.7 (-2.6); self on: 21.2 against 23.9 (-2.7) | **fails (4 of 4)** |
| **E2** self reports reduce wrong merges involving agents | 474 with self reports against 10 without, for each of the three alignment settings | **fails (3 of 3, in the wrong direction)** |
| **E3** anchoring at least as good as plain filtering (vehicle views) | self off: 21.7 against 31.7 (-10.0); self on: 21.2 against 30.9 (-9.7) | **fails (2 of 2)** |

Without `2023-03-17-16-03-02_11_1` the picture is the same (vehicle views 28.8 / 27.5 /
21.8 for self off; wrong merges involving agents 10 against 474).

## More detail

* **Per class (vehicle views, AP@0.5, self off; alignment off / filtered / anchored):**
  vehicle class 84.9 / 84.1 / 74.5, truck class 82.7 / 82.5 / 78.2, pedestrian class 13.2 /
  12.0 / 5.3. The class-pooled number is dominated by pedestrians (59% of labels), whose
  boxes are about 0.6 m wide against a 0.3 m detection noise, so IoU 0.5 is rarely reached;
  the vehicle and truck classes are barely touched by the filter. With constant detection
  scores (0.9) the standard AP coincides with the dataset-style AP, so it adds nothing here.
* **Wrong merges (objects with members attributed to two or more labels), all ego views:**
  13,744 to 16,855 depending on the variant, mostly crowds of pedestrians; they rise with the
  filter (13,744 to 14,331) and with self reports plus the filter (16,855). Per scenario
  and per agent: `val_eval.json`.
* **Offset estimates:** the filter has an estimate in 97.6% (self off) and 98.1% (self on) of
  vehicle-view frames that share an object (1,094 and 1,100 of 1,121).
* **Per scenario (vehicle views, self off; off / filtered / anchored):** `11_1` 57.6 / 57.6 /
  21.9; `1_1` 30.1 / 28.9 / 22.4; `45_0` 27.4 / 27.3 / 26.1; `15_0` 39.9 / 39.3 / 32.9; `33_0`
  21.3 / 19.9 / 14.6; `6_1` 57.4 / 44.5 / 22.4. The scenario with no vehicle-to-vehicle
  overlap (`11_1`) is unaffected by the filter and strongly by anchoring.

## Diagnosis (measured where stated, the rest is my reading)

* **E1 and E3: there is no pose error to correct, so every correction is noise.** The labels
  are in the exact shared frame and the detections are labels plus 0.3 m noise, so the true
  offset between any two agents is zero. The filter's translations (the dumps' alignment)
  have a median of 0.23 m (90th percentile 0.88 m) for vehicle views and 0.17 m (0.63 m) for
  unit views: this is estimation noise applied to the other agents' reports, which costs the
  most where objects are smallest (pedestrians). Anchoring moves the vehicle's own frame, and
  with it all its own (accurate) objects, by the estimated offset to a unit (median 0.15 m,
  90th percentile 0.50 m), which is why it is worse than ego-fixed filtering for vehicle views
  (pedestrian class 13.2 to 5.3, vehicle class 84.9 to 74.5) and equal to it, by construction, for
  unit views. The expectations were written as if pose error were present; this input has none.
* **E2: the self object absorbs neighbouring vehicles.** All 474 agent-involving wrong merges
  with self reports are self objects (identity = the agent) that also hold the label of
  another **vehicle-class** object (measured); without self reports only 10 merges involve an
  agent body. The refinements (same-source exactness, competition) are on; they were
  validated on V2V4Real val, where they cut the neighbour assignments from 18 to 20 percent
  to 1.4 to 2.5 percent. Here the vehicle pose sigma is 1.51 m per axis and traffic is dense
  around the vehicles, so the gate still reaches neighbours. That the competition rule does
  not reject them (an agent that does not itself detect the neighbour, for example one hidden
  from it but seen by a unit, gives no rival) is a hypothesis, not tested.

## What this does and does not show

* Shown: on labels plus noise in the exact map frame, with the committed settings, filtered
  alignment, anchoring and self reports are each a cost, not a gain, by the committed
  metric; the self-report cost is in wrong merges, the alignment cost in AP.
* Not shown: any effect on real detections with real pose error (D3). Labels plus noise carries
  no pose error by construction, so it can only measure the cost of the machinery; whether
  V2X-Real's real poses are that good is not known (vehicle pose sigma 1.51 m was an upper bound
  from label disagreement; the units' 0.1 m is an assumption).
* Not shown: whether the failures are tunable (the filter parameters are V2V4Real val's,
  unchanged; the self gate is 2 sigma). No setting was changed after seeing these results.
* The three expectations failed as committed. Per the rule, I stop here, at the end of D2.


## Record and decisions (2026-11-10, after review)

**D2 as it stands.** E1 to E3 failed because the input had no pose error: the labels are in the
exact shared frame, so every correction the filter or the anchor made was noise. The filter's cost
with no offset to correct is its translation noise, about 0.2 m at the median (0.23 m for
vehicle views, 0.17 m for unit views, 90th percentile up to 0.9 m), which costs AP where objects are
small. **Anchoring moved the vehicle's own objects** (its own frame is corrected into the unit's
by the estimated offset), which contradicts the ego view's principle that the agent's own objects
are placed exactly; that is why it is 10 AP points worse than ego-fixed filtering for vehicle views.
The self-report absorption (474 agent-involving wrong merges against 10) is a separate defect, with
a bounded fix in the next section of `docs/PHASE_D_HANDOFF.md`.

**Decisions.** No injected pose error: V2X-Real's map-frame poses agree to about 0.3 m (the labels'
cross-agent disagreement), so there is little offset to correct, and adding synthetic error would turn
the data back into a simulation. **Alignment and anchoring stay off for V2X-Real from here on** (the
code and the V2V4Real results remain as they are). Wrong-merge accounting and AP for D3 use the
same scoring rules, per class as well as pooled.

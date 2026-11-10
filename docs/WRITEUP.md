# GodView: what it claims, what was shown, where it fails

GodView lets robots and vehicles share what they see. Each agent publishes its
own tracks as signed reports that carry their own uncertainty, and every agent
fuses the reports it receives into the same picture of the world. This page is
the short version of the evidence. The specification is
[PERCEPTION_SHARING.md](PERCEPTION_SHARING.md); the real-data numbers are in
[results/v2v4real.md](results/v2v4real.md).

## What GodView claims

1. **Frames.** Each agent tracks in its own odometry frame and publishes in a
   shared Earth frame (ECEF), with its localization and clock uncertainty
   added to every report's covariance.
2. **No double counting.** Only an agent's own tracks are shared, no fused
   estimate is fed back, and fusion uses covariance intersection, which stays
   consistent under unknown correlation. Relayed or echoed reports cannot make
   the picture look more certain than it is.
3. **Agreement.** The shared world is a pure function of the received
   reports. Agents that hold the same reports compute a bit-identical world,
   whatever order the reports arrived in, including object IDs.
4. **Faults and adversaries.** Loss, duplication, reordering, corruption,
   partitions, restarts and late measurements do not break the above. Every
   batch is Ed25519-signed; forgeries and replays are rejected; a source whose
   reports disagree with its peers is inflated or excluded.
5. **Boxes and IDs.** Objects can carry size and heading, so a car beside a
   truck stays two objects. An object keeps its ID as it passes from one
   agent's sensors to the next.
6. **Honest uncertainty.** Errors that repeat on every frame (a detector that
   sees only part of an object, an annotator who always gets a size wrong, a
   drifting GPS offset) do not average out, so they are declared or estimated,
   not ignored.

## What simulation showed

The deterministic simulator (`godview_sim`) has 20 scenarios, each checking
one stated invariant under a fault or adversary. Every run is reproducible
from its seed. The scenarios that matter most for the story:

| Question | Result |
|:--|:--|
| Do agents agree? | Bit-identical worlds once they hold the same reports: checked in nearly every scenario on every seed run, and across four real node processes over Zenoh with 20% loss and 20% duplication. |
| Do relays and echoes double count? | No: with five agents relaying every batch, each agent's world equals the one computed from each source's own reports exactly, with no uncertainty reduction from echoes. |
| Does a truck seen from both ends stay one object? | Only with the partial-view error declared: without it, duplicated in 15 of 15 seeds; with it, none. |
| Do IDs survive handoffs? | With IDs carried in reports: 0 ID changes across 40 seeds of objects driving past four agents, against about 41 per run when the oldest track holds the ID and about 45 with the original smallest-key rule. No ID was ever given to two objects. |
| Does a drifting GPS offset break fusion? | With relative alignment: all 15 seeds pass, the estimated offsets follow the injected drift to within 0.4 to 0.8 m, and fused error is about 1.0 to 1.2 m against 1.8 to 2.1 m without it. |

The limit of all of this: the simulator was written alongside the code, so it
tests the code against its author's model of the world. Every scenario
includes an ablation that must fail without the mechanism under test, which
shows the scenario exercises it, but not that the model is right. That is
what the real data is for.

## What real data showed

[V2V4Real](https://mobility-lab.seas.ucla.edu/?p=2004) is a recording of two
real cars driving together, with lidar, GPS/INS poses and labeled 3D boxes.
Each car became one GodView agent, and its detections were the dataset's
labels for what its own lidar could see, with added noise.

**First contact (test1, 3 scenes)** found three things the simulator had not:

- The two cars' positions for the same object disagreed by 0.5 to 4 m, a
  slowly drifting offset shared by every object, not noise.
- The human labelers of the two cars gave the same vehicle sizes up to about
  1 m apart. Each track's size estimate grows more certain every frame, so two
  converged tracks of one vehicle stopped matching. This became the declared
  systematic size and heading error.
- Telling GodView the cars knew their position to 1 cm (as RTK would) split
  nearly every shared object in two (994 extra estimates), which is what the
  design predicts when uncertainty is understated.

**Held-out test (test2 and test3, 6 scenes, 1,588 frames).** The settings were
chosen on test1, frozen and committed before test2 and test3 were downloaded,
and run once:

| | Held-out | test1, same settings |
|:--|:--|:--|
| Vehicles both cars detected, fused into one | 93.6% (24,794 of 26,490) | 89.8% |
| Expected vehicles missed | 0.8% (502 of 62,601) | |
| Object-frames a car saw only through sharing | 23% | 22% |
| ID changes per object | 1.42 | 1.24 |
| Position NEES, objects both cars labeled (3 expected) | 0.25 | 0.82 |

In plain terms: on real recordings of two cars, sharing gives each car about a
quarter more of what is around it, and the shared picture rarely misses a
vehicle or shows one twice.

**With relative alignment.** The frozen settings plus alignment, run once on
test2 and test3. The first run hit a bug (below); this is the run after the
fix. Details, including the first run, are in the results; the original
table above is unchanged.

| | With alignment, after the fix | Original held-out |
|:--|:--|:--|
| Vehicles both cars detected, fused into one | 93.3% | 93.6% |
| Expected vehicles missed | 0.8% | 0.8% |
| Object-frames a car saw only through sharing | 22.8% | 22.7% |
| ID changes per object | 1.56 | 1.42 |
| Position NEES, objects both cars labeled | 0.24 | 0.25 |

In plain terms: on two cars, alignment does not improve the result. It
estimated an offset in 65% of car-frames (median 0.40 m) and helped in some
scenes (ID changes in one fell from 1.15 to 0.56 per object), but raised ID
changes in another (0.50 to 1.71). A smaller pose uncertainty, chosen on the
validation split by a rule committed before any validation replay, did not
qualify (the best candidate merged 6 points fewer objects), so that run was
not made. One likely reason it cannot help more with two agents: when only
two cars disagree, there is no way to tell which one is wrong, so alignment
can only split the difference. A third agent, or a roadside unit with a
surveyed position, would give it something to anchor to; that is not tested.
test1 cannot be used to choose alignment settings at all: no frame has the 3
objects both cars see that alignment needs.

### With a real detector (V2V4Real)

The labels-plus-noise detections above were replaced by the output of the
dataset's published PointPillars detector, run per car. The environment
reproduced the published scores first (No Fusion AP@0.5 39.81, Late Fusion
55.07, against 39.8 and 55.0). The detector's error model was measured on the
validation split, settings were committed before any test run, and every
method was scored by the dataset's own evaluation and by a standard
score-ranked AP, because the dataset's AP rewards publishing fewer boxes.

- **First test run (committed settings): GodView lost.** Its own tracker scored
  below the raw detector it was fed (32.5 against 39.9 AP@0.5), because tracks
  that had lost their detection kept being published and one vehicle was
  sometimes tracked twice.
- **Second, separately labeled test run** with tracker settings chosen on
  validation (publish only tracks updated this frame, prune overlapping tracks,
  a score threshold of 0.3), against baselines at the same threshold:

| AP@0.5 / AP@0.7 | dataset AP | standard AP |
|:--|:--|:--|
| One car alone (No Fusion) | 45.6 / 27.9 | 53.6 / 36.9 |
| Late Fusion (published method) | 65.5 / 33.9 | 74.7 / 42.1 |
| **GodView, sharing** | **63.0 / 34.8** | **72.5 / 42.1** |

In plain terms: sharing adds about 16 to 18 AP@0.5 points over one car, and
GodView roughly matches the published late-fusion baseline without beating it,
while also keeping track IDs, uncertainty and signed shared state that late
fusion does not. It is ahead at close range and behind beyond 30 m, where its
boxes are often about 0.8 m off on slow or parked vehicles (cause not found).
An apparent 8-point lead over the published baseline came from the
threshold, and disappears when both are scored the same way. Details:
[docs/results/phase_d_d1_test2.md](results/phase_d_d1_test2.md).

### The demo (V2X-Real)

[V2X-Real](https://mobility-lab.seas.ucla.edu/v2x-real) has two cars and two
roadside units with lidar and cameras. The demo scene was chosen by a rule
committed before searching (the longest stretch where an object is hidden from
a car's camera, seen by another agent, then becomes visible). Its detections
are the dataset's annotations with added noise, not a detector; on this
input, the GPS offset filter and roadside-unit anchoring did not help (the
input has no pose error to correct), and self reports, after a fix, still
caused 21 agent-related wrong merges on validation against 10 without them. Details: [docs/results/phase_d_d2_val.md](results/phase_d_d2_val.md)
and [docs/results/phase_d_d4_scene.md](results/phase_d_d4_scene.md).

## Where it fails, and what is not proven

- **A real detector on two cars only.** The real-detector results are on
  V2V4Real; the four-agent V2X-Real demo uses annotations with added noise.
- **Two cars only.** V2V4Real has two vehicles; scale (many agents, many
  objects) is not benchmarked.
- **The truth is the labels.** There is no ground truth for the poses; truth
  is the mean of the two cars' labels.
- **Too cautious on held-out data.** Position NEES of 0.25 means GodView says
  it is less sure than it is: one fixed pose uncertainty cannot fit offsets
  that vary about 4x between drives. Alignment was the attempt to fix this;
  with two cars it did not (above).
- **Alignment and the label truth.** With two cars, alignment moves each car
  to the midpoint, which is also where the truth (the mean of the two cars'
  labels) sits. That flatters objects both cars labeled, and moved objects
  only one car labeled further from their truth (NEES 0.11 to 0.24 after the fix).
- **Labels that flip front and back** between the two cars split those
  objects at the heading check; a real detector would mark such headings as
  uncertain instead.
- **ID changes on real data** (1.42 per object) come mostly from objects
  flickering between merged and split, not from the ID rule.
- **Alignment estimates translation only**, not a heading offset between
  agents.
- **The trust monitor has no "no consensus" rule.** When every agent drifts
  beyond its declared uncertainty, it excludes all of them, its own reports
  included ([issue #5](https://github.com/Galanafai/Godview/issues/5)).
  With alignment enabled it judges agents after alignment, which avoids this
  in simulation.
- **No real radio link**, no Biscuit capability checks per batch, and
  revocations do not survive a restart.

## Bugs the tests found

- **A young track fixed a heading backwards.** A track 0.15 s old reported
  10.3 ± 2.6 m/s for a vehicle moving at 1.6 m/s, decided front from back from
  that, and kept the decision; fusion then spread it to every agent. Found by
  the 100-seed CI sweep, fixed by letting only tracks at least 1 s old decide,
  and by leaving a heading ambiguous when two agents confidently disagree.
- **The trust monitor excluding everyone** (above), found while building
  alignment.
- **Alignment moving a car by up to about 1e15 m.** Found on real data: in
  51 of 2,070 aligned held-out car-frames the estimated shift exceeded
  8.5 m. With exactly three matched objects the offset covariance is
  singular, and its floor was applied only to the diagonal, so the inverse
  was huge and the solve amplified rounding. Fixed by adding the floor in
  every direction and using a relative tolerance in the solve; a test now
  checks 2,000 three-object cases (it reproduced a shift of about 2e14 m
  before the fix). After the fix the held-out run had 0 such shifts, and
  misses returned from 1.8% to 0.8%.
- **Three rare simulator failures** that also fail on code before this work
  are open with reproduction commands:
  [#2](https://github.com/Galanafai/Godview/issues/2),
  [#3](https://github.com/Galanafai/Godview/issues/3),
  [#4](https://github.com/Galanafai/Godview/issues/4).

## Reproduce

```bash
cargo test --workspace                                         # all tests
cargo run --release -p godview_sim -- --scenario all --seed 1 --seeds 3
cargo run --release -p godview_sim -- --scenario pose_drift --seed 1
```

The V2V4Real replay needs the dataset; see
[results/v2v4real.md](results/v2v4real.md) and `scripts/v2v4real/`.

# Perception Sharing and Spatial Consistency

This document describes how GodView agents share what they perceive and how
each agent builds a shared world model from it. It is the contract referred to
by the module documentation in `godview_core` (`frames`, `report`,
`world_model`) and the specification the DST catalog in `godview_sim` tests.

## 1. Layers

| Layer | Module | Frame | Shared? |
|-------|--------|-------|---------|
| L0 localization | `localization` | odometry to ECEF pose with 6x6 covariance | no |
| L1 observations | `local_tracker::Detection` | sensor / odometry | no |
| L2 local tracks | `local_tracker`, `filter` | odometry | as reports |
| Wire | `report` (`TrackReport`, `ReportBatch`), `topics` | ECEF (or source local, flagged) | yes, signed, per H3 topic cell |
| Store | `report_store` | as received | per agent |
| L3 world model | `world_model`, `consistency` | ECEF, ENU for consumers | derived, never sent |

Data flows one way: detections become local tracks, local tracks become
reports, reports are stored, and the world model is a pure function of the
store, the query time and the agent's trust view. No estimate from L3 is fed
back into a local track or a report. This is what prevents double counting: a
source's information enters any fused estimate at most once, through that
source's own latest report, however many times it was relayed or echoed. The
one thing that does flow back is a label: a report may carry the ID its track
belongs to in its source's world (section 6, step 5). Names never enter
gating, clustering or fusion, and a unit test checks that they change no
estimate or covariance.

## 2. Frames and uncertainty

* Every agent tracks in its own odometry frame (`Pose3`), which is smooth but
  drifts relative to the world.
* L0 supplies `PoseWithCov`: the ECEF from odometry transform with a 6x6
  covariance in the left tangent space (rotation, translation).
* `PoseWithCov::transform_state` maps a 6-D position/velocity state and its
  covariance to ECEF and adds the pose contribution through the Jacobian
  `J_pose = [[I, -[R p]x], [0, -[R v]x]]`. A Monte Carlo test checks the
  linearization (Frobenius error under 5%).
* `add_clock_uncertainty` adds timing error as velocity-scaled position
  variance (`clock_sigma`, default 2 ms).
* `Geodetic::new` validates its ranges, so meters cannot be passed as degrees
  (an audit finding in the old spatial API).
* Process noise for prediction is defined in local ENU (horizontal and
  vertical spectral densities differ) and rotated into ECEF.

A report therefore carries the publisher's tracking, localization and timing
uncertainty. `localization_uncertainty` checks that this covariance is
consistent (pooled NEES within a 4 sigma band around 3 over 60 runs).

An agent without a global pose (`LocalizationStatus::LocalOnly`) publishes
reports flagged `FrameRef::Local`. Receivers store them but the world model
skips them (`skipped_local_frame`) instead of placing them at a wrong position.

## 3. Local tracking (L2)

`LocalTracker` runs one retrodicting constant velocity Kalman filter per
object in the odometry frame.

* Out-of-order detections inside the retrodiction window (0.5 s) are inserted
  in time order and the window is replayed; older ones are rejected as
  `TooOld`, never applied at the wrong time.
* Association uses Hungarian assignment on a likelihood cost
  `NIS + ln det S`. A pure NIS cost favours uncertain new tracks and caused
  duplicate tracks; a regression test fails on the old cost.
* A detection that gates with an existing track at the 99.99% level does not
  spawn a new track, and two tracks that agree in position and velocity are
  merged (the younger is terminated with a tombstone).
* A tentative track that is statistically indistinguishable (position and
  velocity gates) from a published track is discarded before it can confirm.
  Without this, tail detections outside the established gate could feed a
  split track to three hits (found by DST, `slow_loris` seed 33).
* Association runs in two passes. The first assigns at the 99% gate. The
  second pairs published tracks and detections left over from the first at
  the spawn-suppression gate, so a detection between the two gates is not
  silently dropped. Without it a young track with a poor velocity estimate
  could starve while its object went untracked (found by DST,
  `object_expiry`).
* A tentative track not re-observed within 0.5 s is dropped, and it never
  gates a detection across a longer gap. Its velocity is known only from a
  few close observations, so left alone its gate grows until it can capture
  another object's detection and confirm with a confident wrong velocity
  (found by DST, `time_tornado`).
* Lifecycle: `Tentative` until 3 hits, `Confirmed`, `Coasting` after 0.3 s
  without data, `Terminated` after 1.5 s.

## 3a. Object extent (oriented boxes)

Detections may carry an oriented box (`extent::ObjectBox`): length, width and
height with variances, and a heading with a variance. Many detectors cannot
tell front from back, so a heading can be marked ambiguous (known modulo π).
Points without a box remain the base case (radar tracks, aircraft, anything a
sensor reports without a size), and every rule below applies only when both
sides have a box.

* **Tracking.** A track refines its size per axis from every box (with a
  small process noise so it stays adaptive) and its heading with a filter
  that wraps angles, modulo π when ambiguous. Once the track's velocity is
  well determined and above 1 m/s, the direction of travel picks front from
  back. The velocity is not fused as a heading measurement, since objects can
  move sideways.
* **Gating.** Association, spawn suppression and duplicate pruning in the
  local tracker, and pair and cluster gating in the world model, also
  require the sizes (3 DOF, 99.99%) and headings (1 DOF, 99.99%) to agree
  within their uncertainty. A car beside a truck is not merged even when
  poor localization makes their positions indistinguishable (unit test
  `box_gate_keeps_a_car_and_a_truck_apart_when_position_cannot`).
* **Shared frame.** A report's heading is counterclockwise from East at the
  object, converted with the agent's pose; the pose's heading uncertainty is
  added to the box's.
* **Fusion.** Sizes are fused by covariance intersection with a
  trace-minimizing weight; for the scalar heading CI keeps the more certain
  input, flipped by an unambiguous one if needed.
* **Partial views.** A detector that sees only part of an object places
  the center toward the visible side, on every scan. That error does not
  average out, so a filter that treats it as noise becomes confidently wrong,
  and two agents viewing a truck from opposite ends then report it twice. A
  detector's systematic center error is declared as `box_center_sys_frac` (a
  standard deviation in fractions of the half-length and half-width) and
  added to published covariances along the box's axes. In the
  `large_objects` scenario, every seed shows the truck or bus duplicated
  without it and none with it.

The world object, the consumer `Entity` (`oriented_box`), the node's JSON
report and the node's stdin input all carry boxes.

## 4. Reports on the wire

`ReportBatch` is a manually encoded, versioned, signed binary format
(`"GVR"`, schema 3; 50 byte header, 169 bytes per ECEF report with neither a
name nor a box, plus 40 for a name and 33 for a box, 177 per local report
without them, 64 byte Ed25519 signature). Schemas 1 (no box) and 2 (no birth
time, no name) are still decoded; their reports are taken as born at
`t_valid`, without a name. Every byte is covered by the
signature; a test flips each byte in turn and requires rejection.

* `AgentId` is the source's Ed25519 verifying key, so a batch can only be
  signed by the source it names.
* A report is identified by `(source, local track id)` and versioned by
  `(epoch, seq)`. The epoch increases on restart, so a restarted agent that
  reuses track ids replaces its old reports instead of colliding with them.
* `born` is when the source created the track (its own clock). It is
  constant over the track's life, must not be later than `t_valid`, and is
  only used to name world objects (section 6).
* `name` is optional: the ID of the world object the track belonged to in
  the source's latest world, sent only when that ID is another track's key.
  It is a label only (section 6).
* Covariance is sent as the upper triangle in f32, quantized conservatively
  and validated (finite, positive definite, bounded) on receipt.
* Termination is sent as a `Terminated` tombstone, republished for the
  validity period (soft state), so a lost tombstone is repaired by the next
  publish cycle.

## 5. Report store

`ReportStore` is a last-writer-wins map keyed by `(source, local track id)`
with a per-source garbage collection floor. Insert is idempotent,
commutative and associative (tested as CRDT laws), so duplicated, reordered
and relayed batches converge to the same store. Replays below the floor and
versions older than the stored one are counted and ignored. Expired entries
are collected after a grace period and raise the floor, so they cannot be
resurrected by a late replay.

## 6. World model (L3)

`WorldModel::snapshot(store, t_query, trust)` is deterministic: the same store
and query time give a bit-identical result on every agent.

1. Every live global report is predicted to `t_query`.
2. Candidate pairs are found with `SpatialIndex` (H3 partition over exact
   ECEF positions, with exact 3-D distance filtering, so pairs across cell
   edges are never lost) and gated on position and velocity (chi-square,
   3 DOF, 99.99%), and on box size and heading when both reports have boxes.
3. Clusters are built under the constraint that a cluster holds at most one
   track per source; two clusters merge only if their fused estimates also
   gate.
4. Each cluster is fused by covariance intersection, with a trace-minimizing
   weight (golden section search, ties broken to 0.5). CI is used because the
   correlation between sources (common localization infrastructure, shared
   process model) is unknown.
5. Each world object gets an ID (`IdRule::Lineage`). Every member claims
   a name: the name its report carries, or its own key if it carries none.
   The cluster takes the claim of its oldest member (the report whose track
   was created first, ties broken by the smaller key). A claim that is the
   key of a live track in another cluster is refused, because that track's
   cluster owns it. Clusters are settled in order of their oldest member; a
   cluster whose claim was refused or already taken moves to its next
   member's claim, and finally to its oldest member's own key, which no
   other cluster can take, so IDs are unique. The rule needs no memory
   between snapshots, so agents with the same reports agree on IDs (the
   snapshot digest includes them).

**What the ID rule does and does not give.** A node publishes, with each of
its tracks, the ID that track belongs to in its own latest world. When the
track that started an ID ends, the remaining members still carry the name,
so the object keeps its ID through a chain of handoffs as long as each new
source is fused with the object before the previous ones leave. In
`handoff_ids` (objects driving past four short-range agents) objects kept one
ID for the whole drive past in all 40 seeds run, and no ID was ever matched
to two physical objects. In the same runs the oldest-track rule (names
ignored) averaged about 41 ID changes per run and the smallest-key rule about
45, 24 of them when a source joined.

Limits:

* If coverage has a gap, so that no track of the object survives until the
  next source picks it up, the name is lost and the object gets a new ID.
  Recognizing it again would need re-identification, which is not built.
* A name is only as good as the association that produced it. If a source's
  track jumps from one object to another, its name can carry an ID to the
  wrong object. `handoff_ids` checks this does not happen in its geometry;
  crossing objects at close range are covered only by `crossing_targets`,
  which checks positions, not IDs.
* Names and birth times are claims by the source. A validly signed source
  can claim old tracks or names of ended tracks and so choose the IDs of the
  objects it sees (a source with a small key could do this under the old
  rule). It cannot take the ID of a live track in another object, and it
  cannot change any estimate this way.
* Names add up to 41 bytes per report.

`WorldObject::to_entity` converts to the consumer `Entity` (geodetic position,
ENU covariance and, if present, the oriented box) for `SpatialEngine`.

## 7. Trust

`ConsistencyMonitor` compares each source with its partners: for each source
it keeps the median pairwise NIS against at least three other sources within
20 m. A source whose statistic exceeds the inflate threshold has its
covariance inflated; above the exclusion threshold it is excluded, with
hysteresis on the way back. The monitor feeds a `TrustView` into the world
model, so trust changes what an agent fuses but never what it stores or
relays. `world_neutral` ignores trust and is what the convergence checks
compare.

Detection limits are documented by tests: a 6 m unreported bias is excluded,
3.5 m is inflated, 2.5 m is below the detection threshold at the default
sensor noise. The monitor needs partners, so it cannot judge a source that is
alone in its area.

Revocation (`PerceptionNode::revoke`) rejects all later batches from a key and
purges its reports.

## 8. Runtime integration

`GodViewAgent` owns a `PerceptionNode`. `tick` runs track maintenance and
`process_packet` checks envelope age against the context clock and passes the
batch to `PerceptionNode::receive`; rejections are returned as errors and
counted, not dropped silently. The legacy `TrackManager` (Highlander) path is
unchanged and still used by the CARLA agent and the legacy DST scenarios.

## 9. Running nodes over Zenoh (`godview_node`)

`godview_node` runs one `PerceptionNode` per process with real I/O. Zenoh
stays out of `godview_core`: the perception math is synchronous and owned by
one loop, and only the node's I/O is async.

**Cell topics.** `PerceptionNode::publish_by_cell` signs one batch per H3
cell at the topic resolution (default 8, about 530 m edges), and the node
publishes each on `godview/v1/reports/<res>/<cell>`. A node subscribes to
`topics::cover_cells(position, interest radius)`, every cell that can hold a
point within its radius of interest (default 200 m), and follows itself as
it moves. A property test checks the cover never misses a cell. The traffic a
node receives therefore depends on what is around it, not on fleet size: a
node 5 km away in the same Zenoh mesh receives nothing. Subscribers ignore
their own publications. Local-frame reports (no global pose) are not sent,
since no peer could fuse them.

**Loop.** At a fixed rate (default 10 Hz): drain received batches (each is
verified and validated before it touches state), ingest detections, maintain
tracks, publish per cell. A tick that falls more than a period behind is
skipped rather than run late, so lag never accumulates; the report records
lag, skipped ticks and work per tick.

**Inputs.**
* `--synthetic-index N`: agent N of a seeded synthetic scene that every
  process rebuilds identically from `--seed`, with its own sensor noise and
  localization error.
* Otherwise, JSON lines on stdin, one frame per line: detections in the
  agent's odometry frame and the placement of that frame on the Earth (like
  ROS `odom` and `map -> odom`), each with its uncertainty. This is the
  integration point for an external perception stack. Bad lines are counted
  and skipped. See `godview_node/src/input.rs` for the format.

**Faults.** `--loss`, `--duplicate`, `--delay-ms` and `--jitter-ms` apply a
seeded fault stream to outgoing batches, so the network can be made hostile
on purpose even on localhost.

**Report.** On exit a node writes JSON: traffic and rejection counts per
source, fault counts, timing, tracking quality against truth (synthetic),
and the final world with provenance of every object.

```
# Four synthetic nodes on one machine (run each in its own shell):
START=$(( $(date +%s%3N) + 3000 ))
godview-node --listen tcp/127.0.0.1:7447 --synthetic-index 0 --key-seed 1 --start-unix-ms $START
godview-node --listen tcp/127.0.0.1:7448 --connect tcp/127.0.0.1:7447 --synthetic-index 1 --key-seed 2 --start-unix-ms $START
# ... and so on; add --loss 0.2 --duplicate 0.2 --jitter-ms 60 for a hostile network.

# A node fed by an external detector:
my_detector | godview-node --connect tcp/<peer>:7447
```

**Multi-process tests** (`godview_node/tests/multi_process.rs`) start real
node processes on localhost:
* Four nodes with 20% loss, 20% duplication and 10 to 70 ms delays. Every
  node must hear every peer with nothing rejected; during the run, at most
  5% of ticks after warm-up may show a miss or a ghost, and never more than
  3 in a row; after a final fault-free publish, every node must have all
  four objects fused from all four sources, with no ghosts, and all nodes
  must compute a bit-identical world.
* A node 5 km away, connected to the same mesh, receives nothing and shares
  no topic cell.
* A node fed JSON lines on stdin, including junk lines, fuses with a
  synthetic peer that watches the same scene.

These runs are real time and therefore not bit-for-bit reproducible; they
check the end-to-end path under faults with a stated tolerance. The
deterministic, many-seed guarantees are the DST suite's. Run them in
release for realistic timing (`cargo test --release -p godview_node`).
Debug builds verify signatures about a hundred times slower, so the tests
run at 2 Hz there instead of 10 Hz. In release on a two-core machine, four
nodes with four objects each spent about 2.5 ms per tick (at most about
8 ms) and never fell behind schedule; scale has not been benchmarked.

## 10. Deterministic simulation (DST)

`godview_sim::perception` contains a deterministic harness: ground truth
objects, agents with odometry frames, per-agent localization error drawn from
the reported covariance (optionally plus unmodeled bias), sensor noise drawn
in ENU, detection delays, localization outages, a fault-injecting `SimBus`
(latency, jitter, loss, duplication, corruption, partitions, full mesh or
line topology), relaying, fabricating agents, replay and forgery, revocation
and restarts. Every run ends with a reliable synchronization round, after
which convergence is checked exactly.

Every scenario is a function of its seed. Failures print the scenario, seed,
simulated time, agent and the command that reproduces them:

```
cargo run --release -p godview_sim -- --scenario all --seed 42
cargo run --release -p godview_sim -- --scenario split_brain --seed 3
cargo run --release -p godview_sim -- --scenario legacy --seed 42
```

`all` runs the 19 perception scenarios below. The 16 scenarios that exercise
the legacy `TrackManager` path are kept under `legacy`. `--duration` applies
to legacy scenarios only; perception scenarios have fixed durations.

| Scenario | Invariant checked |
|----------|-------------------|
| `two_views` | Two agents, different odometry frames: one world object per physical object fused from both sources, sub-meter error, convergence. |
| `frame_invariance` | The same scene seen through rotated/offset odometry frames gives the same world (1 mm, same cardinality). |
| `localization_uncertainty` | Report NEES against truth is chi-square consistent over 60 runs; covariance grows with range as heading error predicts. |
| `localization_fusion` | 0.4 m / 0.5 deg localization error: reports of one object still associate. |
| `localization_dropout` | Loss of global pose: local-frame reports only, peers stop fusing and resume after recovery. |
| `flash_mob` | Objects crossing H3 cell edges and many objects: no pairs lost at edges, SpatialEngine consistent. |
| `network_faults` | 30% loss, 20% duplication, 5% corruption, reordering: corrupt batches rejected, duplicates idempotent. |
| `slow_loris` | 50% loss: soft state keeps every object present. |
| `biased_source` | Unreported 6 m bias detected and excluded by all honest agents; an ablation with trust disabled shows the ghosts. |
| `crossing_targets` | Two objects within 1.5 m are not merged and keep their identities. |
| `large_objects` | Cars, vans, trucks and buses as oriented boxes, from detectors that see only part of an object and cannot tell front from back: each is one world object from every side, with size consistent with its uncertainty (pooled NEES below 6) and heading within 15 degrees (modulo 180 unless it moves). The same run without the declared systematic center error must show duplicates. |
| `handoff_ids` | Objects drive past a row of short-range agents in both directions, so sources keep joining and leaving. Each object keeps one ID for its whole drive past (at most two changes per run tolerated) and no ID is given to two objects. Comparison runs: under the oldest-track rule every change is its holder leaving or an older track joining and each object passes through at least three holders; the smallest-key rule must rename objects on a join. |
| `object_expiry` | Disappearing objects leave every world within coast plus validity time; new ones appear. |
| `echo_chamber` | Relayed and repeated reports change nothing: world equals the one built from each source's own reports. |
| `split_brain` | Partition: each side fuses its own sources; after healing all converge. |
| `agent_restart` | Restart with a new epoch and reused track ids: old reports replaced, peers fuse it again. |
| `byzantine` | Fabricator creates ghosts until revoked; replay and forgery rejected and leave worlds identical to a control run. |
| `time_warp` | Out-of-order detections inside the retrodiction window are all fused. |
| `time_tornado` | Detections up to 5 s late are rejected, not misapplied, and never create ghosts (estimates no real object explains within their own 99.99% uncertainty); covariance covers the larger errors (pooled NEES below 6). |

"Tracks every object" in these invariants has a precise meaning
(`catalog::TRACKING_CRITERION`): at each 0.5 s sample every expected object is
matched within 5 m by exactly one world object, with error under the
scenario's bound. Gating is statistical, so an isolated sample may show a
duplicate when one report falls outside its gate for one cycle; an error
attributed to the same object at two consecutive samples fails the scenario.
Isolated errors are counted once per sample and object (a duplicate local
track at one source shows in every agent's world at once and is one event);
more than one such event, and more than 2% of expected object-samples, fails
the scenario. The same tolerance applies to checks that every object is fused
from the expected sources, since one report can fall outside its gate for one
snapshot. Checks that something must never happen (fusing across a
partition, ghosts from stale data, worlds that differ from a control run)
stay strict at every sample.

### What the DST suite establishes

A passing scenario shows that its invariant held for that seed's realization
of noise and faults. It does not prove convergence or consistency in general.
The statistical checks pool independent sub-runs and use conservative bounds
so that 100 CI seeds should not raise false alarms; they can detect gross
covariance errors, not small miscalibration. Not covered: real sensor models
(occlusion, clutter beyond fabricated objects, misclassification), map-relative
drift over long missions, more than about 40 objects per agent, the Zenoh
transport (covered by the multi-process tests in section 9), Biscuit
capabilities per batch, and colluding adversaries that coordinate consistent
false reports.

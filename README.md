# GodView

**Shared perception for robots and vehicles.** Each agent shares what it sees as signed track reports that carry their own uncertainty, and every agent fuses the reports it receives into the same picture of the world.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Rust](https://img.shields.io/badge/Rust-2021-orange.svg)](https://www.rust-lang.org/)
[![DST](https://img.shields.io/badge/DST-19%20perception%20scenarios-blue.svg)](docs/PERCEPTION_SHARING.md#10-deterministic-simulation-dst)

> **Status: research prototype.** The pipeline runs end to end, across separate processes over Zenoh, and is tested under injected network faults and adversaries in deterministic simulation. It has not yet been run on real sensor data, over a real radio network, or at fleet scale. See [What is and is not proven](#what-is-and-is-not-proven).

## In plain terms

A robot cannot see around a corner. Another robot can. GodView lets each one share what it sees so that every robot knows what the others know, keeps that picture consistent when the network drops or reorders messages, and stops trusting a robot whose reports do not hold up.

Three things make this hard, and GodView handles each one:

| Problem | What goes wrong | What GodView does |
|:--|:--|:--|
| **Different frames** | Each robot measures in its own drifting coordinate frame, and its own position is uncertain. | Reports are converted to a shared Earth frame (ECEF) with the robot's localization and clock uncertainty folded into each report's covariance. |
| **Double counting** | Relayed and echoed reports make the fused picture look more certain than it is. | Only each agent's own tracks are ever shared, no fused estimate is ever fed back (only object IDs travel back, as labels), and fusion uses covariance intersection, which is safe under unknown correlation. |
| **Different sizes** | A truck seen from opposite ends looks like two objects, and a car beside a truck can look like one. | Detections can carry oriented boxes. Sizes and headings must agree for reports to fuse, and a detector's systematic errors (a center shifted by seeing only part of an object, a size or heading it always gets wrong) are published as uncertainty. |
| **Untrusted data** | A robot can be miscalibrated, compromised or impersonated. | Every batch is Ed25519-signed and validated; replays and forgeries are rejected; a consistency monitor inflates or excludes sources that disagree with their peers. |

## How it works

```
 sensors ─▶ local tracker ─▶ signed reports ─▶ Zenoh topic per H3 cell ─▶ peers
            (own frame)       (Earth frame,                                  │
                               with covariance)                              ▼
                                                     report store (merge-safe, last writer wins)
                                                                             │
                                                                             ▼
                                         shared world model (gate, cluster, fuse; deterministic)
```

1. **Local tracks.** Each agent tracks objects in its own odometry frame with a Kalman filter that accepts late, out-of-order detections inside a short window and rejects older ones rather than applying them at the wrong time.
2. **Signed reports.** Each track is published in ECEF with a full covariance that includes the agent's pose and clock uncertainty, in a versioned binary format with an Ed25519 signature over every byte.
3. **Cell topics.** Reports travel on one Zenoh topic per H3 cell. A node subscribes only to the cells around itself, so what it receives depends on what is near it, not on the size of the fleet.
4. **Report store.** Received reports go into a last-writer-wins store keyed by source and track. Duplicated, reordered and relayed messages all converge to the same store.
5. **World model.** Each agent computes the shared world from its store: predict to a common time, find candidates with an H3 index, gate on position and velocity, cluster with at most one track per source, and fuse by covariance intersection. Each report also carries the ID its track belongs to, so an object keeps its ID as it is handed from one agent's sensors to the next. The same store always gives a bit-identical world, IDs included.

The full contract, frames, formats and parameters are in [docs/PERCEPTION_SHARING.md](docs/PERCEPTION_SHARING.md).

## Quick start

Requires a stable Rust toolchain.

```bash
# All tests (unit, integration, multi-process). Use --release for realistic timing.
cargo test --workspace

# Deterministic simulation: all 19 perception scenarios for one seed, or many seeds
cargo run --release -p godview_sim -- --scenario all --seed 42
cargo run --release -p godview_sim -- --scenario all --seed 1 --seeds 100

# Reproduce one scenario and seed (failures print this command)
cargo run --release -p godview_sim -- --scenario split_brain --seed 3
```

### Run nodes over Zenoh

```bash
cargo build --release -p godview_node
START=$(( $(date +%s%3N) + 3000 ))   # common start time for all nodes

# Two agents watching the same synthetic scene, the second over a lossy link
target/release/godview-node --listen tcp/127.0.0.1:7447 \
  --synthetic-index 0 --ring-agents 2 --key-seed 1 --start-unix-ms $START &
target/release/godview-node --listen tcp/127.0.0.1:7448 --connect tcp/127.0.0.1:7447 \
  --synthetic-index 1 --ring-agents 2 --key-seed 2 --start-unix-ms $START \
  --loss 0.2 --jitter-ms 60
```

Each node prints a JSON report when it finishes: traffic, rejections, faults, timing, tracking quality against the synthetic truth, and the final world with the provenance of every object.

To feed a node from your own perception stack, pipe JSON lines into its stdin, one frame per line: detections in the agent's odometry frame and the placement of that frame on the Earth, each with its uncertainty (like ROS `odom` and `map -> odom`). The format is documented in [`godview_node/src/input.rs`](godview_node/src/input.rs).

```bash
my_detector | target/release/godview-node --connect tcp/<peer>:7447
```

## What is and is not proven

**Tested, with the scope stated in each test:**

- **Deterministic simulation (`godview_sim`).** 19 scenarios, each checking one stated invariant: different coordinate frames, localization error and dropout, H3 cell edges, loss, duplication, corruption and reordering, relays and echoes, partitions, restarts, a biased source, a fabricating source, replay and forgery, late measurements, large objects reported as boxes by detectors that see only part of them, and objects handed between agents. Every run is reproducible from its seed. The 17 earlier scenarios passed for each of 260 seeds checked (4,420 runs) before boxes and IDs changed; since then, all 19 passed for the 9 seeds run so far (171 runs), and the handoff scenario for 40 seeds. The runs check that each invariant held for those seeds; they are not a proof that it holds in general.
- **Multi-process tests (`godview_node`).** Real node processes over Zenoh on localhost:
  - Four nodes with 20% loss, 20% duplication and 10 to 70 ms delays track every object, and they compute a bit-identical world after a final fault-free round.
  - A node 5 km away in the same mesh receives no traffic.
  - A node fed JSON lines on stdin fuses with a synthetic peer.
- **Unit and property tests** for frames and uncertainty propagation (including a Monte Carlo check), the filter, assignment, the wire format (every byte flip rejected), the store's merge laws, the spatial index and cell cover against brute force, and the world model.

**Measured:** in release on a two-core machine, four nodes with four objects each spent about 2.5 ms per 10 Hz tick (at most about 8 ms).

**Not yet:**

- real sensor data or a real detector;
- a real radio network;
- many agents or many objects (scale is not benchmarked);
- boxes from a real detector (simulated detectors only so far);
- re-identification: an object that drops out of every sensor's view and reappears gets a new ID;
- Biscuit capability checks per batch;
- revocations that survive a restart.

## Repository layout

| Path | What it is |
|:--|:--|
| [`godview_core/`](godview_core) | The library. Perception sharing: `frames`, `filter`, `local_tracker`, `report`, `report_store`, `topics`, `spatial_index`, `world_model`, `consistency`, `perception_node`. Also the earlier engines (see below). |
| [`godview_env/`](godview_env) | Environment abstraction (time, randomness, keys, network) so the same code runs in production and in simulation. |
| [`godview_node/`](godview_node) | The `godview-node` binary: Zenoh transport, cell topics, synthetic and stdin inputs, fault injection, multi-process tests. |
| [`godview_sim/`](godview_sim) | Deterministic simulation: the perception harness and scenario catalog, plus the legacy scenarios. |
| [`docs/`](docs) | Documentation. [PERCEPTION_SHARING.md](docs/PERCEPTION_SHARING.md) is the current specification; see [docs/README.md](docs/README.md) for what the other documents cover. |
| [`agent/`](agent) | CARLA detection agent on the earlier `TrackManager` path. Not a workspace member. |
| [`carla_bridge/`](carla_bridge), [`demo/`](demo), [`viewer/`](viewer) | CARLA bridges, demo scripts and a web viewer for the earlier pipeline. |

## Earlier design

GodView began with four engines: an augmented-state EKF for out-of-sequence measurements (`godview_time`), an H3 plus 3D grid spatial index (`godview_space`), capability-based security with Biscuit tokens (`godview_trust`), and a tracker that fused peers' packets with nearest-neighbor association, covariance intersection and a min-UUID identity rule (`godview_tracking`, "Highlander"). An audit found that this tracker could double count, mixed coordinate frames and depended on message order. The perception-sharing pipeline above replaces it for shared perception.

The earlier engines are still in the repository. `godview_time` was repaired (its state augmentation lost correlations), `godview_space` is still used as the store consumers read, and `TrackManager` still drives the CARLA agent, the Rerun examples in `godview_core/examples`, and the legacy simulation scenarios (`--scenario legacy`), including the evolutionary parameter-tuning experiments. The examples build with `--features visualization` (`kitti_demo` also needs `kitti`; `carla_demo` needs `carla` and a system ZeroMQ, and was not checked). Two legacy scenarios, `scale_limit` and `common_bias`, also fail on the earlier code, and `protocol_drift` has no assertions.

## References

1. S. Julier and J. Uhlmann, "A Non-divergent Estimation Algorithm in the Presence of Unknown Correlations," American Control Conference, 1997. (Covariance intersection.)
2. Y. Bar-Shalom, "Update with Out-of-Sequence Measurements in Tracking: Exact Solution," IEEE Transactions on Aerospace and Electronic Systems, 2002.
3. A. Rahmathullah, Á. García-Fernández and L. Svensson, "Generalized Optimal Sub-Pattern Assignment Metric," FUSION, 2017. (GOSPA, used to score estimates.)
4. Uber Engineering, "H3: Uber's Hexagonal Hierarchical Spatial Index."
5. Zenoh, [zenoh.io](https://zenoh.io).

## License

MIT. See [LICENSE](LICENSE).

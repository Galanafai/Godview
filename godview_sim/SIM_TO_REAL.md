# Sim-to-Real: Deploying GodView to Hardware

This document maps the **Deterministic Simulation Testing (DST)** results to real-world hardware and network constraints.

## 1. Network Layer (The "Air Gap")

In simulation (`godview_sim`), we modeled packet loss and jitter. In reality, this maps to:

| Sim Variable | Real World Equivalent | Validated By |
|---|---|---|
| **Packet Loss (30-90%)** | RF Interference, Terrain blocking, jammed LoRa | `DST-010 (NetworkHell)` |
| **Jitter (500ms)** | Mesh store-and-forward delays, 5G latency spikes | `DST-008 (ChaosStorm)` |
| **Bandwidth Limit** | LoRaWAN duty cycles, saturated WiFi | `DST-015 (ResourceStarvation)` |

### Recommendation
*   **Hardware**: LoRa (SX1276) for control/gossip, 5G for high-bandwidth telemetry.
*   **Protocol**: Unreliable Datagrams (UDP-like) are preferred. `DST-010` proved TCP-like retries are fatal in high-loss zones; redundant gossip is superior.

## 2. Compute & Scaling

In simulation, we hit a wall at 200 agents on a server CPU (`DST-009`).

| Sim Variable | Real World Equivalent | Constraint |
|---|---|---|
| **Agents** | Physical Drones / Ground Nodes | **CPU Ops** |
| **Entities** | Tracked targets (planes, cars) | **Memory** |

### Mapping to Embedded Hardware
*   **Server CPU (Ryzen/Xeon)**: Handles ~200 agents.
*   **Nvidia Jetson / Raspberry Pi 4**: Likely limited to **20-30 agents** per node before O(n²) gossip overwhelms the CPU.
*   **Fix**: The Evolutionary Logic (`DST-015`) *must* be enabled on embedded nodes to automatically throttle gossip when CPU load spikes.

## 3. Clock Synchronization

Simulation uses perfect global time. Real hardware drifts.

| Sim Variable | Real World Equivalent | Validated By |
|---|---|---|
| **OOSM (Out of Sequence)** | Clock drift + Network Latency | `DST-011 (TimeTornado)` |

### Critical Requirement
*   **GPS/PTP**: Nodes **MUST** have GPS-disciplined clocks. `DST-011` showed that while we survive 5s delays, accuracy degrades to 76m RMS. Protocol relies on timestamps.

## 4. Trust & Security

| Sim Variable | Real World Equivalent | Validated By |
|---|---|---|
| **Bad Actor** | Hacked Drone, Spoofed Signal | `DST-012 (ZombieApoc)` |
| **Sybil Attack** | Multiple fake identities | `DST-007 (AdaptiveSwarm)` |

### Deployment Strategy
*   Enable **Adaptive Reputation** by default.
*   In hostile environments (Electronic Warfare), `DST-012` proves the swarm will isolate compromised nodes automatically without human intervention.

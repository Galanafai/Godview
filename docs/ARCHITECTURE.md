# GodView Architectural Analysis

## 1. Sequence Diagram: Agent Join, Broadcast & Global Truth

This diagram illustrates the lifecycle of a single Agent within the GodView swarm simulation.

```mermaid
sequenceDiagram
    participant Sim as Simulation Runner
    participant Oracle as Global Oracle (Truth)
    participant Agent as GodView Agent
    participant TM as Tracking Engine
    participant Net as Swarm Network

    Note over Sim, Agent: Initialization & Join
    Sim->>Agent: new(Context, Network, Config)
    Agent->>TM: new(TrackingConfig)
    Sim->>Net: Register Agent NodeID

    Note over Sim, Agent: Simulation Loop (Heartbeat)
    loop Every Tick (30Hz)
        Sim->>Oracle: step(dt)
        Sim->>Oracle: generate_sensor_readings()
        Oracle-->>Sim: [SensorReading]
        
        Note right of Agent: 1. Sensor Input
        Sim->>Agent: ingest_readings(readings)
        Agent->>Agent: Convert to GlobalHazardPacket
        Agent->>TM: process_packet(packet)
        TM-->>Agent: Track Updated
        Agent->>Agent: Buffer packet in recent_packets

        Note right of Agent: 2. Broadcast (Gossip)
        Agent->>Net: queue_gossip(recent_packets)
        
        Note right of Agent: 3. Receive "Global Truth" (Network Consensus)
        Net-->>Agent: receive_gossip(packets from neighbors)
        Agent->>TM: process_packet(remote_packet)
        
        par Consensus Layer
            TM->>TM: Spatial Pruning (H3)
            TM->>TM: Geometric Gating (Mahalanobis)
            TM->>TM: Identity Resolution (Highlander)
            TM->>TM: State Fusion (Covariance Intersection)
        end
        
        TM-->>Agent: Updated Canonical Track
    end
```

## 2. Data Flow Map: Sensor to Network

The data flows through a strict pipeline designed for deterministic consistency.

```mermaid
graph TD
    subgraph Input [Perception Layer]
        S[Sensor Input] -->|Raw Data| O[Oracle / Hardware]
        O -->|SensorReading| A[Agent Ingestion]
        A -->|GlobalHazardPacket| LB[Local Buffer]
    end

    subgraph Core [Consensus & Fusion Layer]
        LB -->|Packet| TM[Tracking Manager]
        
        TM -->|Stage 1| SQ[Spatial Pruning]
        SQ -->|H3 Cell Candidates| GG[Geometric Gating]
        
        GG -->|Stage 2| MD{Mahalanobis Dist < Threshold?}
        MD -- Yes --> GNN[GNN Selection]
        MD -- No --> NT[Create New Track]
        
        GNN -->|Stage 3| HL[Highlander Heuristic]
        HL -->|Min-UUID Resolution| CI[Covariance Intersection]
        
        CI -->|Stage 4| FS[Fused State Update]
    end

    subgraph Output [Network Layer]
        FS -->|Updated Track| G[Gossip Protocol]
        LB -->|Recent Packets| G
        G -->|SignedPacketEnvelope| NP[Network Propagation]
    end

    style TM fill:#f9f,stroke:#333,stroke-width:2px
    style HL fill:#ff9,stroke:#333,stroke-width:2px
```

## 3. The Core & Ownership Model

### The Heartbeat: `godview_core::agent_runtime::GodViewAgent`

The specific module acting as the "heartbeat" of the system is the **`GodViewAgent`** struct (specifically its `tick()` method) located in `godview_core/src/agent_runtime.rs`.

While `godview_sim::runner` drives the *simulation* time, the `GodViewAgent` orchestrates the internal logic of the autonomous unit. 

**Core Responsibilities:**
1.  **Orchestration**: It holds the four mathematical engines (`Time`, `Space`, `Trust`, `Tracking`) and coordinates data flow between them.
2.  **Tick Execution**: Its `tick()` method drives the `TimeEngine` prediction step and `TrackManager` housekeeping (aging tracks) independently of network events.

### Ownership Model & Safety

The system uses a **Single Ownership / Vertical Isolation** model to ensure safety, particularly for Deterministic Simulation Testing (DST).

*   **Vertical Ownership**: 
    *   The `GodViewAgent` strictly **owns** its engines (`TimeEngine`, `SpatialEngine`, `TrackManager`) by value.
    *   It does **not** share these engines behind `Arc<Mutex<...>>` or other interior mutability patterns.
    *   This guarantees that during a `tick()` or `process_packet()`, the agent has exclusive, synchronous mutable access to its entire state (`&mut self`).
    
*   **Environmental Isolation**:
    *   External dependencies (Time, Network, Randomness) are abstracted via the `GodViewContext` and `NetworkTransport` traits.
    *   The Agent holds these as `Arc<Context>`, effectively verifying that it mostly treats the environment as **read-only** or **message-passing only**, preventing side-effect leakage that would break determinism.

*   **Safety Guarantee**:
    *   **Deadlock Freedom**: Since the Agent doesn't internally lock distinct resources, it is immune to deadlocks within its own logic.
    *   **Deterministic Replay**: The ownership model ensures that given the same sequence of inputs (ticks + packets), the internal state transitions are pure functions, essential for the "God View" global truth verification.

//! GodView Deterministic Simulation Testing (DST) Harness
//!
//! This crate provides the "Matrix" - a controlled simulation environment
//! where the entire GodView multi-agent system runs deterministically.
//!
//! # Core Principle: The Reactor Pattern
//!
//! All sources of non-determinism are intercepted and controlled:
//! - **Time**: Virtual clock advances only when all agents block on I/O
//! - **Network**: Channels with configurable latency, jitter, and partitions
//! - **Randomness**: All entropy derived from a single 64-bit seed
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                         SimWorld                            │
//! │  ┌──────────────────────────────────────────────────────┐   │
//! │  │ madsim::Runtime (Virtual Clock + Event Queue)        │   │
//! │  └──────────────────────────────────────────────────────┘   │
//! │       │                        │                            │
//! │  ┌────▼────┐              ┌────▼────┐                       │
//! │  │  Agent  │◄────────────►│  Agent  │     ...               │
//! │  │   #1    │   Network    │   #2    │                       │
//! │  └─────────┘   Channels   └─────────┘                       │
//! │       ▲                        ▲                            │
//! │       │                        │                            │
//! │  ┌────┴────────────────────────┴────┐                       │
//! │  │            Oracle                 │                       │
//! │  │  (Ground Truth Physics Engine)    │                       │
//! │  └───────────────────────────────────┘                       │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Usage
//!
//! ```ignore
//! use godview_sim::{SimWorld, SimConfig};
//!
//! let config = SimConfig {
//!     seed: 42,
//!     num_agents: 6,
//!     ..Default::default()
//! };
//!
//! let mut world = SimWorld::new(config);
//! world.run_scenario("split_brain");
//! ```

mod context;
mod world;
mod oracle;
mod network;
mod keys;
pub mod scenarios;
mod agent;
mod runner;
pub mod visualizer;
pub mod exporter;
pub mod swarm_network;
pub mod adaptive;

pub use context::SimContext;
pub use world::{SimWorld, SimConfig};
pub use oracle::{Oracle, GroundTruthEntity, SensorReading};
pub use network::{SimNetwork, SimNetworkController};
pub use keys::DeterministicKeyProvider;
pub use agent::SimulatedAgent;
pub use runner::{ScenarioRunner, ScenarioResult, ScenarioMetrics};
pub use visualizer::RerunLogger;
pub use exporter::{SimExport, SimFrame, EntityPosition, AgentFrame, TrackPosition, SimEvent};
pub use swarm_network::{SwarmNetwork, SwarmConfig};
pub use adaptive::{AdaptiveState, AdaptiveMetrics, NeighborReputation, TrackConfidence};



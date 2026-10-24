//! Deterministic simulation testing for perception sharing.
//!
//! * [`bus`]: a fault-injecting byte transport (latency, jitter, loss,
//!   duplication, corruption, partitions, topologies).
//! * [`harness`]: runs a [`harness::ScenarioSpec`] with independent per-agent
//!   sensing, per-agent frames and localization error, and records each
//!   agent's view of the world.
//! * [`metrics`]: GOSPA and NEES scoring against ground truth.
//! * [`catalog`]: the scenario definitions and the invariant each one checks.

pub mod bus;
pub mod catalog;
pub mod harness;
pub mod metrics;

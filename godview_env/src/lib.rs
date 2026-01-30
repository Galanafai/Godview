//! GodView Environment Abstraction Layer
//!
//! This crate provides the "Sans-IO" abstraction allowing GodView engines
//! to run in both **Production** (tokio) and **Simulation** (madsim) environments.
//!
//! # Core Concept: The Reactor Pattern
//!
//! For Deterministic Simulation Testing (DST), we intercept all I/O:
//! - Time (`now()`, `sleep()`)
//! - Network (`send()`, `recv()`)
//! - Randomness (`derive_keypair()`)
//!
//! By deriving all entropy from a single 64-bit seed, any bug becomes
//! reproducible via its seed number.
//!
//! # Example
//!
//! ```ignore
//! use godview_env::{GodViewContext, NetworkTransport};
//!
//! async fn agent_loop<Ctx: GodViewContext, Net: NetworkTransport>(
//!     ctx: &Ctx,
//!     net: &Net,
//! ) {
//!     loop {
//!         tokio::select! {
//!             packet = net.recv() => handle_packet(packet),
//!             _ = ctx.sleep(Duration::from_millis(33)) => tick(),
//!         }
//!     }
//! }
//! ```

mod context;
mod network;
mod types;
mod error;
mod tokio_impl;

pub use context::GodViewContext;
pub use network::{NetworkTransport, NetworkController};
pub use types::{NodeId, SignedPacketEnvelope};
pub use error::EnvError;
pub use tokio_impl::TokioContext;


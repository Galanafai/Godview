//! A GodView perception-sharing node over Zenoh.
//!
//! The node wraps one [`godview_core::perception_node::PerceptionNode`] with
//! real I/O. Each agent publishes signed report batches on the topic of the
//! H3 cell its reports fall in (`<prefix>/reports/<res>/<cell>`) and
//! subscribes to the cells around itself, so the traffic a node receives
//! grows with what is near it, not with the size of the fleet.
//!
//! * [`runner`]: the async runtime (Zenoh session, subscriptions, tick loop).
//! * [`synthetic`]: a seeded synthetic world every process can rebuild.
//! * [`input`]: JSON-lines input from an external perception stack.
//! * [`faults`]: seeded loss, delay, jitter and duplication on outgoing traffic.
//! * [`keys`]: Zenoh key expressions.
//! * [`report`]: the JSON report a node writes when it finishes.

pub mod faults;
pub mod input;
pub mod keys;
pub mod report;
pub mod runner;
pub mod synthetic;

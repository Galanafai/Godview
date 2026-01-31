//! Rerun visualization for DST simulations.
//!
//! This module provides visualization of simulation runs using the Rerun SDK.
//! Visualization is optional and only available with the `visualization` feature.
//!
//! # What Gets Logged
//!
//! - Ground truth entities (Oracle) as green points
//! - Agent track estimates as colored points per agent
//! - Entity trajectories as lines
//! - Simulation time as scalar timeline

#[cfg(feature = "visualization")]
use rerun::{RecordingStream, Points3D, Position3D, Color, Radius};
use nalgebra::Vector3;

/// Rerun logger for simulation visualization.
pub struct RerunLogger {
    #[cfg(feature = "visualization")]
    rec: Option<RecordingStream>,
    
    /// Whether visualization is enabled
    enabled: bool,
}

impl RerunLogger {
    /// Creates a new logger with visualization disabled.
    pub fn disabled() -> Self {
        Self {
            #[cfg(feature = "visualization")]
            rec: None,
            enabled: false,
        }
    }
    
    /// Creates a new logger with visualization enabled.
    #[cfg(feature = "visualization")]
    pub fn new(name: &str) -> Self {
        match rerun::RecordingStreamBuilder::new(name).spawn() {
            Ok(rec) => {
                tracing::info!("Rerun visualization enabled - open Rerun Viewer to see simulation");
                Self {
                    rec: Some(rec),
                    enabled: true,
                }
            }
            Err(e) => {
                tracing::warn!("Failed to initialize Rerun: {:?}", e);
                Self {
                    rec: None,
                    enabled: false,
                }
            }
        }
    }
    
    /// Creates a logger - returns disabled if visualization feature not enabled.
    #[cfg(not(feature = "visualization"))]
    pub fn new(_name: &str) -> Self {
        tracing::info!("Rerun visualization not available (compile with --features visualization)");
        Self::disabled()
    }
    
    /// Returns whether visualization is enabled.
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }
    
    /// Sets the simulation time for subsequent logs.
    #[cfg(feature = "visualization")]
    pub fn set_time(&self, seconds: f64) {
        if let Some(ref rec) = self.rec {
            rec.set_time_seconds("sim_time", seconds);
        }
    }
    
    #[cfg(not(feature = "visualization"))]
    pub fn set_time(&self, _seconds: f64) {}
    
    /// Logs ground truth entities from the Oracle.
    #[cfg(feature = "visualization")]
    pub fn log_ground_truth(&self, entities: &[(u64, Vector3<f64>)]) {
        if let Some(ref rec) = self.rec {
            let points: Vec<Position3D> = entities
                .iter()
                .map(|(_, pos)| Position3D::new(pos.x as f32, pos.y as f32, pos.z as f32))
                .collect();
            
            let _ = rec.log(
                "world/ground_truth",
                &Points3D::new(points)
                    .with_colors([Color::from_rgb(0, 255, 0)]) // Green
                    .with_radii([Radius::new_scene_units(2.0)]),
            );
        }
    }
    
    #[cfg(not(feature = "visualization"))]
    pub fn log_ground_truth(&self, _entities: &[(u64, Vector3<f64>)]) {}
    
    /// Logs agent track estimates.
    #[cfg(feature = "visualization")]
    pub fn log_tracks(&self, agent_id: u64, tracks: &[(uuid::Uuid, Vector3<f64>)]) {
        if let Some(ref rec) = self.rec {
            let points: Vec<Position3D> = tracks
                .iter()
                .map(|(_, pos)| Position3D::new(pos.x as f32, pos.y as f32, pos.z as f32))
                .collect();
            
            // Color based on agent ID
            let color = match agent_id % 6 {
                0 => Color::from_rgb(255, 100, 100), // Red
                1 => Color::from_rgb(100, 100, 255), // Blue
                2 => Color::from_rgb(255, 255, 100), // Yellow
                3 => Color::from_rgb(100, 255, 255), // Cyan
                4 => Color::from_rgb(255, 100, 255), // Magenta
                _ => Color::from_rgb(255, 165, 0),   // Orange
            };
            
            let _ = rec.log(
                format!("world/agents/{}/tracks", agent_id),
                &Points3D::new(points)
                    .with_colors([color])
                    .with_radii([Radius::new_scene_units(1.5)]),
            );
        }
    }
    
    #[cfg(not(feature = "visualization"))]
    pub fn log_tracks(&self, _agent_id: u64, _tracks: &[(uuid::Uuid, Vector3<f64>)]) {}
    
    /// Logs a text annotation (e.g., partition event).
    #[cfg(feature = "visualization")]
    pub fn log_event(&self, path: &str, message: &str) {
        if let Some(ref rec) = self.rec {
            let _ = rec.log(
                path,
                &rerun::TextLog::new(message),
            );
        }
    }
    
    #[cfg(not(feature = "visualization"))]
    pub fn log_event(&self, _path: &str, _message: &str) {}
    
    /// Logs RMS error as a scalar metric.
    #[cfg(feature = "visualization")]
    pub fn log_error(&self, agent_id: u64, rms_error: f64) {
        if let Some(ref rec) = self.rec {
            let _ = rec.log(
                format!("metrics/agent_{}/rms_error", agent_id),
                &rerun::Scalar::new(rms_error),
            );
        }
    }
    
    #[cfg(not(feature = "visualization"))]
    pub fn log_error(&self, _agent_id: u64, _rms_error: f64) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_disabled_logger() {
        let logger = RerunLogger::disabled();
        assert!(!logger.is_enabled());
        
        // These should be no-ops
        logger.set_time(1.0);
        logger.log_ground_truth(&[(1, Vector3::new(0.0, 0.0, 0.0))]);
    }
}

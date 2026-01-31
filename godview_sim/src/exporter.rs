//! JSON exporter for Rerun visualization.
//!
//! Exports simulation frames as JSON for the Python Rerun visualizer.

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use std::fs::File;
use std::io::Write;

/// A single frame of simulation data.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimFrame {
    /// Simulation time in seconds
    pub time_sec: f64,
    
    /// Ground truth entity positions
    pub ground_truth: Vec<EntityPosition>,
    
    /// Agent track estimates
    pub agents: Vec<AgentFrame>,
    
    /// Events (partitions, revocations, etc.)
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub events: Vec<SimEvent>,
}

/// Position of an entity.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EntityPosition {
    pub id: u64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl EntityPosition {
    pub fn new(id: u64, pos: Vector3<f64>) -> Self {
        Self {
            id,
            x: pos.x,
            y: pos.y,
            z: pos.z,
        }
    }
}

/// Agent frame data.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgentFrame {
    pub agent_id: u64,
    pub tracks: Vec<TrackPosition>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rms_error: Option<f64>,
}

/// Track position.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrackPosition {
    pub track_id: String,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// Simulation event.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimEvent {
    pub message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub level: Option<String>,
}

/// Complete simulation export.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimExport {
    /// Scenario name
    pub scenario: String,
    
    /// Seed used
    pub seed: u64,
    
    /// Duration in seconds
    pub duration_sec: f64,
    
    /// All frames
    pub frames: Vec<SimFrame>,
    
    /// Final results
    pub passed: bool,
    
    /// Final RMS error if applicable
    #[serde(skip_serializing_if = "Option::is_none")]
    pub final_rms_error: Option<f64>,
}

impl SimExport {
    /// Creates a new export container.
    pub fn new(scenario: &str, seed: u64) -> Self {
        Self {
            scenario: scenario.to_string(),
            seed,
            duration_sec: 0.0,
            frames: Vec::new(),
            passed: false,
            final_rms_error: None,
        }
    }
    
    /// Adds a frame.
    pub fn add_frame(&mut self, frame: SimFrame) {
        self.duration_sec = frame.time_sec;
        self.frames.push(frame);
    }
    
    /// Finalizes the export.
    pub fn finalize(&mut self, passed: bool, rms_error: Option<f64>) {
        self.passed = passed;
        self.final_rms_error = rms_error;
    }
    
    /// Writes to a JSON file.
    pub fn write_to_file(&self, path: &str) -> std::io::Result<()> {
        let json = serde_json::to_string_pretty(self)?;
        let mut file = File::create(path)?;
        file.write_all(json.as_bytes())?;
        Ok(())
    }
}

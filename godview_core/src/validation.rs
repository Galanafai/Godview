//! Validation Module - Ground Truth Comparison for CARLA Integration
//! ===================================================================
//!
//! This module provides tools for validating GodView's sensor fusion
//! output against ground truth data from CARLA simulation.
//!
//! Key metrics:
//! - Position error (RMSE, max error)
//! - Track latency (time from detection to track update)
//! - Track completeness (how many GT objects are tracked)
//! - Ghost rate (false positive tracks)
//!
//! Usage:
//! ```ignore
//! use godview_core::validation::{ValidationSession, GroundTruthEntry};
//!
//! let mut session = ValidationSession::new();
//!
//! // Record ground truth from CARLA
//! session.record_ground_truth(actor_id, position, velocity, heading, class, sim_time);
//!
//! // Record GodView detection
//! session.record_detection(actor_id, detected_position, confidence, source, sim_time);
//!
//! // Get metrics
//! let report = session.generate_report();
//! ```

use std::collections::HashMap;
use nalgebra::Vector3;

// =============================================================================
// GROUND TRUTH ENTRY
// =============================================================================

/// Single ground truth state for an actor at a specific time
#[derive(Debug, Clone)]
pub struct GroundTruthEntry {
    pub actor_id: u32,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub heading_degrees: f64,
    pub actor_class: String,
    pub timestamp: f64,
}

/// Single detection from GodView
#[derive(Debug, Clone)]
pub struct DetectionEntry {
    pub actor_id: u32,
    pub position: Vector3<f64>,
    pub confidence: f64,
    pub source_agent: String,
    pub timestamp: f64,
}

// =============================================================================
// VALIDATION METRICS
// =============================================================================

/// Per-actor validation statistics
#[derive(Debug, Clone, Default)]
pub struct ActorMetrics {
    /// Number of detections for this actor
    pub detection_count: usize,
    /// Sum of position errors (for RMSE calculation)
    pub error_sum_squared: f64,
    /// Maximum position error observed
    pub max_error: f64,
    /// Minimum position error observed
    pub min_error: f64,
    /// Last detection timestamp
    pub last_detection_time: f64,
    /// Number of frames where this actor was visible but not detected
    pub missed_frames: usize,
}

impl ActorMetrics {
    pub fn new() -> Self {
        Self {
            min_error: f64::MAX,
            ..Default::default()
        }
    }
    
    /// Calculate RMSE (Root Mean Square Error)
    pub fn rmse(&self) -> f64 {
        if self.detection_count > 0 {
            (self.error_sum_squared / self.detection_count as f64).sqrt()
        } else {
            0.0
        }
    }
    
    /// Calculate detection rate (assumes 1 expected detection per frame)
    pub fn detection_rate(&self, total_frames: usize) -> f64 {
        if total_frames > 0 {
            self.detection_count as f64 / total_frames as f64
        } else {
            0.0
        }
    }
}

/// Global validation metrics
#[derive(Debug, Clone, Default)]
pub struct GlobalMetrics {
    /// Total frames processed
    pub total_frames: usize,
    /// Total detections across all actors
    pub total_detections: usize,
    /// Total ground truth entries
    pub total_ground_truth: usize,
    /// Number of unique actors in ground truth
    pub unique_actors: usize,
    /// Number of unique actors detected
    pub unique_actors_detected: usize,
    /// Sum of all position errors
    pub global_error_sum: f64,
    /// Maximum error observed globally
    pub global_max_error: f64,
    /// Number of ghost tracks (detections with no matching GT)
    pub ghost_detections: usize,
    /// Latency samples (detection time - GT time)
    pub latency_samples: Vec<f64>,
}

impl GlobalMetrics {
    /// Calculate global average error
    pub fn avg_error(&self) -> f64 {
        if self.total_detections > 0 {
            self.global_error_sum / self.total_detections as f64
        } else {
            0.0
        }
    }
    
    /// Calculate average latency in milliseconds
    pub fn avg_latency_ms(&self) -> f64 {
        if !self.latency_samples.is_empty() {
            (self.latency_samples.iter().sum::<f64>() / self.latency_samples.len() as f64) * 1000.0
        } else {
            0.0
        }
    }
    
    /// Calculate 95th percentile latency
    pub fn p95_latency_ms(&self) -> f64 {
        if self.latency_samples.is_empty() {
            return 0.0;
        }
        let mut sorted: Vec<f64> = self.latency_samples.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let idx = (0.95 * sorted.len() as f64) as usize;
        sorted.get(idx.min(sorted.len() - 1)).copied().unwrap_or(0.0) * 1000.0
    }
    
    /// Calculate track coverage (% of GT actors that were detected)
    pub fn track_coverage(&self) -> f64 {
        if self.unique_actors > 0 {
            self.unique_actors_detected as f64 / self.unique_actors as f64 * 100.0
        } else {
            0.0
        }
    }
    
    /// Calculate ghost rate (% of detections that don't match GT)
    pub fn ghost_rate(&self) -> f64 {
        if self.total_detections > 0 {
            self.ghost_detections as f64 / self.total_detections as f64 * 100.0
        } else {
            0.0
        }
    }
}

// =============================================================================
// VALIDATION SESSION
// =============================================================================

/// Main validation session that collects ground truth and detections
pub struct ValidationSession {
    /// Ground truth entries indexed by (actor_id, frame_index)
    ground_truth: HashMap<u32, Vec<GroundTruthEntry>>,
    /// Detection entries indexed by actor_id
    detections: HashMap<u32, Vec<DetectionEntry>>,
    /// Per-actor metrics
    per_actor_metrics: HashMap<u32, ActorMetrics>,
    /// Global metrics
    global_metrics: GlobalMetrics,
    /// Association threshold (meters) - detections within this distance match GT
    association_threshold: f64,
    /// Current frame index
    current_frame: usize,
}

impl ValidationSession {
    /// Create a new validation session with default settings
    pub fn new() -> Self {
        Self::with_threshold(2.0) // 2 meter association threshold
    }
    
    /// Create a validation session with custom association threshold
    pub fn with_threshold(threshold_meters: f64) -> Self {
        Self {
            ground_truth: HashMap::new(),
            detections: HashMap::new(),
            per_actor_metrics: HashMap::new(),
            global_metrics: GlobalMetrics::default(),
            association_threshold: threshold_meters,
            current_frame: 0,
        }
    }
    
    /// Record a ground truth entry from CARLA
    pub fn record_ground_truth(
        &mut self,
        actor_id: u32,
        position: [f64; 3],
        velocity: [f64; 3],
        heading_degrees: f64,
        actor_class: &str,
        timestamp: f64,
    ) {
        let entry = GroundTruthEntry {
            actor_id,
            position: Vector3::new(position[0], position[1], position[2]),
            velocity: Vector3::new(velocity[0], velocity[1], velocity[2]),
            heading_degrees,
            actor_class: actor_class.to_string(),
            timestamp,
        };
        
        self.ground_truth
            .entry(actor_id)
            .or_insert_with(Vec::new)
            .push(entry);
        
        self.global_metrics.total_ground_truth += 1;
    }
    
    /// Record a detection from GodView
    pub fn record_detection(
        &mut self,
        actor_id: u32,
        position: [f64; 3],
        confidence: f64,
        source_agent: &str,
        detection_timestamp: f64,
    ) {
        let entry = DetectionEntry {
            actor_id,
            position: Vector3::new(position[0], position[1], position[2]),
            confidence,
            source_agent: source_agent.to_string(),
            timestamp: detection_timestamp,
        };
        
        self.detections
            .entry(actor_id)
            .or_insert_with(Vec::new)
            .push(entry.clone());
        
        self.global_metrics.total_detections += 1;
        
        // Find closest ground truth entry for this actor
        if let Some(gt_entries) = self.ground_truth.get(&actor_id) {
            // Find GT entry closest in time
            let closest_gt = gt_entries.iter()
                .min_by(|a, b| {
                    let diff_a = (a.timestamp - detection_timestamp).abs();
                    let diff_b = (b.timestamp - detection_timestamp).abs();
                    diff_a.partial_cmp(&diff_b).unwrap()
                });
            
            if let Some(gt) = closest_gt {
                // Calculate position error
                let error = (entry.position - gt.position).norm();
                
                // Update per-actor metrics
                let metrics = self.per_actor_metrics
                    .entry(actor_id)
                    .or_insert_with(ActorMetrics::new);
                
                metrics.detection_count += 1;
                metrics.error_sum_squared += error * error;
                metrics.max_error = metrics.max_error.max(error);
                metrics.min_error = metrics.min_error.min(error);
                metrics.last_detection_time = detection_timestamp;
                
                // Update global metrics
                self.global_metrics.global_error_sum += error;
                self.global_metrics.global_max_error = 
                    self.global_metrics.global_max_error.max(error);
                
                // Record latency
                let latency = detection_timestamp - gt.timestamp;
                if latency >= 0.0 && latency < 1.0 {
                    self.global_metrics.latency_samples.push(latency);
                }
            }
        } else {
            // No ground truth for this actor - it's a ghost
            self.global_metrics.ghost_detections += 1;
        }
    }
    
    /// Mark end of frame (for tracking missed detections)
    pub fn end_frame(&mut self) {
        self.current_frame += 1;
        self.global_metrics.total_frames += 1;
    }
    
    /// Generate final validation report
    pub fn generate_report(&mut self) -> ValidationReport {
        // Calculate unique actors
        self.global_metrics.unique_actors = self.ground_truth.len();
        self.global_metrics.unique_actors_detected = self.per_actor_metrics.len();
        
        ValidationReport {
            global_metrics: self.global_metrics.clone(),
            per_actor_metrics: self.per_actor_metrics.clone(),
            association_threshold: self.association_threshold,
        }
    }
}

impl Default for ValidationSession {
    fn default() -> Self {
        Self::new()
    }
}

// =============================================================================
// VALIDATION REPORT
// =============================================================================

/// Final validation report with all metrics
#[derive(Debug, Clone)]
pub struct ValidationReport {
    pub global_metrics: GlobalMetrics,
    pub per_actor_metrics: HashMap<u32, ActorMetrics>,
    pub association_threshold: f64,
}

impl ValidationReport {
    /// Print formatted report to console
    pub fn print(&self) {
        println!();
        println!("╔══════════════════════════════════════════════════════════════╗");
        println!("║               GODVIEW VALIDATION REPORT                      ║");
        println!("╠══════════════════════════════════════════════════════════════╣");
        println!("║ SUMMARY                                                      ║");
        println!("╠══════════════════════════════════════════════════════════════╣");
        println!("║ Total Frames:          {:>10}                           ║", 
            self.global_metrics.total_frames);
        println!("║ GT Entries:            {:>10}                           ║", 
            self.global_metrics.total_ground_truth);
        println!("║ Total Detections:      {:>10}                           ║", 
            self.global_metrics.total_detections);
        println!("║ Unique Actors (GT):    {:>10}                           ║", 
            self.global_metrics.unique_actors);
        println!("║ Unique Actors (Det):   {:>10}                           ║", 
            self.global_metrics.unique_actors_detected);
        println!("╠══════════════════════════════════════════════════════════════╣");
        println!("║ ACCURACY                                                     ║");
        println!("╠══════════════════════════════════════════════════════════════╣");
        println!("║ Average Error:         {:>10.3} m                         ║", 
            self.global_metrics.avg_error());
        println!("║ Max Error:             {:>10.3} m                         ║", 
            self.global_metrics.global_max_error);
        println!("║ Track Coverage:        {:>10.1}%                          ║", 
            self.global_metrics.track_coverage());
        println!("║ Ghost Rate:            {:>10.1}%                          ║", 
            self.global_metrics.ghost_rate());
        println!("╠══════════════════════════════════════════════════════════════╣");
        println!("║ LATENCY                                                      ║");
        println!("╠══════════════════════════════════════════════════════════════╣");
        println!("║ Average Latency:       {:>10.2} ms                        ║", 
            self.global_metrics.avg_latency_ms());
        println!("║ P95 Latency:           {:>10.2} ms                        ║", 
            self.global_metrics.p95_latency_ms());
        println!("╚══════════════════════════════════════════════════════════════╝");
        
        // Per-actor summary (top 5 by detection count)
        if !self.per_actor_metrics.is_empty() {
            println!();
            println!("Top Actors by Detection Count:");
            println!("─────────────────────────────────────────────────────────");
            println!("  Actor ID    Detections    RMSE (m)    Max Error (m)");
            println!("─────────────────────────────────────────────────────────");
            
            let mut actors: Vec<_> = self.per_actor_metrics.iter().collect();
            actors.sort_by(|a, b| b.1.detection_count.cmp(&a.1.detection_count));
            
            for (actor_id, metrics) in actors.iter().take(5) {
                println!("  {:>8}    {:>10}    {:>8.3}    {:>12.3}",
                    actor_id,
                    metrics.detection_count,
                    metrics.rmse(),
                    metrics.max_error
                );
            }
        }
    }
    
    /// Check if validation passes acceptance criteria
    pub fn passes_criteria(&self, max_avg_error: f64, max_ghost_rate: f64) -> bool {
        self.global_metrics.avg_error() <= max_avg_error &&
        self.global_metrics.ghost_rate() <= max_ghost_rate
    }
}

// =============================================================================
// TESTS
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_validation_session() {
        let mut session = ValidationSession::new();
        
        // Record ground truth
        session.record_ground_truth(
            1, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 
            0.0, "vehicle", 0.0
        );
        
        // Record detection with small error
        session.record_detection(
            1, [0.5, 0.0, 0.0], 0.9, "ego_vehicle", 0.0
        );
        
        session.end_frame();
        
        let report = session.generate_report();
        
        assert_eq!(report.global_metrics.total_detections, 1);
        assert!(report.global_metrics.avg_error() < 1.0);
    }
    
    #[test]
    fn test_ghost_detection() {
        let mut session = ValidationSession::new();
        
        // Detection with no matching GT = ghost
        session.record_detection(
            999, [10.0, 10.0, 0.0], 0.5, "sensor", 0.0
        );
        
        session.end_frame();
        
        let report = session.generate_report();
        
        assert_eq!(report.global_metrics.ghost_detections, 1);
        assert_eq!(report.global_metrics.ghost_rate(), 100.0);
    }
}

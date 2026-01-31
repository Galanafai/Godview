//! The "TRACKING" Engine - Distributed Data Association Layer
//!
//! Solves the "Duplicate Ghost" problem by using:
//! - Global Nearest Neighbor (GNN) for deterministic association
//! - Covariance Intersection (CI) for loop-safe fusion
//! - Highlander Heuristic (Min-UUID CRDT) for eventual ID consistency
//!
//! This module implements the 4-stage pipeline:
//! 1. Spatial Pruning (H3 cell + k-ring)
//! 2. Geometric Gating (Mahalanobis Distance)
//! 3. Identity Resolution (Highlander)
//! 4. State Fusion (Covariance Intersection)

use h3o::{CellIndex, LatLng, Resolution};
use nalgebra::{Matrix3, Matrix6, Vector3, Vector6};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet, VecDeque};
use uuid::Uuid;
use crate::godview_trust::AdaptiveState;

// ============================================================================
// CONFIGURATION
// ============================================================================

/// Configuration for the TrackManager
#[derive(Debug, Clone)]
pub struct TrackingConfig {
    /// H3 resolution for spatial indexing (default: Resolution::Ten ~66m cells)
    pub h3_resolution: Resolution,
    
    /// Chi-squared threshold for Mahalanobis gating (default: 12.59 for 6 DOF, 95%)
    pub gating_threshold: f64,
    
    /// Maximum age in cycles before track deletion (default: 60 = 2s at 30Hz)
    pub max_age: u32,
    
    /// Base position variance for confidence conversion (default: 25.0 m²)
    pub base_pos_variance: f64,
    
    /// Base velocity variance for confidence conversion (default: 4.0 m²/s²)
    pub base_vel_variance: f64,
}

impl Default for TrackingConfig {
    fn default() -> Self {
        Self {
            h3_resolution: Resolution::Ten,
            gating_threshold: 12.59, // Chi² for 6 DOF at 95%
            max_age: 60,             // 2 seconds at 30 Hz
            base_pos_variance: 25.0, // 5m standard deviation
            base_vel_variance: 4.0,  // 2 m/s standard deviation
        }
    }
}

// ============================================================================
// NETWORK MESSAGE (Input)
// ============================================================================

/// The wire format received from Zenoh - a single observation from a remote agent.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GlobalHazardPacket {
    /// Publisher's local UUID for this object
    pub entity_id: Uuid,
    
    /// Position in WGS84 [latitude, longitude, altitude]
    pub position: [f64; 3],
    
    /// Velocity vector [vx, vy, vz] in m/s
    pub velocity: [f64; 3],
    
    /// Object class: 0=Unknown, 1=Vehicle, 2=Pedestrian, 3=Cyclist, 4=Drone, etc.
    pub class_id: u8,
    
    /// Unix timestamp (seconds since epoch)
    pub timestamp: f64,
    
    /// Confidence score [0.0 - 1.0]
    pub confidence_score: f64,
}

// ============================================================================
// MERGE EVENT (For Deep Inspection Visualization)
// ============================================================================

/// Represents a Highlander merge event where two track IDs collapse into one.
///
/// Used for genealogy visualization and debugging the CRDT logic.
/// Captures the loser's position *before* deletion for the "pop" animation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MergeEvent {
    /// The surviving (canonical) track ID
    pub winner_id: Uuid,
    
    /// The absorbed track ID
    pub loser_id: Uuid,
    
    /// Position where the merge occurred (loser's last position)
    pub merge_position: [f64; 3],
    
    /// Human-readable reason for the merge decision
    pub reason: String,
    
    /// Timestamp of the merge event
    pub timestamp: f64,
}

// ============================================================================
// UNIQUE TRACK (Internal State)
// ============================================================================

/// Internal representation of a fused object in the local world model.
/// 
/// Each UniqueTrack represents one physical object, potentially observed
/// by multiple agents. The Highlander heuristic ensures all agents converge
/// to the same `canonical_id`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UniqueTrack {
    // === Identity (Highlander CRDT) ===
    
    /// The "winning" ID - always the lexicographically smallest UUID seen
    pub canonical_id: Uuid,
    
    /// All UUIDs ever associated with this track (G-Set CRDT)
    pub observed_ids: HashSet<Uuid>,
    
    // === State (6-DOF: Position + Velocity) ===
    
    /// Fused state vector [x, y, z, vx, vy, vz] in ECEF meters
    pub state: Vector6<f64>,
    
    /// 6×6 uncertainty covariance matrix
    pub covariance: Matrix6<f64>,
    
    // === Metadata ===
    
    /// Object class (matches GlobalHazardPacket.class_id)
    pub class_id: u8,
    
    /// Timestamp of the last fusion update
    pub last_update: f64,
    
    /// Cycles since last update (for aging/deletion)
    pub age: u32,
    
    // === Spatial Index Key ===
    
    /// Current H3 cell for spatial indexing
    pub h3_cell: CellIndex,
}

impl UniqueTrack {
    /// Create a new track from an incoming packet.
    pub fn from_packet(packet: &GlobalHazardPacket, covariance: Matrix6<f64>, h3_cell: CellIndex) -> Self {
        let mut observed_ids = HashSet::new();
        observed_ids.insert(packet.entity_id);
        
        Self {
            canonical_id: packet.entity_id,
            observed_ids,
            state: Vector6::new(
                packet.position[0],
                packet.position[1],
                packet.position[2],
                packet.velocity[0],
                packet.velocity[1],
                packet.velocity[2],
            ),
            covariance,
            class_id: packet.class_id,
            last_update: packet.timestamp,
            age: 0,
            h3_cell,
        }
    }
    
    /// Merge a remote ID using the Highlander heuristic.
    /// 
    /// The canonical_id becomes the minimum of all observed IDs.
    /// This is a CRDT merge operation - idempotent and commutative.
    pub fn merge_id(&mut self, remote_id: Uuid) {
        self.observed_ids.insert(remote_id);
        if remote_id < self.canonical_id {
            self.canonical_id = remote_id;
        }
    }
    
    /// Get the position component of the state vector.
    #[inline]
    pub fn position(&self) -> Vector3<f64> {
        Vector3::new(self.state[0], self.state[1], self.state[2])
    }
    
    /// Get the velocity component of the state vector.
    #[inline]
    pub fn velocity(&self) -> Vector3<f64> {
        Vector3::new(self.state[3], self.state[4], self.state[5])
    }
    
    /// Get the position covariance (upper-left 3×3 block).
    #[inline]
    pub fn position_covariance(&self) -> Matrix3<f64> {
        self.covariance.fixed_view::<3, 3>(0, 0).into()
    }
}

// ============================================================================
// TRACK MANAGER (The Engine)
// ============================================================================

/// The core engine for distributed data association.
/// 
/// Maintains a local world model by:
/// 1. Ingesting GlobalHazardPackets from the network
/// 2. Associating them with existing tracks via GNN
/// 3. Fusing state via Covariance Intersection
/// 4. Resolving IDs via the Highlander heuristic
pub struct TrackManager {
    // === Track Store ===
    
    /// All active tracks, keyed by their canonical_id
    tracks: HashMap<Uuid, UniqueTrack>,
    
    // === Spatial Index (H3 → Track IDs) ===
    
    /// Maps H3 cells to the set of track IDs within that cell
    spatial_index: HashMap<CellIndex, HashSet<Uuid>>,
    
    // === Configuration ===
    
    /// Runtime configuration
    config: TrackingConfig,

    /// History of Peer Agreement Cost (J_PA) values
    /// Used for blind fitness evaluation
    pub peer_agreement_history: VecDeque<f64>,

    /// Maximum size of the Peer Agreement rolling window
    pub pa_window_size: usize,
}



impl TrackManager {
    /// Create a new TrackManager with the given configuration.
    pub fn new(config: TrackingConfig) -> Self {
        Self {
            tracks: HashMap::new(),
            spatial_index: HashMap::new(),
            config,
            peer_agreement_history: VecDeque::new(),
            pa_window_size: 30,
        }
    }
    
    /// Create a new TrackManager with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(TrackingConfig::default())
    }
    
    // ========================================================================
    // SPATIAL INDEX OPERATIONS
    // ========================================================================
    
    /// Convert WGS84 coordinates to an H3 cell index.
    pub fn position_to_cell(&self, lat: f64, lon: f64) -> Result<CellIndex, TrackingError> {
        let latlng = LatLng::new(lat, lon)
            .map_err(|e| TrackingError::InvalidCoordinates(format!("{:?}", e)))?;
        Ok(latlng.to_cell(self.config.h3_resolution))
    }
    
    /// Insert a track into the spatial index.
    fn spatial_index_insert(&mut self, cell: CellIndex, track_id: Uuid) {
        self.spatial_index
            .entry(cell)
            .or_insert_with(HashSet::new)
            .insert(track_id);
    }
    
    /// Remove a track from the spatial index.
    fn spatial_index_remove(&mut self, cell: CellIndex, track_id: Uuid) {
        if let Some(set) = self.spatial_index.get_mut(&cell) {
            set.remove(&track_id);
            // Clean up empty cells
            if set.is_empty() {
                self.spatial_index.remove(&cell);
            }
        }
    }
    
    /// Query all track IDs within a cell and its k-ring neighbors.
    /// 
    /// Default k=1 returns the center cell plus its 6 neighbors (7 total).
    pub fn spatial_query_kring(&self, cell: CellIndex, k: u32) -> HashSet<Uuid> {
        let mut result = HashSet::new();
        
        // Query the center cell and all neighbors in the k-ring
        // Using grid_disk_safe which returns an iterator directly
        for neighbor_cell in cell.grid_disk_safe(k) {
            if let Some(track_ids) = self.spatial_index.get(&neighbor_cell) {
                result.extend(track_ids.iter().copied());
            }
        }
        
        result
    }
    
    /// Update a track's position in the spatial index if its cell changed.
    fn reindex_track(&mut self, track_id: Uuid, old_cell: CellIndex, new_cell: CellIndex) {
        if old_cell != new_cell {
            self.spatial_index_remove(old_cell, track_id);
            self.spatial_index_insert(new_cell, track_id);
        }
    }
    
    // ========================================================================
    // TRACK LIFECYCLE
    // ========================================================================
    
    /// Create a new track from an incoming packet.
    fn create_track(&mut self, packet: &GlobalHazardPacket) -> Result<Uuid, TrackingError> {
        let cell = self.position_to_cell(packet.position[0], packet.position[1])?;
        let covariance = self.confidence_to_covariance(packet.confidence_score);
        
        let track = UniqueTrack::from_packet(packet, covariance, cell);
        let track_id = track.canonical_id;
        
        self.spatial_index_insert(cell, track_id);
        self.tracks.insert(track_id, track);
        
        Ok(track_id)
    }
    
    /// Age all tracks by one cycle and remove those that exceed max_age.
    pub fn age_tracks(&mut self) {
        let max_age = self.config.max_age;
        
        // First, increment age for all tracks
        for track in self.tracks.values_mut() {
            track.age += 1;
        }
        
        // Then collect tracks to remove (those at or above max_age)
        let to_remove: Vec<(Uuid, CellIndex)> = self.tracks
            .iter()
            .filter_map(|(id, track)| {
                if track.age >= max_age {
                    Some((*id, track.h3_cell))
                } else {
                    None
                }
            })
            .collect();
        
        // Remove from spatial index and track store
        for (track_id, cell) in to_remove {
            self.spatial_index_remove(cell, track_id);
            self.tracks.remove(&track_id);
        }
    }
    
    // ========================================================================
    // COVARIANCE / CONFIDENCE CONVERSION
    // ========================================================================
    
    /// Convert a scalar confidence score [0.0, 1.0] to a 6×6 covariance matrix.
    /// 
    /// Higher confidence → lower variance.
    /// This allows handling packets that don't transmit full covariance matrices.
    pub fn confidence_to_covariance(&self, confidence: f64) -> Matrix6<f64> {
        // Clamp to avoid division by zero or negative variance
        let inv_confidence = (1.0 - confidence).clamp(0.01, 1.0);
        
        let pos_var = self.config.base_pos_variance * inv_confidence;
        let vel_var = self.config.base_vel_variance * inv_confidence;
        
        // Diagonal covariance (uncorrelated errors)
        Matrix6::from_diagonal(&Vector6::new(
            pos_var, pos_var, pos_var,  // x, y, z
            vel_var, vel_var, vel_var,  // vx, vy, vz
        ))
    }
    
    // ========================================================================
    // PHASE 2: MATH ENGINE (Mahalanobis & GNN)
    // ========================================================================
    
    /// Compute the squared Mahalanobis distance between a track and an observation.
    /// 
    /// D² = (z - Hx)ᵀ S⁻¹ (z - Hx)
    /// 
    /// Where:
    /// - z = measurement vector (from packet)
    /// - x = track state
    /// - H = observation matrix
    /// - S = HPHᵀ + R = innovation covariance
    /// 
    /// Returns f64::MAX if the innovation covariance is singular.
    pub fn mahalanobis_distance_squared(
        &self,
        track: &UniqueTrack,
        packet: &GlobalHazardPacket,
    ) -> f64 {
        // Construct measurement vector (6-DOF: position + velocity)
        let z = Vector6::new(
            packet.position[0],
            packet.position[1],
            packet.position[2],
            packet.velocity[0],
            packet.velocity[1],
            packet.velocity[2],
        );
        
        // Observation matrix H = I (we observe the full state directly)
        // So H*x = x and H*P*Hᵀ = P
        let residual = z - track.state;
        
        // Measurement noise covariance R
        let r = self.confidence_to_covariance(packet.confidence_score);
        
        // Innovation covariance S = P + R (since H = I)
        let s = track.covariance + r;
        
        // Compute S⁻¹
        match s.try_inverse() {
            Some(s_inv) => {
                // D² = residualᵀ * S⁻¹ * residual
                let d_squared = residual.transpose() * s_inv * residual;
                d_squared[(0, 0)] // Extract scalar from 1×1 matrix
            }
            None => {
                // Singular covariance - treat as infinite distance
                f64::MAX
            }
        }
    }
    
    /// Perform geometric gating on a set of candidate tracks.
    /// 
    /// Returns tracks that pass the Chi-squared test, sorted by Mahalanobis distance.
    /// 
    /// Hard gating rules:
    /// 1. Class ID must match (pedestrians don't associate with vehicles)
    /// 2. Mahalanobis distance² must be below threshold
    pub fn gate_candidates(
        &self,
        candidates: &HashSet<Uuid>,
        packet: &GlobalHazardPacket,
    ) -> Vec<(Uuid, f64)> {
        let mut gated: Vec<(Uuid, f64)> = candidates
            .iter()
            .filter_map(|&track_id| {
                let track = self.tracks.get(&track_id)?;
                
                // Hard gate: class must match
                if track.class_id != packet.class_id {
                    return None;
                }
                
                // Soft gate: Mahalanobis distance
                let d_squared = self.mahalanobis_distance_squared(track, packet);
                
                if d_squared < self.config.gating_threshold {
                    Some((track_id, d_squared))
                } else {
                    None
                }
            })
            .collect();
        
        // Sort by distance (ascending) for GNN selection
        gated.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        
        gated
    }
    
    /// Select the best match using Global Nearest Neighbor (GNN).
    /// 
    /// Returns the track ID with the smallest Mahalanobis distance,
    /// or None if no candidates passed gating.
    pub fn select_best_match(&self, gated_candidates: &[(Uuid, f64)]) -> Option<Uuid> {
        gated_candidates.first().map(|(id, _)| *id)
    }
    
    /// Perform spatial pruning and geometric gating for an incoming packet.
    /// 
    /// This combines Stage 1 (Spatial Pruning) and Stage 2 (Geometric Gating)
    /// of the processing pipeline.
    pub fn find_association(&self, packet: &GlobalHazardPacket) -> Result<Option<Uuid>, TrackingError> {
        // Stage 1: Spatial Pruning using H3 k-ring
        let packet_cell = self.position_to_cell(packet.position[0], packet.position[1])?;
        let candidates = self.spatial_query_kring(packet_cell, 1);
        
        if candidates.is_empty() {
            return Ok(None);
        }
        
        // Stage 2: Geometric Gating (Mahalanobis + class check)
        let gated = self.gate_candidates(&candidates, packet);
        
        // GNN: Select best match
        Ok(self.select_best_match(&gated))
    }
    
    // ========================================================================
    // ACCESSORS
    // ========================================================================
    
    /// Get a reference to a track by its canonical ID.
    pub fn get_track(&self, id: &Uuid) -> Option<&UniqueTrack> {
        self.tracks.get(id)
    }
    
    /// Get a mutable reference to a track by its canonical ID.
    pub fn get_track_mut(&mut self, id: &Uuid) -> Option<&mut UniqueTrack> {
        self.tracks.get_mut(id)
    }
    
    /// Get all tracks as an iterator.
    pub fn tracks(&self) -> impl Iterator<Item = &UniqueTrack> {
        self.tracks.values()
    }
    
    /// Get the number of active tracks.
    pub fn track_count(&self) -> usize {
        self.tracks.len()
    }
    
    /// Get statistics about the spatial index.
    pub fn spatial_stats(&self) -> SpatialIndexStats {
        let total_cells = self.spatial_index.len();
        let total_entries: usize = self.spatial_index.values().map(|s| s.len()).sum();
        let avg_per_cell = if total_cells > 0 {
            total_entries as f64 / total_cells as f64
        } else {
            0.0
        };
        
        SpatialIndexStats {
            total_cells,
            total_entries,
            avg_per_cell,
        }
    }

    /// Get average Peer Agreement Cost (J_PA) over the rolling window.
    /// Used for blind fitness evaluation.
    pub fn get_peer_agreement_cost(&self) -> f64 {
        if self.peer_agreement_history.is_empty() {
            return 0.0;
        }
        let sum: f64 = self.peer_agreement_history.iter().sum();
        sum / self.peer_agreement_history.len() as f64
    }
    
    // ========================================================================
    // PHASE 3 & 4: FUSION ENGINE (Highlander + Covariance Intersection)
    // ========================================================================
    
    /// Perform Covariance Intersection fusion of two state estimates.
    /// 
    /// This algorithm is loop-safe: even if the two estimates are fully correlated
    /// (e.g., same data recirculating in the network), the result will not have
    /// artificially reduced covariance.
    /// 
    /// Uses Fast-CI with Trace Minimization:
    ///   ω = tr(P_B) / (tr(P_A) + tr(P_B))
    /// 
    /// Returns (fused_state, fused_covariance), or None if matrices are singular.
    pub fn covariance_intersection(
        x_a: &Vector6<f64>,
        p_a: &Matrix6<f64>,
        x_b: &Vector6<f64>,
        p_b: &Matrix6<f64>,
    ) -> Option<(Vector6<f64>, Matrix6<f64>)> {
        // Step 1: Compute weight using trace minimization heuristic
        let tr_a = p_a.trace();
        let tr_b = p_b.trace();
        let sum_tr = tr_a + tr_b;
        
        // Avoid division by zero
        if sum_tr < 1e-12 {
            return None;
        }
        
        // ω gives more weight to the estimate with smaller trace (lower uncertainty)
        let omega = tr_b / sum_tr;
        
        // Step 2: Compute information matrices (inverse covariances)
        let p_a_inv = p_a.try_inverse()?;
        let p_b_inv = p_b.try_inverse()?;
        
        // Step 3: Fused information matrix
        // P_CI^{-1} = ω * P_A^{-1} + (1-ω) * P_B^{-1}
        let p_ci_inv = p_a_inv * omega + p_b_inv * (1.0 - omega);
        
        // Step 4: Recover fused covariance
        let p_ci = p_ci_inv.try_inverse()?;
        
        // Step 5: Fused state
        // x_CI = P_CI * (ω * P_A^{-1} * x_A + (1-ω) * P_B^{-1} * x_B)
        let info_a = p_a_inv * x_a * omega;
        let info_b = p_b_inv * x_b * (1.0 - omega);
        let x_ci = p_ci * (info_a + info_b);
        
        Some((x_ci, p_ci))
    }
    
    /// Fuse an incoming packet with an existing track using Covariance Intersection.
    /// 
    /// This also applies the Highlander heuristic for ID resolution and updates
    /// the spatial index if the track moves to a new cell.
    /// 
    /// If the canonical_id changes (when the incoming packet has a smaller UUID),
    /// the track will be rekeyed in the HashMap.
    /// 
    /// Returns the new canonical_id (which may have changed due to Highlander merge).
    fn fuse_track(
        &mut self,
        track_id: Uuid,
        packet: &GlobalHazardPacket,
        adaptive_state: Option<&AdaptiveState>,
        neighbor_id: Option<usize>,
    ) -> Result<Uuid, TrackingError> {
        // Get the track
        let track = self.tracks.get(&track_id)
            .ok_or(TrackingError::TrackNotFound(track_id))?;
        
        let old_cell = track.h3_cell;
        let old_canonical_id = track.canonical_id;
        
        // Construct measurement state vector
        let x_meas = Vector6::new(
            packet.position[0],
            packet.position[1],
            packet.position[2],
            packet.velocity[0],
            packet.velocity[1],
            packet.velocity[2],
        );
        
        // Construct measurement covariance
        let p_meas = self.confidence_to_covariance(packet.confidence_score);
        
        // Perform Covariance Intersection
        let (x_fused, p_fused) = Self::covariance_intersection(
            &track.state,
            &track.covariance,
            &x_meas,
            &p_meas,
        ).ok_or(TrackingError::SingularCovariance)?;
        
        // Compute new cell for potentially updated position
        let new_cell = self.position_to_cell(x_fused[0], x_fused[1])?;
        
        // Now we can mutably borrow the track
        let track = self.tracks.get_mut(&track_id).unwrap();
        
        // Update state and covariance
        track.state = x_fused;
        track.covariance = p_fused;
        track.last_update = packet.timestamp;
        track.age = 0;
        track.h3_cell = new_cell;
        
        // --- Blind Fitness Instrumentation (Peer Agreement) ---
        // If this packet came from a neighbor, calculate weighted disagreement
        if let (Some(state), Some(nid)) = (adaptive_state, neighbor_id) {
             // 1. Calculate distance d_ij (Euclidean distance between track and packet)
             let dist = (track.state - x_meas).norm();
             
             // 2. Get neighbor reputation weight w_ij
             // We access the reputation from the passed AdaptiveState
             if let Some(rep) = state.neighbor_reputations.get(&nid) {
                 let w_ij = rep.reliability_score;
                 
                 // 3. Store weighted disagreement: w_ij * d_ij
                 // Note: This is an instantaneous sample. The full J_PA aggregator 
                 // usually sums over all neighbors. Here we store individual samples
                 // which get averaged over time. 
                 
                 // Only count if weight is significant to avoid noise from bad actors
                 if w_ij > 0.1 {
                     let weighted_agreement = w_ij * dist;
                     
                     if self.peer_agreement_history.len() >= self.pa_window_size {
                        self.peer_agreement_history.pop_front();
                     }
                     self.peer_agreement_history.push_back(weighted_agreement);
                 }
             }
        }
        // ----------------------------------------------------

        // Stage 3: Highlander ID Resolution
        track.merge_id(packet.entity_id);
        let new_canonical_id = track.canonical_id;
        
        // Update spatial index if cell changed
        if old_cell != new_cell {
            self.spatial_index_remove(old_cell, track_id);
            self.spatial_index_insert(new_cell, track_id);
        }
        
        // Critical: If canonical_id changed, we need to rekey the track in the HashMap
        // and update the spatial index to point to the new key
        if new_canonical_id != old_canonical_id {
            // Remove track from HashMap, update its key, and reinsert
            if let Some(track) = self.tracks.remove(&track_id) {
                // Update spatial index with new key
                self.spatial_index_remove(track.h3_cell, track_id);
                self.spatial_index_insert(track.h3_cell, new_canonical_id);
                
                // Reinsert track under new canonical_id
                self.tracks.insert(new_canonical_id, track);
            }
        }
        
        // Return the new canonical_id (may have changed due to Highlander merge)
        Ok(new_canonical_id)
    }
    
    /// Process an incoming GlobalHazardPacket through the full 4-stage pipeline.
    /// 
    /// **Stage 1:** Spatial Pruning (H3 k-ring query)
    /// **Stage 2:** Geometric Gating (Mahalanobis distance + class check)
    /// **Stage 3:** Identity Resolution (Highlander min-UUID)
    /// **Stage 4:** State Fusion (Covariance Intersection)
    /// 
    /// Returns the canonical track ID (either existing or newly created).
    pub fn process_packet(
        &mut self, 
        packet: &GlobalHazardPacket,
        adaptive_state: Option<&AdaptiveState>,
        neighbor_id: Option<usize>
    ) -> Result<Uuid, TrackingError> {
        // Stages 1 & 2: Find association
        match self.find_association(packet)? {
            Some(track_id) => {
                // Stages 3 & 4: Fuse with existing track
                // fuse_track returns the (possibly updated) canonical_id
                let canonical_id = self.fuse_track(track_id, packet, adaptive_state, neighbor_id)?;
                Ok(canonical_id)
            }
            None => {
                // No match: Create new track
                self.create_track(packet)
            }
        }
    }
    
    /// Process multiple packets (batch processing).
    /// 
    /// Returns a vector of (result, original_packet_entity_id) tuples.
    pub fn process_packets(&mut self, packets: &[GlobalHazardPacket]) -> Vec<(Result<Uuid, TrackingError>, Uuid)> {
        packets
            .iter()
            .map(|packet| (self.process_packet(packet, None, None), packet.entity_id))
            .collect()
    }
}

// ============================================================================
// STATISTICS
// ============================================================================

/// Statistics about the spatial index.
#[derive(Debug, Clone)]
pub struct SpatialIndexStats {
    pub total_cells: usize,
    pub total_entries: usize,
    pub avg_per_cell: f64,
}

// ============================================================================
// ERRORS
// ============================================================================

/// Errors that can occur during tracking operations.
#[derive(Debug, Clone, thiserror::Error)]
pub enum TrackingError {
    #[error("Invalid coordinates: {0}")]
    InvalidCoordinates(String),
    
    #[error("Track not found: {0}")]
    TrackNotFound(Uuid),
    
    #[error("Covariance matrix is singular")]
    SingularCovariance,
    
    #[error("Gating failed: no candidates within threshold")]
    GatingFailed,
}

// ============================================================================
// TESTS
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    
    fn sample_packet() -> GlobalHazardPacket {
        GlobalHazardPacket {
            entity_id: Uuid::new_v4(),
            position: [37.7749, -122.4194, 10.0], // San Francisco
            velocity: [1.0, 0.0, 0.0],
            class_id: 1, // Vehicle
            timestamp: 1703001600.0,
            confidence_score: 0.9,
        }
    }
    
    #[test]
    fn test_track_manager_creation() {
        let manager = TrackManager::with_defaults();
        assert_eq!(manager.track_count(), 0);
    }
    
    #[test]
    fn test_create_track() {
        let mut manager = TrackManager::with_defaults();
        let packet = sample_packet();
        
        let result = manager.create_track(&packet);
        assert!(result.is_ok());
        
        assert_eq!(manager.track_count(), 1);
        
        // Verify spatial index contains the track
        let stats = manager.spatial_stats();
        assert_eq!(stats.total_entries, 1);
    }
    
    #[test]
    fn test_spatial_query_kring() {
        let mut manager = TrackManager::with_defaults();
        let packet = sample_packet();
        
        let track_id = manager.create_track(&packet).unwrap();
        
        // Get the cell for the track
        let cell = manager.position_to_cell(packet.position[0], packet.position[1]).unwrap();
        
        // Query with k=0 (just the cell itself)
        let results = manager.spatial_query_kring(cell, 0);
        assert!(results.contains(&track_id));
        
        // Query with k=1 (cell + neighbors)
        let results = manager.spatial_query_kring(cell, 1);
        assert!(results.contains(&track_id));
    }
    
    #[test]
    fn test_highlander_merge_id() {
        let packet = sample_packet();
        let cell = LatLng::new(packet.position[0], packet.position[1])
            .unwrap()
            .to_cell(Resolution::Ten);
        
        let covariance = Matrix6::identity();
        let mut track = UniqueTrack::from_packet(&packet, covariance, cell);
        
        let original_id = track.canonical_id;
        
        // Merge with a larger UUID - should not change canonical
        let larger_id = Uuid::max();
        track.merge_id(larger_id);
        assert_eq!(track.canonical_id, original_id);
        assert!(track.observed_ids.contains(&larger_id));
        
        // Merge with a smaller UUID - should become canonical
        let smaller_id = Uuid::nil();
        track.merge_id(smaller_id);
        assert_eq!(track.canonical_id, smaller_id);
        assert!(track.observed_ids.contains(&smaller_id));
        assert!(track.observed_ids.contains(&original_id));
    }
    
    #[test]
    fn test_confidence_to_covariance() {
        let manager = TrackManager::with_defaults();
        
        // High confidence → low variance
        let high_conf = manager.confidence_to_covariance(0.99);
        let low_conf = manager.confidence_to_covariance(0.1);
        
        assert!(high_conf[(0, 0)] < low_conf[(0, 0)]);
    }
    
    #[test]
    fn test_track_aging() {
        let mut manager = TrackManager::new(TrackingConfig {
            max_age: 3,
            ..Default::default()
        });
        
        let packet = sample_packet();
        manager.create_track(&packet).unwrap();
        
        assert_eq!(manager.track_count(), 1);
        
        // Age 3 times
        manager.age_tracks();
        assert_eq!(manager.track_count(), 1);
        manager.age_tracks();
        assert_eq!(manager.track_count(), 1);
        manager.age_tracks();
        // Now age == max_age, should be removed
        assert_eq!(manager.track_count(), 0);
    }
    
    #[test]
    fn test_covariance_intersection_basic() {
        // Two estimates with equal covariance → should average the states
        let x_a = Vector6::new(10.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let x_b = Vector6::new(20.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let p = Matrix6::identity() * 4.0; // Equal covariance
        
        let result = TrackManager::covariance_intersection(&x_a, &p, &x_b, &p);
        assert!(result.is_some());
        
        let (x_fused, _p_fused) = result.unwrap();
        
        // With equal weights, should be near the average
        assert!((x_fused[0] - 15.0).abs() < 1.0);
    }
    
    #[test]
    fn test_covariance_intersection_weights_precise() {
        // One precise estimate, one imprecise → fused should favor precise
        let x_precise = Vector6::new(10.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let x_imprecise = Vector6::new(20.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        
        let p_precise = Matrix6::identity() * 0.1;   // Very precise
        let p_imprecise = Matrix6::identity() * 100.0; // Very imprecise
        
        let result = TrackManager::covariance_intersection(
            &x_precise, &p_precise,
            &x_imprecise, &p_imprecise,
        );
        assert!(result.is_some());
        
        let (x_fused, _p_fused) = result.unwrap();
        
        // Fused should be much closer to the precise estimate
        assert!((x_fused[0] - 10.0).abs() < 2.0);
    }
    
    #[test]
    fn test_covariance_intersection_rumor_safety() {
        // CRITICAL TEST: Fusing an estimate with itself should NOT reduce covariance
        // This validates that CI is loop-safe (no rumor propagation)
        
        let x = Vector6::new(10.0, 20.0, 30.0, 1.0, 2.0, 0.0);
        let p = Matrix6::from_diagonal(&Vector6::new(4.0, 4.0, 4.0, 1.0, 1.0, 1.0));
        let original_trace = p.trace();
        
        // Fuse with exact same data (simulating looped rumor)
        let result = TrackManager::covariance_intersection(&x, &p, &x, &p);
        assert!(result.is_some());
        
        let (_x_fused, p_fused) = result.unwrap();
        let fused_trace = p_fused.trace();
        
        // INVARIANT: Fused covariance should NOT be smaller than original
        // (CI guarantees this even for fully correlated data)
        assert!(
            fused_trace >= original_trace * 0.99, // Allow tiny numerical error
            "Rumor propagation detected! Trace {} < original {}",
            fused_trace, original_trace
        );
    }
    
    #[test]
    fn test_process_packet_creates_new_track() {
        let mut manager = TrackManager::with_defaults();
        let packet = sample_packet();
        
        let result = manager.process_packet(&packet, None, None);
        assert!(result.is_ok());
        
        let track_id = result.unwrap();
        assert_eq!(track_id, packet.entity_id);
        assert_eq!(manager.track_count(), 1);
    }
    
    #[test]
    fn test_process_packet_associates_with_existing() {
        let mut manager = TrackManager::with_defaults();
        
        // First packet creates a track
        let packet1 = GlobalHazardPacket {
            entity_id: Uuid::new_v4(),
            position: [37.7749, -122.4194, 10.0],
            velocity: [1.0, 0.0, 0.0],
            class_id: 1,
            timestamp: 1703001600.0,
            confidence_score: 0.9,
        };
        let track_id1 = manager.process_packet(&packet1, None, None).unwrap();
        
        // Second packet at very similar position should associate
        let packet2 = GlobalHazardPacket {
            entity_id: Uuid::new_v4(), // Different UUID
            position: [37.7749, -122.4194, 10.5], // Slightly different altitude
            velocity: [1.1, 0.0, 0.0],
            class_id: 1, // Same class
            timestamp: 1703001601.0,
            confidence_score: 0.85,
        };
        let track_id2 = manager.process_packet(&packet2, None, None).unwrap();
        
        // Should still be 1 track (associated)
        assert_eq!(manager.track_count(), 1);
        
        // The canonical ID should be the smaller of the two
        let expected_canonical = std::cmp::min(packet1.entity_id, packet2.entity_id);
        assert_eq!(track_id2, expected_canonical);
        
        // Track should have both IDs in observed_ids
        let track = manager.get_track(&track_id2).unwrap();
        assert!(track.observed_ids.contains(&packet1.entity_id));
        assert!(track.observed_ids.contains(&packet2.entity_id));
    }
    
    #[test]
    fn test_process_packet_no_association_different_class() {
        let mut manager = TrackManager::with_defaults();
        
        // First packet: Vehicle
        let packet1 = GlobalHazardPacket {
            entity_id: Uuid::new_v4(),
            position: [37.7749, -122.4194, 10.0],
            velocity: [1.0, 0.0, 0.0],
            class_id: 1, // Vehicle
            timestamp: 1703001600.0,
            confidence_score: 0.9,
        };
        manager.process_packet(&packet1, None, None).unwrap();
        
        // Second packet: Pedestrian at same location
        let packet2 = GlobalHazardPacket {
            entity_id: Uuid::new_v4(),
            position: [37.7749, -122.4194, 10.0],
            velocity: [0.5, 0.0, 0.0],
            class_id: 2, // Pedestrian (different class!)
            timestamp: 1703001600.0,
            confidence_score: 0.9,
        };
        manager.process_packet(&packet2, None, None).unwrap();
        
        // Should be 2 separate tracks (class mismatch prevents association)
        assert_eq!(manager.track_count(), 2);
    }
    
    #[test]
    fn test_mahalanobis_gating() {
        let mut manager = TrackManager::with_defaults();
        
        // Create a track
        let packet1 = sample_packet();
        manager.create_track(&packet1).unwrap();
        
        // Packet very far away should not pass gating
        let far_packet = GlobalHazardPacket {
            entity_id: Uuid::new_v4(),
            position: [40.0, -120.0, 100.0], // Very far from San Francisco
            velocity: [0.0, 0.0, 0.0],
            class_id: 1,
            timestamp: 1703001600.0,
            confidence_score: 0.9,
        };
        
        let association = manager.find_association(&far_packet).unwrap();
        assert!(association.is_none(), "Far packet should not associate");
    }
}

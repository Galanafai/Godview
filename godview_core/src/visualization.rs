//! Visualization module for GodView using Rerun.io
//!
//! This module provides real-time visualization of:
//! - 6D Gaussian uncertainty ellipsoids (position + velocity covariance)
//! - Agent communication packets
//! - Highlander CRDT merge events
//! - Trust verification status
//!
//! Enable with the `visualization` feature flag.

use crate::godview_tracking::GlobalHazardPacket;
use nalgebra::{Matrix3, Matrix6};
use rerun::{RecordingStream, RecordingStreamBuilder};
use uuid::Uuid;

/// Rerun-based visualizer for GodView distributed sensor fusion
pub struct RerunVisualizer {
    /// The underlying Rerun recording stream (public for advanced use)
    pub rec: RecordingStream,
}

impl RerunVisualizer {
    /// Create a new visualizer that spawns the Rerun viewer
    pub fn new(app_id: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let rec = RecordingStreamBuilder::new(app_id)
            .spawn()?;
        
        // Log initial setup
        rec.log_static(
            "world",
            &rerun::ViewCoordinates::RIGHT_HAND_Z_UP(),
        )?;
        
        Ok(Self { rec })
    }
    
    /// Create a visualizer that saves to a file (for web sharing)
    pub fn new_to_file(app_id: &str, path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let rec = RecordingStreamBuilder::new(app_id)
            .save(path)?;
        
        rec.log_static(
            "world",
            &rerun::ViewCoordinates::RIGHT_HAND_Z_UP(),
        )?;
        
        Ok(Self { rec })
    }
    
    /// Get access to the underlying recording stream for direct Rerun API access
    pub fn recording(&self) -> &RecordingStream {
        &self.rec
    }
    
    /// Log a track with its 6D Gaussian uncertainty ellipsoid
    pub fn log_track(
        &self,
        track_id: Uuid,
        position: [f64; 3],
        velocity: [f64; 3],
        covariance: &Matrix6<f64>,
        entity_type: &str,
    ) -> Result<(), Box<dyn std::error::Error>> {
        // Extract position covariance (upper-left 3x3)
        let pos_cov: Matrix3<f64> = covariance.fixed_view::<3, 3>(0, 0).into();
        
        // Eigen decomposition for ellipsoid axes
        let eigen = pos_cov.symmetric_eigen();
        let half_sizes: [f32; 3] = [
            (eigen.eigenvalues[0].abs().sqrt() * 2.0) as f32, // 2-sigma
            (eigen.eigenvalues[1].abs().sqrt() * 2.0) as f32,
            (eigen.eigenvalues[2].abs().sqrt() * 2.0) as f32,
        ];
        
        // Calculate rotation quaternion from eigenvectors
        let rotation = nalgebra::UnitQuaternion::from_matrix(&eigen.eigenvectors);
        let quat = rotation.as_ref();
        
        let path = format!("world/tracks/{}", track_id);
        
        // Log the uncertainty ellipsoid
        self.rec.log(
            format!("{}/ellipsoid", path),
            &rerun::Ellipsoids3D::from_centers_and_half_sizes(
                [[position[0] as f32, position[1] as f32, position[2] as f32]],
                [half_sizes],
            )
            .with_quaternions([[quat.w as f32, quat.i as f32, quat.j as f32, quat.k as f32]])
            .with_colors([[0, 255, 200, 80]]) // Cyan with transparency
            .with_fill_mode(rerun::FillMode::Solid)
        )?;
        
        // Log the center point
        self.rec.log(
            format!("{}/center", path),
            &rerun::Points3D::new([[position[0] as f32, position[1] as f32, position[2] as f32]])
                .with_colors([[255, 255, 255, 255]]) // White
                .with_radii([0.1])
        )?;
        
        // Log velocity vector
        let vel_magnitude = (velocity[0].powi(2) + velocity[1].powi(2) + velocity[2].powi(2)).sqrt();
        if vel_magnitude > 0.01 {
            self.rec.log(
                format!("{}/velocity", path),
                &rerun::Arrows3D::from_vectors([[
                    velocity[0] as f32,
                    velocity[1] as f32,
                    velocity[2] as f32,
                ]])
                .with_origins([[position[0] as f32, position[1] as f32, position[2] as f32]])
                .with_colors([[255, 200, 0, 255]]) // Yellow
            )?;
        }
        
        // Log entity type as text
        self.rec.log(
            format!("{}/label", path),
            &rerun::TextLog::new(format!("{}: {}", entity_type, &track_id.to_string()[..8]))
        )?;
        
        Ok(())
    }
    
    /// Log a simplified track from a hazard packet
    pub fn log_packet_detection(
        &self,
        packet: &GlobalHazardPacket,
        uncertainty: f32,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let path = format!("world/detections/{}", packet.entity_id);
        
        self.rec.log(
            path,
            &rerun::Points3D::new([[
                packet.position[0] as f32,
                packet.position[1] as f32,
                packet.position[2] as f32,
            ]])
            .with_colors([[255, 100, 100, 200]]) // Red-ish
            .with_radii([uncertainty])
        )?;
        
        Ok(())
    }
    
    /// Log a data packet traveling between agents
    pub fn log_data_packet(
        &self,
        from: [f64; 3],
        to: [f64; 3],
        packet_id: &str,
    ) -> Result<(), Box<dyn std::error::Error>> {
        self.rec.log(
            format!("world/packets/{}", packet_id),
            &rerun::Arrows3D::from_vectors([[
                (to[0] - from[0]) as f32,
                (to[1] - from[1]) as f32,
                (to[2] - from[2]) as f32,
            ]])
            .with_origins([[from[0] as f32, from[1] as f32, from[2] as f32]])
            .with_colors([[0, 212, 255, 200]]) // Cyan
        )?;
        
        Ok(())
    }
    
    /// Log a Highlander CRDT merge event
    pub fn log_highlander_merge(
        &self,
        old_id: Uuid,
        new_id: Uuid,
        num_sources: usize,
    ) -> Result<(), Box<dyn std::error::Error>> {
        self.rec.log(
            "logs/crdt",
            &rerun::TextLog::new(format!(
                "🏆 HIGHLANDER: {} → {} ({} sources merged)",
                &old_id.to_string()[..8],
                &new_id.to_string()[..8],
                num_sources
            ))
        )?;
        
        Ok(())
    }
    
    /// Log trust verification status
    pub fn log_trust_event(
        &self,
        agent_id: &str,
        verified: bool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let status = if verified { "✓ VERIFIED" } else { "✗ REJECTED" };
        let color = if verified { "green" } else { "red" };
        
        self.rec.log(
            "logs/trust",
            &rerun::TextLog::new(format!("🔐 {}: {} ({})", agent_id, status, color))
        )?;
        
        Ok(())
    }
    
    /// Log H3 spatial cell activation
    pub fn log_h3_cell(
        &self,
        cell_index: u64,
        center: [f64; 3],
        active: bool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let color = if active { [0, 255, 136, 100] } else { [50, 50, 50, 50] };
        
        self.rec.log(
            format!("world/h3/{:x}", cell_index),
            &rerun::Points3D::new([[center[0] as f32, center[1] as f32, center[2] as f32]])
                .with_colors([color])
                .with_radii([1.0])
        )?;
        
        Ok(())
    }
    
    /// Log uncertainty reduction stats
    pub fn log_stats(
        &self,
        total_tracks: usize,
        avg_uncertainty: f64,
        reduction_percent: f64,
    ) -> Result<(), Box<dyn std::error::Error>> {
        self.rec.log(
            "stats/tracks",
            &rerun::Scalars::new([total_tracks as f64]),
        )?;
        
        self.rec.log(
            "stats/uncertainty",
            &rerun::Scalars::new([avg_uncertainty]),
        )?;
        
        self.rec.log(
            "stats/reduction",
            &rerun::Scalars::new([reduction_percent]),
        )?;
        
        Ok(())
    }
    
    /// Log a ground plane grid for scene context
    pub fn log_ground_plane(&self, size: f32, divisions: usize) -> Result<(), Box<dyn std::error::Error>> {
        let mut points = Vec::new();
        let step = size / divisions as f32;
        
        // Create grid points
        for i in 0..=divisions {
            let coord = -size / 2.0 + i as f32 * step;
            // Along X
            points.push([coord, -size / 2.0, 0.0]);
            points.push([coord, size / 2.0, 0.0]);
            // Along Y
            points.push([-size / 2.0, coord, 0.0]);
            points.push([size / 2.0, coord, 0.0]);
        }
        
        self.rec.log_static(
            "world/ground/grid",
            &rerun::LineStrips3D::new(
                points.chunks(2).map(|c| c.to_vec()).collect::<Vec<_>>()
            )
            .with_colors([[60, 60, 60, 100]]) // Dark gray
        )?;
        
        Ok(())
    }
    
    /// Log an agent (vehicle/drone) as a 3D box at a position with sensor range visualization
    pub fn log_agent(
        &self,
        agent_name: &str,
        position: [f64; 3],
        size: [f32; 3], // [length, width, height]
        color: [u8; 4],
        is_drone: bool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let height_offset = if is_drone { position[2] } else { size[2] as f64 / 2.0 };
        let agent_path = format!("world/agents/{}", agent_name.replace(" ", "_").replace("(", "").replace(")", ""));
        
        // Log the agent box (BIGGER for visibility)
        let visual_size = [size[0] * 2.0, size[1] * 2.0, size[2] * 2.0]; // 2x size
        self.rec.log(
            format!("{}/body", agent_path),
            &rerun::Boxes3D::from_centers_and_sizes(
                [[position[0] as f32, position[1] as f32, height_offset as f32]],
                [visual_size],
            )
            .with_colors([color])
            .with_labels([agent_name])
        )?;
        
        // Log a vertical beam from agent to ground (makes it easy to see)
        self.rec.log(
            format!("{}/beam", agent_path),
            &rerun::LineStrips3D::new([[
                [position[0] as f32, position[1] as f32, 0.0],
                [position[0] as f32, position[1] as f32, height_offset as f32],
            ]])
            .with_colors([[color[0], color[1], color[2], 100]]) // Semi-transparent
            .with_radii([0.1])
        )?;
        
        // Log agent name as 3D text above the agent
        self.rec.log(
            format!("{}/label", agent_path),
            &rerun::Points3D::new([[position[0] as f32, position[1] as f32, (height_offset + 3.0) as f32]])
                .with_colors([color])
                .with_radii([0.3])
                .with_labels([agent_name])
        )?;
        
        Ok(())
    }
    
    /// Log a detection line from an agent to a detected object
    pub fn log_detection_line(
        &self,
        agent_name: &str,
        agent_pos: [f64; 3],
        object_pos: [f64; 3],
        color: [u8; 4],
    ) -> Result<(), Box<dyn std::error::Error>> {
        let agent_path = format!("world/agents/{}", agent_name.replace(" ", "_").replace("(", "").replace(")", ""));
        
        self.rec.log(
            format!("{}/detections", agent_path),
            &rerun::LineStrips3D::new([[
                [agent_pos[0] as f32, agent_pos[1] as f32, agent_pos[2] as f32],
                [object_pos[0] as f32, object_pos[1] as f32, object_pos[2] as f32],
            ]])
            .with_colors([[color[0], color[1], color[2], 30]]) // Very transparent
        )?;
        
        Ok(())
    }
    
    /// Log a sensor range circle on the ground plane
    pub fn log_sensor_range(
        &self,
        agent_name: &str,
        center: [f64; 3],
        range: f32,
        color: [u8; 4],
    ) -> Result<(), Box<dyn std::error::Error>> {
        let agent_path = format!("world/agents/{}", agent_name.replace(" ", "_").replace("(", "").replace(")", ""));
        
        // Draw circle as line strip (32 segments)
        let segments = 32;
        let mut points: Vec<[f32; 3]> = Vec::with_capacity(segments + 1);
        for i in 0..=segments {
            let angle = (i as f64 / segments as f64) * std::f64::consts::TAU;
            points.push([
                (center[0] + range as f64 * angle.cos()) as f32,
                (center[1] + range as f64 * angle.sin()) as f32,
                0.1, // Slightly above ground
            ]);
        }
        
        // Convert to pairs for line strip
        let pairs: Vec<[[f32; 3]; 2]> = points.windows(2)
            .map(|w| [w[0], w[1]])
            .collect();
        
        self.rec.log_static(
            format!("{}/range", agent_path),
            &rerun::LineStrips3D::new(pairs)
                .with_colors([[color[0], color[1], color[2], 80]]) // Semi-transparent
        )?;
        
        Ok(())
    }
    
    /// Log road segments for scene context
    pub fn log_road(&self, from: [f32; 2], to: [f32; 2], width: f32) -> Result<(), Box<dyn std::error::Error>> {
        // Calculate road direction
        let dx = to[0] - from[0];
        let dy = to[1] - from[1];
        let length = (dx * dx + dy * dy).sqrt();
        let center_x = (from[0] + to[0]) / 2.0;
        let center_y = (from[1] + to[1]) / 2.0;
        
        // Rotation angle
        let angle = dy.atan2(dx);
        let quat = nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, angle as f64);
        let q = quat.as_ref();
        
        self.rec.log_static(
            format!("world/roads/{:.0}_{:.0}_{:.0}_{:.0}", from[0], from[1], to[0], to[1]),
            &rerun::Boxes3D::from_centers_and_sizes(
                [[center_x, center_y, 0.01]], // Slightly above ground
                [[length, width, 0.02]],
            )
            .with_quaternions([[q.w as f32, q.i as f32, q.j as f32, q.k as f32]])
            .with_colors([[40, 40, 45, 255]]) // Dark asphalt gray
        )?;
        
        Ok(())
    }
    
    /// Log a track with custom color for the ellipsoid
    pub fn log_track_colored(
        &self,
        track_id: Uuid,
        position: [f64; 3],
        velocity: [f64; 3],
        covariance: &Matrix6<f64>,
        label: &str,
        color: [u8; 4],
    ) -> Result<(), Box<dyn std::error::Error>> {
        // Extract position covariance (upper-left 3x3)
        let pos_cov: Matrix3<f64> = covariance.fixed_view::<3, 3>(0, 0).into();
        
        // Eigen decomposition for ellipsoid axes
        let eigen = pos_cov.symmetric_eigen();
        let half_sizes: [f32; 3] = [
            (eigen.eigenvalues[0].abs().sqrt() * 2.0) as f32, // 2-sigma
            (eigen.eigenvalues[1].abs().sqrt() * 2.0) as f32,
            (eigen.eigenvalues[2].abs().sqrt() * 2.0) as f32,
        ];
        
        // Calculate rotation quaternion from eigenvectors
        let rotation = nalgebra::UnitQuaternion::from_matrix(&eigen.eigenvectors);
        let quat = rotation.as_ref();
        
        let path = format!("world/tracks/{}", label.replace(" ", "_"));
        
        // Log the uncertainty ellipsoid with custom color
        self.rec.log(
            format!("{}/ellipsoid", path),
            &rerun::Ellipsoids3D::from_centers_and_half_sizes(
                [[position[0] as f32, position[1] as f32, position[2] as f32]],
                [half_sizes],
            )
            .with_quaternions([[quat.w as f32, quat.i as f32, quat.j as f32, quat.k as f32]])
            .with_colors([color])
            .with_fill_mode(rerun::FillMode::Solid)
        )?;
        
        // Log the center point
        self.rec.log(
            format!("{}/center", path),
            &rerun::Points3D::new([[position[0] as f32, position[1] as f32, position[2] as f32]])
                .with_colors([[255, 255, 255, 255]]) // White
                .with_radii([0.08])
        )?;
        
        // Log velocity vector
        let vel_magnitude = (velocity[0].powi(2) + velocity[1].powi(2) + velocity[2].powi(2)).sqrt();
        if vel_magnitude > 0.01 {
            self.rec.log(
                format!("{}/velocity", path),
                &rerun::Arrows3D::from_vectors([[
                    velocity[0] as f32,
                    velocity[1] as f32,
                    velocity[2] as f32,
                ]])
                .with_origins([[position[0] as f32, position[1] as f32, position[2] as f32]])
                .with_colors([[255, 200, 0, 255]]) // Yellow
            )?;
        }
        
        Ok(())
    }
    
    // ========================================================================
    // DEEP INSPECTION METHODS (Ghost Hunter, Tension, Merge Events)
    // ========================================================================
    
    /// Log a track with Ghost Hunter coloring based on ghost score.
    /// 
    /// Color scheme:
    /// - Green (< 0.3): Solid consensus
    /// - Orange (0.3 - 0.7): Ambiguous
    /// - Red (> 0.7): Probable ghost (pulsing effect)
    pub fn log_track_with_ghost_score(
        &self,
        track_id: Uuid,
        position: [f64; 3],
        velocity: [f64; 3],
        covariance: &Matrix6<f64>,
        ghost_score: f64,
        frame_idx: usize,
    ) -> Result<(), Box<dyn std::error::Error>> {
        // Map ghost score to color (Green -> Orange -> Red)
        let color: [u8; 4] = if ghost_score > 0.7 {
            // Red with pulsing alpha for high ghost scores
            let pulse = ((frame_idx as f64 * 0.5).sin() * 0.3 + 0.7) * 255.0;
            [255, 50, 50, pulse as u8]
        } else if ghost_score > 0.3 {
            // Orange for ambiguous
            [255, 165, 0, 180]
        } else {
            // Green for solid consensus
            [50, 255, 100, 150]
        };
        
        // Extract position covariance
        let pos_cov: Matrix3<f64> = covariance.fixed_view::<3, 3>(0, 0).into();
        let eigen = pos_cov.symmetric_eigen();
        let half_sizes: [f32; 3] = [
            (eigen.eigenvalues[0].abs().sqrt() * 2.0) as f32,
            (eigen.eigenvalues[1].abs().sqrt() * 2.0) as f32,
            (eigen.eigenvalues[2].abs().sqrt() * 2.0) as f32,
        ];
        
        let rotation = nalgebra::UnitQuaternion::from_matrix(&eigen.eigenvectors);
        let quat = rotation.as_ref();
        
        let path = format!("world/tracks/{}", track_id);
        
        // Log ellipsoid with ghost-hunter coloring
        self.rec.log(
            format!("{}/ellipsoid", path),
            &rerun::Ellipsoids3D::from_centers_and_half_sizes(
                [[position[0] as f32, position[1] as f32, position[2] as f32]],
                [half_sizes],
            )
            .with_quaternions([[quat.w as f32, quat.i as f32, quat.j as f32, quat.k as f32]])
            .with_colors([color])
            .with_fill_mode(rerun::FillMode::Solid)
            .with_labels([format!("👻 {:.2}", ghost_score)])
        )?;
        
        // Log ghost score to time series
        self.rec.log(
            format!("metrics/ghost_score/{}", track_id),
            &rerun::Scalars::new([ghost_score]),
        )?;
        
        Ok(())
    }
    
    /// Log a tension line showing contradiction between detection and fused belief.
    /// 
    /// Draws a magenta line from the detection to the fused track position.
    /// Only drawn when tension exceeds the significance threshold.
    pub fn log_tension_line(
        &self,
        agent_id: &str,
        detection_pos: [f64; 3],
        fused_pos: [f64; 3],
        tension: f64,
        threshold: f64,
    ) -> Result<(), Box<dyn std::error::Error>> {
        if tension <= threshold {
            return Ok(()); // Not significant
        }
        
        // Calculate line intensity based on tension severity
        let intensity = ((tension / threshold).min(3.0) / 3.0 * 255.0) as u8;
        let color = [255, 0, 255, intensity]; // Magenta with variable alpha
        
        self.rec.log(
            format!("world/debug/tension/{}", agent_id),
            &rerun::LineStrips3D::new([[
                [detection_pos[0] as f32, detection_pos[1] as f32, detection_pos[2] as f32],
                [fused_pos[0] as f32, fused_pos[1] as f32, fused_pos[2] as f32],
            ]])
            .with_colors([color])
            .with_radii([0.05])
        )?;
        
        // Log tension value
        self.rec.log(
            format!("metrics/tension/{}", agent_id),
            &rerun::Scalars::new([tension]),
        )?;
        
        Ok(())
    }
    
    /// Log a merge "pop" animation when Highlander resolves two tracks.
    /// 
    /// Creates a visual implosion effect at the merge location.
    pub fn log_merge_pop(
        &self,
        merge_event: &crate::godview_tracking::MergeEvent,
        frame_offset: usize,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let pos = merge_event.merge_position;
        
        // Implosion effect: decreasing radius over frames
        let radius = match frame_offset {
            0 => 3.0,
            1 => 2.5,
            2 => 2.0,
            3 => 1.5,
            4 => 1.0,
            _ => 0.0, // Clear after 5 frames
        };
        
        if radius > 0.0 {
            self.rec.log(
                "world/events/merge_pop",
                &rerun::Points3D::new([[pos[0] as f32, pos[1] as f32, pos[2] as f32]])
                    .with_colors([[0, 255, 255, 150]]) // Cyan
                    .with_radii([radius])
            )?;
        }
        
        // Log detailed text event
        self.rec.log(
            "logs/highlander",
            &rerun::TextLog::new(format!(
                "⚔️ MERGE: {} absorbed {} | Reason: {}",
                &merge_event.winner_id.to_string()[..8],
                &merge_event.loser_id.to_string()[..8],
                merge_event.reason
            ))
        )?;
        
        Ok(())
    }
    
    /// Log a node in the genealogy graph (for merge visualization).
    pub fn log_genealogy_node(
        &self,
        track_id: Uuid,
        node_type: &str, // "seed", "state", "merge"
        position_y: f64, // Time-based Y position (waterfall)
    ) -> Result<(), Box<dyn std::error::Error>> {
        let color = match node_type {
            "seed" => [100, 150, 255, 255],  // Blue
            "merge" => [0, 255, 255, 255],   // Cyan
            _ => [150, 150, 150, 255],       // Gray
        };
        
        self.rec.log(
            "genealogy/nodes",
            &rerun::Points3D::new([[0.0, position_y as f32, 0.0]])
                .with_colors([color])
                .with_radii([0.3])
                .with_labels([format!("{}", &track_id.to_string()[..8])])
        )?;
        
        Ok(())
    }
    
    /// Log an edge in the genealogy graph (merge relationship).
    pub fn log_genealogy_edge(
        &self,
        from_id: Uuid,
        to_id: Uuid,
        from_y: f64,
        to_y: f64,
    ) -> Result<(), Box<dyn std::error::Error>> {
        self.rec.log(
            format!("genealogy/edges/{}_{}", &from_id.to_string()[..8], &to_id.to_string()[..8]),
            &rerun::Arrows3D::from_vectors([[0.0, (to_y - from_y) as f32, 0.0]])
                .with_origins([[0.0, from_y as f32, 0.0]])
                .with_colors([[255, 100, 100, 200]]) // Red for merge direction
        )?;
        
        Ok(())
    }
    
    /// Log entropy reduction metric for a track.
    pub fn log_entropy(
        &self,
        track_id: Uuid,
        entropy: f64,
        entropy_reduction: f64,
    ) -> Result<(), Box<dyn std::error::Error>> {
        self.rec.log(
            format!("metrics/entropy/{}", track_id),
            &rerun::Scalars::new([entropy]),
        )?;
        
        self.rec.log(
            format!("metrics/entropy_reduction/{}", track_id),
            &rerun::Scalars::new([entropy_reduction]),
        )?;
        
        Ok(())
    }
    
    // ========================================================================
    // 3D ASSET METHODS (LiDAR, Meshes, Bounding Boxes)
    // ========================================================================
    
    /// Log a LiDAR point cloud
    ///
    /// Colors by intensity or height if no colors provided.
    pub fn log_lidar_pointcloud(
        &self,
        entity_path: &str,
        points: &[[f32; 3]],
        intensities: Option<&[f32]>, // Optional intensity values [0-1]
        color_by_height: bool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let colors: Vec<[u8; 4]> = if let Some(ints) = intensities {
            // Color by intensity (grayscale)
            ints.iter().map(|i| {
                let v = (i.clamp(0.0, 1.0) * 255.0) as u8;
                [v, v, v, 255]
            }).collect()
        } else if color_by_height {
            // Color by height (Z value) - blue=low, red=high
            let min_z = points.iter().map(|p| p[2]).fold(f32::INFINITY, f32::min);
            let max_z = points.iter().map(|p| p[2]).fold(f32::NEG_INFINITY, f32::max);
            let range = (max_z - min_z).max(0.1);
            
            points.iter().map(|p| {
                let t = ((p[2] - min_z) / range).clamp(0.0, 1.0);
                // Blue -> Cyan -> Green -> Yellow -> Red
                let r = (t * 2.0).min(1.0);
                let g = if t < 0.5 { t * 2.0 } else { 2.0 - t * 2.0 };
                let b = 1.0 - (t * 2.0).min(1.0);
                [(r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8, 255]
            }).collect()
        } else {
            // Default: white
            points.iter().map(|_| [200, 200, 200, 200]).collect()
        };
        
        self.rec.log(
            entity_path,
            &rerun::Points3D::new(points.to_vec())
                .with_colors(colors)
                .with_radii([0.02]) // Small points for LiDAR
        )?;
        
        Ok(())
    }
    
    /// Log a 3D object bounding box (for object detection results)
    ///
    /// Draws a wireframe box with label and confidence.
    pub fn log_3d_detection_box(
        &self,
        entity_path: &str,
        center: [f64; 3],
        size: [f32; 3],       // [length, width, height]
        yaw: f32,             // Rotation around Z axis in radians
        label: &str,
        confidence: f32,
        class_color: [u8; 4],
    ) -> Result<(), Box<dyn std::error::Error>> {
        // Create rotation quaternion from yaw
        let half_yaw = yaw / 2.0;
        let quat = [half_yaw.cos(), 0.0, 0.0, half_yaw.sin()]; // [w, x, y, z]
        
        // Adjust alpha based on confidence
        let alpha = (confidence * 200.0 + 55.0) as u8;
        let color = [class_color[0], class_color[1], class_color[2], alpha];
        
        self.rec.log(
            entity_path,
            &rerun::Boxes3D::from_centers_and_sizes(
                [[center[0] as f32, center[1] as f32, center[2] as f32]],
                [size],
            )
            .with_quaternions([quat])
            .with_colors([color])
            .with_labels([format!("{} ({:.0}%)", label, confidence * 100.0)])
        )?;
        
        Ok(())
    }
    
    /// Log a 3D mesh from file path (GLB/GLTF/OBJ)
    ///
    /// Returns an error if the mesh file doesn't exist.
    pub fn log_3d_mesh(
        &self,
        entity_path: &str,
        mesh_path: &std::path::Path,
        position: [f64; 3],
        scale: f32,
        yaw: f32,
    ) -> Result<(), Box<dyn std::error::Error>> {
        // Read mesh file
        let mesh_data = std::fs::read(mesh_path)?;
        let media_type = match mesh_path
            .extension()
            .and_then(|e| e.to_str())
            .map(|s| s.to_lowercase())
            .as_deref()
        {
            Some("glb") => rerun::MediaType::gltf(),
            Some("gltf") => rerun::MediaType::gltf(),
            Some("obj") => rerun::MediaType::obj(),
            _ => rerun::MediaType::gltf(), // Default
        };
        
        // Create rotation quaternion from yaw
        let half_yaw = yaw / 2.0;
        
        // Log as Asset3D with transform
        self.rec.log(
            entity_path,
            &rerun::Asset3D::from_file_contents(mesh_data, Some(media_type))
        )?;
        
        // Apply transform
        self.rec.log(
            entity_path,
            &rerun::Transform3D::from_translation_rotation_scale(
                [position[0] as f32, position[1] as f32, position[2] as f32],
                rerun::Quaternion::from_xyzw([0.0, 0.0, half_yaw.sin(), half_yaw.cos()]),
                scale,
            )
        )?;
        
        Ok(())
    }
    
    /// Log a colored 3D bounding box for an object class
    ///
    /// Standard class colors:
    /// - Vehicle: Cyan
    /// - Pedestrian: Orange
    /// - Cyclist: Green
    /// - Truck: Purple
    pub fn log_class_bbox(
        &self,
        class_name: &str,
        instance_id: &str,
        center: [f64; 3],
        size: [f32; 3],
        yaw: f32,
        confidence: f32,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let color = match class_name.to_lowercase().as_str() {
            "car" | "vehicle" => [0, 200, 255, 200],      // Cyan
            "pedestrian" | "person" => [255, 150, 50, 200], // Orange
            "cyclist" | "bicycle" | "motorcycle" => [100, 255, 100, 200], // Green
            "truck" | "bus" => [200, 100, 255, 200],      // Purple
            "drone" | "uav" => [255, 215, 0, 200],        // Gold
            _ => [200, 200, 200, 200],                    // Gray
        };
        
        self.log_3d_detection_box(
            &format!("world/detections/{}/{}", class_name, instance_id),
            center,
            size,
            yaw,
            class_name,
            confidence,
            color,
        )
    }
    
    // ========================================================================
    // PROCEDURAL 3D MESH GENERATORS
    // ========================================================================
    
    /// Log a simple car mesh (wedge-shaped body)
    ///
    /// Creates a low-poly car shape with hood, cabin, and trunk.
    pub fn log_car_mesh(
        &self,
        entity_path: &str,
        center: [f64; 3],
        scale: f32,
        yaw: f32,
        color: [u8; 4],
    ) -> Result<(), Box<dyn std::error::Error>> {
        // Car dimensions (before scaling)
        let l = 4.5 * scale; // Length
        let w = 2.0 * scale; // Width  
        let h = 1.5 * scale; // Height
        let hood_h = 0.8 * scale; // Hood height
        
        // Apply rotation
        let cos_y = yaw.cos();
        let sin_y = yaw.sin();
        
        // Helper to rotate and translate a point
        let transform = |x: f32, y: f32, z: f32| -> [f32; 3] {
            let rx = x * cos_y - y * sin_y + center[0] as f32;
            let ry = x * sin_y + y * cos_y + center[1] as f32;
            let rz = z + center[2] as f32;
            [rx, ry, rz]
        };
        
        // Define vertices for a wedge-shaped car
        // Bottom rectangle
        let v0 = transform(-l/2.0, -w/2.0, 0.0);      // Back left bottom
        let v1 = transform(-l/2.0,  w/2.0, 0.0);      // Back right bottom
        let v2 = transform( l/2.0,  w/2.0, 0.0);      // Front right bottom
        let v3 = transform( l/2.0, -w/2.0, 0.0);      // Front left bottom
        
        // Back rectangle (top)
        let v4 = transform(-l/2.0, -w/2.0, h);        // Back left top
        let v5 = transform(-l/2.0,  w/2.0, h);        // Back right top
        
        // Middle (cabin top)
        let v6 = transform(-l/4.0, -w/2.0, h);        // Cabin back left
        let v7 = transform(-l/4.0,  w/2.0, h);        // Cabin back right
        let v8 = transform( l/4.0,  w/2.0, h * 0.9);  // Cabin front right
        let v9 = transform( l/4.0, -w/2.0, h * 0.9);  // Cabin front left
        
        // Hood (lower)
        let v10 = transform(l/2.0, -w/2.0, hood_h);   // Front left hood
        let v11 = transform(l/2.0,  w/2.0, hood_h);   // Front right hood
        
        let vertices = vec![v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11];
        
        // Define triangles (indices)
        let triangles: Vec<[u32; 3]> = vec![
            // Bottom
            [0, 1, 2], [0, 2, 3],
            // Back
            [0, 4, 5], [0, 5, 1],
            // Left side (3 triangles for shape)
            [0, 3, 10], [0, 10, 9], [0, 9, 6], [0, 6, 4],
            // Right side
            [1, 11, 2], [1, 7, 11], [1, 5, 7], [7, 8, 11],
            // Top back (to cabin)
            [4, 6, 7], [4, 7, 5],
            // Cabin top
            [6, 9, 8], [6, 8, 7],
            // Hood top
            [9, 10, 11], [9, 11, 8],
            // Front
            [3, 2, 11], [3, 11, 10],
        ];
        
        self.rec.log(
            entity_path,
            &rerun::Mesh3D::new(vertices)
                .with_triangle_indices(triangles)
                .with_albedo_factor(rerun::Rgba32::from_unmultiplied_rgba(
                    color[0], color[1], color[2], color[3]
                ))
        )?;
        
        Ok(())
    }
    
    /// Log a drone mesh (quadcopter with rotors)
    ///
    /// Creates a drone shape with central body and 4 rotor arms.
    pub fn log_drone_mesh(
        &self,
        entity_path: &str,
        center: [f64; 3],
        scale: f32,
        yaw: f32,
        color: [u8; 4],
    ) -> Result<(), Box<dyn std::error::Error>> {
        let arm_len = 1.5 * scale;
        let body_size = 0.6 * scale;
        let body_h = 0.3 * scale;
        let rotor_r = 0.5 * scale;
        
        let cos_y = yaw.cos();
        let sin_y = yaw.sin();
        
        let transform = |x: f32, y: f32, z: f32| -> [f32; 3] {
            let rx = x * cos_y - y * sin_y + center[0] as f32;
            let ry = x * sin_y + y * cos_y + center[1] as f32;
            let rz = z + center[2] as f32;
            [rx, ry, rz]
        };
        
        let mut vertices = Vec::new();
        let mut triangles: Vec<[u32; 3]> = Vec::new();
        
        // Central body (octagon approximation)
        let n = 8;
        let base_idx = vertices.len() as u32;
        
        // Bottom center
        vertices.push(transform(0.0, 0.0, 0.0));
        // Top center  
        vertices.push(transform(0.0, 0.0, body_h));
        
        // Bottom ring
        for i in 0..n {
            let angle = (i as f32 / n as f32) * std::f32::consts::TAU;
            vertices.push(transform(
                body_size * angle.cos(),
                body_size * angle.sin(),
                0.0
            ));
        }
        
        // Top ring
        for i in 0..n {
            let angle = (i as f32 / n as f32) * std::f32::consts::TAU;
            vertices.push(transform(
                body_size * angle.cos(),
                body_size * angle.sin(),
                body_h
            ));
        }
        
        // Body triangles
        for i in 0..n {
            let next = (i + 1) % n;
            // Bottom
            triangles.push([base_idx, base_idx + 2 + i as u32, base_idx + 2 + next as u32]);
            // Top
            triangles.push([base_idx + 1, base_idx + 2 + n as u32 + next as u32, base_idx + 2 + n as u32 + i as u32]);
            // Sides
            triangles.push([base_idx + 2 + i as u32, base_idx + 2 + n as u32 + i as u32, base_idx + 2 + next as u32]);
            triangles.push([base_idx + 2 + next as u32, base_idx + 2 + n as u32 + i as u32, base_idx + 2 + n as u32 + next as u32]);
        }
        
        // 4 rotor arms
        let arm_positions = [
            (arm_len, arm_len),
            (arm_len, -arm_len),
            (-arm_len, -arm_len),
            (-arm_len, arm_len),
        ];
        
        for (ax, ay) in arm_positions {
            let arm_idx = vertices.len() as u32;
            
            // Arm (simple triangle pointing out)
            vertices.push(transform(0.0, 0.0, body_h * 0.5));
            vertices.push(transform(ax - 0.1, ay - 0.1, body_h * 0.3));
            vertices.push(transform(ax + 0.1, ay + 0.1, body_h * 0.3));
            triangles.push([arm_idx, arm_idx + 1, arm_idx + 2]);
            
            // Rotor disk at end (flat hexagon)
            let rotor_idx = vertices.len() as u32;
            vertices.push(transform(ax, ay, body_h * 0.4));
            
            for i in 0..6 {
                let angle = (i as f32 / 6.0) * std::f32::consts::TAU;
                vertices.push(transform(
                    ax + rotor_r * angle.cos(),
                    ay + rotor_r * angle.sin(),
                    body_h * 0.4
                ));
            }
            
            for i in 0..6 {
                let next = (i + 1) % 6;
                triangles.push([rotor_idx, rotor_idx + 1 + i as u32, rotor_idx + 1 + next as u32]);
            }
        }
        
        self.rec.log(
            entity_path,
            &rerun::Mesh3D::new(vertices)
                .with_triangle_indices(triangles)
                .with_albedo_factor(rerun::Rgba32::from_unmultiplied_rgba(
                    color[0], color[1], color[2], color[3]
                ))
        )?;
        
        Ok(())
    }
    
    // ========================================================================
    // PANEL TITLES AND ANNOTATIONS
    // ========================================================================
    
    /// Log a panel title text in 3D space
    ///
    /// Use this to label different areas of your visualization.
    pub fn log_panel_title(
        &self,
        panel_path: &str,
        position: [f64; 3],
        text: &str,
        subtitle: Option<&str>,
        color: [u8; 4],
    ) -> Result<(), Box<dyn std::error::Error>> {
        let full_text = match subtitle {
            Some(sub) => format!("{}\n{}", text, sub),
            None => text.to_string(),
        };
        
        self.rec.log(
            format!("panels/{}", panel_path),
            &rerun::TextLog::new(full_text.clone()),
        )?;
        
        // Also log as 3D text annotation
        self.rec.log(
            format!("world/labels/{}", panel_path),
            &rerun::Points3D::new([[position[0] as f32, position[1] as f32, position[2] as f32]])
                .with_colors([color])
                .with_radii([0.1])
                .with_labels([full_text])
        )?;
        
        Ok(())
    }
    
    /// Log section titles for different demo areas
    pub fn log_section_titles(&self) -> Result<(), Box<dyn std::error::Error>> {
        // Problem section
        self.log_panel_title(
            "problem",
            [0.0, 80.0, 20.0],
            "🔴 THE PROBLEM",
            Some("Multiple agents = multiple IDs for same object"),
            [255, 100, 100, 255],
        )?;
        
        // Data section  
        self.log_panel_title(
            "data",
            [80.0, 0.0, 20.0],
            "📡 RAW DATA",
            Some("Each agent reports detections with local IDs"),
            [100, 200, 255, 255],
        )?;
        
        // Solution section
        self.log_panel_title(
            "solution",
            [-80.0, 0.0, 20.0],
            "✅ THE SOLUTION",
            Some("Highlander merges duplicate IDs"),
            [100, 255, 150, 255],
        )?;
        
        Ok(())
    }
    
    /// Log a Highlander merge event with visual animation (two tracks becoming one)
    ///
    /// Shows a visual "merge" animation with tracks converging
    pub fn log_highlander_merge_visual(
        &self,
        winner_id: Uuid,
        loser_id: Uuid,
        merge_position: [f64; 3],
        winner_position: [f64; 3],
        loser_position: [f64; 3],
    ) -> Result<(), Box<dyn std::error::Error>> {
        let merge_path = format!("world/highlander/merge_{}", winner_id);
        
        // Log merge point marker (gold flash)
        self.rec.log(
            format!("{}/flash", merge_path),
            &rerun::Points3D::new([[merge_position[0] as f32, merge_position[1] as f32, merge_position[2] as f32]])
                .with_colors([[255, 215, 0, 255]]) // Gold
                .with_radii([2.0])
                .with_labels(["⚡ MERGE!"])
        )?;
        
        // Log connecting lines from old tracks to merge point
        self.rec.log(
            format!("{}/convergence", merge_path),
            &rerun::LineStrips3D::new([[
                [winner_position[0] as f32, winner_position[1] as f32, winner_position[2] as f32],
                [merge_position[0] as f32, merge_position[1] as f32, merge_position[2] as f32],
            ], [
                [loser_position[0] as f32, loser_position[1] as f32, loser_position[2] as f32],
                [merge_position[0] as f32, merge_position[1] as f32, merge_position[2] as f32],
            ]])
            .with_colors([[100, 255, 100, 200], [255, 100, 100, 200]]) // Green winner, red loser
        )?;
        
        // Log text showing the merge
        self.rec.log(
            format!("{}/info", merge_path),
            &rerun::TextLog::new(format!(
                "HIGHLANDER: {} absorbed {}",
                &winner_id.to_string()[..8],
                &loser_id.to_string()[..8]
            ))
        )?;
        
        Ok(())
    }
    
    /// Log summary statistics panel
    pub fn log_stats_panel(
        &self,
        real_objects: usize,
        total_tracks: usize,
        ghost_count: usize,
        merges_performed: usize,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let ghost_rate = if total_tracks > 0 { 
            (ghost_count as f64 / total_tracks as f64 * 100.0) as i32 
        } else { 0 };
        
        let summary = format!(
            "📊 STATS\nReal Objects: {}\nTotal Tracks: {}\nGhost Tracks: {} ({}%)\nMerges: {}",
            real_objects, total_tracks, ghost_count, ghost_rate, merges_performed
        );
        
        self.rec.log("panels/stats", &rerun::TextLog::new(summary))?;
        
        Ok(())
    }
    
    /// Set the current timestamp for timeline scrubbing
    pub fn set_time(&self, name: &str, value: u64) {
        if name == "frame" || name == "step" {
            self.rec.set_time_sequence(name, value as i64);
        } else {
            self.rec.set_time(name, rerun::time::Timestamp::from_nanos_since_epoch(value as i64 * 1_000_000));
        }
    }
    
    /// Set simulation time (seconds) for explicit timeline control
    /// 
    /// This is critical for CARLA integration - ensures Rerun timeline
    /// matches simulation time exactly, not wall clock time.
    pub fn set_sim_time(&self, sim_seconds: f64) {
        let nanos = (sim_seconds * 1e9) as i64;
        self.rec.set_time("sim_time", rerun::time::Timestamp::from_nanos_since_epoch(nanos));
    }
    
    // ========================================================================
    // CINEMATIC CAMERA (LinkedIn-Quality Visualization)
    // ========================================================================
    
    /// Log a cinematic camera for smooth professional visualization.
    /// 
    /// Based on cara_sim_imp.md Section 9: Catmull-Rom splines + quaternion SLERP.
    /// This camera can be selected in Rerun viewer for broadcast-quality recordings.
    pub fn log_cinematic_camera(
        &self,
        position: [f64; 3],
        look_at: [f64; 3],
        up: [f64; 3],
        fov_degrees: f32,
    ) -> Result<(), Box<dyn std::error::Error>> {
        // Calculate forward vector
        let forward = [
            look_at[0] - position[0],
            look_at[1] - position[1],
            look_at[2] - position[2],
        ];
        let forward_len = (forward[0].powi(2) + forward[1].powi(2) + forward[2].powi(2)).sqrt();
        let forward_norm = [
            forward[0] / forward_len,
            forward[1] / forward_len,
            forward[2] / forward_len,
        ];
        
        // Build rotation matrix from forward + up vectors
        // Right = forward × up
        let right = [
            forward_norm[1] * up[2] - forward_norm[2] * up[1],
            forward_norm[2] * up[0] - forward_norm[0] * up[2],
            forward_norm[0] * up[1] - forward_norm[1] * up[0],
        ];
        let right_len = (right[0].powi(2) + right[1].powi(2) + right[2].powi(2)).sqrt();
        let right_norm = [right[0] / right_len, right[1] / right_len, right[2] / right_len];
        
        // Recompute up = right × forward
        let up_corrected = [
            right_norm[1] * forward_norm[2] - right_norm[2] * forward_norm[1],
            right_norm[2] * forward_norm[0] - right_norm[0] * forward_norm[2],
            right_norm[0] * forward_norm[1] - right_norm[1] * forward_norm[0],
        ];
        
        // Build rotation matrix (columns: right, up, -forward for OpenGL convention)
        let rot_mat = nalgebra::Matrix3::new(
            right_norm[0], up_corrected[0], -forward_norm[0],
            right_norm[1], up_corrected[1], -forward_norm[1],
            right_norm[2], up_corrected[2], -forward_norm[2],
        );
        
        let rotation = nalgebra::UnitQuaternion::from_matrix(&rot_mat);
        let q = rotation.as_ref();
        
        let cam_path = "world/cinematic_camera";
        
        // Log transform
        self.rec.log(
            cam_path,
            &rerun::Transform3D::from_translation_rotation(
                [position[0] as f32, position[1] as f32, position[2] as f32],
                rerun::Quaternion::from_wxyz([q.w as f32, q.i as f32, q.j as f32, q.k as f32]),
            ),
        )?;
        
        // Log pinhole camera (lens properties)
        let fov_rad = fov_degrees.to_radians();
        let aspect = 16.0 / 9.0; // Standard widescreen
        let image_width = 1920.0f32;
        let image_height = image_width / aspect;
        let focal_length = (image_width / 2.0) / (fov_rad / 2.0).tan();
        
        self.rec.log(
            cam_path,
            &rerun::Pinhole::from_focal_length_and_resolution(
                [focal_length, focal_length],
                [image_width, image_height],
            ),
        )?;
        
        Ok(())
    }
}

// ============================================================================
// CINEMATIC CAMERA CONTROLLER
// ============================================================================

/// Cinematic camera controller with smooth interpolation.
///
/// Implements Catmull-Rom spline pathing and quaternion SLERP
/// for broadcast-quality visualization recordings.
///
/// Based on cara_sim_imp.md Section 9:
/// - Position history ring buffer for spline interpolation
/// - Delay offset for lazy follow effect
/// - Camera look-at smoothing
pub struct CinematicCamera {
    /// Ring buffer of target positions for spline interpolation
    position_history: std::collections::VecDeque<[f64; 3]>,
    /// Ring buffer of target orientations (yaw only for simplicity)
    heading_history: std::collections::VecDeque<f64>,
    /// Maximum history size
    max_history: usize,
    /// Delay behind target (seconds) for smooth following
    delay_offset: f64,
    /// Camera distance behind target
    follow_distance: f64,
    /// Camera height above target
    follow_height: f64,
    /// Current interpolated position
    current_position: [f64; 3],
    /// Current interpolated look-at
    current_look_at: [f64; 3],
}

impl CinematicCamera {
    /// Create a new cinematic camera controller
    pub fn new(delay_offset: f64, follow_distance: f64, follow_height: f64) -> Self {
        Self {
            position_history: std::collections::VecDeque::with_capacity(32),
            heading_history: std::collections::VecDeque::with_capacity(32),
            max_history: 32,
            delay_offset,
            follow_distance,
            follow_height,
            current_position: [0.0, 0.0, 10.0],
            current_look_at: [0.0, 0.0, 0.0],
        }
    }
    
    /// Update target position (call every frame)
    pub fn update_target(&mut self, position: [f64; 3], heading_degrees: f64) {
        // Add to history
        self.position_history.push_back(position);
        self.heading_history.push_back(heading_degrees);
        
        // Trim to max size
        while self.position_history.len() > self.max_history {
            self.position_history.pop_front();
            self.heading_history.pop_front();
        }
    }
    
    /// Get interpolated camera position and look-at point
    pub fn get_camera_state(&mut self, dt: f64) -> ([f64; 3], [f64; 3]) {
        if self.position_history.len() < 4 {
            // Not enough history for spline, use simple follow
            if let Some(&target) = self.position_history.back() {
                let heading = self.heading_history.back().copied().unwrap_or(0.0);
                let heading_rad = heading.to_radians();
                
                // Position behind target
                let cam_pos = [
                    target[0] - self.follow_distance * heading_rad.cos(),
                    target[1] - self.follow_distance * heading_rad.sin(),
                    target[2] + self.follow_height,
                ];
                
                // Lerp towards target
                self.current_position = self.lerp3(self.current_position, cam_pos, 0.1);
                self.current_look_at = self.lerp3(self.current_look_at, target, 0.15);
            }
            return (self.current_position, self.current_look_at);
        }
        
        // Catmull-Rom spline interpolation
        let t = 0.5; // Interpolate in the middle of history
        let target_pos = self.catmull_rom_interpolate(t);
        let target_heading = self.heading_interpolate(t);
        
        let heading_rad = target_heading.to_radians();
        
        // Calculate camera position behind target
        let cam_target = [
            target_pos[0] - self.follow_distance * heading_rad.cos(),
            target_pos[1] - self.follow_distance * heading_rad.sin(),
            target_pos[2] + self.follow_height,
        ];
        
        // Smooth lerp to target
        self.current_position = self.lerp3(self.current_position, cam_target, dt * 3.0);
        self.current_look_at = self.lerp3(self.current_look_at, target_pos, dt * 5.0);
        
        (self.current_position, self.current_look_at)
    }
    
    /// Catmull-Rom spline interpolation through position history
    /// 
    /// Formula from cara_sim_imp.md Section 9.1.1:
    /// P(t) = 0.5 * [1, t, t², t³] * M * [P₀, P₁, P₂, P₃]ᵀ
    fn catmull_rom_interpolate(&self, t: f64) -> [f64; 3] {
        let n = self.position_history.len();
        if n < 4 {
            return self.position_history.back().copied().unwrap_or([0.0, 0.0, 0.0]);
        }
        
        // Get 4 control points from history
        let idx = (n as f64 * t).floor() as usize;
        let idx = idx.min(n - 4);
        
        let p0 = self.position_history[idx];
        let p1 = self.position_history[idx + 1];
        let p2 = self.position_history[idx + 2];
        let p3 = self.position_history[idx + 3];
        
        // Local t within segment
        let local_t = (t * (n - 3) as f64) - idx as f64;
        let local_t = local_t.clamp(0.0, 1.0);
        
        // Catmull-Rom basis matrix
        let t2 = local_t * local_t;
        let t3 = t2 * local_t;
        
        let mut result = [0.0; 3];
        for i in 0..3 {
            result[i] = 0.5 * (
                (2.0 * p1[i]) +
                (-p0[i] + p2[i]) * local_t +
                (2.0 * p0[i] - 5.0 * p1[i] + 4.0 * p2[i] - p3[i]) * t2 +
                (-p0[i] + 3.0 * p1[i] - 3.0 * p2[i] + p3[i]) * t3
            );
        }
        
        result
    }
    
    /// Interpolate heading angles (handles wrap-around at 360°)
    fn heading_interpolate(&self, t: f64) -> f64 {
        let n = self.heading_history.len();
        if n < 2 {
            return self.heading_history.back().copied().unwrap_or(0.0);
        }
        
        let idx = ((n - 1) as f64 * t).floor() as usize;
        let idx = idx.min(n - 2);
        
        let h0 = self.heading_history[idx];
        let h1 = self.heading_history[idx + 1];
        
        // Handle wrap-around
        let mut diff = h1 - h0;
        if diff > 180.0 { diff -= 360.0; }
        if diff < -180.0 { diff += 360.0; }
        
        let local_t = (t * (n - 1) as f64) - idx as f64;
        h0 + diff * local_t.clamp(0.0, 1.0)
    }
    
    /// Linear interpolation for 3D vectors
    fn lerp3(&self, a: [f64; 3], b: [f64; 3], t: f64) -> [f64; 3] {
        let t = t.clamp(0.0, 1.0);
        [
            a[0] + (b[0] - a[0]) * t,
            a[1] + (b[1] - a[1]) * t,
            a[2] + (b[2] - a[2]) * t,
        ]
    }
}

impl Default for CinematicCamera {
    fn default() -> Self {
        Self::new(0.5, 15.0, 8.0)  // 0.5s delay, 15m behind, 8m above
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    #[ignore] // Requires Rerun viewer
    fn test_visualizer_creation() {
        let viz = RerunVisualizer::new("test_app");
        assert!(viz.is_ok());
    }
}

//! The "SPACE" Engine - Hierarchical Hybrid Indexing (H3 + Sparse Voxel Octree)
//!
//! Solves the "Pancake World" problem by using:
//! - H3 for global 2D surface sharding (handles spherical earth)
//! - Sparse Voxel Octrees for local 3D indexing (handles altitude)

use h3o::{CellIndex, LatLng, Resolution};
use oktree::Octree;
use std::collections::HashMap;
use serde::{Deserialize, Serialize};
use uuid::Uuid;

/// A spatial entity in the GodView world
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Entity {
    /// Unique identifier
    pub id: Uuid,
    
    /// Global position [latitude, longitude, altitude]
    pub position: [f64; 3],
    
    /// Velocity vector [vx, vy, vz] in m/s
    pub velocity: [f64; 3],
    
    /// Entity type (e.g., "human_face", "vehicle", "drone")
    pub entity_type: String,
    
    /// Timestamp of last update (Unix milliseconds)
    pub timestamp: i64,
    
    /// Confidence score (0.0 - 1.0)
    pub confidence: f32,
}

/// A WorldShard represents a columnar volume of space defined by an H3 cell
///
/// The H3 cell defines the 2D footprint (lat/lon), and the internal
/// Octree handles the 3D structure (altitude).
#[derive(Debug)]
pub struct WorldShard {
    /// The H3 cell index defining this shard's 2D footprint
    pub cell_id: CellIndex,
    
    /// Sparse Voxel Octree for local 3D queries
    /// Coordinate system: Quantized meters relative to cell center
    pub local_index: Octree,
    
    /// Map from Octree element IDs to actual entity data
    pub entities: HashMap<u64, Entity>,
    
    /// Counter for generating local element IDs
    next_element_id: u64,
}

impl WorldShard {
    /// Create a new WorldShard for the given H3 cell
    pub fn new(cell_id: CellIndex) -> Self {
        Self {
            cell_id,
            local_index: Oktree::new(),
            entities: HashMap::new(),
            next_element_id: 0,
        }
    }
    
    /// Insert an entity into this shard's octree
    ///
    /// # Arguments
    /// * `entity` - The entity to insert
    /// * `local_coords` - Local coordinates (x, y, z) in meters from cell center
    pub fn insert(&mut self, entity: Entity, local_coords: [f32; 3]) -> u64 {
        let element_id = self.next_element_id;
        self.next_element_id += 1;
        
        // Convert meters to quantized u16 coordinates for octree
        // Range: -1000m to +1000m mapped to 0-65535
        let x = ((local_coords[0] + 1000.0) * 32.767).clamp(0.0, 65535.0) as u16;
        let y = ((local_coords[1] + 1000.0) * 32.767).clamp(0.0, 65535.0) as u16;
        let z = ((local_coords[2] + 1000.0) * 32.767).clamp(0.0, 65535.0) as u16;
        
        // Insert into octree
        self.local_index.insert([x, y, z], element_id);
        
        // Store entity data
        self.entities.insert(element_id, entity);
        
        element_id
    }
    
    /// Query entities within a sphere
    ///
    /// # Arguments
    /// * `center` - Center point in local coordinates [x, y, z]
    /// * `radius` - Radius in meters
    pub fn query_sphere(&self, center: [f32; 3], radius: f32) -> Vec<&Entity> {
        // Convert center to quantized coordinates
        let cx = ((center[0] + 1000.0) * 32.767).clamp(0.0, 65535.0) as u16;
        let cy = ((center[1] + 1000.0) * 32.767).clamp(0.0, 65535.0) as u16;
        let cz = ((center[2] + 1000.0) * 32.767).clamp(0.0, 65535.0) as u16;
        
        // Convert radius to quantized units
        let r_quantized = (radius * 32.767) as u16;
        
        // Query octree (this is a simplified version - real implementation
        // would use oktree's spatial query methods)
        let mut results = Vec::new();
        
        for (element_id, entity) in &self.entities {
            // Simple distance check (octree would optimize this)
            let entity_local = self.global_to_local(entity.position);
            let dx = entity_local[0] - center[0];
            let dy = entity_local[1] - center[1];
            let dz = entity_local[2] - center[2];
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
            
            if dist <= radius {
                results.push(entity);
            }
        }
        
        results
    }
    
    /// Convert global GPS to local coordinates relative to cell center
    fn global_to_local(&self, global_pos: [f64; 3]) -> [f32; 3] {
        // Get cell center
        let center = self.cell_id.to_lat_lng();
        
        // Simple equirectangular approximation for local areas
        const EARTH_RADIUS: f64 = 6378137.0;
        
        let lat_diff = (global_pos[0] - center.lat()) * (std::f64::consts::PI / 180.0);
        let lon_diff = (global_pos[1] - center.lng()) * (std::f64::consts::PI / 180.0);
        
        let x = (lon_diff * EARTH_RADIUS * center.lat().cos()) as f32;
        let z = (-lat_diff * EARTH_RADIUS) as f32;
        let y = global_pos[2] as f32;
        
        [x, y, z]
    }
}

/// The Global Spatial Index
///
/// Maps the spherical world into discrete shards using H3,
/// then uses Octrees within each shard for 3D queries.
pub struct SpatialEngine {
    /// Primary index: H3 Cell -> WorldShard
    pub shards: HashMap<CellIndex, WorldShard>,
    
    /// H3 Resolution level
    /// Resolution 10: ~66m edge length (good for city blocks)
    /// Resolution 9: ~174m edge length (good for neighborhoods)
    pub resolution: Resolution,
}

impl SpatialEngine {
    /// Create a new SpatialEngine
    ///
    /// # Arguments
    /// * `resolution` - H3 resolution level (recommended: 9-11)
    pub fn new(resolution: Resolution) -> Self {
        Self {
            shards: HashMap::new(),
            resolution,
        }
    }
    
    /// Insert or update an entity in the spatial index
    ///
    /// # Arguments
    /// * `entity` - The entity to insert
    ///
    /// # Returns
    /// The H3 cell index where the entity was inserted
    pub fn update_entity(&mut self, entity: Entity) -> Result<CellIndex, String> {
        // Step 1: Convert lat/lon to H3 CellIndex
        let lat_lng = LatLng::new(entity.position[0], entity.position[1])
            .map_err(|e| format!("Invalid coordinates: {:?}", e))?;
        
        let cell_id = lat_lng.to_cell(self.resolution);
        
        // Step 2: Get or create shard
        let shard = self.shards.entry(cell_id).or_insert_with(|| WorldShard::new(cell_id));
        
        // Step 3: Convert global position to local coordinates
        let local_coords = shard.global_to_local(entity.position);
        
        // Step 4: Insert into shard's octree
        shard.insert(entity, local_coords);
        
        Ok(cell_id)
    }
    
    /// Query entities within a 3D sphere
    ///
    /// This respects altitude - a drone at 300m will NOT be returned
    /// when querying for ground-level entities.
    ///
    /// # Arguments
    /// * `center` - Center point [lat, lon, alt]
    /// * `radius_meters` - Search radius in meters
    pub fn query_radius(&self, center: [f64; 3], radius_meters: f64) -> Vec<&Entity> {
        let mut results = Vec::new();
        
        // Step 1: Determine which H3 cells overlap the search radius
        let center_latlng = match LatLng::new(center[0], center[1]) {
            Ok(ll) => ll,
            Err(_) => return results,
        };
        
        let center_cell = center_latlng.to_cell(self.resolution);
        
        // Calculate k-ring radius (number of cell rings to search)
        // Approximate: each resolution 10 cell is ~66m, so k = radius / 66
        let k = ((radius_meters / 66.0).ceil() as u32).max(1);
        
        // Get all cells within k rings
        let cells_to_search: Vec<CellIndex> = center_cell
            .grid_disk(k)
            .collect();
        
        // Step 2: Query each shard's octree
        for cell_id in cells_to_search {
            if let Some(shard) = self.shards.get(&cell_id) {
                // Convert center to local coordinates for this shard
                let local_center = shard.global_to_local(center);
                
                // Query shard
                let shard_results = shard.query_sphere(local_center, radius_meters as f32);
                results.extend(shard_results);
            }
        }
        
        results
    }
    
    /// Get all entities in a specific H3 cell
    pub fn get_cell_entities(&self, cell_id: CellIndex) -> Vec<&Entity> {
        self.shards
            .get(&cell_id)
            .map(|shard| shard.entities.values().collect())
            .unwrap_or_default()
    }
    
    /// Get statistics about the spatial index
    pub fn stats(&self) -> SpatialStats {
        let total_entities: usize = self.shards.values().map(|s| s.entities.len()).sum();
        let total_shards = self.shards.len();
        let avg_entities_per_shard = if total_shards > 0 {
            total_entities as f64 / total_shards as f64
        } else {
            0.0
        };
        
        SpatialStats {
            total_entities,
            total_shards,
            avg_entities_per_shard,
            resolution: self.resolution,
        }
    }
}

/// Statistics about the spatial index
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpatialStats {
    pub total_entities: usize,
    pub total_shards: usize,
    pub avg_entities_per_shard: f64,
    pub resolution: Resolution,
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_spatial_engine_creation() {
        let engine = SpatialEngine::new(Resolution::Ten);
        assert_eq!(engine.shards.len(), 0);
    }
    
    #[test]
    fn test_entity_insertion() {
        let mut engine = SpatialEngine::new(Resolution::Ten);
        
        let entity = Entity {
            id: Uuid::new_v4(),
            position: [37.7749, -122.4194, 10.0], // San Francisco
            velocity: [0.0, 0.0, 0.0],
            entity_type: "human_face".to_string(),
            timestamp: 1702934400000,
            confidence: 0.95,
        };
        
        let result = engine.update_entity(entity);
        assert!(result.is_ok());
        
        let stats = engine.stats();
        assert_eq!(stats.total_entities, 1);
        assert_eq!(stats.total_shards, 1);
    }
    
    #[test]
    fn test_vertical_separation() {
        let mut engine = SpatialEngine::new(Resolution::Ten);
        
        // Ground entity
        let ground = Entity {
            id: Uuid::new_v4(),
            position: [37.7749, -122.4194, 0.0],
            velocity: [0.0, 0.0, 0.0],
            entity_type: "vehicle".to_string(),
            timestamp: 1702934400000,
            confidence: 0.95,
        };
        
        // Aerial entity (same lat/lon, different altitude)
        let aerial = Entity {
            id: Uuid::new_v4(),
            position: [37.7749, -122.4194, 300.0],
            velocity: [0.0, 0.0, 0.0],
            entity_type: "drone".to_string(),
            timestamp: 1702934400000,
            confidence: 0.95,
        };
        
        engine.update_entity(ground).unwrap();
        engine.update_entity(aerial).unwrap();
        
        // Query at ground level with 50m radius
        let results = engine.query_radius([37.7749, -122.4194, 0.0], 50.0);
        
        // Should only return ground entity, not aerial
        assert_eq!(results.len(), 1);
        assert_eq!(results[0].entity_type, "vehicle");
    }
}

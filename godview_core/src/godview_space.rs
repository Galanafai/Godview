//! The "SPACE" Engine - Hierarchical Hybrid Indexing (H3 + 3D Grid)
//!
//! Solves the "Pancake World" problem by using:
//! - H3 for global 2D surface sharding (handles spherical earth)
//! - 3D grid-based spatial index within each shard (handles altitude)

use h3o::{CellIndex, LatLng, Resolution};
use std::collections::{HashMap, HashSet};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

/// Average edge length in meters for each H3 resolution level
/// Source: https://h3geo.org/docs/core-library/restable/
const H3_EDGE_LENGTH_M: [f64; 16] = [
    1107712.591,  // Res 0 - ~1108 km
    418676.005,   // Res 1 - ~419 km
    158244.655,   // Res 2 - ~158 km
    59810.857,    // Res 3 - ~60 km
    22606.379,    // Res 4 - ~23 km
    8544.408,     // Res 5 - ~8.5 km
    3229.482,     // Res 6 - ~3.2 km
    1220.629,     // Res 7 - ~1.2 km
    461.354,      // Res 8 - ~461 m
    174.375,      // Res 9 - ~174 m
    65.907,       // Res 10 - ~66 m
    24.910,       // Res 11 - ~25 m
    9.415,        // Res 12 - ~9.4 m
    3.559,        // Res 13 - ~3.6 m
    1.348,        // Res 14 - ~1.3 m
    0.509,        // Res 15 - ~0.5 m
];

/// Get edge length in meters for a given H3 resolution
fn edge_length_meters(resolution: Resolution) -> f64 {
    H3_EDGE_LENGTH_M[resolution as usize]
}

/// 3D grid cell index for spatial hashing within a shard
#[derive(Debug, Clone, Copy, Hash, Eq, PartialEq)]
struct GridCell {
    x: i32,
    y: i32,
    z: i32,
}

impl GridCell {
    fn from_local_coords(coords: [f32; 3], cell_size: f32) -> Self {
        Self {
            x: (coords[0] / cell_size).floor() as i32,
            y: (coords[1] / cell_size).floor() as i32,
            z: (coords[2] / cell_size).floor() as i32,
        }
    }
    
    /// Get all neighboring cells within radius (in cells)
    fn neighbors(&self, radius: i32) -> Vec<GridCell> {
        let mut cells = Vec::new();
        for dx in -radius..=radius {
            for dy in -radius..=radius {
                for dz in -radius..=radius {
                    cells.push(GridCell {
                        x: self.x + dx,
                        y: self.y + dy,
                        z: self.z + dz,
                    });
                }
            }
        }
        cells
    }
}

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
/// 3D grid provides O(1) spatial lookup for altitude-aware queries.
#[derive(Debug)]
pub struct WorldShard {
    /// The H3 cell index defining this shard's 2D footprint
    pub cell_id: CellIndex,
    
    /// Map from element IDs to actual entity data
    pub entities: HashMap<u64, Entity>,
    
    /// 3D spatial index: GridCell -> set of entity IDs in that cell
    spatial_grid: HashMap<GridCell, HashSet<u64>>,
    
    /// Grid cell size in meters (determines spatial resolution)
    grid_cell_size: f32,
    
    /// Counter for generating local element IDs
    next_element_id: u64,
}

impl WorldShard {
    /// Create a new WorldShard for the given H3 cell
    /// 
    /// # Arguments
    /// * `cell_id` - The H3 cell index
    /// * `grid_cell_size` - Size of 3D grid cells in meters (default: 10m)
    pub fn new(cell_id: CellIndex) -> Self {
        Self::with_grid_size(cell_id, 10.0)  // 10m grid cells by default
    }
    
    /// Create a WorldShard with custom grid cell size
    pub fn with_grid_size(cell_id: CellIndex, grid_cell_size: f32) -> Self {
        Self {
            cell_id,
            entities: HashMap::new(),
            spatial_grid: HashMap::new(),
            grid_cell_size,
            next_element_id: 0,
        }
    }
    
    /// Insert an entity into this shard
    ///
    /// # Arguments
    /// * `entity` - The entity to insert
    /// * `local_coords` - Local coordinates (x, y, z) in meters from cell center
    pub fn insert(&mut self, entity: Entity, local_coords: [f32; 3]) -> u64 {
        let element_id = self.next_element_id;
        self.next_element_id += 1;
        
        // Compute grid cell for spatial indexing
        let grid_cell = GridCell::from_local_coords(local_coords, self.grid_cell_size);
        
        // Add to spatial index
        self.spatial_grid
            .entry(grid_cell)
            .or_insert_with(HashSet::new)
            .insert(element_id);
        
        // Store entity data
        self.entities.insert(element_id, entity);
        
        element_id
    }
    
    /// Query entities within a sphere using 3D grid acceleration
    ///
    /// Complexity: O(k³ × avg_entities_per_cell) where k = ceil(radius / cell_size)
    /// For typical scenarios with 10m cells and 50m radius, k=5, so ~125 cells checked.
    ///
    /// # Arguments
    /// * `center` - Center point in local coordinates [x, y, z]
    /// * `radius` - Radius in meters
    pub fn query_sphere(&self, center: [f32; 3], radius: f32) -> Vec<&Entity> {
        let mut results = Vec::new();
        
        // Calculate how many grid cells to search
        let cell_radius = (radius / self.grid_cell_size).ceil() as i32;
        let center_cell = GridCell::from_local_coords(center, self.grid_cell_size);
        
        // Query only the neighboring grid cells (not all entities!)
        for grid_cell in center_cell.neighbors(cell_radius) {
            if let Some(entity_ids) = self.spatial_grid.get(&grid_cell) {
                for &id in entity_ids {
                    if let Some(entity) = self.entities.get(&id) {
                        let entity_local = self.global_to_local(entity.position);
                        let dx = entity_local[0] - center[0];
                        let dy = entity_local[1] - center[1];
                        let dz = entity_local[2] - center[2];
                        let dist_sq = dx * dx + dy * dy + dz * dz;
                        
                        if dist_sq <= radius * radius {
                            results.push(entity);
                        }
                    }
                }
            }
        }
        
        results
    }
    
    /// Convert global GPS to local coordinates relative to cell center
    pub fn global_to_local(&self, global_pos: [f64; 3]) -> [f32; 3] {
        // Get cell center - LatLng implements From<CellIndex>
        let center = LatLng::from(self.cell_id);
        
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
/// then uses 3D grid-based spatial index within each shard for fast queries.
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
        
        // Step 4: Insert into shard's spatial index
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
        
        // Calculate k-ring radius using resolution-aware edge length
        // FIX: Use dynamic edge length based on actual resolution
        let edge_length = edge_length_meters(self.resolution);
        let k = ((radius_meters / edge_length).ceil() as u32).max(1);
        
        // Get all cells within k rings using grid_disk_safe (returns iterator)
        let cells_to_search: Vec<CellIndex> = center_cell
            .grid_disk_safe(k)
            .collect();
        
        // Step 2: Query each shard's 3D grid
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

//! Ground truth oracle for simulation.
//!
//! The Oracle maintains the "God's eye view" of the simulated world:
//! - True positions of all entities
//! - Physics simulation (kinematics)
//! - Sensor reading generation (with noise)

use nalgebra::{Vector3, Vector6};
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;
use rand_distr::{Distribution, Normal};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// A ground truth entity in the simulation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GroundTruthEntity {
    /// Unique entity ID
    pub id: u64,
    
    /// Position [x, y, z] in meters (global frame)
    pub position: Vector3<f64>,
    
    /// Velocity [vx, vy, vz] in m/s
    pub velocity: Vector3<f64>,
    
    /// Entity class (e.g., "drone", "vehicle", "pedestrian")
    pub class: String,
    
    /// Entity is active (not destroyed/removed)
    pub active: bool,
}

impl GroundTruthEntity {
    /// Creates a new entity at the given position.
    pub fn new(id: u64, position: Vector3<f64>, class: &str) -> Self {
        Self {
            id,
            position,
            velocity: Vector3::zeros(),
            class: class.to_string(),
            active: true,
        }
    }
    
    /// Creates a new entity with initial velocity.
    pub fn with_velocity(
        id: u64,
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        class: &str,
    ) -> Self {
        Self {
            id,
            position,
            velocity,
            class: class.to_string(),
            active: true,
        }
    }
    
    /// Returns the state as a 6D vector [pos, vel].
    pub fn state(&self) -> Vector6<f64> {
        Vector6::new(
            self.position.x, self.position.y, self.position.z,
            self.velocity.x, self.velocity.y, self.velocity.z,
        )
    }
}

/// A sensor reading generated from ground truth with noise.
#[derive(Debug, Clone)]
pub struct SensorReading {
    /// Entity ID this reading corresponds to
    pub entity_id: u64,
    
    /// Noisy position measurement
    pub position: Vector3<f64>,
    
    /// Velocity (typically from derivative or sensor)
    pub velocity: Vector3<f64>,
}

/// The Oracle - maintains ground truth and generates sensor readings.
pub struct Oracle {
    /// Master seed (separate from network to avoid interference)
    physics_seed: u64,
    
    /// RNG for physics (noise, random events)
    physics_rng: ChaCha8Rng,
    
    /// All ground truth entities
    entities: HashMap<u64, GroundTruthEntity>,
    
    /// Next entity ID
    next_id: u64,
    
    /// Current simulation time (seconds)
    current_time: f64,
    
    /// Position noise standard deviation (meters)
    position_noise_std: f64,
}

impl Oracle {
    /// Creates a new Oracle with the given physics seed.
    ///
    /// Note: The physics seed should be derived separately from the network seed
    /// so that changing network topology doesn't affect entity trajectories.
    pub fn new(physics_seed: u64) -> Self {
        Self {
            physics_seed,
            physics_rng: ChaCha8Rng::seed_from_u64(physics_seed),
            entities: HashMap::new(),
            next_id: 0,
            current_time: 0.0,
            position_noise_std: 0.5, // 50cm noise by default
        }
    }
    
    /// Sets the position noise standard deviation.
    pub fn set_position_noise(&mut self, std_dev: f64) {
        self.position_noise_std = std_dev;
    }
    
    /// Spawns a new entity and returns its ID.
    pub fn spawn_entity(
        &mut self,
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        class: &str,
    ) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        
        let entity = GroundTruthEntity::with_velocity(id, position, velocity, class);
        self.entities.insert(id, entity);
        
        id
    }
    
    /// Removes an entity from the simulation.
    pub fn remove_entity(&mut self, id: u64) {
        if let Some(entity) = self.entities.get_mut(&id) {
            entity.active = false;
        }
    }
    
    /// Advances physics by dt seconds.
    pub fn step(&mut self, dt: f64) {
        self.current_time += dt;
        
        // Simple constant-velocity model
        for entity in self.entities.values_mut() {
            if entity.active {
                entity.position += entity.velocity * dt;
            }
        }
    }
    
    /// Returns the current simulation time.
    pub fn time(&self) -> f64 {
        self.current_time
    }
    
    /// Returns all active entities.
    pub fn active_entities(&self) -> Vec<&GroundTruthEntity> {
        self.entities.values().filter(|e| e.active).collect()
    }
    
    /// Returns a specific entity by ID.
    pub fn entity(&self, id: u64) -> Option<&GroundTruthEntity> {
        self.entities.get(&id)
    }
    
    /// Generates a noisy sensor reading for an entity.
    ///
    /// Uses Box-Muller transform to add Gaussian noise.
    pub fn generate_sensor_reading(&mut self, entity_id: u64) -> Option<Vector3<f64>> {
        let entity = self.entities.get(&entity_id)?;
        if !entity.active {
            return None;
        }
        
        // Add Gaussian noise to position
        let normal = Normal::new(0.0, self.position_noise_std).unwrap();
        let noise = Vector3::new(
            normal.sample(&mut self.physics_rng),
            normal.sample(&mut self.physics_rng),
            normal.sample(&mut self.physics_rng),
        );
        
        Some(entity.position + noise)
    }
    
    /// Generates sensor readings for all active entities.
    pub fn generate_all_readings(&mut self) -> Vec<(u64, Vector3<f64>)> {
        let entity_ids: Vec<u64> = self.entities
            .values()
            .filter(|e| e.active)
            .map(|e| e.id)
            .collect();
        
        entity_ids
            .into_iter()
            .filter_map(|id| {
                self.generate_sensor_reading(id).map(|pos| (id, pos))
            })
            .collect()
    }
    
    /// Generates SensorReading structs for all active entities.
    ///
    /// This is the preferred method for agent consumption.
    pub fn generate_sensor_readings(&mut self) -> Vec<SensorReading> {
        let entity_ids: Vec<(u64, Vector3<f64>)> = self.entities
            .values()
            .filter(|e| e.active)
            .map(|e| (e.id, e.velocity))
            .collect();
        
        entity_ids
            .into_iter()
            .filter_map(|(id, velocity)| {
                self.generate_sensor_reading(id).map(|position| {
                    SensorReading {
                        entity_id: id,
                        position,
                        velocity,
                    }
                })
            })
            .collect()
    }
    
    /// Returns ground truth positions for error calculation.
    pub fn ground_truth_positions(&self) -> Vec<(u64, Vector3<f64>)> {
        self.entities
            .values()
            .filter(|e| e.active)
            .map(|e| (e.id, e.position))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_oracle_spawn_entity() {
        let mut oracle = Oracle::new(42);
        
        let id = oracle.spawn_entity(
            Vector3::new(100.0, 200.0, 50.0),
            Vector3::new(10.0, 0.0, 0.0),
            "drone",
        );
        
        let entity = oracle.entity(id).unwrap();
        assert_eq!(entity.position.x, 100.0);
        assert_eq!(entity.class, "drone");
    }
    
    #[test]
    fn test_oracle_physics_step() {
        let mut oracle = Oracle::new(42);
        
        let id = oracle.spawn_entity(
            Vector3::new(0.0, 0.0, 100.0),
            Vector3::new(20.0, 0.0, 0.0), // 20 m/s in x direction
            "drone",
        );
        
        oracle.step(1.0); // 1 second
        
        let entity = oracle.entity(id).unwrap();
        assert!((entity.position.x - 20.0).abs() < 0.001);
    }
    
    #[test]
    fn test_oracle_deterministic_noise() {
        let mut oracle1 = Oracle::new(42);
        let mut oracle2 = Oracle::new(42);
        
        let id1 = oracle1.spawn_entity(Vector3::zeros(), Vector3::zeros(), "drone");
        let id2 = oracle2.spawn_entity(Vector3::zeros(), Vector3::zeros(), "drone");
        
        let reading1 = oracle1.generate_sensor_reading(id1).unwrap();
        let reading2 = oracle2.generate_sensor_reading(id2).unwrap();
        
        // Same seed = same noise
        assert_eq!(reading1, reading2);
    }
}

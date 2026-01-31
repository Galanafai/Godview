use godview_core::godview_tracking::GlobalHazardPacket;
use rand::Rng;

/// Parameters that can be evolved/adapted at runtime.
#[derive(Debug, Clone, Copy)]
pub struct EvoParams {
    /// How many ticks between gossip rounds.
    pub gossip_interval_ticks: u64,
    
    /// Maximum number of neighbors to gossip with.
    pub max_neighbors_gossip: usize,
    
    /// Minimum confidence to accept a track.
    pub confidence_threshold: f64,
}

impl Default for EvoParams {
    fn default() -> Self {
        Self {
            gossip_interval_ticks: 5,
            max_neighbors_gossip: 100, // Effectively infinite (all neighbors)
            confidence_threshold: 0.0,
        }
    }
}

/// State for the evolutionary learning process.
#[derive(Debug, Clone)]
pub struct EvolutionaryState {
    /// Current active parameters.
    pub current_params: EvoParams,
    
    /// Parameters from the previous epoch (for rollback).
    prev_params: EvoParams,
    
    /// Fitness score of the current epoch.
    current_fitness: f64,
    
    /// Fitness score of the previous epoch.
    prev_fitness: f64,
    
    /// Current mutation being tested.
    active_mutation: Option<MutationType>,
    
    // Metrics accumulator for current epoch
    epoch_msgs_sent: u64,
    epoch_error_sum: f64,
    epoch_samples: u64,
}

#[derive(Debug, Clone, Copy)]
enum MutationType {
    IncreaseGossipInterval,
    DecreaseGossipInterval,
    IncreaseMaxNeighbors,
    DecreaseMaxNeighbors,
    IncreaseConfidence,
    DecreaseConfidence,
}

impl EvolutionaryState {
    pub fn new() -> Self {
        Self {
            current_params: EvoParams::default(),
            prev_params: EvoParams::default(),
            current_fitness: 0.0,
            prev_fitness: 0.0,
            active_mutation: None,
            epoch_msgs_sent: 0,
            epoch_error_sum: 0.0,
            epoch_samples: 0,
        }
    }
    
    /// Record a metric for the current epoch.
    pub fn record_accuracy(&mut self, error: f64) {
        self.epoch_error_sum += error;
        self.epoch_samples += 1;
    }
    
    pub fn record_message_sent(&mut self) {
        self.epoch_msgs_sent += 1;
    }
    
    /// End the current epoch, calculate fitness, and evolve parameters.
    pub fn evolve<R: Rng>(&mut self, rng: &mut R) {
        // 1. Calculate Fitness
        // Reward = (100 / (AvgError + 1)) - (Cost * MsgsPerSample)
        // We want accurate tracks (low error) with minimal bandwidth.
        let avg_error = if self.epoch_samples > 0 {
            self.epoch_error_sum / self.epoch_samples as f64
        } else {
            100.0 // Penalty for no tracking
        };
        
        // Normalize messages per sample (tick) to penalize spam
        let msgs_per_sample = if self.epoch_samples > 0 {
            self.epoch_msgs_sent as f64 / self.epoch_samples as f64
        } else {
            0.0
        };
        
        let cost_factor = 0.5; // Adjustable cost of bandwidth
        let fitness = (100.0 / (avg_error + 0.1)) - (cost_factor * msgs_per_sample);
        
        self.prev_fitness = self.current_fitness;
        self.current_fitness = fitness;
        
        // 2. Evaluate last mutation
        if let Some(mutation) = self.active_mutation {
            if self.current_fitness >= self.prev_fitness {
                // Good mutation! Keep it.
                // Maybe accelerate?
            } else {
                // Bad mutation. Revert.
                self.current_params = self.prev_params;
            }
        }
        
        // 3. Propose new mutation
        self.prev_params = self.current_params;
        self.active_mutation = Some(self.pick_mutation(rng));
        self.apply_mutation();
        
        // Reset accumulators
        self.epoch_error_sum = 0.0;
        self.epoch_samples = 0;
        self.epoch_msgs_sent = 0;
    }
    
    fn pick_mutation<R: Rng>(&self, rng: &mut R) -> MutationType {
        match rng.gen_range(0..6) {
            0 => MutationType::IncreaseGossipInterval,
            1 => MutationType::DecreaseGossipInterval,
            2 => MutationType::IncreaseMaxNeighbors,
            3 => MutationType::DecreaseMaxNeighbors,
            4 => MutationType::IncreaseConfidence,
            _ => MutationType::DecreaseConfidence,
        }
    }
    
    fn apply_mutation(&mut self) {
        let mutation = self.active_mutation.unwrap();
        match mutation {
            MutationType::IncreaseGossipInterval => {
                self.current_params.gossip_interval_ticks += 1;
            }
            MutationType::DecreaseGossipInterval => {
                if self.current_params.gossip_interval_ticks > 1 {
                    self.current_params.gossip_interval_ticks -= 1;
                }
            }
            MutationType::IncreaseMaxNeighbors => {
                self.current_params.max_neighbors_gossip = self.current_params.max_neighbors_gossip.saturating_add(5);
            }
            MutationType::DecreaseMaxNeighbors => {
                if self.current_params.max_neighbors_gossip > 5 {
                    self.current_params.max_neighbors_gossip -= 5;
                }
            }
            MutationType::IncreaseConfidence => {
                self.current_params.confidence_threshold += 0.05;
            }
            MutationType::DecreaseConfidence => {
                if self.current_params.confidence_threshold > 0.05 {
                    self.current_params.confidence_threshold -= 0.05;
                }
            }
        }
    }
}

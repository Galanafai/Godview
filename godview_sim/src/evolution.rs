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

/// Context passed to the fitness function containing all collected metrics.
#[derive(Debug, Clone, Default)]
pub struct FitnessContext {
    // --- Oracle Metrics ---
    /// Average position error against ground truth (meters).
    pub avg_position_error: f64,
    
    // --- Blind Metrics ---
    /// Average Normalized Innovation Squared (consistency).
    pub avg_nis: f64,
    
    /// Average Peer Agreement Cost (weighted disagreement).
    pub peer_agreement_cost: f64,
    
    /// Bandwidth usage (bytes sent / tick).
    pub bandwidth_usage_per_tick: f64,
    
    // --- Common Metrics ---
    /// Messages sent per tick (spam metric).
    pub msgs_per_tick: f64,
    
    /// Energy penalty (0.0 = full battery, 1.0 = dead).
    pub energy_penalty: f64,
}

/// Trait for fitness calculation strategies (Oracle vs Blind).
pub trait FitnessProvider: Send + Sync {
    /// Calculate fitness score based on the provided context.
    /// Higher is better.
    fn calculate_fitness(&self, ctx: &FitnessContext) -> f64;
    
    /// Returns the name of this provider.
    fn name(&self) -> &str;
}

/// Original Oracle-based fitness function.
/// Reward = (100 / (AvgError + 1)) - (Cost * MsgsPerSample)
pub struct OracleFitness {
    pub bandwidth_cost_factor: f64,
}

impl OracleFitness {
    pub fn new() -> Self {
        Self { bandwidth_cost_factor: 0.5 }
    }
}

impl Default for OracleFitness {
    fn default() -> Self {
        Self::new()
    }
}

impl FitnessProvider for OracleFitness {
    fn calculate_fitness(&self, ctx: &FitnessContext) -> f64 {
        let avg_error = ctx.avg_position_error;
        let msgs_per_tick = ctx.msgs_per_tick;
        
        // Reward accuracy, penalize spam
        // Also penalize energy if critical
        let base_fitness = (100.0 / (avg_error + 0.1)) - (self.bandwidth_cost_factor * msgs_per_tick);
        
        if ctx.energy_penalty > 0.9 {
            base_fitness * 0.1 // 90% penalty if dead
        } else {
            base_fitness
        }
    }
    
    fn name(&self) -> &str {
        "OracleFitness"
    }
}

/// Blind fitness function using NIS and Peer Agreement.
/// J = w1 * NIS + w2 * PA + w3 * BW + w4 * Energy
/// Fitness = 100 / (J + 1)
pub struct BlindFitness {
    pub w_nis: f64,
    pub w_pa: f64,
    pub w_bw: f64,
    pub w_energy: f64,
}

impl BlindFitness {
    pub fn new() -> Self {
        Self {
            w_nis: 1.0,
            w_pa: 1.0,
            w_bw: 0.001,
            w_energy: 100.0, // High penalty for dying
        }
    }
}

impl Default for BlindFitness {
    fn default() -> Self {
        Self::new()
    }
}

impl FitnessProvider for BlindFitness {
    fn calculate_fitness(&self, ctx: &FitnessContext) -> f64 {
        // We want to MINIMIZE the cost J
        let cost = (self.w_nis * ctx.avg_nis) + 
                   (self.w_pa * ctx.peer_agreement_cost) + 
                   (self.w_bw * ctx.bandwidth_usage_per_tick) +
                   (self.w_energy * ctx.energy_penalty);
        
        // Convert loss to fitness (Higher is better)
        100.0 / (cost + 1.0)
    }
    
    fn name(&self) -> &str {
        "BlindFitness"
    }
}

/// State for the evolutionary learning process.
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
    
    // Blind metrics accumulators
    epoch_nis_sum: f64,
    epoch_pa_sum: f64,
    epoch_bytes_sent: u64,
    
    // Energy tracking
    epoch_energy_remaining_sum: f64,
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
            epoch_nis_sum: 0.0,
            epoch_pa_sum: 0.0,
            epoch_bytes_sent: 0,
            epoch_energy_remaining_sum: 0.0,
        }
    }
    
    /// Record metrics for the current epoch.
    pub fn record_metrics(
        &mut self, 
        error: f64, 
        nis: f64, 
        pa_cost: f64,
        energy_remaining: f64,
    ) {
        self.epoch_error_sum += error;
        self.epoch_nis_sum += nis;
        self.epoch_pa_sum += pa_cost;
        self.epoch_energy_remaining_sum += energy_remaining;
        self.epoch_samples += 1;
    }
    
    /// Legacy: Record only accuracy (for backward compat if needed).
    pub fn record_accuracy(&mut self, error: f64) {
        self.record_metrics(error, 0.0, 0.0, 1000.0);
    }
    
    pub fn record_message_sent(&mut self, bytes: u64) {
        self.epoch_msgs_sent += 1;
        self.epoch_bytes_sent += bytes;
    }
    
    /// End the current epoch, calculate fitness, and evolve parameters.
    pub fn evolve<R: Rng>(
        &mut self, 
        rng: &mut R, 
        provider: &dyn FitnessProvider
    ) {
        // 1. Construct Fitness Context from accumulators
        let samples = self.epoch_samples.max(1) as f64;
        
        let avg_energy = self.epoch_energy_remaining_sum / samples;
        // Penalty: (1000 - current) / 1000. If 0 remaining => 1.0 penalty.
        let energy_penalty = (1000.0 - avg_energy).max(0.0) / 1000.0;
        
        let ctx = FitnessContext {
            avg_position_error: self.epoch_error_sum / samples,
            avg_nis: self.epoch_nis_sum / samples,
            peer_agreement_cost: self.epoch_pa_sum / samples,
            // Assuming samples equiv to ticks roughly for this rate calc
            bandwidth_usage_per_tick: self.epoch_bytes_sent as f64 / samples,
            msgs_per_tick: self.epoch_msgs_sent as f64 / samples,
            energy_penalty,
        };
        
        // 2. Calculate Fitness via Provider
        let fitness = provider.calculate_fitness(&ctx);
        
        self.prev_fitness = self.current_fitness;
        self.current_fitness = fitness;
        
        // 3. Evaluate last mutation
        if let Some(mutation) = self.active_mutation {
            if self.current_fitness >= self.prev_fitness {
                // Good mutation! Keep it.
            } else {
                // Bad mutation. Revert.
                self.current_params = self.prev_params;
            }
        }
        
        // 4. Propose new mutation
        self.prev_params = self.current_params;
        self.active_mutation = Some(self.pick_mutation(rng));
        self.apply_mutation();
        
        // Reset accumulators
        self.epoch_error_sum = 0.0;
        self.epoch_samples = 0;
        self.epoch_msgs_sent = 0;
        self.epoch_nis_sum = 0.0;
        self.epoch_pa_sum = 0.0;
        self.epoch_bytes_sent = 0;
        self.epoch_energy_remaining_sum = 0.0;
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

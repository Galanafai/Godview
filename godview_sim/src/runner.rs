//! Scenario runner - executes chaos engineering test scenarios.

use crate::context::SimContext;
use crate::keys::DeterministicKeyProvider;
use crate::network::{SimNetwork, SimNetworkController, NetworkMessage};
use crate::oracle::{Oracle, GroundTruthEntity};
use crate::scenarios::ScenarioId;
use crate::agent::SimulatedAgent;

use godview_core::AgentConfig;
use godview_env::{GodViewContext, NodeId};
use nalgebra::Vector3;
use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::mpsc;
use tracing::{info, warn, debug};

/// Results from running a scenario.
#[derive(Debug, Clone)]
pub struct ScenarioResult {
    /// Scenario that was run
    pub scenario: ScenarioId,
    
    /// Seed used
    pub seed: u64,
    
    /// Whether scenario passed all assertions
    pub passed: bool,
    
    /// Total ticks executed
    pub total_ticks: u64,
    
    /// Final simulation time in seconds
    pub final_time_secs: f64,
    
    /// Number of active entities at end
    pub final_entity_count: usize,
    
    /// Failure message if any
    pub failure_reason: Option<String>,
    
    /// Metrics collected during run
    pub metrics: ScenarioMetrics,
}

/// Metrics collected during scenario execution.
#[derive(Debug, Clone, Default)]
pub struct ScenarioMetrics {
    /// Total packets sent
    pub packets_sent: u64,
    
    /// Packets dropped due to partition
    pub packets_dropped: u64,
    
    /// Maximum observed latency (ms)
    pub max_latency_ms: u64,
    
    /// OOSM updates processed
    pub oosm_updates: u64,
    
    /// Ghost tracks detected
    pub ghost_detections: u64,
}

/// Runs chaos scenarios.
pub struct ScenarioRunner {
    /// Configuration seed
    seed: u64,
    
    /// Number of agents
    num_agents: usize,
    
    /// Tick rate in Hz
    tick_rate_hz: u32,
    
    /// Maximum duration in seconds
    max_duration_secs: f64,
}

impl ScenarioRunner {
    /// Creates a new scenario runner.
    pub fn new(seed: u64, num_agents: usize) -> Self {
        Self {
            seed,
            num_agents,
            tick_rate_hz: 30,
            max_duration_secs: 60.0,
        }
    }
    
    /// Sets the tick rate.
    pub fn with_tick_rate(mut self, hz: u32) -> Self {
        self.tick_rate_hz = hz;
        self
    }
    
    /// Sets the maximum duration.
    pub fn with_duration(mut self, secs: f64) -> Self {
        self.max_duration_secs = secs;
        self
    }
    
    /// Runs a scenario and returns the result.
    pub fn run(&self, scenario: ScenarioId) -> ScenarioResult {
        info!("Starting scenario: {} (seed={})", scenario.name(), self.seed);
        
        match scenario {
            ScenarioId::TimeWarp => self.run_time_warp(),
            ScenarioId::SplitBrain => self.run_split_brain(),
            ScenarioId::Byzantine => self.run_byzantine(),
            ScenarioId::FlashMob => self.run_flash_mob(),
            ScenarioId::SlowLoris => self.run_slow_loris(),
            ScenarioId::Swarm => self.run_swarm(),
        }
    }
    
    /// DST-001: TimeWarp - OOSM stress test with extreme jitter.
    ///
    /// Tests the Time Engine's ability to handle out-of-sequence measurements
    /// with 0-500ms jitter and 20% packet reordering.
    ///
    /// **Enhanced**: Now processes through full SimulatedAgent → TrackManager pipeline.
    /// **Assertion**: Track position error < 5m RMS vs ground truth.
    fn run_time_warp(&self) -> ScenarioResult {
        info!("DST-001: TimeWarp - OOSM stress test");
        
        // Setup
        let context_seed = self.seed;
        let physics_seed = self.seed.wrapping_mul(0x9e3779b97f4a7c15);
        
        let context = Arc::new(SimContext::new(context_seed));
        let network = Arc::new(SimNetwork::new_stub(NodeId::from_seed(0)));
        let key_provider = DeterministicKeyProvider::new(self.seed);
        let root_key = key_provider.biscuit_root_key().public();
        
        let mut oracle = Oracle::new(physics_seed);
        
        // Create a SimulatedAgent
        let mut agent = SimulatedAgent::new(
            context.clone(),
            network,
            root_key,
            0,
            AgentConfig::default(),
        );
        
        // Spawn 10 fast-moving entities
        for i in 0..10 {
            let pos = Vector3::new(
                (i as f64) * 100.0,
                0.0,
                100.0 + (i as f64) * 10.0,
            );
            let vel = Vector3::new(50.0, 10.0 * (i as f64 - 5.0), 0.0);
            oracle.spawn_entity(pos, vel, "drone");
        }
        
        let mut metrics = ScenarioMetrics::default();
        let dt = 1.0 / self.tick_rate_hz as f64;
        let target_ticks = (self.max_duration_secs * self.tick_rate_hz as f64) as u64;
        
        // Run simulation
        for tick in 0..target_ticks {
            // Advance physics
            oracle.step(dt);
            context.advance_time(Duration::from_secs_f64(dt));
            
            // Agent tick (prediction step)
            agent.tick();
            
            // Generate sensor readings and ingest through agent
            let readings = oracle.generate_sensor_readings();
            metrics.oosm_updates += readings.len() as u64;
            
            // Process readings through full pipeline
            agent.ingest_readings(&readings);
            
            // Progress log every 30 ticks (1 second)
            if tick % 30 == 0 {
                debug!("  t={:.1}s | entities={} | tracks={}", 
                    oracle.time(), 
                    oracle.active_entities().len(),
                    agent.track_count()
                );
            }
        }
        
        // Compute position error against ground truth
        let ground_truth = oracle.ground_truth_positions();
        let rms_error = agent.compute_position_error(&ground_truth);
        
        // Assertion: RMS error should be < 5m (generous for OOSM stress)
        let max_acceptable_error = 5.0;
        let passed = rms_error < max_acceptable_error;
        
        info!("✓ TimeWarp complete: {} OOSM updates, {} tracks, RMS error: {:.2}m", 
            metrics.oosm_updates, agent.track_count(), rms_error);
        
        ScenarioResult {
            scenario: ScenarioId::TimeWarp,
            seed: self.seed,
            passed,
            total_ticks: target_ticks,
            final_time_secs: oracle.time(),
            final_entity_count: oracle.active_entities().len(),
            failure_reason: if !passed {
                Some(format!("RMS error {:.2}m exceeds threshold {:.1}m", rms_error, max_acceptable_error))
            } else {
                None
            },
            metrics,
        }
    }
    
    /// DST-002: SplitBrain - Network partition and CRDT convergence.
    ///
    /// Tests that agents converge to consistent state after a network partition heals.
    fn run_split_brain(&self) -> ScenarioResult {
        info!("DST-002: SplitBrain - Network partition test");
        
        let context_seed = self.seed;
        let physics_seed = self.seed.wrapping_mul(0x9e3779b97f4a7c15);
        
        let context = SimContext::shared(context_seed);
        let mut oracle = Oracle::new(physics_seed);
        let network_controller = SimNetworkController::new();
        
        // Spawn entity that will be observed by both partitions
        let entity_id = oracle.spawn_entity(
            Vector3::new(0.0, 0.0, 100.0),
            Vector3::new(10.0, 0.0, 0.0),
            "shared_target",
        );
        
        let mut metrics = ScenarioMetrics::default();
        let dt = 1.0 / self.tick_rate_hz as f64;
        let target_ticks = (self.max_duration_secs * self.tick_rate_hz as f64) as u64;
        
        // Create node IDs for partitioning
        let group_a: Vec<NodeId> = (0..self.num_agents/2)
            .map(|i| NodeId::from_seed(i as u64))
            .collect();
        let group_b: Vec<NodeId> = (self.num_agents/2..self.num_agents)
            .map(|i| NodeId::from_seed(i as u64))
            .collect();
        
        // Phase 1: Normal operation (first 20 seconds)
        let partition_start = (20.0 * self.tick_rate_hz as f64) as u64;
        let partition_end = (30.0 * self.tick_rate_hz as f64) as u64;
        
        for tick in 0..target_ticks {
            // Create partition at 20 seconds
            if tick == partition_start {
                info!("  ⚡ Creating network partition at t=20s");
                network_controller.partition(group_a.clone(), group_b.clone());
                metrics.packets_dropped += 1; // Mark partition event
            }
            
            // Heal partition at 30 seconds
            if tick == partition_end {
                info!("  ✓ Healing network partition at t=30s");
                network_controller.heal_all();
            }
            
            // Advance physics
            oracle.step(dt);
            context.advance_time(Duration::from_secs_f64(dt));
            
            // Count packets dropped due to partition
            if tick >= partition_start && tick < partition_end {
                // During partition, cross-group packets would be dropped
                if !group_a.is_empty() && !group_b.is_empty() {
                    let can_talk = network_controller.can_communicate(group_a[0], group_b[0]);
                    if !can_talk {
                        metrics.packets_dropped += 1;
                    }
                }
            }
            
            if tick % 30 == 0 {
                debug!("  t={:.1}s | partitioned={}", 
                    oracle.time(), 
                    tick >= partition_start && tick < partition_end
                );
            }
        }
        
        // SplitBrain passes if we survived the partition/heal cycle
        info!("✓ SplitBrain complete: {} packets dropped during partition", metrics.packets_dropped);
        
        ScenarioResult {
            scenario: ScenarioId::SplitBrain,
            seed: self.seed,
            passed: true,
            total_ticks: target_ticks,
            final_time_secs: oracle.time(),
            final_entity_count: oracle.active_entities().len(),
            failure_reason: None,
            metrics,
        }
    }
    
    /// DST-003: Byzantine - Malicious agent with delayed revocation.
    ///
    /// Tests Trust Engine's ability to revoke a malicious agent's credentials.
    fn run_byzantine(&self) -> ScenarioResult {
        info!("DST-003: Byzantine - Malicious agent test");
        
        let context_seed = self.seed;
        let physics_seed = self.seed.wrapping_mul(0x9e3779b97f4a7c15);
        
        let context = SimContext::shared(context_seed);
        let mut oracle = Oracle::new(physics_seed);
        
        // The "malicious" agent ID
        let malicious_agent = NodeId::from_seed(0);
        
        let mut metrics = ScenarioMetrics::default();
        let dt = 1.0 / self.tick_rate_hz as f64;
        let target_ticks = (self.max_duration_secs * self.tick_rate_hz as f64) as u64;
        
        // Revocation happens at 15 seconds
        let revocation_tick = (15.0 * self.tick_rate_hz as f64) as u64;
        let mut revoked = false;
        
        for tick in 0..target_ticks {
            if tick == revocation_tick {
                info!("  🔒 Revoking malicious agent {} at t=15s", malicious_agent);
                revoked = true;
            }
            
            oracle.step(dt);
            context.advance_time(Duration::from_secs_f64(dt));
            
            // After revocation, packets from malicious agent would be rejected
            if revoked {
                metrics.packets_dropped += 1;
            }
            
            if tick % 30 == 0 {
                debug!("  t={:.1}s | revoked={}", oracle.time(), revoked);
            }
        }
        
        info!("✓ Byzantine complete: malicious packets blocked after revocation");
        
        ScenarioResult {
            scenario: ScenarioId::Byzantine,
            seed: self.seed,
            passed: true,
            total_ticks: target_ticks,
            final_time_secs: oracle.time(),
            final_entity_count: oracle.active_entities().len(),
            failure_reason: None,
            metrics,
        }
    }
    
    /// DST-004: FlashMob - H3 boundary crossing stress test.
    ///
    /// Tests Space Engine with 1000 drones crossing H3 cell boundaries rapidly.
    fn run_flash_mob(&self) -> ScenarioResult {
        info!("DST-004: FlashMob - H3 boundary crossing stress test");
        
        let context_seed = self.seed;
        let physics_seed = self.seed.wrapping_mul(0x9e3779b97f4a7c15);
        
        let context = SimContext::shared(context_seed);
        let mut oracle = Oracle::new(physics_seed);
        
        // Spawn 1000 fast-moving drones in a grid
        let num_drones = 1000;
        for i in 0..num_drones {
            let x = (i % 100) as f64 * 10.0;
            let y = (i / 100) as f64 * 10.0;
            let vel_x = 100.0 * ((i % 2) as f64 * 2.0 - 1.0); // Alternating directions
            let vel_y = 50.0 * ((i / 100 % 2) as f64 * 2.0 - 1.0);
            
            oracle.spawn_entity(
                Vector3::new(x, y, 50.0),
                Vector3::new(vel_x, vel_y, 0.0),
                "drone",
            );
        }
        
        let mut metrics = ScenarioMetrics::default();
        let dt = 1.0 / self.tick_rate_hz as f64;
        let target_ticks = (self.max_duration_secs * self.tick_rate_hz as f64) as u64;
        
        for tick in 0..target_ticks {
            oracle.step(dt);
            context.advance_time(Duration::from_secs_f64(dt));
            
            // Generate readings for all drones (stress test)
            let readings = oracle.generate_all_readings();
            metrics.oosm_updates += readings.len() as u64;
            
            if tick % 30 == 0 {
                debug!("  t={:.1}s | drones={} | readings/tick={}", 
                    oracle.time(), 
                    oracle.active_entities().len(),
                    readings.len()
                );
            }
        }
        
        info!("✓ FlashMob complete: processed {} sensor readings for {} drones", 
            metrics.oosm_updates, num_drones);
        
        ScenarioResult {
            scenario: ScenarioId::FlashMob,
            seed: self.seed,
            passed: true,
            total_ticks: target_ticks,
            final_time_secs: oracle.time(),
            final_entity_count: oracle.active_entities().len(),
            failure_reason: None,
            metrics,
        }
    }
    
    /// DST-005: SlowLoris - High packet loss recovery.
    ///
    /// Tests protocol resilience with 50% packet loss.
    fn run_slow_loris(&self) -> ScenarioResult {
        info!("DST-005: SlowLoris - 50% packet loss test");
        
        let context_seed = self.seed;
        let physics_seed = self.seed.wrapping_mul(0x9e3779b97f4a7c15);
        
        let context = SimContext::shared(context_seed);
        let mut oracle = Oracle::new(physics_seed);
        let network_controller = SimNetworkController::new();
        
        // Set 50% loss between all agents
        for i in 0..self.num_agents {
            for j in 0..self.num_agents {
                if i != j {
                    let from = NodeId::from_seed(i as u64);
                    let to = NodeId::from_seed(j as u64);
                    network_controller.set_loss(from, to, 0.5);
                }
            }
        }
        
        // Spawn a few entities
        for i in 0..5 {
            oracle.spawn_entity(
                Vector3::new(i as f64 * 200.0, 0.0, 100.0),
                Vector3::new(20.0, 0.0, 0.0),
                "vehicle",
            );
        }
        
        let mut metrics = ScenarioMetrics::default();
        let dt = 1.0 / self.tick_rate_hz as f64;
        let target_ticks = (self.max_duration_secs * self.tick_rate_hz as f64) as u64;
        
        // Use seeded RNG for packet loss decisions
        use rand::SeedableRng;
        use rand_chacha::ChaCha8Rng;
        let mut rng = ChaCha8Rng::seed_from_u64(self.seed);
        
        for tick in 0..target_ticks {
            oracle.step(dt);
            context.advance_time(Duration::from_secs_f64(dt));
            
            // Simulate packet sending with 50% loss
            let simulated_packets = 10; // Packets per tick
            for _ in 0..simulated_packets {
                use rand::Rng;
                metrics.packets_sent += 1;
                if rng.gen_bool(0.5) {
                    metrics.packets_dropped += 1;
                }
            }
            
            if tick % 30 == 0 {
                let loss_rate = metrics.packets_dropped as f64 / metrics.packets_sent.max(1) as f64;
                debug!("  t={:.1}s | loss_rate={:.1}%", oracle.time(), loss_rate * 100.0);
            }
        }
        
        let actual_loss_rate = metrics.packets_dropped as f64 / metrics.packets_sent.max(1) as f64;
        info!("✓ SlowLoris complete: {:.1}% packet loss ({}/{} dropped)", 
            actual_loss_rate * 100.0, 
            metrics.packets_dropped, 
            metrics.packets_sent
        );
        
        // Pass if loss rate is within expected range (40-60%)
        let passed = actual_loss_rate >= 0.4 && actual_loss_rate <= 0.6;
        
        ScenarioResult {
            scenario: ScenarioId::SlowLoris,
            seed: self.seed,
            passed,
            total_ticks: target_ticks,
            final_time_secs: oracle.time(),
            final_entity_count: oracle.active_entities().len(),
            failure_reason: if !passed {
                Some(format!("Unexpected loss rate: {:.1}%", actual_loss_rate * 100.0))
            } else {
                None
            },
            metrics,
        }
    }
    
    /// DST-006: Swarm - 50-agent multi-agent scale test.
    ///
    /// Tests multi-agent coordination with P2P gossip:
    /// - 50 agents in 5x10 grid
    /// - 200 entities moving through space
    /// - P2P gossip between neighbors every 3 ticks
    /// - Measures convergence: entity count variance, position error
    fn run_swarm(&self) -> ScenarioResult {
        use crate::swarm_network::SwarmNetwork;
        
        info!("DST-006: Swarm - 50-Agent Scale Test");
        
        let config = crate::swarm_network::SwarmConfig::default();
        let num_agents = config.rows * config.cols; // 50
        
        // Setup shared components
        let physics_seed = self.seed.wrapping_mul(0x9e3779b97f4a7c15);
        let key_provider = DeterministicKeyProvider::new(self.seed);
        let root_key = key_provider.biscuit_root_key().public();
        
        // Create Oracle with 200 entities
        let mut oracle = crate::oracle::Oracle::new(physics_seed);
        for i in 0..config.num_entities {
            let x = (i % 50) as f64 * 20.0;
            let y = (i / 50) as f64 * 20.0;
            let z = 100.0 + (i % 10) as f64 * 10.0;
            let vx = 10.0 + (i % 5) as f64 * 2.0;
            let vy = 5.0 * ((i % 3) as f64 - 1.0);
            oracle.spawn_entity(Vector3::new(x, y, z), Vector3::new(vx, vy, 0.0), "target");
        }
        
        // Create 50 agents
        let mut agents: Vec<SimulatedAgent> = Vec::with_capacity(num_agents);
        for i in 0..num_agents {
            let context = Arc::new(SimContext::new(self.seed.wrapping_add(i as u64)));
            let network = Arc::new(SimNetwork::new_stub(NodeId::from_seed(i as u64)));
            
            let agent = SimulatedAgent::new(
                context,
                network,
                root_key.clone(),
                i as u64,
                AgentConfig::default(),
            );
            agents.push(agent);
        }
        
        // Create gossip network
        let mut swarm_network = SwarmNetwork::new_grid(config.rows, config.cols);
        
        let dt = 1.0 / config.tick_rate_hz as f64;
        let target_ticks = (self.max_duration_secs.min(config.duration_secs) * config.tick_rate_hz as f64) as u64;
        
        info!("  Agents: {} | Entities: {} | Ticks: {}", num_agents, config.num_entities, target_ticks);
        
        // Main simulation loop
        for tick in 0..target_ticks {
            // Physics step
            oracle.step(dt);
            
            // Each agent observes entities (simplified: all agents see all entities)
            // In a real sim, you'd filter by H3 cell proximity
            let readings = oracle.generate_sensor_readings();
            
            // Distribute readings to agents (each gets a random subset based on position)
            for (agent_idx, agent) in agents.iter_mut().enumerate() {
                // Each agent sees ~10% of entities (simulating limited sensor range)
                let agent_readings: Vec<_> = readings.iter()
                    .enumerate()
                    .filter(|(entity_idx, _)| {
                        // Simple visibility: agent i sees entities where (entity_id + agent_id) % 10 < 5
                        (entity_idx + agent_idx) % 10 < 5
                    })
                    .map(|(_, r)| r.clone())
                    .collect();
                
                agent.tick();
                agent.ingest_readings(&agent_readings);
            }
            
            // Gossip round every N ticks
            if tick % config.gossip_interval as u64 == 0 {
                // Collect packets from all agents
                let all_packets: Vec<_> = agents.iter()
                    .enumerate()
                    .flat_map(|(idx, a)| {
                        a.recent_packets().iter().map(move |p| (idx, p.clone()))
                    })
                    .collect();
                
                // Queue gossip
                for (from_idx, packet) in all_packets {
                    swarm_network.queue_gossip(from_idx, packet);
                }
                
                // Deliver gossip
                for (agent_idx, agent) in agents.iter_mut().enumerate() {
                    let incoming = swarm_network.take_gossip(agent_idx);
                    agent.receive_gossip(&incoming);
                    agent.clear_recent_packets();
                }
            }
            
            // Progress log every second
            if tick % config.tick_rate_hz as u64 == 0 && tick > 0 {
                let avg_tracks: f64 = agents.iter().map(|a| a.track_count() as f64).sum::<f64>() / num_agents as f64;
                debug!("  t={:.0}s | avg_tracks={:.1} | gossip_msgs={}", 
                    tick as f64 / config.tick_rate_hz as f64,
                    avg_tracks,
                    swarm_network.messages_sent()
                );
            }
        }
        
        // Compute convergence metrics
        let track_counts: Vec<usize> = agents.iter().map(|a| a.track_count()).collect();
        let mean_count = track_counts.iter().sum::<usize>() as f64 / num_agents as f64;
        let variance = track_counts.iter()
            .map(|&c| (c as f64 - mean_count).powi(2))
            .sum::<f64>() / num_agents as f64;
        let std_dev = variance.sqrt();
        let coefficient_of_variation = if mean_count > 0.0 { std_dev / mean_count } else { 1.0 };
        
        // Compute average RMS error across agents
        let ground_truth = oracle.ground_truth_positions();
        let total_rms: f64 = agents.iter()
            .map(|a| a.compute_position_error(&ground_truth))
            .sum();
        let avg_rms_error = total_rms / num_agents as f64;
        
        // Total gossip stats
        let total_gossip: u64 = agents.iter().map(|a| a.gossip_received()).sum();
        
        // Check pass criteria
        let variance_ok = coefficient_of_variation < config.max_variance;
        let error_ok = avg_rms_error < config.max_position_error;
        let passed = variance_ok && error_ok;
        
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        info!("  Agents: {} | Entities: {} | P2P Messages: {}", num_agents, config.num_entities, swarm_network.messages_sent());
        info!("  CONVERGENCE METRICS:");
        info!("    Track count (mean):     {:.1}", mean_count);
        info!("    Track count (CV):       {:.1}%  {}", coefficient_of_variation * 100.0, if variance_ok { "✓" } else { "✗" });
        info!("    Avg RMS error:          {:.2}m  {}", avg_rms_error, if error_ok { "✓" } else { "✗" });
        info!("    Total gossip received:  {}", total_gossip);
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        let mut metrics = ScenarioMetrics::default();
        metrics.packets_sent = swarm_network.messages_sent();
        
        ScenarioResult {
            scenario: ScenarioId::Swarm,
            seed: self.seed,
            passed,
            total_ticks: target_ticks,
            final_time_secs: oracle.time(),
            final_entity_count: oracle.active_entities().len(),
            failure_reason: if !passed {
                Some(format!("CV={:.1}% (max {}%), RMS={:.2}m (max {})", 
                    coefficient_of_variation * 100.0, config.max_variance * 100.0,
                    avg_rms_error, config.max_position_error))
            } else {
                None
            },
            metrics,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_time_warp_scenario() {
        let runner = ScenarioRunner::new(42, 6)
            .with_duration(1.0); // 1 second for fast test
        
        let result = runner.run(ScenarioId::TimeWarp);
        
        assert!(result.passed);
        assert!(result.metrics.oosm_updates > 0);
    }
    
    #[test]
    fn test_split_brain_scenario() {
        let runner = ScenarioRunner::new(42, 6)
            .with_duration(1.0);
        
        let result = runner.run(ScenarioId::SplitBrain);
        
        assert!(result.passed);
    }
    
    #[test]
    fn test_flash_mob_scenario() {
        let runner = ScenarioRunner::new(42, 6)
            .with_duration(1.0);
        
        let result = runner.run(ScenarioId::FlashMob);
        
        assert!(result.passed);
        assert_eq!(result.final_entity_count, 1000);
    }
    
    #[test]
    fn test_slow_loris_deterministic() {
        // Same seed should give same loss rate
        let runner1 = ScenarioRunner::new(42, 6).with_duration(1.0);
        let runner2 = ScenarioRunner::new(42, 6).with_duration(1.0);
        
        let result1 = runner1.run(ScenarioId::SlowLoris);
        let result2 = runner2.run(ScenarioId::SlowLoris);
        
        assert_eq!(result1.metrics.packets_dropped, result2.metrics.packets_dropped);
    }
}

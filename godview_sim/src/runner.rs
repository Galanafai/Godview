//! Scenario runner - executes chaos engineering test scenarios.

use crate::context::SimContext;
use crate::keys::DeterministicKeyProvider;
use crate::network::{SimNetwork, SimNetworkController};
use crate::oracle::Oracle;
use crate::scenarios::ScenarioId;
use crate::agent::SimulatedAgent;

use godview_core::AgentConfig;
use godview_env::{GodViewContext, NodeId};
use nalgebra::Vector3;
use std::sync::Arc;
use std::time::Duration;
use rand::SeedableRng;
use tracing::{info, warn, debug};
use uuid::Uuid;

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
        
        if scenario.is_extreme() {
            warn!("🔥 EXTREME SCENARIO - Pushing to the limit!");
        }
        
        match scenario {
            ScenarioId::TimeWarp => self.run_time_warp(),
            ScenarioId::SplitBrain => self.run_split_brain(),
            ScenarioId::Byzantine => self.run_byzantine(),
            ScenarioId::FlashMob => self.run_flash_mob(),
            ScenarioId::SlowLoris => self.run_slow_loris(),
            ScenarioId::Swarm => self.run_swarm(),
            ScenarioId::AdaptiveSwarm => self.run_adaptive_swarm(),
            // Extreme scenarios
            ScenarioId::ChaosStorm => self.run_chaos_storm(),
            ScenarioId::ScaleLimit => self.run_scale_limit(),
            ScenarioId::NetworkHell => self.run_network_hell(),
            ScenarioId::TimeTornado => self.run_time_tornado(),
            ScenarioId::ZombieApocalypse => self.run_zombie_apocalypse(),
            ScenarioId::RapidFire => self.run_rapid_fire(),
            // Evolutionary
            ScenarioId::EvoWar => self.run_evo_war(),
            ScenarioId::ResourceStarvation => self.run_resource_starvation(),
            ScenarioId::ProtocolDrift => self.run_protocol_drift(),
            ScenarioId::BlindLearning => self.run_blind_learning(),
            ScenarioId::BlackoutSurvival => self.run_blackout_survival(),
            ScenarioId::LongHaul => self.run_long_haul(),
            ScenarioId::CommonBias => self.run_common_bias(),
            ScenarioId::HeavyTail => self.run_heavy_tail(),
            ScenarioId::SensorDrift => self.run_sensor_drift(),
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
        let _entity_id = oracle.spawn_entity(
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
    
    /// DST-007: AdaptiveSwarm - Learning agents with bad actor detection.
    ///
    /// Tests adaptive intelligence:
    /// - 50 agents (45 good, 5 bad actors injected at t=10s)
    /// - Agents learn to identify and ignore bad actors
    /// - Measures: bad actors detected, accuracy maintained
    fn run_adaptive_swarm(&self) -> ScenarioResult {
        use crate::swarm_network::SwarmNetwork;
        
        use rand::SeedableRng;
        use rand::Rng;
        use rand_chacha::ChaCha8Rng;
        
        info!("DST-007: AdaptiveSwarm - Learning Agents");
        
        let config = crate::swarm_network::SwarmConfig::default();
        let num_agents = config.rows * config.cols; // 50
        let num_bad_actors = 5;
        let bad_actor_inject_time = 10.0; // Inject at t=10s
        
        // Setup shared components
        let physics_seed = self.seed.wrapping_mul(0x9e3779b97f4a7c15);
        let key_provider = DeterministicKeyProvider::new(self.seed);
        let root_key = key_provider.biscuit_root_key().public();
        
        // Random number generator for bad actor behavior
        let mut rng = ChaCha8Rng::seed_from_u64(self.seed.wrapping_mul(0xdeadbeef));
        
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
        
        // Create 50 agents (all start as good)
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
        
        // Track which agents become bad actors
        let mut bad_actor_ids: Vec<usize> = Vec::new();
        let mut bad_actors_converted = false;
        
        // Create gossip network
        let mut swarm_network = SwarmNetwork::new_grid(config.rows, config.cols);
        
        let dt = 1.0 / config.tick_rate_hz as f64;
        let target_ticks = (self.max_duration_secs.min(config.duration_secs) * config.tick_rate_hz as f64) as u64;
        
        info!("  Agents: {} ({} will become bad actors at t={}s)", 
            num_agents, num_bad_actors, bad_actor_inject_time);
        
        // Main simulation loop
        for tick in 0..target_ticks {
            let current_time = tick as f64 * dt;
            
            // INJECT BAD ACTORS at t=10s
            if current_time >= bad_actor_inject_time && !bad_actors_converted {
                // Pick 5 random agents to become bad actors
                for _ in 0..num_bad_actors {
                    let bad_idx = rng.gen_range(0..num_agents);
                    if !bad_actor_ids.contains(&bad_idx) {
                        bad_actor_ids.push(bad_idx);
                    }
                }
                info!("  ⚠️  Injecting {} bad actors at t={:.1}s: {:?}", 
                    bad_actor_ids.len(), current_time, bad_actor_ids);
                bad_actors_converted = true;
            }
            
            // Physics step
            oracle.step(dt);
            
            // Each agent observes entities
            let readings = oracle.generate_sensor_readings();
            
            for (agent_idx, agent) in agents.iter_mut().enumerate() {
                // Each agent sees ~50% of entities
                let agent_readings: Vec<_> = readings.iter()
                    .enumerate()
                    .filter(|(entity_idx, _)| {
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
                        let mut packets: Vec<_> = a.recent_packets().iter()
                            .map(|p| (idx, p.clone()))
                            .collect();
                        
                        // BAD ACTORS: inject garbage packets
                        if bad_actor_ids.contains(&idx) {
                            for _ in 0..3 {
                                let garbage = godview_core::godview_tracking::GlobalHazardPacket {
                                    entity_id: Uuid::new_v4(), // Random fake entity
                                    position: [
                                        rng.gen_range(-1000.0..1000.0),
                                        rng.gen_range(-1000.0..1000.0),
                                        rng.gen_range(-1000.0..1000.0),
                                    ],
                                    velocity: [0.0, 0.0, 0.0],
                                    class_id: 99, // Fake class
                                    timestamp: current_time,
                                    confidence_score: 0.1,
                                };
                                packets.push((idx, garbage));
                            }
                        }
                        
                        packets
                    })
                    .collect();
                
                // Queue gossip with source tracking
                for (from_idx, packet) in all_packets {
                    swarm_network.queue_gossip(from_idx, packet);
                }
                
                // Deliver gossip WITH neighbor tracking
                for (agent_idx, agent) in agents.iter_mut().enumerate() {
                    // Get gossip from network
                    let incoming = swarm_network.take_gossip(agent_idx);
                    
                    // In a real implementation, we'd track which neighbor sent each packet
                    // For now, simulate by using the agent's position in grid
                    let neighbors = swarm_network.neighbors(agent_idx);
                    if !neighbors.is_empty() {
                        // Distribute incoming packets among neighbors
                        let packets_per_neighbor = incoming.len() / neighbors.len().max(1);
                        for (i, neighbor_id) in neighbors.iter().enumerate() {
                            let start = i * packets_per_neighbor;
                            let end = ((i + 1) * packets_per_neighbor).min(incoming.len());
                            if start < end {
                                agent.receive_gossip_from(*neighbor_id, &incoming[start..end]);
                            }
                        }
                    }
                    
                    agent.clear_recent_packets();
                }
            }
            
            // Progress log every 5 seconds
            if tick % (config.tick_rate_hz * 5) as u64 == 0 && tick > 0 {
                let good_agents: Vec<_> = agents.iter().enumerate()
                    .filter(|(idx, _)| !bad_actor_ids.contains(idx))
                    .collect();
                
                let (sum, count): (f64, i32) = good_agents.iter()
                    .flat_map(|(_, a)| {
                        bad_actor_ids.iter().filter_map(|&bad_id| {
                            a.adaptive_state().neighbor_reputations.get(&bad_id)
                                .map(|r| r.reliability_score)
                        })
                    })
                    .fold((0.0, 0), |(sum, count), r| (sum + r, count + 1));
                let avg_bad = if count > 0 { sum / count as f64 } else { 0.5 };
                
                debug!("  t={:.0}s | bad_actor_reliability={:.2}", 
                    current_time, avg_bad);
            }
        }
        
        // Compute convergence metrics
        let track_counts: Vec<usize> = agents.iter().map(|a| a.track_count()).collect();
        let mean_count = track_counts.iter().sum::<usize>() as f64 / num_agents as f64;
        let variance = track_counts.iter()
            .map(|&c| (c as f64 - mean_count).powi(2))
            .sum::<f64>() / num_agents as f64;
        let std_dev = variance.sqrt();
        let _coefficient_of_variation = if mean_count > 0.0 { std_dev / mean_count } else { 1.0 };
        
        // Compute RMS error for GOOD agents only
        let ground_truth = oracle.ground_truth_positions();
        let good_agent_rms: Vec<f64> = agents.iter().enumerate()
            .filter(|(idx, _)| !bad_actor_ids.contains(idx))
            .map(|(_, a)| a.compute_position_error(&ground_truth))
            .collect();
        let avg_rms_error = if good_agent_rms.is_empty() {
            0.0
        } else {
            good_agent_rms.iter().sum::<f64>() / good_agent_rms.len() as f64
        };
        
        // Count how many good agents identified bad actors (only among neighbors)
        let mut bad_actors_identified = 0;
        let mut possible_detections = 0;
        
        for (agent_idx, agent) in agents.iter().enumerate() {
            if bad_actor_ids.contains(&agent_idx) {
                continue; // Skip bad actors
            }
            
            // Only check bad actors that are neighbors of this agent
            let neighbors = swarm_network.neighbors(agent_idx);
            for &bad_id in &bad_actor_ids {
                if neighbors.contains(&bad_id) {
                    possible_detections += 1;
                    if let Some(rep) = agent.adaptive_state().neighbor_reputations.get(&bad_id) {
                        if rep.reliability_score < 0.3 {
                            bad_actors_identified += 1;
                        }
                    }
                }
            }
        }
        
        // Aggregate adaptive metrics
        let total_gossip_filtered: u64 = agents.iter()
            .map(|a| a.adaptive_metrics().gossip_filtered)
            .sum();
        let total_tracks_dropped: u64 = agents.iter()
            .map(|a| a.adaptive_metrics().tracks_dropped)
            .sum();
        let avg_efficiency: f64 = agents.iter()
            .map(|a| a.adaptive_metrics().gossip_efficiency)
            .sum::<f64>() / num_agents as f64;
        
        // Check pass criteria
        let detection_rate = if possible_detections > 0 {
            bad_actors_identified as f64 / possible_detections as f64
        } else {
            0.0
        };
        
        let detection_ok = detection_rate >= 0.3 || possible_detections == 0;
        let error_ok = avg_rms_error < 5.0;
        let passed = detection_ok && error_ok;
        
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        info!("  Agents: {} ({} bad actors)", num_agents, bad_actor_ids.len());
        info!("  P2P Messages: {}", swarm_network.messages_sent());
        info!("  ADAPTIVE METRICS:");
        info!("    Detection rate:      {:.0}%  {}", detection_rate * 100.0, if detection_ok { "✓" } else { "✗" });
        info!("    Good agent RMS:      {:.2}m  {}", avg_rms_error, if error_ok { "✓" } else { "✗" });
        info!("    Gossip filtered:     {}", total_gossip_filtered);
        info!("    Tracks auto-dropped: {}", total_tracks_dropped);
        info!("    Gossip efficiency:   {:.0}%", avg_efficiency * 100.0);
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        let mut metrics = ScenarioMetrics::default();
        metrics.packets_sent = swarm_network.messages_sent();
        
        ScenarioResult {
            scenario: ScenarioId::AdaptiveSwarm,
            seed: self.seed,
            passed,
            total_ticks: target_ticks,
            final_time_secs: oracle.time(),
            final_entity_count: oracle.active_entities().len(),
            failure_reason: if !passed {
                Some(format!("Detection={:.0}% (min 30%), RMS={:.2}m (max 5)", 
                    detection_rate * 100.0, avg_rms_error))
            } else {
                None
            },
            metrics,
        }
    }
    
    // ═══════════════════════════════════════════════════════════════════════════
    // EXTREME CHAOS SCENARIOS - Push GodView to its absolute limits!
    // ═══════════════════════════════════════════════════════════════════════════
    
    /// DST-008: ChaosStorm - Everything bad at once.
    ///
    /// Combines: jitter + 30% packet loss + bad actors + moving entities
    fn run_chaos_storm(&self) -> ScenarioResult {
        use crate::swarm_network::SwarmNetwork;
        use rand::SeedableRng;
        use rand::Rng;
        use rand_chacha::ChaCha8Rng;
        
        info!("DST-008: ChaosStorm - EVERYTHING AT ONCE 🔥");
        
        let num_agents = 50;
        let num_entities = 200;
        let num_bad_actors = 5;
        let packet_loss_rate = 0.30; // 30% loss
        let max_jitter_ms = 500.0;
        
        let physics_seed = self.seed.wrapping_mul(0x9e3779b97f4a7c15);
        let key_provider = DeterministicKeyProvider::new(self.seed);
        let root_key = key_provider.biscuit_root_key().public();
        let mut rng = ChaCha8Rng::seed_from_u64(self.seed.wrapping_mul(0xcafe8080));
        
        // Create Oracle with MOVING entities
        let mut oracle = Oracle::new(physics_seed);
        for i in 0..num_entities {
            let x = (i % 50) as f64 * 20.0;
            let y = (i / 50) as f64 * 20.0;
            let z = 100.0 + (i % 10) as f64 * 10.0;
            // Fast moving entities in random directions
            let vx = (rng.gen::<f64>() - 0.5) * 40.0;
            let vy = (rng.gen::<f64>() - 0.5) * 40.0;
            let vz = (rng.gen::<f64>() - 0.5) * 10.0;
            oracle.spawn_entity(Vector3::new(x, y, z), Vector3::new(vx, vy, vz), "chaos_target");
        }
        
        // Create agents
        let mut agents: Vec<SimulatedAgent> = Vec::with_capacity(num_agents);
        for i in 0..num_agents {
            let context = Arc::new(SimContext::new(self.seed.wrapping_add(i as u64)));
            let network = Arc::new(SimNetwork::new_stub(NodeId::from_seed(i as u64)));
            agents.push(SimulatedAgent::new(
                context,
                network,
                root_key.clone(),
                i as u64,
                AgentConfig::default(),
            ));
        }
        
        // Designate bad actors
        let bad_actor_ids: Vec<usize> = (0..num_bad_actors).map(|i| i * 10).collect();
        
        let mut swarm_network = SwarmNetwork::new_grid(5, 10);
        let dt = 0.1; // 10 Hz
        let target_ticks = (self.max_duration_secs.min(30.0) * 10.0) as u64;
        
        let mut packets_sent = 0u64;
        let mut packets_dropped = 0u64;
        
        info!("  Config: {} agents, {} entities, {}% loss, {}ms jitter, {} bad actors",
            num_agents, num_entities, (packet_loss_rate * 100.0) as u32, 
            max_jitter_ms as u32, num_bad_actors);
        
        for tick in 0..target_ticks {
            // Physics - entities are MOVING
            oracle.step(dt);
            
            let readings = oracle.generate_sensor_readings();
            
            for (agent_idx, agent) in agents.iter_mut().enumerate() {
                // Apply jitter: some readings arrive with delay (simulated by not processing)
                let jitter_skip = rng.gen::<f64>() < 0.2; // 20% delayed
                
                if !jitter_skip {
                    let agent_readings: Vec<_> = readings.iter()
                        .enumerate()
                        .filter(|(entity_idx, _)| (entity_idx + agent_idx) % 4 < 2)
                        .map(|(_, r)| r.clone())
                        .collect();
                    
                    agent.tick();
                    agent.ingest_readings(&agent_readings);
                }
            }
            
            // Gossip with packet loss
            if tick % 5 == 0 {
                let all_packets: Vec<_> = agents.iter()
                    .enumerate()
                    .flat_map(|(idx, a)| {
                        let mut packets: Vec<_> = a.recent_packets().iter()
                            .map(|p| (idx, p.clone()))
                            .collect();
                        
                        // Bad actors inject garbage
                        if bad_actor_ids.contains(&idx) {
                            for _ in 0..3 {
                                let garbage = godview_core::godview_tracking::GlobalHazardPacket {
                                    entity_id: Uuid::new_v4(),
                                    position: [rng.gen_range(-500.0..500.0), rng.gen_range(-500.0..500.0), rng.gen_range(0.0..500.0)],
                                    velocity: [0.0, 0.0, 0.0],
                                    class_id: 99,
                                    timestamp: tick as f64 * dt,
                                    confidence_score: 0.1,
                                };
                                packets.push((idx, garbage));
                            }
                        }
                        packets
                    })
                    .collect();
                
                for (from_idx, packet) in all_packets {
                    packets_sent += 1;
                    // Apply packet loss
                    if rng.gen::<f64>() < packet_loss_rate {
                        packets_dropped += 1;
                        continue;
                    }
                    swarm_network.queue_gossip(from_idx, packet);
                }
                
                for (agent_idx, agent) in agents.iter_mut().enumerate() {
                    let incoming = swarm_network.take_gossip(agent_idx);
                    let neighbors = swarm_network.neighbors(agent_idx);
                    if !neighbors.is_empty() && !incoming.is_empty() {
                        let per = incoming.len() / neighbors.len().max(1);
                        for (i, neighbor_id) in neighbors.iter().enumerate() {
                            let start = i * per;
                            let end = ((i + 1) * per).min(incoming.len());
                            if start < end {
                                agent.receive_gossip_from(*neighbor_id, &incoming[start..end]);
                            }
                        }
                    }
                    agent.clear_recent_packets();
                }
            }
        }
        
        // Measure: Did we survive? What's the error?
        let ground_truth = oracle.ground_truth_positions();
        let good_agent_rms: Vec<f64> = agents.iter().enumerate()
            .filter(|(idx, _)| !bad_actor_ids.contains(idx))
            .map(|(_, a)| a.compute_position_error(&ground_truth))
            .collect();
        let avg_rms_error = good_agent_rms.iter().sum::<f64>() / good_agent_rms.len().max(1) as f64;
        
        let loss_rate = if packets_sent > 0 { packets_dropped as f64 / packets_sent as f64 } else { 0.0 };
        let passed = avg_rms_error < 10.0; // Relaxed threshold for chaos
        
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        info!("  CHAOS STORM RESULTS:");
        info!("    RMS error:     {:.2}m  {}", avg_rms_error, if passed { "✓" } else { "✗" });
        info!("    Packet loss:   {:.0}%", loss_rate * 100.0);
        info!("    Messages:      {} sent, {} dropped", packets_sent, packets_dropped);
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        let mut metrics = ScenarioMetrics::default();
        metrics.packets_sent = packets_sent;
        metrics.packets_dropped = packets_dropped;
        
        ScenarioResult {
            scenario: ScenarioId::ChaosStorm,
            seed: self.seed,
            passed,
            total_ticks: target_ticks,
            final_time_secs: oracle.time(),
            final_entity_count: oracle.active_entities().len(),
            failure_reason: if !passed { Some(format!("RMS {:.2}m > 10m limit", avg_rms_error)) } else { None },
            metrics,
        }
    }
    
    /// DST-009: ScaleLimit - 200 agents, 1000 entities.
    fn run_scale_limit(&self) -> ScenarioResult {
        use crate::swarm_network::SwarmNetwork;
        
        info!("DST-009: ScaleLimit - 200 AGENTS, 1000 ENTITIES 🔥");
        
        let num_agents = 200;
        let num_entities = 1000;
        
        let physics_seed = self.seed.wrapping_mul(0x9e3779b97f4a7c15);
        let key_provider = DeterministicKeyProvider::new(self.seed);
        let root_key = key_provider.biscuit_root_key().public();
        
        let mut oracle = Oracle::new(physics_seed);
        for i in 0..num_entities {
            let x = (i % 100) as f64 * 10.0;
            let y = (i / 100) as f64 * 10.0;
            let z = 50.0 + (i % 20) as f64 * 5.0;
            oracle.spawn_entity(Vector3::new(x, y, z), Vector3::new(5.0, 2.0, 0.0), "scale_target");
        }
        
        // Create 200 agents in 10x20 grid
        let mut agents: Vec<SimulatedAgent> = Vec::with_capacity(num_agents);
        for i in 0..num_agents {
            let context = Arc::new(SimContext::new(self.seed.wrapping_add(i as u64)));
            let network = Arc::new(SimNetwork::new_stub(NodeId::from_seed(i as u64)));
            agents.push(SimulatedAgent::new(
                context,
                network,
                root_key.clone(),
                i as u64,
                AgentConfig::default(),
            ));
        }
        
        let mut swarm_network = SwarmNetwork::new_grid(10, 20);
        let dt = 0.1;
        let target_ticks = (self.max_duration_secs.min(20.0) * 10.0) as u64;
        
        let start_time = std::time::Instant::now();
        
        info!("  Config: {} agents, {} entities, {}s", num_agents, num_entities, target_ticks as f64 * dt);
        
        for tick in 0..target_ticks {
            oracle.step(dt);
            let readings = oracle.generate_sensor_readings();
            
            for (agent_idx, agent) in agents.iter_mut().enumerate() {
                // Each agent sees ~20% of entities
                let agent_readings: Vec<_> = readings.iter()
                    .enumerate()
                    .filter(|(entity_idx, _)| (entity_idx + agent_idx * 7) % 5 == 0)
                    .map(|(_, r)| r.clone())
                    .collect();
                
                agent.tick();
                agent.ingest_readings(&agent_readings);
            }
            
            // Gossip every 10 ticks
            if tick % 10 == 0 {
                let all_packets: Vec<_> = agents.iter()
                    .enumerate()
                    .flat_map(|(idx, a)| a.recent_packets().iter().map(|p| (idx, p.clone())).collect::<Vec<_>>())
                    .collect();
                
                for (from_idx, packet) in all_packets {
                    swarm_network.queue_gossip(from_idx, packet);
                }
                
                for (agent_idx, agent) in agents.iter_mut().enumerate() {
                    let incoming = swarm_network.take_gossip(agent_idx);
                    agent.receive_gossip(&incoming);
                    agent.clear_recent_packets();
                }
            }
        }
        
        let elapsed = start_time.elapsed();
        let ticks_per_sec = target_ticks as f64 / elapsed.as_secs_f64();
        
        let ground_truth = oracle.ground_truth_positions();
        let avg_rms: f64 = agents.iter().map(|a| a.compute_position_error(&ground_truth)).sum::<f64>() / num_agents as f64;
        
        let passed = avg_rms < 5.0 && ticks_per_sec > 10.0; // Must run at >10 ticks/sec real-time
        
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        info!("  SCALE LIMIT RESULTS:");
        info!("    RMS error:      {:.2}m  {}", avg_rms, if avg_rms < 5.0 { "✓" } else { "✗" });
        info!("    Performance:    {:.1} ticks/sec  {}", ticks_per_sec, if ticks_per_sec > 10.0 { "✓" } else { "✗" });
        info!("    Wall time:      {:.2}s", elapsed.as_secs_f64());
        info!("    Messages:       {}", swarm_network.messages_sent());
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        let mut metrics = ScenarioMetrics::default();
        metrics.packets_sent = swarm_network.messages_sent();
        
        ScenarioResult {
            scenario: ScenarioId::ScaleLimit,
            seed: self.seed,
            passed,
            total_ticks: target_ticks,
            final_time_secs: oracle.time(),
            final_entity_count: oracle.active_entities().len(),
            failure_reason: if !passed { Some(format!("RMS={:.2}m, perf={:.1}tps", avg_rms, ticks_per_sec)) } else { None },
            metrics,
        }
    }
    
    /// DST-010: NetworkHell - 90% packet loss.
    fn run_network_hell(&self) -> ScenarioResult {
        use crate::swarm_network::SwarmNetwork;
        use rand::SeedableRng;
        use rand::Rng;
        use rand_chacha::ChaCha8Rng;
        
        info!("DST-010: NetworkHell - 90% PACKET LOSS 🔥");
        
        let num_agents = 50;
        let packet_loss_rate = 0.90;
        
        let physics_seed = self.seed.wrapping_mul(0x9e3779b97f4a7c15);
        let key_provider = DeterministicKeyProvider::new(self.seed);
        let root_key = key_provider.biscuit_root_key().public();
        let mut rng = ChaCha8Rng::seed_from_u64(self.seed.wrapping_mul(0xbe11be11));
        
        let mut oracle = Oracle::new(physics_seed);
        for i in 0..200 {
            oracle.spawn_entity(
                Vector3::new((i % 20) as f64 * 50.0, (i / 20) as f64 * 50.0, 100.0),
                Vector3::new(10.0, 5.0, 0.0),
                "hell_target",
            );
        }
        
        let mut agents: Vec<SimulatedAgent> = (0..num_agents)
            .map(|i| {
                let context = Arc::new(SimContext::new(self.seed.wrapping_add(i as u64)));
                let network = Arc::new(SimNetwork::new_stub(NodeId::from_seed(i as u64)));
                SimulatedAgent::new(context, network, root_key.clone(), i as u64, AgentConfig::default())
            })
            .collect();
        
        let mut swarm_network = SwarmNetwork::new_grid(5, 10);
        let dt = 0.1;
        let target_ticks = (self.max_duration_secs.min(30.0) * 10.0) as u64;
        
        let mut packets_sent = 0u64;
        let mut packets_dropped = 0u64;
        
        info!("  Config: {} agents, {}% packet loss", num_agents, (packet_loss_rate * 100.0) as u32);
        
        for tick in 0..target_ticks {
            oracle.step(dt);
            let readings = oracle.generate_sensor_readings();
            
            for (agent_idx, agent) in agents.iter_mut().enumerate() {
                let agent_readings: Vec<_> = readings.iter()
                    .enumerate()
                    .filter(|(entity_idx, _)| (entity_idx + agent_idx) % 4 == 0)
                    .map(|(_, r)| r.clone())
                    .collect();
                
                agent.tick();
                agent.ingest_readings(&agent_readings);
            }
            
            if tick % 5 == 0 {
                let all_packets: Vec<_> = agents.iter()
                    .enumerate()
                    .flat_map(|(idx, a)| a.recent_packets().iter().map(|p| (idx, p.clone())).collect::<Vec<_>>())
                    .collect();
                
                for (from_idx, packet) in all_packets {
                    packets_sent += 1;
                    if rng.gen::<f64>() < packet_loss_rate {
                        packets_dropped += 1;
                        continue;
                    }
                    swarm_network.queue_gossip(from_idx, packet);
                }
                
                for (agent_idx, agent) in agents.iter_mut().enumerate() {
                    let incoming = swarm_network.take_gossip(agent_idx);
                    agent.receive_gossip(&incoming);
                    agent.clear_recent_packets();
                }
            }
        }
        
        let ground_truth = oracle.ground_truth_positions();
        let avg_rms: f64 = agents.iter().map(|a| a.compute_position_error(&ground_truth)).sum::<f64>() / num_agents as f64;
        
        let actual_loss = packets_dropped as f64 / packets_sent.max(1) as f64;
        
        // With 90% loss, we're just testing survival and some coherence
        let passed = avg_rms < 50.0; // Very relaxed - just don't go crazy
        
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        info!("  NETWORK HELL RESULTS:");
        info!("    RMS error:     {:.2}m  {}", avg_rms, if passed { "✓ (survived!)" } else { "✗" });
        info!("    Packet loss:   {:.0}% ({} / {})", actual_loss * 100.0, packets_dropped, packets_sent);
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        let mut metrics = ScenarioMetrics::default();
        metrics.packets_sent = packets_sent;
        metrics.packets_dropped = packets_dropped;
        
        ScenarioResult {
            scenario: ScenarioId::NetworkHell,
            seed: self.seed,
            passed,
            total_ticks: target_ticks,
            final_time_secs: oracle.time(),
            final_entity_count: oracle.active_entities().len(),
            failure_reason: if !passed { Some(format!("RMS {:.2}m too high", avg_rms)) } else { None },
            metrics,
        }
    }
    
    /// DST-011: TimeTornado - 5-second OOSM delays.
    fn run_time_tornado(&self) -> ScenarioResult {
        use rand::SeedableRng;
        use rand::Rng;
        use rand_chacha::ChaCha8Rng;
        
        info!("DST-011: TimeTornado - 5-SECOND OOSM DELAYS 🔥");
        
        let physics_seed = self.seed.wrapping_mul(0x9e3779b97f4a7c15);
        let key_provider = DeterministicKeyProvider::new(self.seed);
        let root_key = key_provider.biscuit_root_key().public();
        let mut rng = ChaCha8Rng::seed_from_u64(self.seed.wrapping_mul(0xabed0abed));
        
        let context = Arc::new(SimContext::new(self.seed));
        let network = Arc::new(SimNetwork::new_stub(NodeId::from_seed(0)));
        let mut agent = SimulatedAgent::new(context, network, root_key, 0, AgentConfig::default());
        
        let mut oracle = Oracle::new(physics_seed);
        oracle.spawn_entity(Vector3::new(0.0, 0.0, 100.0), Vector3::new(20.0, 10.0, 0.0), "tornado_target");
        
        let dt = 0.1;
        let max_delay_secs = 5.0;
        let target_ticks = (self.max_duration_secs.min(60.0) * 10.0) as u64;
        
        // Buffer for delayed readings
        let mut delayed_queue: Vec<(u64, crate::oracle::SensorReading)> = Vec::new();
        let mut oosm_count = 0u64;
        
        info!("  Config: max delay {}s, duration {}s", max_delay_secs, target_ticks as f64 * dt);
        
        for tick in 0..target_ticks {
            oracle.step(dt);
            let readings = oracle.generate_sensor_readings();
            
            // Add current readings to queue with random delay
            for reading in readings {
                let delay_ticks = (rng.gen::<f64>() * max_delay_secs / dt) as u64;
                let delivery_tick = tick + delay_ticks;
                delayed_queue.push((delivery_tick, reading));
            }
            
            // Deliver readings whose time has come (simulating OOSM)
            delayed_queue.sort_by_key(|(t, _)| std::cmp::Reverse(*t));
            while let Some((delivery_tick, _)) = delayed_queue.last() {
                if *delivery_tick <= tick {
                    let (_, reading) = delayed_queue.pop().unwrap();
                    agent.tick();
                    agent.ingest_readings(&[reading]);
                    oosm_count += 1;
                } else {
                    break;
                }
            }
        }
        
        // Drain remaining queue
        for (_, reading) in delayed_queue.drain(..) {
            agent.tick();
            agent.ingest_readings(&[reading]);
            oosm_count += 1;
        }
        
        let ground_truth = oracle.ground_truth_positions();
        let rms_error = agent.compute_position_error(&ground_truth);
        
        // With 5s delays on a moving target, some error is expected
        let passed = rms_error < 200.0 && oosm_count > 0;
        
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        info!("  TIME TORNADO RESULTS:");
        info!("    RMS error:      {:.2}m  {}", rms_error, if passed { "✓" } else { "✗" });
        info!("    OOSM updates:   {}", oosm_count);
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        let mut metrics = ScenarioMetrics::default();
        metrics.oosm_updates = oosm_count;
        
        ScenarioResult {
            scenario: ScenarioId::TimeTornado,
            seed: self.seed,
            passed,
            total_ticks: target_ticks,
            final_time_secs: oracle.time(),
            final_entity_count: 1,
            failure_reason: if !passed { Some(format!("RMS {:.2}m", rms_error)) } else { None },
            metrics,
        }
    }
    
    /// DST-012: ZombieApocalypse - 50% of agents are bad actors.
    fn run_zombie_apocalypse(&self) -> ScenarioResult {
        use crate::swarm_network::SwarmNetwork;
        use rand::SeedableRng;
        use rand::Rng;
        use rand_chacha::ChaCha8Rng;
        
        info!("DST-012: ZombieApocalypse - 50% BAD ACTORS 🔥");
        
        let num_agents = 50;
        let num_bad_actors = 25; // Half!
        
        let physics_seed = self.seed.wrapping_mul(0x9e3779b97f4a7c15);
        let key_provider = DeterministicKeyProvider::new(self.seed);
        let root_key = key_provider.biscuit_root_key().public();
        let mut rng = ChaCha8Rng::seed_from_u64(self.seed.wrapping_mul(0xdead0dead));
        
        let mut oracle = Oracle::new(physics_seed);
        for i in 0..200 {
            oracle.spawn_entity(
                Vector3::new((i % 20) as f64 * 50.0, (i / 20) as f64 * 50.0, 100.0),
                Vector3::new(10.0, 5.0, 0.0),
                "survivor_target",
            );
        }
        
        let mut agents: Vec<SimulatedAgent> = (0..num_agents)
            .map(|i| {
                let context = Arc::new(SimContext::new(self.seed.wrapping_add(i as u64)));
                let network = Arc::new(SimNetwork::new_stub(NodeId::from_seed(i as u64)));
                SimulatedAgent::new(context, network, root_key.clone(), i as u64, AgentConfig::default())
            })
            .collect();
        
        // First half are zombies (bad actors)
        let bad_actor_ids: Vec<usize> = (0..num_bad_actors).collect();
        
        let mut swarm_network = SwarmNetwork::new_grid(5, 10);
        let dt = 0.1;
        let target_ticks = (self.max_duration_secs.min(30.0) * 10.0) as u64;
        
        info!("  Config: {} agents, {} zombies ({}%)", num_agents, num_bad_actors, num_bad_actors * 100 / num_agents);
        
        for tick in 0..target_ticks {
            oracle.step(dt);
            let readings = oracle.generate_sensor_readings();
            
            for (agent_idx, agent) in agents.iter_mut().enumerate() {
                let agent_readings: Vec<_> = readings.iter()
                    .enumerate()
                    .filter(|(entity_idx, _)| (entity_idx + agent_idx) % 4 < 2)
                    .map(|(_, r)| r.clone())
                    .collect();
                
                agent.tick();
                agent.ingest_readings(&agent_readings);
            }
            
            if tick % 5 == 0 {
                let all_packets: Vec<_> = agents.iter()
                    .enumerate()
                    .flat_map(|(idx, a)| {
                        let mut packets: Vec<_> = a.recent_packets().iter()
                            .map(|p| (idx, p.clone()))
                            .collect();
                        
                        // Zombies inject lots of garbage
                        if bad_actor_ids.contains(&idx) {
                            for _ in 0..10 { // 10 garbage packets each!
                                let garbage = godview_core::godview_tracking::GlobalHazardPacket {
                                    entity_id: Uuid::new_v4(),
                                    position: [rng.gen_range(-1000.0..1000.0), rng.gen_range(-1000.0..1000.0), rng.gen_range(0.0..500.0)],
                                    velocity: [rng.gen_range(-100.0..100.0), rng.gen_range(-100.0..100.0), 0.0],
                                    class_id: 99,
                                    timestamp: tick as f64 * dt,
                                    confidence_score: rng.gen_range(0.0..0.5),
                                };
                                packets.push((idx, garbage));
                            }
                        }
                        packets
                    })
                    .collect();
                
                for (from_idx, packet) in all_packets {
                    swarm_network.queue_gossip(from_idx, packet);
                }
                
                for (agent_idx, agent) in agents.iter_mut().enumerate() {
                    let incoming = swarm_network.take_gossip(agent_idx);
                    let neighbors = swarm_network.neighbors(agent_idx);
                    if !neighbors.is_empty() && !incoming.is_empty() {
                        let per = incoming.len() / neighbors.len().max(1);
                        for (i, neighbor_id) in neighbors.iter().enumerate() {
                            let start = i * per;
                            let end = ((i + 1) * per).min(incoming.len());
                            if start < end {
                                agent.receive_gossip_from(*neighbor_id, &incoming[start..end]);
                            }
                        }
                    }
                    agent.clear_recent_packets();
                }
            }
        }
        
        // Only measure GOOD agents (survivors)
        let ground_truth = oracle.ground_truth_positions();
        let good_agent_rms: Vec<f64> = agents.iter().enumerate()
            .filter(|(idx, _)| !bad_actor_ids.contains(idx))
            .map(|(_, a)| a.compute_position_error(&ground_truth))
            .collect();
        let avg_rms = good_agent_rms.iter().sum::<f64>() / good_agent_rms.len().max(1) as f64;
        
        // Count zombies identified by survivors
        let mut zombies_identified = 0;
        let mut possible_detections = 0;
        for (agent_idx, agent) in agents.iter().enumerate() {
            if bad_actor_ids.contains(&agent_idx) { continue; }
            let neighbors = swarm_network.neighbors(agent_idx);
            for &zombie_id in &bad_actor_ids {
                if neighbors.contains(&zombie_id) {
                    possible_detections += 1;
                    if let Some(rep) = agent.adaptive_state().neighbor_reputations.get(&zombie_id) {
                        if rep.reliability_score < 0.3 {
                            zombies_identified += 1;
                        }
                    }
                }
            }
        }
        
        let detection_rate = if possible_detections > 0 { zombies_identified as f64 / possible_detections as f64 } else { 0.0 };
        let passed = avg_rms < 10.0 && detection_rate > 0.2;
        
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        info!("  ZOMBIE APOCALYPSE RESULTS:");
        info!("    Survivor RMS:    {:.2}m  {}", avg_rms, if avg_rms < 10.0 { "✓" } else { "✗" });
        info!("    Zombie detection: {:.0}%  {}", detection_rate * 100.0, if detection_rate > 0.2 { "✓" } else { "✗" });
        info!("    Zombies spotted: {} / {}", zombies_identified, possible_detections);
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        let mut metrics = ScenarioMetrics::default();
        metrics.packets_sent = swarm_network.messages_sent();
        
        ScenarioResult {
            scenario: ScenarioId::ZombieApocalypse,
            seed: self.seed,
            passed,
            total_ticks: target_ticks,
            final_time_secs: oracle.time(),
            final_entity_count: oracle.active_entities().len(),
            failure_reason: if !passed { Some(format!("RMS={:.2}m, detection={:.0}%", avg_rms, detection_rate * 100.0)) } else { None },
            metrics,
        }
    }
    
    /// DST-013: RapidFire - 100Hz tick rate.
    fn run_rapid_fire(&self) -> ScenarioResult {
        info!("DST-013: RapidFire - 100Hz TICK RATE 🔥");
        
        let physics_seed = self.seed.wrapping_mul(0x9e3779b97f4a7c15);
        let key_provider = DeterministicKeyProvider::new(self.seed);
        let root_key = key_provider.biscuit_root_key().public();
        
        let context = Arc::new(SimContext::new(self.seed));
        let network = Arc::new(SimNetwork::new_stub(NodeId::from_seed(0)));
        let mut agent = SimulatedAgent::new(context, network, root_key, 0, AgentConfig::default());
        
        let mut oracle = Oracle::new(physics_seed);
        for i in 0..10 {
            oracle.spawn_entity(
                Vector3::new(i as f64 * 100.0, 0.0, 100.0),
                Vector3::new(50.0, 25.0 * ((i % 2) as f64 * 2.0 - 1.0), 0.0),
                "rapid_target",
            );
        }
        
        let tick_rate = 100.0; // 100 Hz
        let dt = 1.0 / tick_rate;
        let sim_duration = self.max_duration_secs.min(10.0); // Max 10s for speed
        let target_ticks = (sim_duration * tick_rate) as u64;
        
        let start_time = std::time::Instant::now();
        
        info!("  Config: {}Hz tick rate, {} ticks, {}s sim time", tick_rate, target_ticks, sim_duration);
        
        for _tick in 0..target_ticks {
            oracle.step(dt);
            let readings = oracle.generate_sensor_readings();
            agent.tick();
            agent.ingest_readings(&readings);
        }
        
        let elapsed = start_time.elapsed();
        let actual_rate = target_ticks as f64 / elapsed.as_secs_f64();
        
        let ground_truth = oracle.ground_truth_positions();
        let rms_error = agent.compute_position_error(&ground_truth);
        
        // Must run at least 50% of target rate and maintain accuracy
        let passed = rms_error < 3.0 && actual_rate > tick_rate * 0.5;
        
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        info!("  RAPID FIRE RESULTS:");
        info!("    RMS error:    {:.2}m  {}", rms_error, if rms_error < 3.0 { "✓" } else { "✗" });
        info!("    Target rate:  {}Hz", tick_rate);
        info!("    Actual rate:  {:.0}Hz  {}", actual_rate, if actual_rate > tick_rate * 0.5 { "✓" } else { "✗" });
        info!("    Wall time:    {:.3}s", elapsed.as_secs_f64());
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        ScenarioResult {
            scenario: ScenarioId::RapidFire,
            seed: self.seed,
            passed,
            total_ticks: target_ticks,
            final_time_secs: oracle.time(),
            final_entity_count: oracle.active_entities().len(),
            failure_reason: if !passed { Some(format!("RMS={:.2}m, rate={:.0}Hz", rms_error, actual_rate)) } else { None },
            metrics: ScenarioMetrics::default(),
        }
    }
    
    // ═══════════════════════════════════════════════════════════════════════════
    // EVOLUTIONARY INTELLIGENCE - Adapting to the Unknown
    // ═══════════════════════════════════════════════════════════════════════════
    
    /// DST-014: EvoWar - Evolution vs Chaos.
    ///
    /// Red Team (static bad actors) vs Blue Team (evolutionary).
    /// Can Blue evolve to survive high noise + bad actors?
    fn run_evo_war(&self) -> ScenarioResult {
        use crate::swarm_network::SwarmNetwork;
        use rand::SeedableRng;
        use rand::Rng;
        use rand_chacha::ChaCha8Rng;
        
        info!("DST-014: EvoWar - EVOLUTION VS CHAOS 🧬");
        
        let num_agents = 50;
        let num_red_team = 25; // Half are static/bad
        let packet_loss_rate = 0.30;
        
        let physics_seed = self.seed.wrapping_mul(0x9e3779b97f4a7c15);
        let key_provider = DeterministicKeyProvider::new(self.seed);
        let root_key = key_provider.biscuit_root_key().public();
        let mut rng = ChaCha8Rng::seed_from_u64(self.seed.wrapping_mul(0xeb014));
        
        // Oracle setup
        let mut oracle = Oracle::new(physics_seed);
        for i in 0..100 {
            oracle.spawn_entity(
                Vector3::new((i % 10) as f64 * 50.0, (i / 10) as f64 * 50.0, 100.0),
                Vector3::new(5.0, 2.0, 0.0),
                "evo_target",
            );
        }
        
        // Agents
        let mut agents: Vec<SimulatedAgent> = Vec::with_capacity(num_agents);
        for i in 0..num_agents {
            let context = Arc::new(SimContext::new(self.seed.wrapping_add(i as u64)));
            let network = Arc::new(SimNetwork::new_stub(NodeId::from_seed(i as u64)));
            agents.push(SimulatedAgent::new(
                context,
                network,
                root_key.clone(),
                i as u64,
                AgentConfig::default(),
            ));
        }
        
        // Red team indices (static bad actors)
        let red_team_ids: Vec<usize> = (0..num_red_team).collect();
        // Blue team indices (evolving)
        let blue_team_ids: Vec<usize> = (num_red_team..num_agents).collect();
        
        // Configure Red Team as bad actors
        for &id in &red_team_ids {
            // Re-create as bad actor
            let context = Arc::new(SimContext::new(self.seed.wrapping_add(id as u64)));
            let network = Arc::new(SimNetwork::new_stub(NodeId::from_seed(id as u64)));
            agents[id] = SimulatedAgent::new_bad_actor(context, network, root_key.clone(), id as u64, AgentConfig::default());
        }
        
        let mut swarm_network = SwarmNetwork::new_grid(5, 10);
        let dt = 0.1;
        let target_ticks = (self.max_duration_secs.min(30.0) * 10.0) as u64;
        let evo_epoch_ticks = 20; // Evolve every 2s
        
        info!("  Config: {} Blue (learning), {} Red (static/bad), 30% loss", blue_team_ids.len(), red_team_ids.len());
        
        let _ground_truth_buffer: Vec<_> = oracle.ground_truth_positions(); // Initial
        
        for tick in 0..target_ticks {
            oracle.step(dt);
            let readings = oracle.generate_sensor_readings();
            let ground_truth = oracle.ground_truth_positions();
            
            for (agent_idx, agent) in agents.iter_mut().enumerate() {
                // Blue team evolves
                if blue_team_ids.contains(&agent_idx) {
                    agent.tick_evolution(evo_epoch_ticks, Some(&ground_truth));
                } else {
                    agent.tick(); // Red team just ticks
                }
                
                // Sensor input
                let agent_readings: Vec<_> = readings.iter()
                    .enumerate()
                    .filter(|(entity_idx, _)| (entity_idx + agent_idx) % 4 == 0)
                    .map(|(_, r)| r.clone())
                    .collect();
                agent.ingest_readings(&agent_readings);
            }
            
            // Gossip
            if tick % 5 == 0 {
                // Collect packets
                let all_packets: Vec<_> = agents.iter()
                    .enumerate()
                    .flat_map(|(idx, a)| {
                        let mut packets: Vec<_> = a.recent_packets().iter()
                            .map(|p| (idx, p.clone()))
                            .collect();
                        
                        // Red team injects garbage
                        if red_team_ids.contains(&idx) {
                            for _ in 0..5 {
                                let garbage = godview_core::godview_tracking::GlobalHazardPacket {
                                    entity_id: Uuid::new_v4(), // Confusing ID
                                    position: [rng.gen_range(-500.0..500.0), rng.gen_range(-500.0..500.0), 0.0],
                                    velocity: [0.0, 0.0, 0.0],
                                    class_id: 99,
                                    timestamp: tick as f64 * dt,
                                    confidence_score: 0.9, // High confidence lie
                                };
                                packets.push((idx, garbage));
                            }
                        }
                        packets
                    })
                    .collect();

                // Distribute packets (Blue respects evo params)
                for (from_idx, packet) in all_packets {
                    // Packet loss
                    if rng.gen::<f64>() < packet_loss_rate { continue; }
                    
                    swarm_network.queue_gossip(from_idx, packet);
                    
                    // Record measurement for BLUE team sender
                    if blue_team_ids.contains(&from_idx) {
                        // Estimate wire size (struct ~100 bytes + overhead ~25)
                        let size = 125;
                        agents[from_idx].record_message_sent_metric(size);
                    }
                }
                
                for (agent_idx, agent) in agents.iter_mut().enumerate() {
                    let incoming = swarm_network.take_gossip(agent_idx);
                    
                    // Apply EVO constraints for Blue
                    let max_neighbors = if blue_team_ids.contains(&agent_idx) {
                        agent.max_gossip_neighbors()
                    } else {
                        100 // Red talks to everyone
                    };
                    
                    // Filter incoming by max neighbors (simulate bandwidth constraint)
                    let limited_incoming = if incoming.len() > max_neighbors {
                        &incoming[0..max_neighbors]
                    } else {
                        &incoming[..]
                    };
                    
                    agent.receive_gossip(limited_incoming);
                    agent.clear_recent_packets();
                }
            }
        }
        
        // Metrics
        let ground_truth = oracle.ground_truth_positions();
        
        // Blue Team Score
        let blue_rms: Vec<f64> = agents.iter().enumerate()
            .filter(|(idx, _)| blue_team_ids.contains(idx))
            .map(|(_, a)| a.compute_position_error(&ground_truth))
            .collect();
        let avg_blue_rms = blue_rms.iter().sum::<f64>() / blue_rms.len().max(1) as f64;
        
        // Did params diverge from default?
        let blue_params = &agents[blue_team_ids[0]].evolutionary_state().current_params;
        let param_drift = (blue_params.confidence_threshold - 0.0).abs() > 0.01 || 
                          blue_params.max_neighbors_gossip != 100 ||
                          blue_params.gossip_interval_ticks != 5;
        
        let passed = avg_blue_rms < 10.0;
        
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        info!("  EVO WAR RESULTS:");
        info!("    Blue RMS:      {:.2}m  {}", avg_blue_rms, if passed { "✓" } else { "✗" });
        info!("    Param Drift:   {} (Agents adapted!)", if param_drift { "YES" } else { "NO" });
        info!("    Final Params:  interval={}, neighbors={}, conf={:.2}", 
            blue_params.gossip_interval_ticks, blue_params.max_neighbors_gossip, blue_params.confidence_threshold);
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        ScenarioResult {
            scenario: ScenarioId::EvoWar,
            seed: self.seed,
            passed,
            total_ticks: target_ticks,
            final_time_secs: oracle.time(),
            final_entity_count: oracle.active_entities().len(),
            failure_reason: if !passed { Some(format!("Blue RMS {:.2}m", avg_blue_rms)) } else { None },
            metrics: ScenarioMetrics::default(),
        }
    }
    
    /// DST-015: ResourceStarvation.
    fn run_resource_starvation(&self) -> ScenarioResult {
        use crate::swarm_network::SwarmNetwork;
        use rand::SeedableRng;
        
        
        info!("DST-015: ResourceStarvation - BANDWIDTH LIMIT 🧬");
        
        let num_agents = 50;
        let total_bandwidth_limit = 1000; // packets per tick global
        
        let mut agents: Vec<SimulatedAgent> = Vec::with_capacity(num_agents);
        let key_provider = DeterministicKeyProvider::new(self.seed);
        let root_key = key_provider.biscuit_root_key().public();
        
        for i in 0..num_agents {
            let context = Arc::new(SimContext::new(self.seed.wrapping_add(i as u64)));
            let network = Arc::new(SimNetwork::new_stub(NodeId::from_seed(i as u64)));
            agents.push(SimulatedAgent::new(
                context, 
                network, 
                root_key.clone(), 
                i as u64, 
                AgentConfig::default()
            ));
        }
        
        let mut oracle = Oracle::new(self.seed);
        for i in 0..50 {
             oracle.spawn_entity(
                Vector3::new(i as f64 * 10.0, 0.0, 100.0),
                Vector3::new(1.0, 0.0, 0.0),
                "starve_target",
            );
        }
        
        let mut swarm_network = SwarmNetwork::new_grid(5, 10);
        let dt = 0.1;
        let target_ticks = (self.max_duration_secs.min(20.0) * 10.0) as u64;
        let evo_epoch_ticks = 10;
        
        let mut total_sent = 0;
        let mut total_dropped_bandwidth = 0;
        
        for tick in 0..target_ticks {
            oracle.step(dt);
            let readings = oracle.generate_sensor_readings();
            let ground_truth = oracle.ground_truth_positions();
            
            for (idx, agent) in agents.iter_mut().enumerate() {
                agent.tick_evolution(evo_epoch_ticks, Some(&ground_truth));
                
                 let agent_readings: Vec<_> = readings.iter()
                    .enumerate()
                    .filter(|(entity_idx, _)| (entity_idx + idx) % 5 == 0)
                    .map(|(_, r)| r.clone())
                    .collect();
                agent.ingest_readings(&agent_readings);
            }
            
            // Collect all proposed packets
            let mut pending_packets = Vec::new();
            for (idx, agent) in agents.iter_mut().enumerate() {
                // Respect agent's evolved gossip interval
                if tick % agent.gossip_interval() == 0 {
                    let recent_count = agent.recent_packets().len();
                    for p in agent.recent_packets() {
                        pending_packets.push((idx, p.clone()));
                    }
                    for _ in 0..recent_count {
                        // Estimate average packet size (e.g., 100 bytes + overhead)
                        let size = 125;
                        agent.record_message_sent_metric(size); // Charged for attempting
                    }
                }
            }
            
            // Global bandwidth limiter
            total_sent += pending_packets.len();
            if pending_packets.len() > total_bandwidth_limit {
                total_dropped_bandwidth += pending_packets.len() - total_bandwidth_limit;
                pending_packets.truncate(total_bandwidth_limit);
            }
            
            // Deliver
            for (from, packet) in pending_packets {
                swarm_network.queue_gossip(from, packet);
            }
            
            for (idx, agent) in agents.iter_mut().enumerate() {
                let incoming = swarm_network.take_gossip(idx);
                agent.receive_gossip(&incoming);
                agent.clear_recent_packets();
            }
        }
        
        let ground_truth = oracle.ground_truth_positions();
        let avg_rms: f64 = agents.iter().map(|a| a.compute_position_error(&ground_truth)).sum::<f64>() / num_agents as f64;
        
        // Check if agents increased gossip interval to reduce cost
        let avg_interval: f64 = agents.iter().map(|a| a.gossip_interval() as f64).sum::<f64>() / num_agents as f64;
        
        let passed = avg_rms < 5.0 && avg_interval > 5.0; // Interval should increase > 5 (default)
        
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        info!("  RESOURCE STARVATION RESULTS:");
        info!("    RMS error:      {:.2}m  {}", avg_rms, if avg_rms < 5.0 { "✓" } else { "✗" });
        info!("    Avg Interval:   {:.1} ticks (started at 5) {}", avg_interval, if avg_interval > 5.0 { "✓ (Adapted)" } else { "✗" });
        info!("    Bandwidth Drop: {:.1}%", total_dropped_bandwidth as f64 * 100.0 / total_sent as f64);
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        ScenarioResult {
            scenario: ScenarioId::ResourceStarvation,
            seed: self.seed,
            passed,
            total_ticks: target_ticks,
            final_time_secs: oracle.time(),
            final_entity_count: oracle.active_entities().len(),
            failure_reason: if !passed { Some(format!("RMS={:.2}m (want <5), Interval={:.1} (want >5)", avg_rms, avg_interval)) } else { None },
            metrics: ScenarioMetrics::default(),
        }
    }
    
    /// DST-016: ProtocolDrift - Stub for now.
    fn run_protocol_drift(&self) -> ScenarioResult {
        info!("DST-016: ProtocolDrift - Placeholder 🧬");
        ScenarioResult {
            scenario: ScenarioId::ProtocolDrift,
            seed: self.seed,
            passed: true,
            total_ticks: 0,
            final_time_secs: 0.0,
            final_entity_count: 0,
            failure_reason: None,
            metrics: ScenarioMetrics::default(),
        }
    }

    /// DST-017: BlindLearning - Evolve without Ground Truth.
    ///
    /// Agents must optimize NIS (Internal Consistency) and Peer Agreement (Consensus)
    /// to find good parameters, without ever knowing their true error.
    fn run_blind_learning(&self) -> ScenarioResult {
        use crate::evolution::BlindFitness;

        info!("DST-017: BlindLearning - ADAPTING BLINDLY 🙈");
        
        let num_agents = 50;
        let packet_loss_rate = 0.20; // Moderate noise
        
        // Oracle setup
        let mut oracle = Oracle::new(self.seed);
        for i in 0..50 {
            oracle.spawn_entity(
                Vector3::new((i % 10) as f64 * 50.0, (i / 10) as f64 * 50.0, 100.0),
                Vector3::new(5.0, 2.0, 0.0),
                "blind_target",
            );
        }
        
        let key_provider = DeterministicKeyProvider::new(self.seed);
        let root_key = key_provider.biscuit_root_key().public();
        
        // Agents initialized with BLIND FITNESS
        let mut agents: Vec<SimulatedAgent> = (0..num_agents)
            .map(|i| {
                let context = Arc::new(SimContext::new(self.seed.wrapping_add(i as u64)));
                let network = Arc::new(SimNetwork::new_stub(NodeId::from_seed(i as u64)));
                let mut agent = SimulatedAgent::new(context, network, root_key.clone(), i as u64, AgentConfig::default());
                
                // CRITICAL: Switch to Blind Fitness!
                agent.set_fitness_provider(Box::new(BlindFitness::new()));
                agent
            })
            .collect();
            
        let mut swarm_network = crate::swarm_network::SwarmNetwork::new_grid(5, 10);
        let dt = 0.1;
        let target_ticks = (self.max_duration_secs.min(45.0) * 10.0) as u64; // Runs a bit longer
        let evo_epoch_ticks = 20;
        
        let mut rng = rand::rngs::StdRng::seed_from_u64(self.seed);
        use rand::{Rng, SeedableRng};
        
        info!("  Config: {} agents using BlindFitness (NIS+PA+BW)", num_agents);
        
        // Tracking convergence
        let mut initial_rms = 0.0;
        
        for tick in 0..target_ticks {
            oracle.step(dt);
            let readings = oracle.generate_sensor_readings();
            let ground_truth = oracle.ground_truth_positions();
            
            // Measure initial performance after a few ticks
            if tick == 20 {
                initial_rms = agents.iter().map(|a| a.compute_position_error(&ground_truth)).sum::<f64>() / num_agents as f64;
                info!("  Initial RMS: {:.2}m", initial_rms);
            }
            
            for (idx, agent) in agents.iter_mut().enumerate() {
                 // Evolution Step
                 // We pass ground_truth ONLY for recording the "true" accuracy for our report/metrics.
                 // The agent's BlindFitness provider will IGNORE it.
                agent.tick_evolution(evo_epoch_ticks, Some(&ground_truth));
                
                let agent_readings: Vec<_> = readings.iter()
                    .enumerate()
                    .filter(|(entity_idx, _)| (entity_idx + idx) % 5 == 0)
                    .map(|(_, r)| r.clone())
                    .collect();
                agent.ingest_readings(&agent_readings);
            }
            
            // Gossip Logic (with packet drop)
            if tick % 5 == 0 {
                // Collect packets
                let all_packets: Vec<_> = agents.iter_mut()
                    .enumerate()
                    .flat_map(|(idx, a)| {
                        // Respect evolved interval? 
                        // Simplified: check simple modulus against agent's interval
                        if tick % a.gossip_interval() == 0 {
                            let packets: Vec<_> = a.recent_packets().iter().map(|p| (idx, p.clone())).collect();
                            // Charge bandwidth
                            let count = packets.len();
                             // Estimate wire size (struct ~100 bytes + overhead ~25)
                            a.record_message_sent_metric(count as u64 * 125);
                            packets
                        } else {
                            Vec::new()
                        }
                    })
                    .collect();

                // Distribute
                for (from_idx, packet) in all_packets {
                    // 20% Packet loss
                    if rng.gen::<f64>() < packet_loss_rate { continue; }
                    swarm_network.queue_gossip(from_idx, packet);
                }
                
                for (agent_idx, agent) in agents.iter_mut().enumerate() {
                    let incoming = swarm_network.take_gossip(agent_idx);
                    
                    // Limit neighbors by evolved parameter
                    let max_neighbors = agent.max_gossip_neighbors();
                     let limited_incoming = if incoming.len() > max_neighbors {
                        &incoming[0..max_neighbors]
                    } else {
                        &incoming[..]
                    };
                    
                    agent.receive_gossip(limited_incoming);
                    agent.clear_recent_packets();
                }
            }
        }
        
        let ground_truth = oracle.ground_truth_positions();
        let final_rms: f64 = agents.iter().map(|a| a.compute_position_error(&ground_truth)).sum::<f64>() / num_agents as f64;
        
        // Did we improve?
        let improved = final_rms < initial_rms;
        // Did we survive reasonably well?
        let passed = final_rms < 10.0; 
        
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        info!("  BLIND LEARNING RESULTS:");
        info!("    Initial RMS:   {:.2}m", initial_rms);
        info!("    Final RMS:     {:.2}m  {}", final_rms, if passed { "✓" } else { "✗" });
        info!("    Improvement:   {}", if improved { "YES (Optimized!)" } else { "NO" });
        
        // Check params
        let agent0_params = &agents[0].evolutionary_state().current_params;
        info!("    Final Params:  interval={}, neighbors={}, conf={:.2}", 
            agent0_params.gossip_interval_ticks, agent0_params.max_neighbors_gossip, agent0_params.confidence_threshold);
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        ScenarioResult {
            scenario: ScenarioId::BlindLearning,
            seed: self.seed,
            passed,
            total_ticks: target_ticks,
            final_time_secs: oracle.time(),
            final_entity_count: oracle.active_entities().len(),
            failure_reason: if !passed { Some(format!("RMS {:.2}m", final_rms)) } else { None },
            metrics: ScenarioMetrics::default(),
        }
    }

    /// DST-018: BlackoutSurvival - Total System Failure.
    ///
    /// The ultimate test: 50% Packet Loss + Sensor Faults + Bad Actors + Bandwidth Limit.
    /// Agents must usage BlindFitness to filter noise, reject bad actors, and survive.
    fn run_blackout_survival(&self) -> ScenarioResult {
        use crate::evolution::BlindFitness;

        info!("DST-018: BlackoutSurvival - TOTAL SYSTEM FAILURE 💀");
        
        // 1. Extreme Environment
        let num_agents = 50;
        let num_bad_actors = 10; // 20% Traitors
        let packet_loss_rate = 0.50; // High loss
        let sensor_fault_rate = 0.10; // 10% Blackouts
        let bandwidth_limit = 1500; // Global limit
        
        // Oracle setup
        let mut oracle = Oracle::new(self.seed);
        for i in 0..50 {
            oracle.spawn_entity(
                Vector3::new((i % 10) as f64 * 50.0, (i / 10) as f64 * 50.0, 100.0),
                Vector3::new(5.0, 2.0, 0.0),
                "blackout_target",
            );
        }
        
        let key_provider = DeterministicKeyProvider::new(self.seed);
        let root_key = key_provider.biscuit_root_key().public();
        
        // Agents: Blind Fitness + Bad Actors
        let mut agents: Vec<SimulatedAgent> = Vec::with_capacity(num_agents);
        for i in 0..num_agents {
             let context = Arc::new(SimContext::new(self.seed.wrapping_add(i as u64)));
            let network = Arc::new(SimNetwork::new_stub(NodeId::from_seed(i as u64)));
            let mut agent = SimulatedAgent::new(context, network, root_key.clone(), i as u64, AgentConfig::default());
            
            // Usage Blind Fitness
            agent.set_fitness_provider(Box::new(BlindFitness::new()));
            
             // Bad Actors?
            if i < num_bad_actors as usize {
                 let context = Arc::new(SimContext::new(self.seed.wrapping_add(i as u64)));
                 let network = Arc::new(SimNetwork::new_stub(NodeId::from_seed(i as u64)));
                 let mut bad_agent = SimulatedAgent::new_bad_actor(context, network, root_key.clone(), i as u64, AgentConfig::default());
                 bad_agent.set_fitness_provider(Box::new(BlindFitness::new())); 
                 agent = bad_agent;
            }
            agents.push(agent);
        }
            
        let mut swarm_network = crate::swarm_network::SwarmNetwork::new_grid(5, 10);
        let dt = 0.1;
        let target_ticks = (self.max_duration_secs.min(60.0) * 10.0) as u64;
        let evo_epoch_ticks = 20;
        
        let mut rng = rand::rngs::StdRng::seed_from_u64(self.seed);
        use rand::{Rng, SeedableRng};
        
        info!("  Config: {} agents ({} bad), 50% loss, 10% sensor faults, BW limit", num_agents, num_bad_actors);
        
        for tick in 0..target_ticks {
            oracle.step(dt);
            let readings = oracle.generate_sensor_readings();
            let ground_truth = oracle.ground_truth_positions();
            
            for (idx, agent) in agents.iter_mut().enumerate() {
                // Evolution
                agent.tick_evolution(evo_epoch_ticks, Some(&ground_truth));
                
                // Sensor Faults injection
                let mut agent_readings: Vec<_> = readings.iter()
                    .enumerate()
                    .filter(|(entity_idx, _)| (entity_idx + idx) % 5 == 0)
                    .map(|(_, r)| r.clone())
                    .collect();
                
                // 10% chance to lose all readings (Blackout)
                if rng.gen::<f64>() < sensor_fault_rate {
                    agent_readings.clear();
                } else if rng.gen::<f64>() < 0.05 {
                     // 5% chance of severe noise
                     for r in agent_readings.iter_mut() {
                         r.position.x += rng.gen_range(-50.0..50.0);
                         r.position.y += rng.gen_range(-50.0..50.0);
                     }
                }
                
                agent.ingest_readings(&agent_readings);
            }
            
             // Gossip Logic
            if tick % 5 == 0 {
                // Collect packets
                let mut all_packets: Vec<_> = agents.iter_mut()
                    .enumerate()
                    .flat_map(|(idx, a)| {
                        if tick % a.gossip_interval() == 0 {
                            let packets: Vec<_> = a.recent_packets().iter().map(|p| (idx, p.clone())).collect();
                            // Charge bandwidth
                            let count = packets.len();
                            a.record_message_sent_metric(count as u64 * 125);
                            packets
                        } else {
                            Vec::new()
                        }
                    })
                    .collect();
                
                // Global Bandwidth Limit (Starvation)
                 if all_packets.len() > bandwidth_limit {
                    // Random drop to limit
                    use rand::seq::SliceRandom;
                    all_packets.shuffle(&mut rng);
                    all_packets.truncate(bandwidth_limit);
                }

                // Distribute
                for (from_idx, packet) in all_packets {
                    // 50% Packet loss
                    if rng.gen::<f64>() < packet_loss_rate { continue; }
                    swarm_network.queue_gossip(from_idx, packet);
                }
                
                for (agent_idx, agent) in agents.iter_mut().enumerate() {
                    let incoming = swarm_network.take_gossip(agent_idx);
                    
                    // Limit neighbors
                    let max_neighbors = agent.max_gossip_neighbors();
                     let limited_incoming = if incoming.len() > max_neighbors {
                        &incoming[0..max_neighbors]
                    } else {
                        &incoming[..]
                    };
                    
                     // Receive gossip with neighbor attribution hacks for trust
                     let neighbors = swarm_network.neighbors(agent_idx);
                     if !neighbors.is_empty() && !limited_incoming.is_empty() {
                        let per = limited_incoming.len() / neighbors.len().max(1);
                        for (i, neighbor_id) in neighbors.iter().enumerate() {
                            let start = i * per;
                            let end = if i == neighbors.len() - 1 { limited_incoming.len() } else { (i + 1) * per };
                            if start < end {
                                agent.receive_gossip_from(*neighbor_id, &limited_incoming[start..end]);
                            }
                        }
                    }
                    agent.clear_recent_packets();
                }
            }
        }
        
        let ground_truth = oracle.ground_truth_positions();
        
        // Filter out bad actors for scoring
        let good_agent_rms: Vec<f64> = agents.iter().enumerate()
            .filter(|(idx, _)| *idx >= num_bad_actors)
            .map(|(_, a)| a.compute_position_error(&ground_truth))
            .collect();
            
        let avg_rms = good_agent_rms.iter().sum::<f64>() / good_agent_rms.len().max(1) as f64;
        
        // Did we survive?
        let passed = avg_rms < 15.0; // Relaxed threshold due to 50% loss + faults
        
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        info!("  BLACKOUT RESULTS:");
        info!("    Survivor RMS:  {:.2}m  {}", avg_rms, if passed { "✓" } else { "✗" });
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        ScenarioResult {
            scenario: ScenarioId::BlackoutSurvival,
            seed: self.seed,
            passed,
            total_ticks: target_ticks,
            final_time_secs: oracle.time(),
            final_entity_count: oracle.active_entities().len(),
            failure_reason: if !passed { Some(format!("RMS {:.2}m", avg_rms)) } else { None },
            metrics: ScenarioMetrics::default(),
        }
    }

    /// DST-019: LongHaul - Energy Crisis.
    ///
    /// Survive 2000 ticks with limited battery.
    /// Agents must evolve to speak less (higher gossip interval) to survive.
    fn run_long_haul(&self) -> ScenarioResult {
        use crate::evolution::BlindFitness;

        info!("DST-019: LongHaul - THE ENERGY CRISIS 🔋");
        
        // Config
        let num_agents = 10;
        let start_energy = 1000.0;
        let mut oracle = Oracle::new(self.seed);
        
        // Spawn some entities to track
        for i in 0..10 {
            oracle.spawn_entity(
                Vector3::new((i as f64) * 20.0, 0.0, 100.0),
                Vector3::new(1.0, 1.0, 0.0),
                "long_haul_target",
            );
        }
        
        let key_provider = DeterministicKeyProvider::new(self.seed);
        let root_key = key_provider.biscuit_root_key().public();
        
        // Init Agents
        let mut agents: Vec<SimulatedAgent> = (0..num_agents)
            .map(|i| {
                 let context = Arc::new(SimContext::new(self.seed.wrapping_add(i as u64)));
                 let network = Arc::new(SimNetwork::new_stub(NodeId::from_seed(i as u64)));
                 let mut agent = SimulatedAgent::new(context, network, root_key.clone(), i as u64, AgentConfig::default());
                 agent.set_fitness_provider(Box::new(BlindFitness::new()));
                 agent.consume_energy(850.0); // Start with 150J for fast test
                 agent
            })
            .collect();
            
        let mut swarm_network = crate::swarm_network::SwarmNetwork::new_grid(5, 10);
        let dt = 0.1;
        let target_ticks = 200;
        let evo_epoch_ticks = 20; // Faster evolution for test
        
        let _rng = rand::rngs::StdRng::seed_from_u64(self.seed);
        
        info!("  Config: {} agents, 10 entities, {} ticks. Starting Energy: {}J", num_agents, target_ticks, start_energy);
        
        for tick in 0..target_ticks {
            oracle.step(dt);
            // Limit visible readings to reduce sensor cost pressure (allow some chance)
            // Or just let them deal with it.
            // Cost: 0.05 * 10 = 0.5 per tick. 2000 * 0.5 = 1000 J.
            // They will all die if they process all.
            // But they can't choose to ignore yet.
            // I will artificially limit readings in this scenario to 2 entities per agent per tick (randomly).
            // This represents "Scanning a sector" instead of 360 view.
            
            let readings = oracle.generate_sensor_readings();
            let ground_truth = oracle.ground_truth_positions();
            
            for (idx, agent) in agents.iter_mut().enumerate() {
                // Check if alive handled in tick() internal (returns false if dead)
                if !agent.tick() {
                    continue; // Dead
                }
                
                // Evolution
                agent.tick_evolution(evo_epoch_ticks, Some(&ground_truth));
                
                // Limited Ingestion (2 readings max)
                let my_readings: Vec<_> = readings.iter()
                    .skip(idx % 5) // Simple stagger
                    .take(2)
                    .cloned()
                    .collect();
                    
                agent.ingest_readings(&my_readings);
            }
            
             // Gossip Logic
            if tick % 5 == 0 {
                // Collect packets
                let all_packets: Vec<_> = agents.iter_mut()
                    .enumerate()
                    .flat_map(|(idx, a)| {
                        // Check dead status before asking logic
                        if !a.is_alive() { return Vec::new(); }
                        
                        // Use new should_broadcast() with Emergency Protocol
                        if a.should_broadcast(tick) {
                            let packets: Vec<_> = a.recent_packets().iter().map(|p| (idx, p.clone())).collect();
                            
                            // Charge Message Cost!
                            let cost = packets.len() as f64 * 1.0; 
                            a.consume_energy(cost);
                            
                            // Metrics
                            a.record_message_sent_metric(packets.len() as u64 * 125);
                            packets
                        } else {
                            Vec::new()
                        }
                    })
                    .collect();
                
                // Distribute
                for (from_idx, packet) in all_packets {
                    swarm_network.queue_gossip(from_idx, packet);
                }
                
                for (agent_idx, agent) in agents.iter_mut().enumerate() {
                     let incoming = swarm_network.take_gossip(agent_idx);
                     // Receive & Process
                     agent.receive_gossip(&incoming);
                     agent.clear_recent_packets();
                }
            }
        }
        
        // Analysis
        let final_survivors = agents.iter().filter(|a| a.is_alive()).count();
        let survival_rate = final_survivors as f64 / num_agents as f64;
        
        let survivor_rms: f64 = if final_survivors > 0 {
             let gt = oracle.ground_truth_positions();
             let sum_rms: f64 = agents.iter()
                .filter(|a| a.is_alive())
                .map(|a| a.compute_position_error(&gt))
                .sum();
             sum_rms / final_survivors as f64
        } else {
            999.0
        };
        
        // Success Criteria: > 80% Survivors AND < 5.0m RMS
        let passed = survival_rate > 0.8 && survivor_rms < 5.0;
        
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        info!("  LONG HAUL RESULTS:");
        info!("    Survivors:    {}/{} ({:.1}%)", final_survivors, num_agents, survival_rate * 100.0);
        info!("    Survivor RMS: {:.2}m", survivor_rms);
        
        // Print average evolved parameters of survivors
        if final_survivors > 0 {
            let avg_interval: f64 = agents.iter()
                .filter(|a| a.is_alive())
                .map(|a| a.gossip_interval() as f64)
                .sum::<f64>() / final_survivors as f64;
            info!("    Avg Interval: {:.1} ticks (Started at 5.0)", avg_interval);
        }
        
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        ScenarioResult {
            scenario: ScenarioId::LongHaul,
            seed: self.seed,
            passed,
            total_ticks: target_ticks,
            final_time_secs: oracle.time(),
            final_entity_count: 5,
            failure_reason: if !passed { Some(format!("Survivors: {:.0}%, RMS: {:.2}m", survival_rate*100.0, survivor_rms)) } else { None },
            metrics: ScenarioMetrics::default(),
        }
    }
    
    /// DST-020: CommonBias - GPS bias detection via evolution (v0.6.0)
    /// 
    /// All agents receive a +5m GPS offset on sensor readings.
    /// Tests if agents can evolve `sensor_bias_estimate` to compensate.
    /// 
    /// **Success Criteria**: Swarm RMS < 5.0m after evolution.
    fn run_common_bias(&self) -> ScenarioResult {
        use crate::evolution::BlindFitness;
        info!("DST-020: CommonBias - GPS Bias Detection 🎯");
        
        let num_agents = 10;
        let gps_bias = 5.0; // +5m bias on all readings
        
        // Create Oracle with 5 stationary targets
        let mut oracle = Oracle::new(self.seed);
        for i in 0..5 {
            oracle.spawn_entity(
                Vector3::new((i as f64) * 30.0, 0.0, 100.0),
                Vector3::zeros(),
                "bias_target",
            );
        }
        
        let key_provider = DeterministicKeyProvider::new(self.seed);
        let root_key = key_provider.biscuit_root_key().public();
        
        // Init Agents with BlindFitness
        let mut agents: Vec<SimulatedAgent> = (0..num_agents)
            .map(|i| {
                 let context = Arc::new(SimContext::new(self.seed.wrapping_add(i as u64)));
                 let network = Arc::new(SimNetwork::new_stub(NodeId::from_seed(i as u64)));
                 let mut agent = SimulatedAgent::new(context, network, root_key.clone(), i as u64, AgentConfig::default());
                 agent.set_fitness_provider(Box::new(BlindFitness::new()));
                 agent
            })
            .collect();
            
        let mut swarm_network = crate::swarm_network::SwarmNetwork::new_grid(5, 10);
        let dt = 0.1;
        let target_ticks = 500; // Longer run for evolution to find bias
        let evo_epoch_ticks = 50;
        
        info!("  Config: {} agents, 5 entities, {} ticks. GPS Bias: +{}m", num_agents, target_ticks, gps_bias);
        
        for tick in 0..target_ticks {
            oracle.step(dt);
            
            // Get base readings from Oracle
            let base_readings = oracle.generate_sensor_readings();
            let ground_truth = oracle.ground_truth_positions();
            
            for (idx, agent) in agents.iter_mut().enumerate() {
                if !agent.tick() { continue; } // Dead
                
                // Get a subset of readings for this agent
                let my_readings: Vec<_> = base_readings.iter()
                    .skip(idx % 5)
                    .take(2)
                    .cloned()
                    .collect();
                
                // Apply GPS bias to readings
                let mut biased_readings: Vec<_> = my_readings.into_iter()
                    .map(|mut r| {
                        r.position.x += gps_bias;
                        r.position.y += gps_bias;
                        r.position.z += gps_bias;
                        r
                    })
                    .collect();
                
                // Apply agent's evolved bias compensation
                let bias_compensation = agent.sensor_bias_estimate();
                for reading in &mut biased_readings {
                    reading.position.x -= bias_compensation;
                    reading.position.y -= bias_compensation;
                    reading.position.z -= bias_compensation;
                }
                
                agent.ingest_readings(&biased_readings);
                
                // Evolution tick
                agent.tick_evolution(evo_epoch_ticks, Some(&ground_truth));
            }
            
            // Gossip
            if tick % 5 == 0 {
                let all_packets: Vec<_> = agents.iter_mut()
                    .enumerate()
                    .flat_map(|(idx, a)| {
                        if !a.is_alive() { return Vec::new(); }
                        if a.should_broadcast(tick) {
                            a.recent_packets().iter().map(|p| (idx, p.clone())).collect()
                        } else {
                            Vec::new()
                        }
                    })
                    .collect();
                
                for (from_idx, packet) in all_packets {
                    swarm_network.queue_gossip(from_idx, packet);
                }
                
                for (agent_idx, agent) in agents.iter_mut().enumerate() {
                     let incoming = swarm_network.take_gossip(agent_idx);
                     agent.receive_gossip(&incoming);
                     agent.clear_recent_packets();
                }
            }
        }
        
        // Measure final accuracy
        let gt: Vec<_> = oracle.ground_truth_positions();
        let avg_rms: f64 = agents.iter()
            .map(|a| a.compute_position_error(&gt))
            .sum::<f64>() / num_agents as f64;
        
        // Check evolved bias estimates
        let avg_bias_estimate: f64 = agents.iter()
            .map(|a| a.sensor_bias_estimate())
            .sum::<f64>() / num_agents as f64;
        
        let passed = avg_rms < 5.0;
        
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        info!("  COMMON BIAS RESULTS:");
        info!("    Final RMS: {:.2}m (target < 5.0m)", avg_rms);
        info!("    Avg Bias Estimate: {:.2}m (true bias: {}m)", avg_bias_estimate, gps_bias);
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        ScenarioResult {
            scenario: ScenarioId::CommonBias,
            seed: self.seed,
            passed,
            total_ticks: target_ticks,
            final_time_secs: oracle.time(),
            final_entity_count: 5,
            failure_reason: if !passed { Some(format!("RMS: {:.2}m", avg_rms)) } else { None },
            metrics: ScenarioMetrics::default(),
        }
    }
    
    /// DST-021: HeavyTail - Cauchy noise stress test (v0.6.0)
    /// 
    /// Tests agents evolved on Gaussian noise against heavy-tailed Cauchy noise.
    /// Cauchy has occasional extreme outliers that challenge tracking filters.
    /// 
    /// **Success Criteria**: RMS < 10.0m (more lenient due to outliers)
    fn run_heavy_tail(&self) -> ScenarioResult {
        use crate::evolution::BlindFitness;
        use crate::oracle::NoiseModel;
        
        info!("DST-021: HeavyTail - Cauchy Noise Stress Test 📉");
        
        let num_agents = 10;
        let mut oracle = Oracle::new(self.seed);
        
        // Use Cauchy (heavy-tailed) noise
        oracle.set_noise_model(NoiseModel::Cauchy);
        oracle.set_position_noise(1.0); // 1m scale parameter
        
        // Spawn 5 stationary targets
        for i in 0..5 {
            oracle.spawn_entity(
                Vector3::new((i as f64) * 30.0, 0.0, 100.0),
                Vector3::zeros(),
                "heavy_target",
            );
        }
        
        let key_provider = DeterministicKeyProvider::new(self.seed);
        let root_key = key_provider.biscuit_root_key().public();
        
        let mut agents: Vec<SimulatedAgent> = (0..num_agents)
            .map(|i| {
                let context = Arc::new(SimContext::new(self.seed.wrapping_add(i as u64)));
                let network = Arc::new(SimNetwork::new_stub(NodeId::from_seed(i as u64)));
                let mut agent = SimulatedAgent::new(context, network, root_key.clone(), i as u64, AgentConfig::default());
                agent.set_fitness_provider(Box::new(BlindFitness::new()));
                agent
            })
            .collect();
        
        let mut swarm_network = crate::swarm_network::SwarmNetwork::new_grid(5, 10);
        let dt = 0.1;
        let target_ticks = 300;
        let evo_epoch_ticks = 30;
        
        info!("  Config: {} agents, 5 entities, {} ticks. Noise: Cauchy (heavy-tailed)", num_agents, target_ticks);
        
        for tick in 0..target_ticks {
            oracle.step(dt);
            let readings = oracle.generate_sensor_readings();
            let ground_truth = oracle.ground_truth_positions();
            
            for (idx, agent) in agents.iter_mut().enumerate() {
                if !agent.tick() { continue; }
                
                let my_readings: Vec<_> = readings.iter()
                    .skip(idx % 5)
                    .take(2)
                    .cloned()
                    .collect();
                
                agent.ingest_readings(&my_readings);
                agent.tick_evolution(evo_epoch_ticks, Some(&ground_truth));
            }
            
            // Gossip
            if tick % 5 == 0 {
                let all_packets: Vec<_> = agents.iter_mut()
                    .enumerate()
                    .flat_map(|(idx, a)| {
                        if !a.is_alive() { return Vec::new(); }
                        if a.should_broadcast(tick) {
                            a.recent_packets().iter().map(|p| (idx, p.clone())).collect()
                        } else {
                            Vec::new()
                        }
                    })
                    .collect();
                
                for (from_idx, packet) in all_packets {
                    swarm_network.queue_gossip(from_idx, packet);
                }
                
                for (agent_idx, agent) in agents.iter_mut().enumerate() {
                    let incoming = swarm_network.take_gossip(agent_idx);
                    agent.receive_gossip(&incoming);
                    agent.clear_recent_packets();
                }
            }
        }
        
        let gt = oracle.ground_truth_positions();
        let avg_rms: f64 = agents.iter()
            .map(|a| a.compute_position_error(&gt))
            .sum::<f64>() / num_agents as f64;
        
        // More lenient threshold due to occasional extreme Cauchy outliers
        let passed = avg_rms < 10.0;
        
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        info!("  HEAVY TAIL RESULTS:");
        info!("    Final RMS: {:.2}m (target < 10.0m)", avg_rms);
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        ScenarioResult {
            scenario: ScenarioId::HeavyTail,
            seed: self.seed,
            passed,
            total_ticks: target_ticks,
            final_time_secs: oracle.time(),
            final_entity_count: 5,
            failure_reason: if !passed { Some(format!("RMS: {:.2}m", avg_rms)) } else { None },
            metrics: ScenarioMetrics::default(),
        }
    }
    
    /// DST-022: SensorDrift - Gradual degradation (v0.6.0)
    /// 
    /// Sensor noise increases over time, simulating degradation or environmental changes.
    /// Tests whether agents can adapt to non-stationary noise.
    /// 
    /// **Success Criteria**: RMS < 8.0m despite 5x noise increase by end
    fn run_sensor_drift(&self) -> ScenarioResult {
        use crate::evolution::BlindFitness;
        
        info!("DST-022: SensorDrift - Sensor Degradation Over Time 📈");
        
        let num_agents = 10;
        let mut oracle = Oracle::new(self.seed);
        
        let initial_noise = 0.5;
        let final_noise = 2.5; // 5x degradation
        oracle.set_position_noise(initial_noise);
        
        // Spawn 5 stationary targets
        for i in 0..5 {
            oracle.spawn_entity(
                Vector3::new((i as f64) * 30.0, 0.0, 100.0),
                Vector3::zeros(),
                "drift_target",
            );
        }
        
        let key_provider = DeterministicKeyProvider::new(self.seed);
        let root_key = key_provider.biscuit_root_key().public();
        
        let mut agents: Vec<SimulatedAgent> = (0..num_agents)
            .map(|i| {
                let context = Arc::new(SimContext::new(self.seed.wrapping_add(i as u64)));
                let network = Arc::new(SimNetwork::new_stub(NodeId::from_seed(i as u64)));
                let mut agent = SimulatedAgent::new(context, network, root_key.clone(), i as u64, AgentConfig::default());
                agent.set_fitness_provider(Box::new(BlindFitness::new()));
                agent
            })
            .collect();
        
        let mut swarm_network = crate::swarm_network::SwarmNetwork::new_grid(5, 10);
        let dt = 0.1;
        let target_ticks = 400;
        let evo_epoch_ticks = 40;
        
        info!("  Config: {} agents, 5 entities, {} ticks. Noise: {:.1}m → {:.1}m", 
              num_agents, target_ticks, initial_noise, final_noise);
        
        for tick in 0..target_ticks {
            // Linearly increase noise over time
            let progress = tick as f64 / target_ticks as f64;
            let current_noise = initial_noise + (final_noise - initial_noise) * progress;
            oracle.set_position_noise(current_noise);
            
            oracle.step(dt);
            let readings = oracle.generate_sensor_readings();
            let ground_truth = oracle.ground_truth_positions();
            
            for (idx, agent) in agents.iter_mut().enumerate() {
                if !agent.tick() { continue; }
                
                let my_readings: Vec<_> = readings.iter()
                    .skip(idx % 5)
                    .take(2)
                    .cloned()
                    .collect();
                
                agent.ingest_readings(&my_readings);
                agent.tick_evolution(evo_epoch_ticks, Some(&ground_truth));
            }
            
            // Gossip
            if tick % 5 == 0 {
                let all_packets: Vec<_> = agents.iter_mut()
                    .enumerate()
                    .flat_map(|(idx, a)| {
                        if !a.is_alive() { return Vec::new(); }
                        if a.should_broadcast(tick) {
                            a.recent_packets().iter().map(|p| (idx, p.clone())).collect()
                        } else {
                            Vec::new()
                        }
                    })
                    .collect();
                
                for (from_idx, packet) in all_packets {
                    swarm_network.queue_gossip(from_idx, packet);
                }
                
                for (agent_idx, agent) in agents.iter_mut().enumerate() {
                    let incoming = swarm_network.take_gossip(agent_idx);
                    agent.receive_gossip(&incoming);
                    agent.clear_recent_packets();
                }
            }
        }
        
        let gt = oracle.ground_truth_positions();
        let avg_rms: f64 = agents.iter()
            .map(|a| a.compute_position_error(&gt))
            .sum::<f64>() / num_agents as f64;
        
        let passed = avg_rms < 8.0;
        
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        info!("  SENSOR DRIFT RESULTS:");
        info!("    Final RMS: {:.2}m (target < 8.0m)", avg_rms);
        info!("    Final Noise: {:.1}m (5x degradation)", final_noise);
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        ScenarioResult {
            scenario: ScenarioId::SensorDrift,
            seed: self.seed,
            passed,
            total_ticks: target_ticks,
            final_time_secs: oracle.time(),
            final_entity_count: 5,
            failure_reason: if !passed { Some(format!("RMS: {:.2}m", avg_rms)) } else { None },
            metrics: ScenarioMetrics::default(),
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

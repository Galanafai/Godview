//! Chaos engineering scenarios for DST.

/// Scenario identifiers
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScenarioId {
    /// DST-001: OOSM stress test with extreme jitter
    TimeWarp,
    
    /// DST-002: Network partition and CRDT convergence
    SplitBrain,
    
    /// DST-003: Malicious agent with delayed revocation
    Byzantine,
    
    /// DST-004: H3 boundary crossing stress test
    FlashMob,
    
    /// DST-005: High packet loss recovery
    SlowLoris,
    
    /// DST-006: 50-agent multi-agent scale test
    Swarm,
    
    /// DST-007: 50-agent with learning + bad actors
    AdaptiveSwarm,
    
    // ═══════════════════════════════════════════════════
    // EXTREME CHAOS SCENARIOS - Push to the limit!
    // ═══════════════════════════════════════════════════
    
    /// DST-008: Everything bad at once
    ChaosStorm,
    
    /// DST-009: 200 agents, 1000 entities
    ScaleLimit,
    
    /// DST-010: 90% packet loss
    NetworkHell,
    
    /// DST-011: 5-second OOSM delays
    TimeTornado,
    
    /// DST-012: 50% of agents are bad actors
    ZombieApocalypse,
    
    /// DST-013: 100Hz tick rate
    RapidFire,
    
    // ═══════════════════════════════════════════════════
    // EVOLUTIONARY SCENARIOS
    // ═══════════════════════════════════════════════════
    
    /// DST-014: Evolution vs Chaos
    EvoWar,
    
    /// DST-015: Bandwidth constrained evolution
    ResourceStarvation,
    
    /// DST-016: Protocol divergence adaptation
    ProtocolDrift,
    
    /// DST-017: Blind fitness learning (no oracle)
    BlindLearning,
    
    /// DST-018: Total system failure survival
    BlackoutSurvival,
    
    /// DST-019: Energy Conserved Survival
    LongHaul,
    
    /// DST-020: Common GPS Bias Detection (v0.6.0)
    CommonBias,
    
    /// DST-021: Heavy-Tailed Noise (v0.6.0)
    HeavyTail,
    
    /// DST-022: Sensor Drift (v0.6.0)
    SensorDrift,
}

impl ScenarioId {
    /// Returns a list of all scenarios.
    pub fn all() -> Vec<ScenarioId> {
        vec![
            ScenarioId::TimeWarp,
            ScenarioId::SplitBrain,
            ScenarioId::Byzantine,
            ScenarioId::FlashMob,
            ScenarioId::SlowLoris,
            ScenarioId::Swarm,
            ScenarioId::AdaptiveSwarm,
            // Extreme scenarios
            ScenarioId::ChaosStorm,
            ScenarioId::ScaleLimit,
            ScenarioId::NetworkHell,
            ScenarioId::TimeTornado,
            ScenarioId::ZombieApocalypse,
            ScenarioId::RapidFire,
            // Evolutionary
            ScenarioId::EvoWar,
            ScenarioId::ResourceStarvation,
            ScenarioId::ProtocolDrift,
            ScenarioId::BlindLearning,
            ScenarioId::BlackoutSurvival,
            ScenarioId::LongHaul,
            ScenarioId::CommonBias,
            ScenarioId::HeavyTail,
            ScenarioId::SensorDrift,
        ]
    }
    
    /// Returns standard scenarios (not extreme).
    pub fn standard() -> Vec<ScenarioId> {
        vec![
            ScenarioId::TimeWarp,
            ScenarioId::SplitBrain,
            ScenarioId::Byzantine,
            ScenarioId::FlashMob,
            ScenarioId::SlowLoris,
            ScenarioId::Swarm,
            ScenarioId::AdaptiveSwarm,
        ]
    }
    
    /// Returns extreme scenarios only.
    pub fn extreme() -> Vec<ScenarioId> {
        vec![
            ScenarioId::ChaosStorm,
            ScenarioId::ScaleLimit,
            ScenarioId::NetworkHell,
            ScenarioId::TimeTornado,
            ScenarioId::ZombieApocalypse,
            ScenarioId::RapidFire,
            ScenarioId::BlackoutSurvival,
        ]
    }
    
    /// Returns evolutionary scenarios.
    pub fn evolutionary() -> Vec<ScenarioId> {
        vec![
            ScenarioId::EvoWar,
            ScenarioId::ResourceStarvation,
            ScenarioId::ProtocolDrift,
            ScenarioId::CommonBias,
            ScenarioId::HeavyTail,
            ScenarioId::SensorDrift,
        ]
    }
    
    /// Returns scenarios suitable for strict CI enforcement.
    /// Excludes: performance-dependent (ScaleLimit), research (CommonBias).
    pub fn stable_ci() -> Vec<ScenarioId> {
        vec![
            // Baseline
            ScenarioId::TimeWarp,
            ScenarioId::SplitBrain,
            ScenarioId::Byzantine,
            ScenarioId::FlashMob,
            ScenarioId::SlowLoris,
            ScenarioId::Swarm,
            ScenarioId::AdaptiveSwarm,
            // Extreme (not perf-dependent)
            ScenarioId::ChaosStorm,
            ScenarioId::NetworkHell,
            ScenarioId::TimeTornado,
            ScenarioId::ZombieApocalypse,
            ScenarioId::RapidFire,
            // Evolutionary (stable)
            ScenarioId::EvoWar,
            ScenarioId::ResourceStarvation,
            ScenarioId::BlindLearning,
            ScenarioId::BlackoutSurvival,
            ScenarioId::LongHaul,
            ScenarioId::HeavyTail,
            ScenarioId::SensorDrift,
        ]
    }
    
    /// Returns research scenarios (allowed to fail in CI).
    pub fn research() -> Vec<ScenarioId> {
        vec![
            ScenarioId::ScaleLimit,     // Performance-dependent
            ScenarioId::CommonBias,     // Evolving bias (needs more epochs)
            ScenarioId::ProtocolDrift,  // Experimental
            ScenarioId::SensorDrift,
        ]
    }
    
    /// Returns the scenario name.
    pub fn name(&self) -> &'static str {
        match self {
            ScenarioId::TimeWarp => "time_warp",
            ScenarioId::SplitBrain => "split_brain",
            ScenarioId::Byzantine => "byzantine",
            ScenarioId::FlashMob => "flash_mob",
            ScenarioId::SlowLoris => "slow_loris",
            ScenarioId::Swarm => "swarm",
            ScenarioId::AdaptiveSwarm => "adaptive_swarm",
            // Extreme
            ScenarioId::ChaosStorm => "chaos_storm",
            ScenarioId::ScaleLimit => "scale_limit",
            ScenarioId::NetworkHell => "network_hell",
            ScenarioId::TimeTornado => "time_tornado",
            ScenarioId::ZombieApocalypse => "zombie_apocalypse",
            ScenarioId::RapidFire => "rapid_fire",
            // Evolutionary
            ScenarioId::EvoWar => "evo_war",
            ScenarioId::ResourceStarvation => "resource_starvation",
            ScenarioId::ProtocolDrift => "protocol_drift",
            ScenarioId::BlindLearning => "blind_learning",
            ScenarioId::BlackoutSurvival => "blackout_survival",
            ScenarioId::LongHaul => "long_haul",
            ScenarioId::CommonBias => "common_bias",
            ScenarioId::HeavyTail => "heavy_tail",
            ScenarioId::SensorDrift => "sensor_drift",
        }
    }
    
    /// Returns a description of the scenario.
    pub fn description(&self) -> &'static str {
        match self {
            ScenarioId::TimeWarp => "OOSM stress test with 0-500ms jitter and 20% reordering",
            ScenarioId::SplitBrain => "Network partition for 10s, verify Min-UUID convergence",
            ScenarioId::Byzantine => "Malicious agent with delayed revocation propagation",
            ScenarioId::FlashMob => "1000 drones crossing H3 boundaries rapidly",
            ScenarioId::SlowLoris => "50% packet loss, verify protocol recovery",
            ScenarioId::Swarm => "50 agents, 200 entities, P2P gossip, convergence test",
            ScenarioId::AdaptiveSwarm => "50 agents + 5 bad actors, learning to identify them",
            // Extreme
            ScenarioId::ChaosStorm => "🔥 EVERYTHING AT ONCE: jitter + loss + bad actors + moving",
            ScenarioId::ScaleLimit => "🔥 200 AGENTS, 1000 ENTITIES: stress test scalability",
            ScenarioId::NetworkHell => "🔥 90% PACKET LOSS: find the breaking point",
            ScenarioId::TimeTornado => "🔥 5-SECOND DELAYS: extreme OOSM stress",
            ScenarioId::ZombieApocalypse => "🔥 50% BAD ACTORS: can good agents survive?",
            ScenarioId::RapidFire => "🔥 100Hz TICK RATE: high-frequency stress test",
            // Evolutionary
            ScenarioId::EvoWar => "🧬 Evolution vs Chaos: agents adapt parameters to survive",
            ScenarioId::ResourceStarvation => "🧬 Bandwidth Constrained: evolve efficiency",
            ScenarioId::ProtocolDrift => "🧬 Protocol Divergence: one group changes logic",
            ScenarioId::BlindLearning => "🧬 Blind Learning: adapt without ground truth",
            ScenarioId::BlackoutSurvival => "💀 BLACKOUT: Loss + Faults + Bad Actors + Bandwidth Limit",
            ScenarioId::LongHaul => "🔋 LONG HAUL: Energy Crisis Survival",
            ScenarioId::CommonBias => "🎯 COMMON BIAS: All agents get +5m GPS offset, evolve compensation",
            ScenarioId::HeavyTail => "📉 HEAVY TAIL: Cauchy noise tests Gaussian-evolved parameters",
            ScenarioId::SensorDrift => "📈 SENSOR DRIFT: Gradual sensor degradation over time",
        }
    }
    
    /// Returns true if this is an extreme scenario.
    pub fn is_extreme(&self) -> bool {
        matches!(self, 
            ScenarioId::ChaosStorm | 
            ScenarioId::ScaleLimit | 
            ScenarioId::NetworkHell |
            ScenarioId::TimeTornado |
            ScenarioId::ZombieApocalypse |
            ScenarioId::RapidFire |
            ScenarioId::BlackoutSurvival
        )
    }

    /// Returns true if this is an evolutionary scenario.
    pub fn is_evolutionary(&self) -> bool {
        matches!(self,
            ScenarioId::EvoWar |
            ScenarioId::ResourceStarvation |
            ScenarioId::ProtocolDrift |
            ScenarioId::BlindLearning
        )
    }
}

impl std::fmt::Display for ScenarioId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.name())
    }
}

impl std::str::FromStr for ScenarioId {
    type Err = String;
    
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "time_warp" | "timewarp" | "dst-001" => Ok(ScenarioId::TimeWarp),
            "split_brain" | "splitbrain" | "dst-002" => Ok(ScenarioId::SplitBrain),
            "byzantine" | "dst-003" => Ok(ScenarioId::Byzantine),
            "flash_mob" | "flashmob" | "dst-004" => Ok(ScenarioId::FlashMob),
            "slow_loris" | "slowloris" | "dst-005" => Ok(ScenarioId::SlowLoris),
            "swarm" | "dst-006" => Ok(ScenarioId::Swarm),
            "adaptive_swarm" | "adaptiveswarm" | "dst-007" => Ok(ScenarioId::AdaptiveSwarm),
            // Extreme
            "chaos_storm" | "chaosstorm" | "dst-008" => Ok(ScenarioId::ChaosStorm),
            "scale_limit" | "scalelimit" | "dst-009" => Ok(ScenarioId::ScaleLimit),
            "network_hell" | "networkhell" | "dst-010" => Ok(ScenarioId::NetworkHell),
            "time_tornado" | "timetornado" | "dst-011" => Ok(ScenarioId::TimeTornado),
            "zombie_apocalypse" | "zombieapocalypse" | "dst-012" => Ok(ScenarioId::ZombieApocalypse),
            "rapid_fire" | "rapidfire" | "dst-013" => Ok(ScenarioId::RapidFire),
            // Evolutionary
            "evo_war" | "evowar" | "dst-014" => Ok(ScenarioId::EvoWar),
            "resource_starvation" | "resourcestarvation" | "dst-015" => Ok(ScenarioId::ResourceStarvation),
            "protocol_drift" | "protocoldrift" | "dst-016" => Ok(ScenarioId::ProtocolDrift),
            "blind_learning" | "blindlearning" | "dst-017" => Ok(ScenarioId::BlindLearning),
            "blackout_survival" | "blackoutsurvival" | "dst-018" => Ok(ScenarioId::BlackoutSurvival),
            "long_haul" | "longhaul" | "dst-019" => Ok(ScenarioId::LongHaul),
            "common_bias" | "commonbias" | "dst-020" => Ok(ScenarioId::CommonBias),
            "heavy_tail" | "heavytail" | "dst-021" => Ok(ScenarioId::HeavyTail),
            "sensor_drift" | "sensordrift" | "dst-022" => Ok(ScenarioId::SensorDrift),
            // Groups
            "extreme" => Err("Use --scenario extreme (or all) for extreme scenarios".to_string()),
            "evolutionary" => Err("Use --scenario evolutionary (or all) for evolutionary scenarios".to_string()),
            _ => Err(format!("Unknown scenario: {}", s)),
        }
    }
}

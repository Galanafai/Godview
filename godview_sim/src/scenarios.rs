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
        }
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
            _ => Err(format!("Unknown scenario: {}", s)),
        }
    }
}


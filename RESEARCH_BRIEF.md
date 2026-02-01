# GodView Project: Evolutionary Swarm Intelligence Research Brief

## 1. Project Overview
GodView is a distributed tracking system for drone swarms that uses **Deterministic Simulation Testing (DST)** to verify resilience.
Recently, we implemented **Online Evolutionary Learning** to allow agents to "self-tune" their parameters (e.g., gossip frequency, Kalman filter noise models) in real-time.

## 2. Core Hypothesis: "Blind Fitness"
Our main theoretical contribution is **Blind Fitness** (`docs/blind_fitness.md`).
We posit that agents can optimize their tracking accuracy *without* independent ground truth by minimizing a composite cost function of:
1.  **NIS (Normalized Innovation Squared)**: Internal consistency of the Kalman Filter.
2.  **Peer Agreement**: Consensus with trusted neighbors (weighted by reputation).
3.  **Bandwidth**: Penalty for excessive communication.

**Status**: PROVEN in Simulation.
- Scenarios enabled agents to adapt to noise and packet loss, achieving ~3m RMS error without ground truth guidance.

## 3. The Current Challenge: "The Energy Crisis" (DST-019)
We stress-tested this system with `DST-019: LongHaul` (`godview_sim/FINAL_REPORT.md`), introducing a strict energy economy:
- **Constraints**: Limited battery (150J), costs for sensing (0.05J) and messaging (1J).
- **Goal**: Evolve efficiency strategies (e.g., silence) to survive.
- **Result**: **0% Survival**. 
- **Analysis**: The evolutionary pressure was too slow (linear `+1` tick mutations) to outpace the exponential energy drain. The agents "evolved to death."

## 4. Why We Need Review
We are preparing to publish/deploy this system. We need an unbiased "Red Team" analysis of:
1.  **Theoretical Gaps**: Is "Blind Fitness" mathematically robust, or are there "Groupthink" local minima we haven't found?
2.  **Evolutionary Stagnation**: Why did `DST-019` fail so hard? Is our mutation strategy (small incremental steps) fundamentally flawed for critical survival?
3.  **Simulation Artifacts**: Are we overfitting to our specific simulation logic?

## 5. Key Files Code Map
- `godview_sim/src/evolution.rs`: The genotype/phenotype logic.
- `godview_sim/src/runner.rs`: The simulation environment and scenarios.
- `godview_core/src/godview_tracking.rs`: The consensus mechanism (Peer Agreement).
- `godview_core/src/godview_time.rs`: The math (NIS calculation).
- `godview_sim/src/agent.rs`: How the agent ties these inputs together.

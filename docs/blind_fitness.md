BLIND_FITNESS
Overview and Motivation
In the current GodView evolutionary learning system (see evolution.rs ), agent fitness is evaluated
using an oracle-based ground truth error (e.g. RMS position error in simulation). This is feasible in
simulation where a deterministic Oracle provides true state data 1 . However, on real hardware we cannot
rely on any omniscient ground truth. We need to transition to a “blind” fitness function that each agent
can compute autonomously using only onboard sensor data and peer communications. The goal is to
achieve comparable adaptive performance in the real world by using internal consistency and consensus
metrics in place of direct error-to-truth.
Key Goals: - Remove Oracle Dependence: Replace the ground-truth-based fitness metric with one
computable from local information alone.
- Internal Consistency: Leverage the Kalman Filter’s Normalized Innovation Squared (NIS) as a proxy for
how well the agent’s internal state estimates match its sensor measurements (i.e. filter consistency).
- Peer Agreement: Introduce a reputation-weighted peer agreement metric (using the existing
AdaptiveState neighbor reputation system) to encourage consensus with other agents’ estimates while
discounting faulty peers.
- Resource Efficiency: Incorporate a Bandwidth/Communication cost term to penalize excessive gossip
or network use, ensuring evolution favors solutions that conserve bandwidth.
- Composite Fitness: Define a single composite fitness function that combines NIS, peer agreement, and
bandwidth cost. This composite “blind” fitness should guide the mutation-selection loop on hardware,
enabling agents to self-tune in real time without any oracle.
By carefully designing this blind fitness function, the evolutionary algorithm can continue to “self-heal” and
adapt the system on real drones/nodes just as it did in simulation 2 , but now using only on-board
feedback signals.
Components of the New Blind Fitness Function
The proposed fitness function will be a weighted combination of three components: (1) Normalized
Innovation Squared (NIS) for sensor consistency, (2) Peer Agreement (with AdaptiveState reputation
weighting), and (3) Bandwidth Cost for communication overhead. Each component is defined below with
its mathematical formulation and rationale.
1. Normalized Innovation Squared (NIS) – Internal Consistency
Definition: NIS is a well-known statistic from Kalman filtering that measures how consistent the filter’s
predictions are with incoming measurements. At each sensor update, the filter computes a measurement
residual ν(k) = y(k) − y
^(k) (the difference between the actual measurement and the predicted
measurement). The filter also computes the innovation covariance S(k), which is the expected covariance
1of that residual given the current state uncertainty and measurement noise. The Normalized Innovation
Squared is defined as:
NIS(k) = ν(k)T S(k)−1 ν(k) .
This scalar ϵν (k) is chi-square distributed with degrees of freedom equal to the measurement dimension
nz
3
. Intuitively, NIS tells us how “surprising” a new sensor reading is, relative to what the filter expected.
Lower NIS means the measurement falls within the filter’s predicted uncertainty envelope (good
consistency), whereas high NIS indicates the measurement deviated more than expected (potential
inconsistency or unmodeled effects).
Rationale: NIS can be evaluated without any ground truth state – it uses only the filter’s internal
covariance and the actual sensor reading. Therefore, it is ideal as a proxy for accuracy in the real world
where truth is unknown 4 . A well-tuned filter with correct noise models should produce an average NIS
roughly equal to the measurement dimension, and NIS values should mostly lie within a confidence interval
derived from the chi-square distribution 5 6 . If the agent’s filter is mis-tuned or the environment
changed (e.g. sensor noise increased, or the agent’s state estimate drifted), the NIS will tend to be larger
than expected (filter is inconsistent with reality). Thus, by minimizing NIS, an agent is effectively tuning
itself to maintain consistency between its predictions and observations.
For the fitness function, we will use an aggregated NIS metric over a time window. For example, each
agent can compute the average NIS (ANIS) over the last N sensor updates:
N
NIS =
1
∑ ν(i)T S(i)−1 ν(i) .
N i=1
This time-averaged NIS smooths out momentary fluctuations and provides a stable measure of filter
consistency. The fitness component derived from NIS can be simply this average (to be minimized), or an
equivalent “consistency score” that is higher when NIS is within expected bounds. We prefer to treat it as a
cost term: lower is better. In practice, we may also define thresholds: e.g. if NIS exceeds the upper bound
of the 95% chi-square confidence interval regularly, that indicates poor filter consistency 7 . The fitness
function could heavily penalize such threshold violations (to encourage evolutionary pressure to fix gross
inconsistencies, such as adding process noise or re-tuning sensor noise parameters). Conversely, if NIS is
consistently too low (below the expected range), the filter might be overestimating its uncertainty
(inefficient). Extremely low NIS might receive a mild penalty as well, to avoid trivial solutions like inflating
covariance estimates. Overall, the NIS term drives the agent to self-calibrate its Kalman filter: for
instance, adjusting process noise, measurement noise, or gating out outlier readings to keep innovations in
check.
Why NIS? This metric allows each agent to verify filter performance without true data by
checking innovation consistency 8 . It has been recommended to use NIS when ground
truth is unavailable 4 , making it a perfect foundation for blind fitness.
2. Peer Agreement – Consensus via AdaptiveState Reputation
Definition: The peer agreement component measures how well an agent’s state estimates align with those
of its neighbors, taking into account the trustworthiness of those neighbors. Let x
^i be the state vector
2estimated by the agent in question (e.g. the positions/velocities of all tracked entities that this agent is
maintaining). Let each neighbor j provide its own state estimates x
^j . We define a disagreement metric for
each neighbor as the distance between the two estimates, e.g. dij = ∥x
^i − x
^j ∥ (this could be an Euclidean
norm for continuous state differences, or a more complex measure combining state differences and any
difference in the set of tracked objects). Because not all neighbors are equally reliable, we introduce a
reputation weight wij for neighbor j, as maintained by the AdaptiveState system. The AdaptiveState
module is the neighbor reputation learning mechanism ( adaptive.rs ) that was validated in simulation
to detect and isolate bad actors 9 . Each agent maintains a reputation score for peers based on past
behavior (e.g. whether that neighbor’s data often conflicts with others or is flagged as malicious). These
weights satisfy 0 ≤ wij ≤ 1 and are dynamically adjusted; a high weight means neighbor j is deemed
trustworthy, whereas a neighbor that frequently broadcasts inconsistent or erroneous data will have its
weight reduced (approaching 0).
Using these, we define the Peer Agreement error for agent i as a weighted average of its disagreement
with all peers:
Epeer,i =
∑j∈neighbors wij dij
∑j∈neighbors wij
.
In words, Epeer is the average difference between agent i’s state and its neighbors’ states, weighted by
neighbor reputation. If the agent has perfect agreement with its most trusted peers, this error will be low.
Large divergence from trustworthy peers will increase this metric. (If an agent has no neighbors or all
weights are zero, we can define Epeer to be 0 or neutral by convention – effectively that term is ignored
when isolated.)
Importantly, because we assume all agents are estimating the same underlying truth (e.g. tracking the same
set of entities in a shared environment), a correct agent should eventually converge with other correct
agents on the same estimates 10 11 . In simulation, one metric of multi-agent consistency was the track
count coefficient of variation (CV) across agents, which was required to stay below 15% 12 (i.e. all agents
should see roughly the same number of tracks). Peer Agreement extends this idea: not only should the
count of entities match, but the estimated state values (positions, IDs, etc.) should match across agents. Any
persistent disagreement likely indicates an error in one of the agents (or a lack of communication).
Rationale: The peer agreement term incentivizes cooperative consistency. An agent that evolves better
consensus behavior (e.g. tuning its parameters to more closely match neighbors on shared estimates) will
score better. This is crucial in a decentralized system – we want all agents to converge on a common
operational picture. By incorporating peer agreement into fitness, we encourage behaviors like: - Trust
alignment: If an agent finds itself in disagreement with high-reputation peers, the evolutionary pressure
will push it to adjust its filter or fusion strategy (or even increase communication) to reduce that
discrepancy. In effect, the swarm will gravitate toward a common estimate through this evolutionary
feedback. - Bad actor resistance: If a neighbor is malicious or faulty (sending divergent data), its
reputation weight wij will drop (AdaptiveState’s job) 13 . This means disagreements with that neighbor
contribute very little to the fitness penalty. In other words, the fitness function does not punish an agent
for ignoring or differing from an untrusted peer. This is critical: we don’t want agents to be penalized for
not agreeing with a bad actor. Instead, they are rewarded for aligning with the majority of honest peers. The
reputation weighting ensures robustness to Byzantine behavior – a lesson learned and validated in
3simulation (e.g. 100% of bad actors were detected and isolated in the AdaptiveSwarm scenario 9 ). - Stable
consensus vs. sensor bias: The peer term also helps detect if an agent’s own sensors are biased or
malfunctioning. For instance, if one agent’s sensor is consistently off, that agent’s state will diverge from
others; it will see a high peer disagreement, which will push the evolutionary algorithm to adjust (perhaps
to trust its sensor less or rely more on neighbor updates). In the absence of ground truth, the best proxy for
“correctness” is agreement among multiple independent observers. The peer metric harnesses the wisdom
of the swarm as a stand-in for truth. So long as a majority of agents are healthy, a lone straggler will be
nudged to conform. (We do assume at least some neighbors are seeing the truth or consensus; if all agents
share the same bias, see Edge Cases below.)
Implementation: Each agent will leverage the existing trust/adaptive reputation subsystem (the
AdaptiveState logic in godview_sim/src/adaptive.rs ) to compute this metric. In practice, agents
already gossip their state estimates (track data) to neighbors; they can compute discrepancy for each
incoming neighbor update. The reputation values wij are updated continuously by that subsystem (for
example, if neighbor j frequently sends data that the agent deems implausible or far off from others, j’s rep
drops). We will integrate those weights into the fitness calculation as shown above. The Peer Agreement
fitness component will then be proportional to Epeer – lower is better (lower disagreement with trusted
peers). This encourages each agent to keep its state in line with the swarm’s consensus. Essentially, this
forms an internal distributed error metric: instead of error to ground truth, we use error to neighbors
(trust-weighted) as a surrogate for error to the true state.
3. Bandwidth Cost – Communication Overhead Penalty
Definition: The bandwidth cost component measures how much network resource an agent is consuming,
and penalizes high usage. In our system, agents communicate via a peer-to-peer gossip protocol to share
track updates and other data. We can quantify “bandwidth usage” per agent as, for example, the number
of bytes or packets sent per unit time, or the fraction of allotted bandwidth/time slots used. For
fitness purposes, we will define a normalized cost metric Cbw such as:
Cbw =
(packets sent per second)
(bandwidth budget per second)
This would yield Cbw = 1.0 when an agent is exactly at the expected bandwidth budget, >1 if it exceeds
(undesirable), and <1 if it’s using less. Alternatively, a simpler definition could be total packets or bytes sent
in a fixed window (since in many scenarios we have a hard cap on bandwidth, e.g. LoRa duty cycle). We may
also include compute cost implicitly here, but the main focus is network usage. We treat Cbw as a penalty
term to minimize (lower communication overhead yields better fitness).
Rationale: In a real deployment, communications are limited (some agents may be on low-bandwidth
radios) and excessive chatter can congest the network or waste energy. We need to ensure the evolutionary
process doesn’t “cheat” the above metrics by, say, sending an overwhelming number of messages to
perfectly agree with peers. The bandwidth cost term provides a counterbalance, rewarding agents that
achieve good NIS and peer agreement efficiently. In simulation, we observed scenarios where reducing
gossip frequency was beneficial: for example, in the ResourceStarvation test (DST-015), agents evolved to
increase their gossip interval from 5 to 7.1 ticks to reduce load, cutting bandwidth usage while maintaining
accuracy. By explicitly including a bandwidth term in fitness, we bake in this preference for using minimal
communication necessary to reach consensus.
4In practice, this component will push the agents to optimize the trade-off between accuracy and
bandwidth. If an agent can achieve similar peer agreement with fewer messages (e.g. by smartly
scheduling updates or compressing data), it will have a better fitness. Conversely, an agent that spam-sends
every tiny update may score slightly better on agreement, but will pay a cost in the fitness for heavy
bandwidth usage.
Implementation: We will instrument the networking layer (or the agent runtime) to track each agent’s
outgoing data. For example, each agent can keep a rolling count of packets or bytes sent in the last T
seconds. We then normalize that by a target value or maximum allowed. The Bandwidth cost term in the
fitness will be proportional to this usage. Likely we will scale it so that using 100% of the allowed bandwidth
(budget) gives a baseline cost of 1.0, and using 50% gives 0.5, etc., making it easy to interpret. The exact
weight in the composite fitness will determine how strongly evolution favors bandwidth thrift versus other
metrics (more on weight selection below).
4. Composite Fitness Formulation
With the above components defined, we now construct the composite fitness function. We will treat
fitness as a cost to minimize (lower composite score = better). Let:
• JNIS = NIS = average NIS over the evaluation period (as defined above). This reflects filter
inconsistency (so lower is better).
• JPA = Epeer = reputation-weighted peer disagreement (the average difference from neighbors’
estimates). Lower is better (more consensus).
• JBW = Cbw = normalized bandwidth consumption. Lower is better (more efficient).
We combine them as a weighted sum (or linear combination):
Jfitness = wnis ⋅ JNIS + wpeer ⋅ JPA + wbw ⋅ JBW .
Here wnis , wpeer , wbw are weighting coefficients that balance the contribution of each component. These
weights will be chosen based on the relative importance and scaling of the metrics: - Normalization: We
will likely normalize each component to a comparable scale before weighting. For example, NIS is
dimensionless but on the order of the measurement dimension (~a few units), peer disagreement might be
in physical units (meters) or could be normalized by some typical error, and bandwidth is normalized as
described. We want changes in each term to influence fitness comparably. One approach is to scale each
raw metric by its expected acceptable value, so that a value of 1.0 represents roughly the borderline of good
performance for that term. - Initial Weights: For a first implementation, we might set all w equal
(assuming normalization has been done). Alternatively, if we know that (for instance) maintaining a
reasonable accuracy is paramount, we might weight NIS and peer higher, and bandwidth slightly lower. In
DST results, accuracy was maintained under 1 m RMS even when agents adapted gossip rate, suggesting
that we should not sacrifice too much accuracy for bandwidth. So we might set wnis and wpeer higher to
ensure evolution doesn’t overly starve communication just to save bandwidth. Fine-tuning can be done via
simulation or A/B tests (see Validation section). - Fitness Maximization: If the evolutionary algorithm
expects a higher fitness value to be better (rather than a cost to minimize), we can invert this formulation. For
example, define Fitness = K − Jfitness for some large constant, or define individual reward terms as the
inverse of each cost. In practice, it’s straightforward to implement minimization by picking individuals with
lowest score; we just need to be consistent. We will treat Jfitness as something to minimize in concept.
5Interpretation: The composite fitness essentially encodes “well-tuned, in-agreement, and efficient” as
the ideal behavior: - If an agent’s filter is well-tuned (low NIS) and it agrees with peers and it uses little
bandwidth, it will have a very low Jfitness (good). - If it tries to cheat one aspect (e.g. spams messages to gain
perfect peer agreement), the bandwidth term rises and hurts fitness. - If it isolates itself to save bandwidth,
peer disagreement rises. - If it ignores sensor updates to avoid high NIS (e.g. by giving them low weight),
then it may fall out of sync with reality and peers, affecting both NIS (in long term if filter diverges) and peer
terms. - Thus, the agent is driven to find a balance: just enough communication to stay in sync, just the
right filter tuning to respect sensor data, and alignment with trustworthy neighbors. This mirrors the multi-
objective nature of the original problem (accuracy vs. resources vs. resilience) but collapses it into one scalar
for the evolutionary algorithm to optimize.
Evolutionary Loop Architecture (Mutation-Selection without
Ground Truth)
With the new fitness function in place, we now describe how the evolutionary mutation-selection loop will
operate on real hardware using only local and peer data. The core idea is that each agent will periodically
mutate its own internal parameters, evaluate its performance via the blind fitness metrics (NIS, peer,
bandwidth), and then select whether to adopt the new parameters based on improvement in those metrics.
Key Parameters to Evolve: In simulation, the evolutionary system was used to tune parameters like gossip
frequency, neighbor selection, and filter settings to optimize performance 14 . On hardware, we target
similar parameters. Examples include: - Gossip interval / rate: How often to send state updates to
neighbors. (A higher interval means less frequent messages, saving bandwidth at risk of lower consensus.) -
Neighbor count or topology: How many neighbors to actively communicate with, or weighting of
neighbor inputs. (E.g. an agent might evolve to listen to more neighbors in a hostile scenario to ensure it
isn’t misled by one bad actor 14 .) - Kalman filter tuning: Process noise covariance Q, measurement noise
covariance R, and possibly gating thresholds for outlier rejection. (These directly affect NIS; evolution
might, for example, increase R if sensors are noisier than assumed, to bring NIS down into consistent
range, or adjust how aggressively to fuse neighbor information vs. local sensor.) - Data fusion parameters:
e.g. how to weight incoming neighbor track data versus self-detected data, how long to persist tracks
without updates, etc. - AdaptiveState sensitivity: Possibly meta-parameters of the reputation system
(though likely that is fixed by design).
We don’t enumerate all possible genes here, but the system should be designed to allow mutation of any
tunable parameter that could improve the blind fitness metrics. The mutation can be random
perturbations drawn from some distribution, similar to how it was done in simulation (e.g. slight increase/
decrease in a parameter, or random reconfiguration).
Single-Agent Optimization vs. Population: In simulation, we had many agents and could evaluate
multiple mutated variants in parallel using the oracle fitness. On hardware, each physical agent is
essentially on its own (though all are running the same algorithm). We intend for each agent to perform its
own evolutionary adaptation locally. This can be thought of as 50 (for example) independent evolutionary
processes running if there are 50 drones, each tuning itself. Since all agents share the same code logic, in
practice they will likely converge to similar parameters (especially because peer agreement pressures them
toward similar communication patterns and filter settings). We are not doing a “genetic exchange” between
agents at this time (no explicit crossover of strategies between different agents), though indirectly,
6successful strategies spread because if one agent finds a setting that yields better agreement, others might
benefit by adapting towards it to agree.
Feedback Loop Structure: Pseudocode-wise, each agent will continuously execute a loop as follows:
1. Collect Baseline Metrics: The agent runs with its current parameter set Θcurrent and observes its
ongoing NIS, peer agreement, and bandwidth usage. It can maintain a sliding window or
periodically compute the fitness score Jfitness,current for recent performance (e.g. every 30 seconds or
appropriate timescale). This serves as the baseline performance.
2. Mutation (Propose New Parameters): The agent creates a mutated candidate Θnew by randomly
perturbing one or more parameters of Θcurrent . For example, it might try increasing the gossip
interval by +20%, or tweaking the process noise scalar, etc. The mutation step can be stochastic
(possibly guided by previous successes). In code, this could involve calling a mutation function
similar to the simulation’s, but without reliance on the oracle.
3. Deploy & Evaluate: The agent temporarily switches to Θnew and runs the system under these new
parameters for a trial period (e.g. the next N sensor updates or M seconds). During this trial, it
collects the same metrics: compute average NIS, peer disagreement, bandwidth usage under the
new parameter set. At the end of the trial, the agent computes a candidate fitness score Jfitness,new
using the composite formula. (The trial period must be long enough to get statistically meaningful
metrics; it could be a fixed time window or perhaps one could run both parameter sets in parallel if
resources allow – see below).
4. Selection: The agent compares the new fitness score to the baseline. If Jnew < Jcurrent (the new
parameters improve the composite fitness, i.e. lower cost), then the agent accepts the mutation –
Θnew becomes the current parameter set going forward (we essentially “evolve” into that state). If
instead Jnew ≥ Jcurrent (no improvement or worse), the agent reverts to Θcurrent (discard the
mutation). Either way, the loop then repeats by proposing another random mutation after some
time. This resembles a simple evolutionary strategy (1+1 EA or hill-climbing). Over time, beneficial
mutations accumulate, tuning the agent.
5. Repeat Continuously: This process is continuous and on-line. The agent is always trying occasional
tweaks and keeping those that help. The rate of mutation trials can be tuned so as not to disturb the
system too frequently. For example, the agent might spend most of the time with its best-known
parameters, but every minute or so, initiate a short trial of a mutation to see if it can do even better.
Diagram – Blind Evolutionary Feedback Loop:
We can illustrate the above loop in a simplified flow:
• Agent with current parameters → (Periodic Mutation) → Candidate parameter set → Apply
candidate in live operation → Measure NIS, Peer, Bandwidth → Compute composite fitness →
Compare to baseline → Select best (keep or revert) → (loop back to Mutation).
Each agent essentially has a self-contained feedback loop where experience (sensor data + peer data)
informs adaptation. Figure 1 (conceptually) would show the cycle: Mutate → Test → Feedback (NIS/Peer) →
Select → Mutate…, all happening on-board without external supervision.
Parallel/Concurrent Evaluation: One consideration is that environment conditions can change over time,
which might make sequential trial comparisons noisy. If feasible, an agent could run two internal filter
instances concurrently, one with current parameters and one with mutated parameters, on the same
7stream of data, to directly compare outcomes in parallel. For example, it could maintain a shadow Kalman
filter or a ghost agent with variant settings. Both would process the same sensor inputs and peer updates
(the mutated one perhaps not broadcasting to others to avoid confusion), and we compare their metrics
side-by-side. This would control for environmental variations. However, this doubles computational load and
complexity. Alternatively, the agent can alternate parameters in rapid succession (e.g. switch every other
measurement) to intermix conditions – but that might destabilize things. The simplest approach is likely as
described: assume the environment changes slowly enough that a short evaluation is representative. We
will need to smooth out random fluctuations by using sufficiently long evaluation windows or repeated
trials.
Integration into System: The evolutionary loop will be integrated into the agent’s runtime. Likely, we will
create a background task (or incorporate into the main event loop at a lower frequency) that handles the
mutation and evaluation. The existing code in evolution.rs can be repurposed: instead of using the
simulation’s global fitness, it will fetch metrics from the agent’s own sensors and neighbor data. Each agent
can instantiate an EvolutionManager that periodically triggers this process. We must ensure that switching
parameters on the fly is handled carefully (e.g. reinitialize Kalman filter if Q/R change significantly, etc., or
smoothly transition). We will also include safety checks – for example, do not allow extremely detrimental
mutations to persist (if a new parameter causes obvious instability, we might revert immediately rather
than waiting full trial). Storing the last known “good” configuration is important so we can roll back if
needed.
In summary, the mutation-selection loop gives the agent a blind self-tuning capability. It continuously
asks “if I change this, do my consistency and agreement get better or worse?” and iteratively finds a near-
optimal setting. This is analogous to how in simulation the system discovered surprising adaptations (like
throttling gossip to reduce load) purely via evolutionary pressure, but now the pressure comes from
internal metrics rather than an external evaluator.
Edge Cases and Robustness Considerations
Designing a blind fitness function and on-line evolution raises important edge cases. We address how the
system should handle these to ensure stability and reliability:
• Malicious or Divergent Peers: In cases where one or more neighbors are broadcasting incorrect
information (due to malfunction or a spoofing attack), agents might see high peer disagreement.
The Adaptive Reputation mechanism mitigates this by rapidly lowering the weight of such peers
13 . An agent that disagrees with a low-reputation peer will not be heavily penalized in the peer
agreement term. Thus, the fitness function naturally steers agents to ignore outliers: they will focus
on agreeing with the trusted majority. If an entire cluster of neighbors is malicious (Sybil attack), the
reputation system (backed by cryptographic identity and detection of inconsistencies) will isolate
them as a group 15 , and honest agents will weight them low. It’s important to ensure the peer
metric calculation uses the up-to-date weights so that once a neighbor is flagged, its influence on
fitness is negligible. Evolutionarily, this means an agent won’t erroneously adjust its filter to match a
rogue neighbor; instead, it might evolve to increase its own trust threshold or seek more inputs
from other peers. This was proven out in simulation: the swarm automatically isolated bad actors
9 . We expect the same in real deployment: the blind fitness should cause any agent that
accidentally tried to follow a malicious peer (increasing disagreement with others and likely spiking
its NIS due to contradictory data) to get a worse score, thereby discouraging that behavior.
8• Consensus vs. Global Bias (“Groupthink”): A tricky scenario is if all agents share a similar bias or
error (e.g. a systematic sensor bias affecting everyone or a common model error). In that case, the
agents might all agree with each other (peer agreement is fine) but all be wrong relative to the true
state. NIS should catch this if the bias leads to a sustained non-zero residual. For example, if every
drone’s GPS is off by 5 meters due to ionospheric error, each filter’s residuals will show a bias (unless
the filter includes a bias state). The average NIS would rise because measurements consistently
deviate in one direction more than expected noise. The evolutionary system may respond by
increasing process noise or adding a bias term (if such mutation is possible) to accommodate that
discrepancy, thereby bringing NIS down. If the bias is static and the filter cannot inherently correct it,
NIS stays high and the system will try various adaptations; one could be to overly trust peers (but
they have the same bias, so that doesn’t solve NIS). This is a limitation: the swarm as a whole might
converge to a biased solution that looks consistent internally. Mitigation: in the design, we can allow
evolution of certain global calibration parameters if known (e.g. a bias estimate that all agents carry).
Additionally, during validation we should test scenarios with common-mode errors to ensure NIS
metric indeed flags them (it should, because the innovation will have a mean offset). The agents
might then evolve to inflate the measurement noise model to treat the bias as noise – not ideal for
accuracy, but it keeps NIS in check. This is an area to watch during real testing.
• Sensor Noise Spikes and Outliers: Real sensors occasionally produce wild readings (outliers). A
single large residual could spike the NIS and temporarily worsen fitness, possibly triggering a wrong
adaptation if misinterpreted. We handle this by using averaged metrics and gating:
• The Kalman filter already likely uses an innovation gating threshold (e.g. a chi-square 99% cutoff) to
reject obviously bad measurements. If a reading produces NIS above a threshold, the filter can
discard it as an outlier, preventing it from skewing the state and also from affecting the NIS average
too much. (Even if we count a rejected measurement’s NIS, we know such events should be rare if
things are working. Frequent gating means something else is wrong with the model, which evolution
should address by increasing noise assumptions or similar.)
• The fitness computation uses a windowed average NIS, so one spike among many nominal readings
will only slightly raise the average. We could also use a robust measure (like median NIS or trim the
top 5%) to avoid single spikes dominating. Thus, the evolutionary loop won’t overreact to one-off
noise. Only sustained changes in NIS will drive parameter evolution.
• If a particular sensor is extremely noisy (e.g. hardware issue causing many spikes), that will reflect in
consistently high NIS and the agent should respond by, say, putting less weight on that sensor or
increasing its noise parameter.
• Communication Dropouts / Peer Absence: If an agent temporarily has no communication (e.g. out
of range or radio failure), the peer agreement term becomes irrelevant. We design the fitness such
that in absence of peer data, JPA can be treated as 0 (or simply not counted). The agent then relies
on NIS alone during that period. We should ensure that lack of peers doesn’t inadvertently penalize
the agent – it’s not the agent’s fault if it’s isolated. In fact, in isolation the best it can do is keep its
sensor consistency optimal. So the fitness function effectively degrades to just NIS + bandwidth
(bandwidth would be low due to no comms). This is fine; the agent will tune itself for solo operation
in that time. Once peers reappear, the peer term naturally comes back into play. If an agent is
permanently alone, the evolutionary process simply optimizes NIS and minimal bandwidth (which
9likely means it’ll stop sending useless messages if nobody is there to hear). This adaptive silence is
acceptable.
• A related case is delayed or infrequent peer updates: if neighbors send data slowly or irregularly,
an agent might see stale neighbor states causing apparent differences. The design should account
for timestamps – compare states in a time-consistent way (e.g. if neighbor’s data is 5s old, perhaps
don’t penalize difference too strongly as it may be due to propagation delay). Possibly the
agreement metric can include a time decay or only consider relatively fresh data. We will incorporate
such logic to avoid penalizing agents for lags that are beyond their control.
• Evolution Stability and Oscillation: On-line evolution could, if not managed, cause oscillations
where an agent keeps toggling between two parameter sets. For example, one parameter might
yield slightly better NIS but worse peer agreement, and another yields vice versa, resulting in near-
equal composite fitness with noise. To prevent thrashing, we can introduce hysteresis or
smoothing in the selection:
• Require a statistically significant improvement (beyond some epsilon or noise margin) before
accepting a new parameter set. This could be based on confidence intervals of the measured
metrics.
• Alternatively, use a fitness memory or multi-armed bandit approach: occasionally test past
configurations to ensure the current one is still the best in the current environment.
• We might also allow the algorithm to adjust multiple parameters simultaneously in one mutation
once it’s near an optimum (to explore the joint effect).
• Another safeguard is to periodically re-introduce small random perturbations even after converging,
to ensure robustness against slow environmental drift (a form of continuous adaptation). This is
already inherent in the loop as described, but we might taper mutation magnitude over time to get
finer adjustments as it converges.
• System Safety: We must ensure that during a mutation trial, the system doesn’t violate safety or
mission constraints. For instance, a mutated parameter could conceivably make the agent filter
diverge or drop tracks. We will put sanity limits on parameters (e.g. don’t set gossip interval to
extremely high so that tracking fails, or process noise to zero which could break covariance, etc.). If a
mutation causes a severe immediate degradation (like NIS skyrockets meaning the filter is going
unstable), the agent can terminate that trial early and revert. In essence, we implement a monitor
for catastrophic performance (maybe threshold on NIS or on track count drop). This way, the
evolutionary process won’t drive the system into unsafe territory. It’s similar to how simulation had
pass criteria thresholds 16 ; on hardware we can enforce certain baseline criteria (e.g. “must
maintain track of target” or “NIS must not exceed X for more than Y seconds”) and abort bad
mutations.
By accounting for these edge cases, the blind fitness system is designed to be resilient. Many of these
strategies (reputation weighting, gating, etc.) have already been proven in simulation tests (Byzantine
attacks, high noise, network partitions, etc. were handled in DST scenarios 17 18 ). We will carry over those
lessons. The key new challenge is ensuring the evolutionary adaptation remains stable and beneficial in a
non-stationary real world – which the above considerations address by smoothing, safeguarding, and using
robust statistics.
10Validation and Benchmarking Plan
To gain confidence that the blind fitness function performs comparably to the simulation-based approach,
we will undertake a series of validation steps:
• Simulated A/B Comparison: First, test the new fitness function in the simulation environment,
treating it as if the agents were blind. We can run the same scenarios from Deterministic Simulation
Testing (DST) but modify evolution.rs to use Jfitness instead of the oracle error. The oracle
(ground truth) will not be fed into the evolution; however, since it’s a sim, we can still record the
actual RMS errors for evaluation. We will verify that the evolutionary process driven by blind metrics
still yields good outcomes. For example, in the ResourceStarvation (DST-015) scenario with a
bandwidth cap, check that agents using blind fitness indeed evolve a larger gossip interval (as they
did before) and that the final RMS error achieved is low (within ~0.85 m as before 14 ). Similarly, in a
bad actor scenario (DST-014), verify that blind-fitness-driven agents isolate bad actors and maintain
accuracy (we’d expect to see the reputation weights drop for bad actors and the honest agents’ peer
agreement mostly with each other, preserving tracking accuracy ~0.8 m 14 ). Essentially, we ensure
that for each major scenario tested with oracle fitness, the blind fitness yields comparable
performance metrics. If there’s a divergence (e.g. maybe blind fitness agents end up with slightly
higher error because they over-prioritize bandwidth), we can adjust the weighting or formulation.
• Weight Tuning in Simulation: Using the simulation as a sandbox, we will systematically vary the
weights wnis , wpeer , wbw to see how the outcomes change. For instance, run a suite of simulations
where we emphasize NIS vs. peer differently, and identify which setting best matches the optimal
ground-truth results. We might find, for example, that too high wbw causes agents to starve comm
and then their truth RMS goes up; too low wbw and they spam unnecessarily. Through such
experiments, pick a balanced set of weights that yields near-optimal ground-truth performance in a
variety of conditions. These will be our default for real deployments. (We can still allow evolution of
behavior beyond that, but the weights guide the trade-offs.)
• Live Hardware Functional Tests: Deploy a small number of agents (e.g. 3-5) in a controlled
environment with known ground truth references (for testing purposes). For example, set up a
scenario where all agents track a moving target with known trajectory (or use motion capture for
ground truth). Let the on-board blind evolution run. Over time, record the agents’ internal metrics
and also the actual error to truth. We expect to see: internal metrics like NIS trending down as they
tune filters, peer agreement improving (states converging), and ideally the actual error to truth also
decreasing or staying low. We will verify that no agent diverges or fails. This real-world dry run helps
catch any unmodeled issues (e.g. sensor bias not handled, or too slow adaptation). It also lets us
ensure that the blind metrics correlate well with actual performance. If we find cases where an
agent’s internal metrics look “good” but its real error is bad, that indicates a gap in our fitness
formulation (and we’d investigate why – possibly the groupthink bias scenario or a mis-weighting).
• Benchmark against Simulation Baseline: For more quantitative comparison, we can use the
simulation to generate baseline “optimal” parameter sets (by using the oracle fitness or by manually
tuning). Then on hardware, measure how close the evolved parameters get to those baselines. For
example, if in sim the best gossip interval was 7.1 ticks for a given scenario, do the real agents evolve
towards ~7 ticks as well under similar conditions? If yes, that’s a good sign the blind fitness is
effective. Similarly compare filter noise settings, etc.
11• Stress Testing Edge Cases: Intentionally create some edge conditions in a test:
• Introduce a bad actor node in a real test (or simulate one in the loop) to see if the reputation system
and peer metric respond correctly (bad actor’s weight goes to 0, honest agents ignore it, maintain
consensus among themselves).
• Increase sensor noise artificially (e.g. add noise to one agent’s sensor) and see if that agent evolves a
higher measurement noise parameter (which would show as its NIS dropping back to normal range
after adaptation).
• Limit bandwidth drastically (simulate poor network) and confirm agents back off their
communication rates and still keep reasonable peer agreement via evolution.
• Time-varying conditions: perhaps start with one set of conditions (low noise, plenty bandwidth) then
mid-test change conditions (increase noise or drop bandwidth) and observe if agents adapt further
(they should, e.g. if noise increases, NIS spikes, then they retune).
• Performance Benchmarks: Define success criteria similar to DST but for blind adaptation:
• Accuracy: Final RMS error within X% of simulation oracle-based evolution’s result.
• Convergence Time: How long it takes for the on-line evolution to reach a stable parameter set after
deployment (we want this reasonably fast, e.g. within a few minutes or a few dozen iterations).
• Stability: Once converged, the variance in metrics (no oscillation, minimal further change if
environment static).
• Bad Actor Detection Rate: Should remain high (as in simulation, near 100% isolation of bad nodes
9 ).
• Bandwidth Usage: Meets the real constraints (e.g. if limit 1kB/s, verify agents stay at or below that
after adaptation, as intended).
We will utilize logging and telemetry from the agents to gather all these metrics during field tests. If any of
the benchmarks are not met, that indicates either a need to adjust the fitness weights or perhaps add
another component. For instance, if we find that track count consistency (all agents seeing the same
number of targets) is not adequate, we might explicitly include a term for that or ensure the peer
disagreement covers it (e.g. treating missing track as large error).
• Iterative Refinement: After initial deployment tests, we’ll iterate on the fitness formulation if
needed. Perhaps the linear combination is sufficient, or we might discover non-linear interactions
requiring a more complex function. For now, we hypothesize the linear weighted sum will work,
given our ability to tune weights.
• Fallback Testing: It’s wise to test that if the evolutionary system is turned off (static parameters), the
system performs at least reasonably. The blind fitness-driven adaptation should then demonstrably
improve upon that baseline. This ensures that even in worst case (if our blind metrics failed to guide
correctly), the system isn’t worse than a fixed well-chosen config. In simulation we have such
baseline configs; on hardware we can choose the sim-derived optimum as a fixed config and
compare.
By following this validation plan, we will ensure the blind fitness function is not just theoretically sound,
but practically effective. We will have demonstrated that optimizing NIS + Peer + Bandwidth leads to the
same kind of real-world performance (tracking accuracy, reliability) that optimizing directly for error did in
12the simulation. Only after this validation will we deploy the blind evolutionary system broadly (e.g. enabling
it by default on all nodes for live missions).
Implementation Notes and Next Steps
To implement the above in the codebase, we outline the tasks and design considerations for the
engineering team:
• Refactor Fitness Computation in evolution.rs : Abstract the fitness evaluation so that it can
use either the simulation oracle (when running in SimContext) or the new blind metrics (when
running in TokioContext on real hardware). This likely means introducing hooks for the agent’s own
data. For example, in production mode, GodViewAgent can provide a method to retrieve its recent
NIS statistics, peer disagreement, etc. The evolution module can call this instead of oracle functions.
Ensure thread-safety or synchronization since these metrics might be updated in real-time by other
parts of the system (e.g. Kalman updates computing NIS).
• Kalman Filter Instrumentation: Modify/extend the Time Engine ( godview_time.rs ) or
wherever the Kalman filter update occurs to compute the innovation residual and covariance for
each measurement. Compute ν T S −1 ν per update and possibly maintain a running average. This
could be as simple as accumulating a sum of NIS and count of updates, or using an exponential
moving average to weigh recent more. We might also log how many times NIS exceeds certain
thresholds (for gating stats). This data should be accessible to the fitness function.
• AdaptiveState
Integration:
In
the
Trust/Neighbor
system
( godview_trust.rs
or
adaptive.rs ), ensure each agent can compute its reputation-weighted disagreement. Likely, the
tracking engine or CRDT engine ( godview_tracking.rs ) that fuses neighbor data can measure
differences. We may implement a routine that, for each track in the agent’s local store, compares
with neighbor versions of that track (if neighbors gossip track state, the agent can see if positions
differ). Summing those differences weighted by the trust score (which is maintained in
AdaptiveState) will yield JPA . This might require maintaining a cache of last received neighbor states
for each entity and the neighbor’s rep. We should be careful to compute this consistently (e.g. at a
certain tick or over a small interval). We can reuse the simulation’s agreement metrics (like track
count CV was computed from all agents – here each agent can approximate something similar by
looking at how many tracks it has vs. how many its neighbors report, which contributes to
disagreement if there’s mismatch).
• Bandwidth Monitoring: Hook into the NetworkTransport (e.g. the Zenoh/UDP layer or gossip
protocol implementation) to count outgoing bytes/messages. Many systems have this
instrumentation for debugging or QoS; if not, we add a simple counter for each agent for packets
sent. Possibly we can piggyback on existing stats (simulation had total messages, etc.). We will likely
need a sliding window count (last N seconds) to use in fitness. A small buffer or circular queue of
timestamped send events could work, or just a decaying average. This should be light-weight.
• Parameter Mutation Mechanism: Implement functions to mutate each tunable parameter. For
numerical parameters (intervals, noise values), a random delta (e.g. 5-10% change) can be applied.
For discrete choices (like neighbor count or which neighbor to drop/add), define a mutation like “try
13adding one neighbor” or “remove one neighbor link randomly.” These should be encoded such that
the evolutionary loop can iterate through them. Perhaps represent the set of parameters as a vector
or struct, and have a mutation operator that picks one field at random to tweak. This logic might
already exist in evolution.rs for simulation (if it used GA, it might have crossover/mutation
defined). We will reuse what we can.
• Running the Loop: Determine how to schedule the evolutionary loop on hardware. Possibly use a
Tokio task that wakes up at a fixed interval to initiate a trial. We must balance adaptation speed with
not interfering with real-time operation. A reasonable approach is to run trials during operation, as
we planned, but perhaps ensure that during a trial, the agent’s performance is still acceptable. If
needed, we could reduce normal functionality slightly during a trial (though ideally not). We should
also log trials and outcomes for offline analysis (to see how often it accepts vs rejects, etc., during
testing).
• Diagnostics and Telemetry: Develop tools to visualize the evolution in real-time. For example, we
can log the fitness components continuously and maybe send them to a ground station. This would
allow engineers to see “Agent 3’s NIS just dropped after it adjusted process noise” or “Agent 5 is not
improving – maybe stuck.” Such insight will be valuable during initial deployment tuning.
• Gradual Rollout: Initially, we can run the system in a “fitness monitor” mode: calculate the blind
fitness metrics but do not actuate any mutations, just to see what the baseline metrics are with a
known good parameter set. This can be step 1 in field testing. Step 2, allow mutations but perhaps
with narrow bounds (tiny adjustments) to ensure nothing crazy. Step 3, fully enable the evolutionary
adaptation once confidence is gained.
• Documentation & Training: Since this is an internal spec for research engineers, we’ll also
document the math and logic clearly in the code comments (especially the rationale for chosen
weights or formulas). Future developers (e.g. Google Antigravity team members) should be able to
understand why we combine NIS, peer, and bandwidth in this manner.
• Future Extensions: Note that while this spec focuses on transitioning the fitness function, the same
framework could be extended. For instance, if later we introduce a new sensor or a new objective
(like power consumption), we could add another term to the composite fitness (with appropriate
weight). The system is modular enough to handle multi-objective optimization by blending into one
score.
By following this plan, the team will implement a robust blind evolutionary learning system. Each agent
will effectively “close the loop” on itself using internal feedback, achieving near-simulation-level
performance in reality. This internal specification serves as a roadmap to make that transition smooth,
ensuring that our adaptive swarm remains just as accurate, cooperative, and efficient without the
omniscient simulator oracle.
1
README.md
file://file_00000000cb2471f79f2a2066f216f15c
142
9
10
11
12
14
16
17
18
DST.md
file://file_000000007f8c71f5b133371fdc7591ba
3
4
Normalized Innovation Squared (NIS) | Kalman filter for embedded systems
https://kalman-filter.com/normalized-innovation-squared/
5
6
7
Normalized Estimation Error Squared (NEES) | Kalman filter for embedded systems
https://kalman-filter.com/normalized-estimation-error-squared/
8
How to verify Kalman filter performance without true data - Cross Validated
https://stats.stackexchange.com/questions/502427/how-to-verify-kalman-filter-performance-without-true-data
13
15
SIM_TO_REAL.md
file://file_00000000170c71f590a43544cbbf9d9a
15
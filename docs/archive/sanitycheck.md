Sanity Check for Project GodView v3: Distributed
Perception Architecture
Overview of the Proposed Architecture
Elevator Pitch Recap: Project GodView v3 aims to create an “HTTP of Reality” – a decentralized protocol for
cooperative perception among autonomous agents. Instead of sharing raw sensor feeds (which is
bandwidth-prohibitive), vehicles and drones exchange lightweight semantic 3D information (object positions,
velocities, etc.) over the Zenoh pub/sub network. This promises to dramatically reduce bandwidth (by ~99%,
from video streams to kilobyte-level messages) while running on consumer-grade hardware (e.g. a GTX
1050 Ti GPU). In essence, each agent contributes to a real-time shared world model by publishing
GlobalHazardPacket messages containing high-level state (object ID, position, velocity, class, etc.),
enabling others to “see” beyond their line of sight.
Key Claimed Benefits: By sharing structured object-level data instead of raw images or point clouds, the
system slashes network load and latency. For example, 50 agents would transmit on the order of 1.5 MB/s
total, making cooperative perception feasible even on 4G/5G networks. This approach aligns with the
broader trend in collaborative perception toward transmitting only minimal essential data needed for
safety-critical awareness, since “collaborative perception systems must finish a perception cycle within a
hard deadline (e.g., 100 ms)… therefore, only sharing minimal, essential data is advisable to save
bandwidth” 1 .
However, building such a distributed perception network faces several fundamental challenges (“impossible
physics”), which the proposal addresses via three specific fixes:
• Flaw 1: The Time Travel Problem (Latency & Out-of-Sequence Measurements) – Network latency
and jitter (100 ms+ on cellular links) mean that by the time one vehicle’s observation reaches another,
it may be stale or out-of-order relative to that vehicle’s own timeline. This can cause “ghost” objects
jumping backward or forward in time if not handled correctly.
• Proposed Fix: Augmented State EKF (AS-EKF) – Each agent runs an Extended Kalman Filter that
maintains a fixed-size history buffer of past states (e.g. the last N=20 state estimates at 30 Hz,
~600 ms). If a delayed observation arrives out-of-sequence, the filter can “rewind” to update the
earlier state and then re-propagate the corrections to the present. In effect, this is a form of fixed-lag
smoothing to gracefully absorb delayed measurements rather than naïvely discarding or applying
them late.
• Flaw 2: The Pancake World Problem (Verticality) – Using a 2D geospatial index (like a plain
geohash or an Uber H3 cell) for partitioning data collapses the altitude dimension. A drone at 100 m
altitude above a car at ground level would naively share the same 2D cell, causing a false “collision”
in the data index.
• Proposed Fix: Hierarchical Hybrid Sharding (H3 + Octree) – Partition the environment first by an
H3 hexagonal cell (to localize data in latitude/longitude), and then within each cell maintain a sparse
voxel octree to index altitude. This two-tier spatial index prevents mixing ground-level objects with
1aerial objects in the same bucket. Essentially, an H3 cell (~res 10 chosen for urban granularity) is
subdivided in the vertical dimension so that, for example, drones at 300 ft and cars at 0 ft appear in
distinct octree nodes even if their lat/long coincide.
• Flaw 3: The Sybil Attack (Trust and Security) – In an open decentralized system, a malicious actor
could inject fake “phantom hazards” (e.g. a bogus report of a tank on the freeway) to disrupt
vehicles. Traditional V2X networks face similar issues of authentication and data trust.
• Proposed Fix: Capability-Based Access Control (CapBAC) with Signed Tokens – Every data
publisher must possess a cryptographic token (based on Biscuit, a Datalog-enabled capability token)
issued by a trusted authority. The Biscuit token encodes fine-grained permissions (e.g. which H3 cell/
region the agent is allowed to write to). Each GlobalHazardPacket is signed with Ed25519, and
peers verify the signature and the token’s validity (offline, using the root public key) before accepting
the data. This means only authorized agents (e.g. city-approved vehicles/drones) can inject
perception data, and even those are constrained to their geographic scope (“allow write if resource ==
nyc/sector_7” in the token, for instance). There is no reliance on a central server at runtime – trust is
decentralized via public-key crypto.
Execution Plan (Next Step): The project will move from simplistic webcam demos to a more complex
simulation environment using CARLA (an open-source autonomous driving simulator). The immediate goal
is to demonstrate “seeing around corners” – e.g. one vehicle detecting a pedestrian occluded from another
vehicle’s view, and sharing that information in time for the second vehicle to react. The CARLA integration
will involve running the simulator in low-quality, off-screen mode on a modest GPU (GTX 1050 Ti with 4 GB
VRAM), extracting ground-truth positions of agents in real time via a Python API, and publishing these as
GlobalHazardPacket feeds into the Rust-based core system. This setup will test multi-agent data
exchange and the AS-EKF fusion in a controlled, repeatable setting before attempting real-world trials.
1. Architecture Roast: Potential Breaking Points
Overall, the design tackles known pain points in collaborative autonomous perception, but we need to
examine if each solution is practical and sufficient. Let’s scrutinize each aspect and identify where things
might “break” or underperform in a real deployment:
1.1 Latency and the Augmented State EKF (Time-Travel Fix)
Maintaining a historical state buffer to handle out-of-sequence measurements is a sound approach
grounded in estimation theory. In fact, this approach is essentially a form of fixed-lag smoothing using a
Kalman Filter. Literature on multi-sensor fusion acknowledges that one of the “easiest and simplest” ways to
incorporate delayed or out-of-sequence measurements (OOSMs) is to augment the state with older copies
and perform a retroactive update 2 3 . By treating some past states as part of the current state vector, a
delayed measurement can update the corresponding component, and the filter’s covariance then
propagates that correction forward to the present estimate. This avoids the alternative of complex
backtracking or discarding stale data.
Viability on GTX 1050 Ti: An EKF with (for example) 20 historical states, each containing position and
velocity (say 6 variables per state if [x,y,z, vx,vy,vz]), results in a ~120-dimensional state vector for one
tracked object. The covariance matrix would be 120×120. Performing an EKF update in that space is not
trivial, but it’s within the capabilities of a modern CPU – likely on the order of a few hundred microseconds
2or a few milliseconds per update in optimized C++. Even with dozens of tracked objects, this is manageable
on a mid-range CPU core. In other words, the AS-EKF math itself is not too heavy for a 1050 Ti system – in
fact the filter will run on CPU (EKFs don’t typically require GPU acceleration). The GPU would be more taxed
by other tasks (like CARLA’s rendering or any neural nets, if used).
However, there are a few subtle challenges: - State Management Complexity: Augmenting state for each
object and handling variable lags means the filter code becomes quite complex. Ensuring numerical
stability of a large EKF is tricky – covariance matrices can become ill-conditioned, and tuning process/
measurement noise for a multi-lag state might require careful calibration. If the implementation is naïve, it
could break down with frequent updates (e.g. every incoming OOSM triggers a flurry of covariance
propagation). A well-known issue with EKFs is their sensitivity to tuning; an AS-EKF might be even harder to
tune since it must remain stable over retroactive corrections. - Data Association & Duplicate Tracking: The
architecture assumes each incoming measurement can be associated to the correct object track (via
entity_id or by spatial proximity). In practice, if two vehicles detect the same pedestrian independently,
will they assign the same UUID? If not, your system might treat them as separate hazards initially.
Duplication or mis-association could “break” the global view – two ghost entries for one object, or worse, a
failure to fuse data from multiple observers. The proposal’s use of a UUIDv4 persistent ID hints at trying to
have global IDs, but generating and reconciling those in a decentralized way is non-trivial. A common
approach is to use gating and track fusion algorithms to merge observations that appear to correspond to
the same object. This data association problem is a big potential failure point: without a robust scheme,
the GodView could either double-count hazards or drop a critical update because it wasn’t recognized as
referring to an existing object. - Out-of-Sequence Window Limits: The design sets a 20-frame (~600 ms) lag
buffer, discarding data older than that. While 500 ms is a generous tolerance for typical V2V communication,
there will be corner cases (e.g. momentary network outages or highly variable 4G delays) where a packet
arrives just outside the window. In those cases the data is thrown away. If that data was the only
observation of a hazard (say a pedestrian that is no longer visible later), the system might miss it entirely.
There’s a trade-off here: a longer buffer catches more late data but further bloats state and increases CPU
cost. The chosen 600 ms seems reasonable, but it does impose a hard latency ceiling – beyond ~0.5 s
delay, information is lost. Real cellular networks under load can occasionally experience >500 ms latencies
or jitter spikes 4 5 . This means in extremely bad network conditions, the system’s effectiveness could
sharply degrade (though arguably, driving in such conditions is risky anyway). - Dynamic Objects and
Prediction Errors: Even with velocity transmitted and a predictive filter, large delays mean extrapolating
object motion over a significant fraction of a second. A lot can happen in 0.5 s – a pedestrian could start or
stop running, a car could brake, etc. The EKF assumes a motion model to predict forward, but if the target
maneuvers unexpectedly, a late update might correct a large error. The filter will smooth it out rather than
causing a jump (which is good), but the inherent lag means the information content of a 500 ms-old
measurement is low. In fast urban traffic scenarios, 500 ms old data is nearly outdated. The design claims to
“successfully fuse” at 500 ms, which is fine in a mathematical sense, but one must be cautious about safety
implications – e.g., a car moving at 15 m/s travels 7.5 m in 0.5 s. If you only find out about an obstacle with
that much delay, you might not avoid it in time. So, while the filter prevents instability (no jittery ghost cars),
the real requirement is to keep latency as low as possible, in practice well under 200 ms for hazard
avoidance. It will be important to measure actual end-to-end delays in the system and possibly optimize the
network or message prioritization to keep them low.
In summary, the AS-EKF approach is technically sound – it’s analogous to known techniques for handling
delayed measurements 2 and fixed-lag smoothing. It shouldn’t break the system from a computational
perspective on a GTX 1050 Ti. The risk lies more in the integration details: data association must be rock-
3solid, and the system must still strive to minimize latency (through network optimizations or sending
predictive state) because simply being able to handle 500 ms delays doesn’t mean it’s safe to routinely have
500 ms delays. As long as these factors are managed, the time-travel fix is a strong point of the design
rather than a weakness.
1.2 Spatial Sharding with H3 and Sparse Voxel Octree (Verticality Fix)
Using H3 hexagonal cells for spatial partitioning is a clever choice – H3 is a well-tested global discrete grid
system (originally from Uber) that provides hierarchical spatial indexing. By choosing a resolution (Res 10 in
this case), you define the horizontal granularity of data clustering. At H3 Res 10, each hexagon is on the
order of ~0.015 km² (roughly a hexagon of ~75 m edge length, covering ~15000 m² on average) 6 7 . This
is roughly the size of a city block or smaller, which seems appropriate for urban scenarios. Two vehicles or
objects within the same Res 10 cell are reasonably near each other (on the order of <100 m apart), so
grouping data by these cells makes sense – subscribers can pull information from local cells of interest
rather than the entire city.
“Is H3 res 10 too fine?” – Not necessarily; it is a trade-off: - If the cell size is too coarse (lower resolution like
Res 7 or 8), each cell covers a very large area (hundreds of meters across). You’d have fewer topics to
subscribe to, but each would contain many objects, including some that are far irrelevant. It could also
reintroduce the vertical mixing problem (e.g. a tall building and a street underneath might still share a cell if
the cell is huge). - If the cell size is too fine (Res 12–15), you get very small cells (a few meters across at the
extreme). That might over-shard the data – vehicles would have to subscribe to dozens of neighboring cells
to cover their perception range, and maintaining those subscriptions (and the overhead of many small
topics) could be inefficient. There’s a sweet spot where each cell is about the radius of a single agent’s
immediate hazard zone. Res 10 (hex ~150 m across) is a reasonable heuristic for city driving: an
autonomous car typically doesn’t need to ingest every object beyond 100–150 m (especially if occluded), and
if it does, subscribing to one layer out (the neighboring hexes) would catch those.
So Res 10 seems like a practical choice to start with. We might discover through testing that adjusting to Res
9 or 11 optimizes performance, but any fine-tuning can be done later. The architecture could allow dynamic
resolution or multi-resolution if needed (H3’s hierarchy makes it possible to adapt).
Now, the vertical dimension: Pure H3 indexing has no concept of altitude – it’s a 2D grid on the Earth’s
surface. The proposal’s solution of an octree inside each hex cell is sound from a data-structuring
perspective. Essentially, for each hex cell (which corresponds to a column of airspace above a patch of
ground), we maintain a sparse 3D tree that subdivides altitude. This is like having “buckets” for 0–10 m, 10–
20 m, … or more adaptive if using an octree that refines where needed. It ensures that a drone flying at
100 m and a car on the ground, despite sharing an (x,y) location, will end up in different leaf nodes of the
octree. So queries for “what’s in my vicinity” can be constrained not just by horizontal distance but also by
altitude band, preventing false collision merges.
Potential breaking points or complexities: - Octree granularity and complexity: How is the octree
structured exactly? If it’s truly sparse and only populated where objects are, this is fine. But consider
dynamic objects: as a drone moves up or down, does it get reindexed into different octree nodes in real-
time? Likely yes. The overhead of updating the spatial index might be non-trivial if there are lots of moving
objects. However, since this is distributed (each agent might maintain its own local map), it’s not like a
central server managing the entire octree – each subscriber can maintain a lightweight structure of what it
4knows in each cell. This should scale, but it’s an area to watch for performance issues. - Subscription
model: A vehicle on the ground probably only cares about hazards also near ground level (e.g., other
vehicles, pedestrians). It might safely ignore drones at 300 ft. Conversely, a low-flying drone cares about
other drones and maybe tall infrastructure. The octree allows filtering that out. But the application logic
must ensure, for example, that a ground vehicle doesn’t get spurious “nearby object” alerts for a drone
overhead. The design appears to handle this by design, which is good. It just means each agent likely
subscribes not just to an H3 cell but to layers within that cell (or subscribes to all and then filters by altitude).
If using Zenoh, the topics/keys could encode the cell ID and perhaps a layer or altitude range. - Coordinate
precision and alignment: To use H3 effectively, all agents must share the same coordinate frame and high-
precision localization. H3 is geodetic (lat/long); a small error in GPS (say 5 m) won’t usually throw you into a
different H3 cell at res 10 (cells are ~100+ m wide), so that’s forgiving. But altitude errors could cause some
confusion in octree indexing (less likely since altitude differences of tens of meters are usually distinct). As
long as each vehicle knows its own position reasonably well (likely via GPS+IMU or localization to a map),
the partitioning will be consistent. If a vehicle’s self-position was off by a large margin, all its reported
hazard locations would be offset and possibly fall into wrong cells. This could “break” data consistency. For
critical infrastructure, you might want RTK GPS or high-accuracy localization on each participant to avoid
that problem. The design doesn’t explicitly mention this, but it’s implicitly needed for any collaborative
perception to work (since all data is tagged with global coordinates). - Edge cases: How to handle objects
that span altitude layers (e.g. a crane or a building)? Typically those wouldn’t be “hazards” broadcast –
mostly we’re talking point-like dynamic objects. So likely not an issue. Another edge case is if an object is
exactly at cell boundary (either horizontal or vertical) – you might get slight oscillation between cells if its
position estimate noise crosses a boundary. This could cause thrashing (e.g., an object appears in cell A,
then next update in cell B). With a hex grid ~75 m edges, a small noise won’t typically cause cell flips unless
near the edge. Even if so, the EKF smoothing on positions might alleviate jitter on that scale. But it’s
something to test (maybe hysteresis or overlap subscriptions to neighbor cells is needed to not lose track
when an object crosses a cell boundary). - Alternative Approaches: Just to sanity-check, are there simpler
ways to account for vertical separation? One could simply include altitude tags in messages and have
subscribers ignore mismatched altitudes. But that doesn’t solve collisions in data storage – if you used a
plain key like “cellID = X” to index, a drone and car share that key. The octree fix is essentially implementing
a 3D index (like converting lat,lon,alt -> (H3_cell, octree_node)). This is reasonable. Another approach might
have been to use a different spatial index that’s inherently 3D (some projects use a 3D grid or a “Layered
H3”). I haven’t seen a standard “H4” for 3D, so this custom solution is fine. It just adds a bit of complexity in
implementation.
All things considered, the Hierarchical sharding by H3 + SVO is a sound solution to the “pancake world”
issue. It will not likely break in theory. The main caution is ensuring the resolution and octree parameters
are tuned to the use case: - If Res 10 proves too fine (too many cells to subscribe), you might move to Res 9
(cells ~3× larger linear dimension). But that risks more objects per cell and more vertical mixing – not ideal
for dense urban mixed traffic. I suspect Res 10 is chosen wisely for that scenario. - Ensure that each agent
subscribes to all relevant cells (probably the one it’s in and adjacent ones). Missing a cell could mean
missing a hazard around a corner. This is more of an application detail than a flaw in the concept.
In summary, the spatial partitioning fix is robust in concept. It’s unlikely to be the breaking point unless
implementation is mishandled. If anything, the question will be performance tuning: how many cells can an
agent subscribe to? Zenoh might handle many subscriptions efficiently (it’s built for scalable data
distribution). We might need to optimize how often data is published per cell to avoid flooding subscribers
(more on this in redundancy discussion later). But no fundamental roadblocks here.
51.3 Security and Trust: CapBAC with Biscuit Tokens (Sybil Attack Fix)
Security is often the most overlooked aspect in such systems, so it’s refreshing that the architecture
addresses it head-on. The proposed use of Capability-Based Access Control tokens (Biscuit) and Ed25519
signatures is quite advanced and granular. Essentially, instead of blindly trusting any V2V message or using
simple PKI identity certs, this system requires each hazard publisher to present cryptographic proof of
authority and context (where they are allowed to publish). This design is akin to having a federated root of
trust: e.g., the city transportation authority issues a root token to a fleet operator or regional manager,
which delegates a scoped token to each vehicle/drone. All other participants can verify the entire chain
offline. This yields some clear benefits: - Only authorized entities can inject data, mitigating random
spoofers. This addresses the Sybil attack where an attacker might create many fake “vehicles” broadcasting
false info. Without a valid token signed by the root, those messages are ignored. - It localizes trust: a token
might say this vehicle is trusted for region X. If that vehicle somehow tried to claim a hazard in region Y
where it’s not authorized, verification fails. This is a clever way to enforce that, for example, a compromised
drone in San Francisco Sector 7 can’t suddenly spawn fake hazards in Sector 1 where it isn’t present. - The
use of Biscuit (with its Datalog-based caveats) means policies can be quite expressive (time-bound, region-
bound, type-bound, etc.). And unlike simple JWTs, Biscuit tokens support offline attenuation (delegation) –
exactly what is needed for distributed systems where connectivity to a central authority isn’t guaranteed.
What could break or be challenging: - Token Overhead & Latency: Each GlobalHazardPacket
presumably carries a signature and perhaps the token or a reference to it. If the token is small (Biscuit
tokens can be a few hundred bytes depending on the number of caveats/blocks), this is not huge – likely on
par with the message size. But it is overhead. Signing each message with Ed25519 is fast (signing or
verifying ~1000s of messages per second on typical hardware), so not a bottleneck. The overall bandwidth
budget (1.5 MB/s for 50 agents) must account for these cryptographic bytes too. It should be fine, but in
extreme densities, this overhead might add up. In comparison, standard ETSI V2X messages also carry
signatures and often certificate attachments, which are on the order of 200–300 bytes per message, so this
is in the same ballpark. - Bootstrapping and Revocation: The architecture assumes a shared root of trust
(e.g., “City of SF”). How are tokens distributed initially? Likely out-of-band (when a vehicle joins the network).
More importantly, revocation is tricky in offline systems. If a token or key is compromised, how do others
learn to distrust it? With short-lived tokens or having expiration times, you can limit risk, but real-world
deployments usually need a mechanism to revoke credentials (like a certificate revocation list or update
broadcast). Biscuit might allow embedding revocation checks or versioning, but offline verification
inherently means you aren’t consulting a central list each time. A pragmatic approach is periodic refresh of
tokens or requiring vehicles to update credentials regularly. This isn’t a show-stopper, just an operational
consideration. - Insider Threats: CapBAC stops unauthorized outsiders, but what if an authorized node
misbehaves (malfunctioning sensor or a hacked vehicle that does hold a valid token)? It could still inject
misleading data that passes cryptographic checks. This is where higher-level logic is needed: e.g., if one
vehicle reports a “tank on freeway” but none of the other 10 vehicles around see it, the system should flag
or down-weight that outlier. The current design doesn’t detail any data-level credibility filtering aside from
trust tokens. In mission-critical systems, you might incorporate plausibility checks or require consensus
(maybe not for every hazard, but certainly for unusual events). This borders on “Byzantine resilience” –
detecting and ignoring faulty or malicious contributors even if they are authenticated. Implementing such
logic could be complex (maybe using Bayesian filters or cross-verification among peers). At minimum, the
EKF fusion could treat extremely divergent measurements as outliers (e.g., gated out if they don’t fit the
predicted state). That would mitigate some single-sensor errors or lies. - Comparison to Industry
Standards: It’s worth noting that current V2X safety systems (DSRC/802.11p and C-V2X) use a PKI with
6certificates and signatures on messages. Those ensure authenticity but not the kind of fine-grained control
your system offers. In fact, the ETSI Collective Perception Message (CPM) standard simply trusts that any
vehicle broadcasting a perceived object is an honest participant with a valid certificate. Your approach adds
an extra layer by constraining “who can say what where.” This is forward-thinking, but it’s also relatively
untested in deployed vehicular networks. One challenge might be integration – if this needed to
interoperate with standard systems, you’d have to either piggyback or translate. But since this is a self-
contained protocol (“HTTP of Reality”), you have freedom to design anew. - Performance of Datalog
policies: Biscuit’s Datalog-based validation means every peer will run a small logic program to verify token
+ caveats for each message. This is usually very fast (microseconds) given the small rule set, but it’s
something to test under load (50 vehicles * X messages/sec). Written in Rust, it’s likely fine.
Overall, the trust architecture is strong and not a typical point of failure. The main risk is if operational
details (like revocation or misbehaving insiders) aren’t addressed – but those are known issues that can be
incrementally solved (e.g., add a revocation list distribution, and an anomaly detection for weird hazard
reports). It elevates the system from “toy” to something that could realistically be deployed in a safety-critical
environment, because trust and security are first-class concerns.
One might argue it’s over-engineered for a prototype, but given the end goal as critical infrastructure, it’s
appropriate. And the idea of decentralized verification of data is echoed by researchers: peer-to-peer
networks and even blockchain-like approaches are suggested to improve trustworthiness in AV
collaborative perception 8 . Your CapBAC+Biscuit solution is effectively a tailored, efficient realization of
that idea (without the heavy overhead of a blockchain, but achieving similar decentralization of trust).
1.4 Additional Challenges and “Unknown Unknowns”
Beyond the three identified flaws and fixes, there are a few other areas where the architecture could face
difficulties:
• Redundancy & Bandwidth Management: If many agents observe the same object, will they all
broadcast it? For example, 5 cars all see a pedestrian – do we get 5 hazard packets for one
pedestrian? The ETSI Collective Perception standards have encountered this issue of duplicate
messages flooding the network. They have defined rules and even AI-based strategies to mitigate
redundant transmissions 9 10 . Your system might need a similar strategy. Perhaps the first
detector of a hazard “owns” it and others either refrain from sending or only send if they have a
significantly better estimate. This could be implemented with some simple rules (e.g., if I hear
someone else broadcasting an object I also see, and its data is fresh and within my error bounds, I
might not transmit to avoid clutter). Without such mitigation, the 99% bandwidth reduction claim
could be jeopardized in dense environments – many vehicles could otherwise multiply-report the
same information. It’s something to keep an eye on as you integrate multiple agents. The good news
is that your capability tokens could potentially be used here too (e.g., designate a leader in an area),
but more practically, algorithms from literature (like dynamic transmit rate based on perceived value
11
10 or machine-learning policies 12 ) could be adapted later.
• Pose and Calibration Errors: Since this system fuses data from different vehicles, any error in their
localization or calibration can introduce inconsistency. If Car A’s positioning has a 1 m bias, and Car
B’s is accurate, a pedestrian might have two slightly different reported positions. The AS-EKF can fuse
those, essentially averaging them out, but a consistent bias might confuse it (it might think the
object is moving or split the difference). Over time, the filter might estimate a correct track, but initial
7mismatches could cause lag in recognizing it’s the same object. In practice, high-quality GPS or a
shared map coordinate frame (e.g., all vehicles localize against HD map landmarks) is assumed. It
might be worth eventually allowing the filter to also estimate an “offset” per peer if biases are
detected (that gets complicated though). This is not a fatal flaw, just an inherent noise issue to
manage.
• Scalability and Network Topology: Zenoh is designed for scalable and efficient data distribution,
blending peer-to-peer with brokered modes. But if we imagine hundreds of agents in a city, how does
Zenoh perform? The protocol can do peer-to-peer in a local area and route messages by interest,
which is good. One risk is if network infrastructure (4G/5G) is patchy, some messages might need to
hop across multiple relays or fall back to opportunistic routing. Zenoh is capable of such “store and
forward” if configured, but then latency can increase. In essence, your system will be as robust as the
networking allows. It’s good that you assume unreliable networks with high jitter – the system is
built to tolerate that. However, keep in mind real cellular networks also have occasional packet loss.
Your EKF will need to handle dropped packets (likely just fine, it will predict and carry on). But large
drop rates might reduce the benefit of collaboration. That’s more of a deployment concern (maybe
eventually integrating with edge computing or RSUs to assist).
• Privacy and Data Ownership: If this becomes city infrastructure, data governance issues arise (as
pointed out in research 8 ). Your use of local tokens under a city root already implies a governance
model – the city (or fleet operator) owns the data sharing in their domain. That’s fine. Just be aware
that privacy laws might consider certain shared sensor data as sensitive (e.g., identifying
pedestrians). However, since you share only abstracted hazard info (and likely ephemeral UUIDs),
privacy risks are minimized relative to sharing raw video. This is actually a selling point of your
approach (no raw camera feed broadcast).
In summary, the architecture is thoughtfully addressing the main challenges. The likely breakpoints are not
in the core concepts, but in implementation and system integration details: - Robust data association to
avoid ghost duplicates. - Effective strategies to reduce redundant messaging in dense scenarios. - Ensuring
the system remains within safe latency bounds in practice. - Handling misbehaving but authenticated
nodes. - Tuning spatial and temporal parameters (H3 resolution, octree depth, EKF noise covariances, etc.).
None of these are fatal flaws; they are engineering challenges that will need iteration and testing. The
architecture provides the framework to solve them (the hardest part is knowing the problems – you’ve done
that). Now it’s about execution and refinement.
2. Validation Approach: CARLA Simulation vs. Real-World Testing
The proposal to pivot into the CARLA simulator for the next phase is well-justified. Here’s why moving to
CARLA is a logical step, and some considerations around it:
Advantages of CARLA Integration: - Controlled Environment: CARLA allows creating complex traffic
scenarios (urban layouts, vehicles, pedestrians, occlusions) in a deterministic way. You can spawn a
situation where one vehicle’s view is occluded and another’s isn’t, exactly the corner-case you want to test
(“seeing around corners”). In the real world, setting up such a scenario and repeating it for testing would be
hard and costly. Simulation lets you iterate quickly and adjust parameters systematically. - Ground Truth
Access: As you noted, you plan to use CARLA’s ground-truth data (exact positions of agents). This is
extremely valuable for testing the best-case performance of your network and fusion algorithms. Essentially,
8you remove perception errors from the equation and first ask: “If vehicles could perfectly detect objects and
share them, does our system fuse them correctly and timely?” This isolates the network and algorithm
aspects. If something fails here, you know it’s the system, not sensor noise. Later, you could introduce
simulated sensor noise or use CARLA camera/LiDAR data with an actual detector to see how perception
errors affect things. But one step at a time is wise. - Resource Constraints: Running CARLA on a GTX 1050
Ti is pushing it, but if you use low-quality rendering and perhaps a headless mode (Off-screen), it can run.
CARLA’s requirements note that a mid-range GPU can run simple scenes at lower resolution. You also might
not need to enable camera sensors at high resolution – since you’re pulling positions directly. That will save
a lot of GPU time (rendering the environment is the heavy part; just computing actor transforms is light). If
needed, you can disable rendering entirely ( -opengl or no-display mode, which uses minimal GPU just
for physics). Physics and simulation might still load the CPU, but a 1050 Ti machine likely has a decent CPU
that can handle a moderate number of actors. - Networking in a Single Machine: A practical trick – you
might run multiple CARLA clients or a single client that controls multiple vehicles and simulate the network
latency by delaying messages. Or run the hazard-sharing logic in separate processes that communicate via
Zenoh even on one PC. This lets you emulate a distributed system without requiring 50 physical cars! CARLA
is often used with such network simulators or even real networking if you run multiple PCs. There’s also
AutoCastSim and other frameworks built on CARLA to simulate V2V comms with delay models 13 . You
could either integrate a simplistic delay (e.g., add 100 ms sleep randomly to messages) or use a more
advanced network model. This approach will validate if your AS-EKF indeed corrects the “ghosting” when
updates arrive late.
Comparing to Real-World Tests: - Repeatability & Safety: Real-world testing of cooperative perception
would require at least two vehicles (or one vehicle and one roadside unit) and careful choreography to
create occluded hazard scenarios. It’s expensive and potentially risky (you don’t actually want to put real
pedestrians in danger to test if the system stops…). Simulation allows testing edge cases (a child runs from
behind a parked truck) safely. Only after proving in simulation that the system works (and identifying any
bugs) should it move to limited field trials. Skipping straight to real-world would almost certainly result in
wasted time chasing issues that are easier debugged in sim. - Sensor and Environment Complexity: In the
real world, you’d have to deal with sensor mounting, calibration, environment distractions, etc. Those will
ultimately need testing, but they can cloud the focus at this stage. CARLA gives you an idealized world (or a
configurable level of noise). You can incrementally add complexity – e.g., use CARLA’s camera + your own
object detector if you want to see the effect of detection latency or false positives on the system. Many
academic papers on collaborative perception use simulation (like CARLA or bespoke simulators) to evaluate
algorithms before attempting any real demo 14 . It’s a standard approach. - Development Speed: Iterating
in software (with CARLA) is faster. Real-world tests have logistical delays (charging vehicles, finding test
locations, GPS reception issues, etc.). Since you also mentioned a constraint (only a 1050 Ti available), you
likely don’t have access to a fleet of real autonomous cars yet. So simulation is the pragmatic path to
demonstrate viability and hopefully attract further resources.
Potential Downsides of Simulation: - Reality Gap: One must always be aware of the “simulation-to-reality
gap.” CARLA’s physics and models can be idealized. Networks in sim won’t exactly capture real 4G behavior
unless explicitly modeled. There’s a risk of overfitting the solution to the sim conditions. For example, CARLA
will give perfectly accurate object positions; in reality, each vehicle’s detected object position has error and
could be biased. Your EKF might behave slightly differently when fed perfect vs noisy data. To mitigate this,
once the system works in sim, consider adding some artificial noise: e.g., perturb the reported positions a
bit, or inject an occasional false positive, to see how the filter and logic handle it. Also test with a variety of
latencies (simulate 50 ms vs 200 ms vs 500 ms delays) to ensure your buffer logic truly holds up. -
9Performance on Single Machine: Running CARLA plus your Rust core plus the Zenoh router and clients on
one machine might be heavy. If the machine starts swapping or maxing out CPU/GPU, the timing of your
system could be affected (e.g., simulated sensor data might lag). Monitor your system load. If needed,
simplify the CARLA scenario (fewer vehicles, no expensive sensors) or run parts of the system on a second
machine (maybe CARLA on one, your Rust core on another, communicating over a real network to better
mimic reality). Even with one GPU, you could run CARLA on CPU-only mode (it’s slow but if you only simulate
at, say, 10 FPS, it might suffice). - Carla Integration Effort: Minor point – writing the Python bridge to
CARLA and getting synchronization right (especially if you want synchronous mode simulation) will take
some work. CARLA’s API can give you transforms each tick; you’ll need to package those into hazard
messages. This is straightforward. Just ensure that your simulation loop doesn’t flood the system faster
than it can handle – if CARLA runs at 30 Hz and you publish 30 messages per object per second, is Zenoh
and your consumers able to handle that? Likely yes, 30 Hz is fine for small messages. But if you spawn, say,
20 objects each at 30 Hz, that’s 600 messages/sec – still trivial for Zenoh and modern networks, but with all
the overhead it might approach the 1.5 MB/s you budgeted. Keep an eye on throughput and perhaps start
with a smaller frequency or fewer agents, then scale up.
Real-World Tests (Future): Ultimately, after CARLA validation, you should plan small-scale field demos. For
example, two cars with laptops and maybe an RC car or drone as the hazard. That will expose any
integration issues with real sensors and comms (like GPS inaccuracies, cellular modem lag spikes, etc.). But
doing this after confidence is built in sim will save a lot of headache.
So, moving to CARLA is absolutely the right immediate step. It is the standard path in academic and
industrial R&D to prove out cooperative perception. You’re not abandoning real-world tests; you’re de-
risking them by simulation first. Given your hardware limits, CARLA gives you a lot of bang for the buck in
terms of validating the concept under many scenarios that would be impractical to stage in reality at this
point.
3. Verdict: Toy or Critical Infrastructure?
My verdict: The Project GodView v3 architecture is on a trajectory toward critical infrastructure, but it’s
not there yet – which is perfectly fine at this stage. In other words, it’s far more than “toy code” in concept,
but to earn the label of critical infrastructure, it will need thorough testing, hardening, and likely integration
with standards down the line.
Why it’s not just a toy: - It addresses real, fundamental problems (latency, spatial indexing, security) that
any city-scale cooperative perception system must solve. The solutions proposed aren’t hacks; they are
rooted in known techniques (Kalman smoothing for OOSM 2 , hierarchical spatial indexes, capability
security models) and extend them in an innovative combination. This isn’t a quick demo that ignores edge
cases – quite the opposite, you’ve identified the edge cases up front. - The inclusion of a robust trust
framework is a hallmark of serious infrastructure thinking. Most hobby or “toy” projects would not bother
with security until much later, if ever. Your use of CapBAC and decentralized verification shows an intention
to build something that could be safely deployed in the wild, not just run in a lab. That aligns with
requirements for critical infrastructure (which demands security and resilience). - Scalability is considered
(50 agents, bandwidth budgets, etc.). Many toy demos would just connect a couple of vehicles with no
thought to bandwidth optimization or subscription sharding. The use of Zenoh and H3 indicates you’re
thinking at city scale (thousands of objects, distributed systems). This is infrastructure-level thinking. It
10aligns with emerging standards like ETSI’s Collective Perception messages (which also define object lists
with position and velocity) 15 . In fact, your GlobalHazardPacket (with lat, lon, alt, velocity vector, class,
confidence) is almost exactly the data that ETSI’s Perceived Object Container in Cooperative Perception
Messages contains (location, speed, heading, etc.) 15 . This parallel evolution suggests you’re solving the
right problem – bringing standardized ideas into a more decentralized, software-defined realm.
What’s needed to graduate to critical infrastructure: - Reliability and Redundancy: Critical
infrastructure implies high availability and fail-safe behavior. The current design has no single point of
failure in theory (decentralized), which is good. But you’ll need to ensure things like: what if Zenoh broker
node goes down (if you use one), or if GPS goes out, or if one vehicle spews malformed data (despite auth)?
Building resilience (maybe multiple communication paths or local fallbacks) will be a next step. -
Interoperability: For something like this to be adopted city-wide, it might need to work with various
vendors and possibly integrate with existing V2X systems. That might mean translating
GlobalHazardPackets to/from standard CPMs, or ensuring your system can run on automotive hardware. A
GTX 1050 Ti is fine for a prototype, but an actual autonomous vehicle might use more modern compute (or
need this to run on an embedded system). Fortunately, none of your components are insanely heavy – an
optimized EKF, some networking, and crypto can all run on automotive-grade systems. It’s more a matter of
porting and testing on those platforms. - Validation & Certification: As with any safety-critical system,
proving it works under all conditions is the hard part. You’ll need extensive simulation and real-world
testing, and eventually external validation (maybe using formal verification for parts of the logic, or at least
following automotive software standards like ISO 26262 for the implementation). This is far ahead, but
mentioning it to contrast that initial prototypes are far from that bar. The journey from prototype to
deployed infrastructure involves rigorous test plans and incremental deployment (first in pilot zones, etc.).
Toy vs Critical: At this moment, GodView v3 is an experimental prototype with a strong design. It’s not
deployed, so one might call it a “proof of concept.” But it is not a toy in the derogatory sense – it’s not
something trivial or naive. It’s a sophisticated system tackling the right problems. The next steps (CARLA
tests, then real-world demos) will determine how close it gets to a production-ready system.
If I may use an analogy: It’s like an early version of an operating system for a new network – it has the
architecture of a serious product, but it hasn’t gone through the years of hardening that, say, TCP/IP or
HTTP did before being critical infrastructure. Calling it “toy code” would ignore the depth and foresight in
the design. On the contrary, it has potential to influence how future cooperative perception networks are
built, especially if you demonstrate its effectiveness.
Final Thoughts: Continue with the rigorous approach: - Test under increasingly realistic conditions (you
might incorporate the actual timing of sensor processing in CARLA by eventually using camera feeds
through an object detection model – to simulate real perception delay and errors). - Monitor the system’s
performance metrics closely in sim (latency distribution, CPU load for EKF, network usage, false positive/
negative rates for hazards). - Refine the parameters (H3 res, EKF noise values, etc.) based on those
experiments. - Document and communicate results – if you can show e.g. “Car B reacted 200 ms sooner to
an occluded hazard because of Car A’s data, avoiding an accident,” that’s a compelling case to move
forward.
Given the modular nature of your design, none of the potential issues we discussed are deal-breakers;
they’re areas for improvement. The architecture passes the sanity check overall – it’s internally consistent
and founded on known principles. The devil will be in the details of implementation and tuning, but you
11appear to be aware of those. So, I would encourage proceeding full steam with the CARLA validation, while
keeping the above caveats in mind. With careful development, Project GodView could evolve from a
prototype into a piece of the critical infrastructure needed for safe autonomous urban mobility.
Sources:
• Malik, S. et al. (2023). Collaborative Perception—The Missing Piece in Realizing Fully Autonomous Driving.
Sensors, 23(18), 7854. – Overview of collaborative (cooperative) perception approaches, including data-
sharing models and challenges like trust and bandwidth 16 17 8 .
• ETSI TR 103 562 (2021). Collective Perception Service (CPS); Cooperative Perception Messages and
Performance. – Describes the standard format for sharing perceived objects (location, speed, heading) and
the need for redundancy mitigation in V2X perception sharing 15 10 .
• Ullah, I. et al. (2018). Multisensor-Based Target-Tracking with Out-of-Sequence Measurements. Sensors,
18(11), 4043. – Demonstrates using fixed-lag smoothing (Rauch-Tung-Striebel smoother) to handle out-of-
sequence sensor data for improved tracking accuracy 2 .
• Wu, Q. et al. (2023). On Data Fabrication in Collaborative Vehicular Perception: Attacks and Mitigations.
USENIX Security Symposium. – Notes that collaborative perception must meet tight deadlines
(~100 ms) and therefore should transmit only minimal essential data to be feasible 1 .
• Biscuit Authorization (2020). Decentralized Capability Tokens. – Documentation on Biscuit tokens, which
allow embedding Datalog policies in a bearer token and offline verification/attenuation, supporting robust
distributed access control 18 19 .
121
[PDF] On Data Fabrication in Collaborative Vehicular Perception: Attacks ...
https://www.usenix.org/system/files/sec23winter-prepub-37-zhang-qingzhao.pdf
Multisensor-Based Target-Tracking Algorithm with Out-of-Sequence-Measurements in Cluttered
Environments
2
https://www.mdpi.com/1424-8220/18/11/4043
3
Kalman Filtering with Uncertain and Asynchronous Measurement ...
https://navi.ion.org/content/71/3/navi.652
4
Network Latency in Teleoperation of Connected and Autonomous ...
https://www.mdpi.com/1424-8220/24/12/3957
5
PACP: Priority-Aware Collaborative Perception for Connected and ...
https://www.computer.org/csdl/journal/tm/2024/12/10646529/1ZIPSvkShRS
6
7
Tables of Cell Statistics Across Resolutions | H3
https://h3geo.org/docs/core-library/restable/
8
9
10
11
12
13
14
15
16
17
Collaborative Perception—The Missing Piece in Realizing Fully
Autonomous Driving | MDPI
https://www.mdpi.com/1424-8220/23/18/7854
18
Biscuit tokens
https://www.biscuitsec.org/
19
Auth.Biscuit - Hackage - Haskell
https://hackage.haskell.org/package/biscuit-haskell/docs/Auth-Biscuit.html
13
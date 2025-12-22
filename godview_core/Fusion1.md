Introduction
Project GodView v3 is a decentralized multi-agent perception system where ~50 autonomous agents
(vehicles, drones, etc.) share observed hazards over a mesh network (Zenoh). Without a central server
assigning “ground truth” IDs, the system suffers the “Duplicate Ghost” phenomenon – the same physical
object appears with multiple track IDs (UUIDs) in the global model 1 . This leads to visual flicker, double-
counting, and downstream planning errors 2 . Another challenge is “rumor propagation,” where agents
recirculate fused data in loops, causing overconfident covariances (“mathematical incest” in filter terms 3 ).
The current v3 solution addresses these with:
• Mahalanobis Gating + Global Nearest Neighbor (GNN) for data association (matching tracks
across agents).
• Covariance Intersection (CI) for track-to-track fusion, instead of naive Kalman merging, to prevent
overconfident filters in a loop-prone peer-to-peer network 3 4 .
• “Highlander” ID heuristic for distributed identity resolution: all agents adopt the smallest UUID as
the canonical ID for a track, ensuring they converge on a single ID without negotiation 5 6
(“There can be only one”).
This GNN+CI+Highlander stack meets strict constraints (mid-range CPU only, <100 ms cycle, <1.5 MB/s for
comms 7 ) and has shown to minimize CPU load and bandwidth while guaranteeing eventual consistency
in track IDs 8 9 . However, it has known limitations in complex scenarios. GNN provides fast,
deterministic associations but can mis-associate or spawn duplicates in high clutter or when objects are
closely spaced. CI yields consistent but sometimes overly conservative state estimates. The Highlander ID
merge (min-UUID) resolves ID conflicts but is a simple heuristic that could be augmented for faster or
smarter convergence.
This report explores creative, efficient alternatives and augmentations to the GNN+CI+Highlander
pipeline, aiming to reduce duplicate ghosts and rumor propagation under the same real-time, decentralized
constraints. We consider improvements at each stage: data association, track fusion, and identity
management. Approaches like Probabilistic Data Association (JPDAF), clustering-based soft gating
(DBSCAN), feature/embedding-based matching, advanced CRDT-style label consensus, and fusion-free
deduplication (tracklet threading) are examined. We evaluate each for accuracy (duplicate reduction,
tracking quality), robustness to sensor heterogeneity and delays, computational complexity, bandwidth
usage, and ease of decentralization. Clear comparisons and integration strategies with the existing Rust +
nalgebra + Zenoh stack are provided, along with pseudocode and an architectural sketch of how these
alternatives could fit into Project GodView v3.
1Limitations of the Current GNN+CI+Highlander Approach
Before diving into alternatives, it’s important to summarize why the current approach, while effective, may
need augmentation:
• Data Association Ambiguities: GNN (with Mahalanobis gating) assigns each incoming observation
to the nearest existing track or spawns a new track if none is within threshold. It’s fast and mostly
reliable, especially with Mahalanobis distance properly normalizing differences for each sensor’s
precision 10 . However, GNN is a hard assignment – each detection is either committed to one track
or not at all. In scenes with multiple closely spaced objects or high clutter, a greedy nearest-neighbor
can make incorrect matches or oscillate assignments. There is no notion of uncertainty in the
assignment; a marginal match might still either be taken or dropped. This can lead to track ID
“ping-pong” (two real objects swapping IDs if association flips) or duplicate tracks (if an object’s
detection falls outside a strict gate one cycle, a new UUID is spawned). The current system mitigates
some of this by gating on object class/type (e.g. do not associate a pedestrian detection to a car
track) 11 , but subtle issues remain.
• Fusion Overconfidence in Loops: CI fusion is used specifically to handle the unknown correlations
in a mesh network. Standard Kalman track fusion would optimally combine estimates if errors are
independent, but in a loop (A → B → C → A), it would over-trust redundant data and collapse
covariance 4 . CI addresses this by computing a convex combination of information matrices,
effectively “never more confident than the least confident source” 12 , thus guaranteeing
consistency even if two agents are sharing the same info in disguise. The trade-off is that CI can be
overly conservative – the fused estimate might have a larger covariance (less confidence) than a
proper Kalman fusion would give if the data were truly independent 13 . This conservatism can
slightly degrade tracking precision or responsiveness. Moreover, CI assumes all fused tracks are
possibly correlated; it doesn’t distinguish fresh independent info from a rumor. In cases where an
agent’s observation is genuinely new, CI may under-weight it, slowing how quickly covariance
tightens compared to a perfect fusion. There is also the computational aspect: CI (especially the
“Fast-CI” heuristic used in v3) is chosen for being $O(1)$ per fusion 14 , but more complex fusion
algorithms could incorporate weighting or out-of-sequence handling at some cost.
• Highlander ID Heuristic: The current ID consensus is elegantly simple: each track stores all
encountered UUIDs (aliases) and always switches to the lexicographically smallest one 15 6 . This
Conflict-free Replicated Data Type (CRDT) strategy guarantees eventual consistency (all agents will
agree on one ID for the object) without any extra messaging 16 . It’s a monotonic rule (IDs only ever
switch in one direction to a “lower” ID), so no oscillation or endless conflict. The downside is mostly
in initial behavior – for a brief time, different agents may use different IDs until the smallest ID
propagates. The flicker is short-lived (milliseconds) but present 17 . Also, lexicographic min of a
UUID (often a random number) is effectively arbitrary; it doesn’t consider which agent had better
view or higher confidence. In theory, a smarter consensus might choose the best representative
track’s ID rather than a random min. But implementing that without breaking the beautiful simplicity
and eventual consistency of Highlander is non-trivial.
Given these considerations, we explore the following alternatives and augmentations:
2Alternative Data Association Methods
Joint Probabilistic Data Association (JPDAF)
Mechanism: JPDAF is a soft data association technique that, instead of choosing a single “best” match for
each observation, computes association probabilities for each possible pairing of tracks and observations.
In a multi-target scenario, it essentially evaluates the likelihood of each assignment hypothesis and updates
tracks with a weighted sum of observations. This means if two agents’ hazard reports are ambiguous (e.g.
two close objects and two detections), JPDAF can partially update each track with each detection
probabilistically, rather than committing a potentially wrong one-to-one pairing.
Advantages: JPDAF excels in high clutter or closely spaced targets where GNN might either mis-associate or
create duplicate tracks. By considering multiple hypotheses, it can reduce instances of one object becoming
two tracks or vice versa. It’s also robust to missed detections – a track can still be updated by a weaker
association if no strong match, rather than being dropped or duplicated. Notably, JPDA has better
tolerance for false alarms and clutter; it will down-weight unlikely matches instead of either accepting or
rejecting them outright 18 . This could mitigate the “duplicate ghost” issue by more gracefully handling
borderline cases where an object’s observation doesn’t squarely hit an existing track’s gate – JPDAF might
still associate it with some probability instead of spawning a new ID immediately.
Challenges: The probabilistic approach comes at a cost. First, JPDAF is computationally heavier than GNN.
In the worst case, solving the JPDA assignment grows exponentially with the number of close objects/
detections (it must sum over all feasible association combinations), though practical algorithms use
clustering and pruning to manage this. Still, compared to GNN’s $O(n^3)$ or better for assignment, JPDA
can be significantly more CPU intensive if many objects are within gating range of many observations 19 .
Given the 100 ms frame budget on a mid-range CPU, a full JPDAF might be borderline if dozens of hazards
are in view. Second, in a decentralized context, JPDAF’s usual implementation would require sharing the
association probabilities or jointly optimizing across agents, which is bandwidth prohibitive 20 . Each
agent could run a local JPDAF using both its own detections and the received tracks from others as
“measurements,” but ensuring that all agents converge to the same probabilistic assignment without a
central coordinator is tricky. The v3 analysis noted that synchronizing JPDA’s marginal probabilities across
the mesh would impose prohibitive overhead 20 .
Potential Augmentation & Variants: One compromise is to apply JPDAF locally per agent in a limited scope.
For example, each agent could treat incoming remote tracks similar to its own sensor detections in a JPDA
framework for that cycle. This doesn’t require sending probabilities; each agent does its own calculation.
They might momentarily disagree on associations, but the Highlander ID merge and continued
communication can correct any ID inconsistencies after a few cycles. Another variant is Joint Integrated
Probabilistic Data Association (JIPDA) or Distributed JPDA with consensus. Some research has proposed
distributed JPDA filters where each node runs JPDA locally and then fuses the resulting track posteriors with
neighbors, though typically these assume some tree network or approximate the correlation. That likely
violates our 100 ms budget.
JPDAF’s impact on ghosts & rumors: JPDAF can reduce duplicate ghosts because it inherently tries to
coalesce duplicate observations into the same track when statistically justified. However, it has a known
drawback: track coalescence of distinct objects. If two hazards stay very close, JPDAF sometimes merges
them into one “average” track (because it assigns both detections partially to both tracks over time) 21 . In
3other words, JPDAF might solve one kind of ghost (duplicate IDs for one object) at the risk of the opposite
problem – mistakenly fusing two separate objects into one track (losing an object). This track coalescence is
documented as a tendency of JPDA in dense scenarios 22 . Our baseline GNN would at least maintain two
separate IDs if two objects are present (it might swap them incorrectly, but won’t merge them into one
track). Thus, JPDAF must be used with caution; gating thresholds or additional logic would be needed to
prevent merging truly distinct targets. On the rumor propagation side, JPDAF by itself doesn’t solve
covariance inflation from loops – we would still likely apply CI for the fusion step of the resulting tracks to be
safe (JPDAF covers data association, not how we fuse state estimates). If anything, JPDA could worsen
bandwidth usage if naïvely implemented, since sharing probability matrices or multiple hypothesis info
would blow past the ~1.5 MB/s limit. Therefore, a practical recommendation is a hybrid approach: use a
simplified JPDAF only in ambiguous situations. For example, run GNN normally for most objects, but if an
agent detects that multiple tracks and detections are mutually within each other’s gates (an ambiguous
cluster), invoke a JPDA sub-routine for just that cluster (a small number of objects) to resolve association
more optimally. This keeps worst-case complexity constrained and avoids system-wide overhead.
In summary, JPDAF offers improved association robustness (fewer duplicate IDs in clutter, graceful
handling of missed detections) at the cost of higher complexity and potential ID merge issues of its own.
It could augment the current system by handling edge cases where GNN struggles, but a full JPDA solution
across the mesh is likely too heavy. We will compare JPDAF’s trade-offs to other methods in the
Comparative Trade-offs section.
Cluster-Based Association (DBSCAN and Soft Gating)
Mechanism: Instead of associating tracks pairwise, we can approach the data association as a clustering
problem in state space. The idea is to group track estimates that likely correspond to the same object. A
popular method is DBSCAN (Density-Based Spatial Clustering), which can cluster points (e.g. track
positions) that lie within a certain distance ε of each other. In a multi-agent scenario, each agent at each
cycle could take the set of all track estimates it knows (its own tracks and recently received tracks from
others) and perform clustering in the state space (e.g. positions, possibly velocities). Each cluster of tracks
would represent a single hypothesized physical object. If two or more track estimates from different agents
fall into the same DBSCAN cluster, the system labels them as the same object and can merge their IDs (and
possibly states). Compared to strict gating, clustering provides a “soft gating”: points that are within some
radius of any other point in the cluster can be grouped, even if one pairwise distance was slightly over a
threshold. For example, if Agent A’s and Agent B’s position estimates for a hazard are a bit off due to noise,
they still end up in one cluster if they’re all within ε distance chain-wise, avoiding creation of separate
ghosts.
Advantages: Clustering methods like DBSCAN are inherently good at discovering groups of points without
prior assignment of which belongs where. This can naturally solve the “multiple duplicate” issue – if the
same object has three different IDs in the network, all three track points will lie in the same vicinity and
DBSCAN will group them. The cluster can then be resolved into a single track (by some fusion or selection),
eliminating the duplicates. GNN, by contrast, typically handles associations pairwise; if three agents have
separate IDs for one object, pairwise association logic might require multiple iterations or could end up
with one leftover. Clustering handles many-to-many associations seamlessly. Another benefit is
computational efficiency in practice: DBSCAN has average-case complexity around $O(n \log n)$ (can be
worst-case $O(n^2)$, but with spatial indexing (e.g., H3 octree already in use) it’s efficient). A recent study
proposed a distributed DBSCAN-based track association and found it more computationally efficient than
4conventional approaches like full JPDA or MHT in distributed tracking 23 24 . Each agent can run the
clustering independently on its local view of tracks; they should reach similar cluster results if their inputs
are eventually consistent (slight timing differences might temporarily cause minor discrepancies, but the
system can correct next cycle). Importantly, DBSCAN doesn’t require exchanging any information beyond
what the system already shares (track states). It’s a local algorithm: each agent gets tracks via Zenoh and
clusters them – no additional network overhead or synchronization of probabilities is needed, unlike JPDA.
Using a confidence-aware distance metric can further improve clustering. Instead of plain Euclidean distance,
we can use Mahalanobis distance (which factors in covariance) or dynamically adjust ε based on the
combined uncertainties of tracks. For example, if one track is LiDAR-derived (very precise) and another is
from GPS (fuzzier), their allowed spatial gating distance should account for that (the design doc notes 2 m
might be “far” for LiDAR but “close” for GPS 10 ). A Mahalanobis or covariance-weighted distance in the
clustering ensures we group tracks if their error ellipses overlap, not just raw positions. This is effectively a
“soft gating” – tracks with large uncertainty can cluster with others even at larger physical separation, while
high-precision tracks demand tight proximity to cluster.
Outcome and Fusion in Clusters: Once we identify a cluster of tracks representing the same object, how to
fuse or choose the final track? One approach is to designate a cluster leader (e.g., pick the track with the
highest confidence or smallest covariance as the representative) and have all agents adopt that leader’s ID
(similar to Highlander but based on quality rather than arbitrary UUID). Alternatively, one could perform a
track fusion: for all tracks in the cluster, combine their state estimates. Covariance Intersection can be
applied to more than two sources iteratively – effectively fusing the cluster into one track with a safe
covariance. Another simpler approach is just averaging positions with some weights (though that might
ignore correlation; CI or at least averaging with covariance weighting is preferable). Notably, if the cluster
contains one track that originated locally (the agent itself saw the object) and others that came from remote
agents, the local agent might decide not to fuse the remote ones every cycle but simply to use them as
confirmation. This ties into the fusion-free concept later (clustering can be used purely for deduplication of
IDs, without necessarily merging state estimates each time, to reduce rumor propagation).
Challenges: Choosing the right ε (cluster radius) is critical. If too small, legitimate duplicates might not
cluster (defeating the purpose). If too large, distinct objects risk being merged into one cluster (similar to
JPDA’s coalescence issue). We can mitigate this by making ε scale with estimated covariance – effectively
using probabilistic distance. Also, DBSCAN has a parameter for minimum points per cluster; here we might
set minPoints = 1 or 2 since even a single track should form a cluster by itself. Another challenge is real-time
performance if the number of tracks is large; however, with spatial indexing (already mentioned in the code
for GNN association 25 ), we typically only cluster local neighborhood of tracks (no need to cluster objects
far apart in the world).
Distributed Consistency: In a decentralized system, each agent running DBSCAN on its current set of
tracks should ideally produce the same clustering of global objects. Minor differences can occur if, say,
Agent A has not yet received a track from Agent C that Agent B has. But as data propagates (with Zenoh’s
eventual consistency), those clusters will align. Even if temporarily one agent clusters two tracks and
another hasn’t yet, the Highlander ID rule can still enforce eventual convergence (once Agent A hears the
third track, it will merge IDs too). There might be a need for a unique cluster ID so that agents know they
refer to the same cluster/object. We could use, for example, the smallest UUID in the cluster as the cluster’s
global ID (which is exactly Highlander applied at cluster level). In effect, clustering plus taking the min ID of
5each cluster yields a very similar outcome to the current system’s ID merge, but clustering helps ensure we
found all duplicates in one go.
Resilience: Clustering is naturally robust to sensor heterogeneity and jitter if using Mahalanobis distance –
it adapts the grouping to account for timing and uncertainty. If packets are delayed, an object might
temporarily appear as two clusters, but as soon as the delayed data arrives, they merge. One can even
incorporate a temporal dimension in clustering (e.g., consider recent track positions over a short time
window) to account for objects moving. A variation, DBSCAN-Time clustering, could link tracklets across
time – effectively threading track segments when communication delays cause them to appear disjoint.
Overall, a DBSCAN-based association offers a middle ground between rigid GNN and complex JPDA: it’s
computationally efficient and fully distributed (each node does its own clustering) 26 , and it can reduce
duplicate ghosts by grouping all related tracks of one object. We must carefully handle how to then fuse or
select the track from each cluster, which we discuss later (it can integrate with CI or a fusion-free selection).
The next section on Fusion Strategies will consider both fusing and not fusing the cluster information.
Embedding-Based Similarity for Association
Mechanism: Purely geometric association (distances, gating) can be augmented by incorporating
descriptive features or embeddings of the tracked objects. In many tracking domains (especially vision),
using an appearance or feature embedding greatly improves the ability to re-associate objects across
sensors. In Project GodView, agents might have additional data about hazards: e.g., object class (pedestrian,
car, cyclist), color or shape (if camera data is available), radar cross-section, or even a learned descriptor
from a neural network. We propose generating a compact feature vector for each track – either a learned
embedding (e.g., from a small CNN or even the output of a classifier network) or a handcrafted feature set
(size, velocity pattern, LiDAR point cloud signature, etc.). This feature would be transmitted along with the
track’s state in the Zenoh “Global Hazard Packet.” Then, when performing association between an incoming
observation and existing tracks, or when clustering tracks, agents can use a combined metric: a function of
both spatial proximity and feature similarity. For example, one could require not only that the Mahalanobis
distance is within a gate, but also that the feature embeddings are sufficiently similar (above some cosine
similarity threshold, for instance) before considering two tracks a match. Alternatively, a weighted score
could be computed that mixes distance and feature affinity.
Example: NVIDIA’s multi-camera tracking workflow uses visual appearance embeddings to maintain
consistent IDs for objects seen across different camera views 27 . They track objects through cameras by
matching embedding vectors rather than relying solely on position, achieving a unique ID per object across
the camera network. We can adopt a similar idea: treat each agent like a “camera” in a multi-camera system
– each agent produces an embedding for an object, and if another agent’s object embedding is near in
feature space (and time/space), they likely correspond to the same object. By tracking IDs via these
embeddings, the system becomes more robust to scenarios where geometry is ambiguous but appearance
or other attributes are distinctive. For instance, two pedestrians standing close might be hard to distinguish
by position alone, but if one is wearing red and another blue (and the system has color or clothing
embeddings), we can avoid merging them. Conversely, if the same person is seen by two different agents
from different angles, geometry might be a bit off due to sensor noise, but the appearance embedding
could help confirm it’s the same person, thus preventing duplicate ghost tracks.
6Handcrafted vs Learned Features: If GPU were available, a deep learned embedding (like a small ResNet
for appearance or a point cloud embedding network) would be ideal. However, GPU is mostly tied up with
CARLA and any local perception (YOLO) in this project 28 . We likely need a lightweight solution. We can
leverage the output of existing detectors: for example, if YOLO or another model is used to detect the
hazard, the class label is already used (we saw class gating in code 11 ). We could extend that with other
metadata: bounding box size (which correlates with distance in camera), object orientation, or a few basic
color histograms from the image. Alternatively, an agent might maintain a simple appearance model (like
the average color of the object’s bounding box or a d-dimensional feature from the detector’s intermediate
layer). If cameras are not available (e.g., only LiDAR), we might use shape features (bounding box in 3D,
reflectance) or even driving trajectory patterns (e.g., a vehicle on a straight road vs a zig-zag pedestrian –
patterns which could be embedded via a simple RNN or just a feature vector of recent motion). The key is
any additional discriminative information beyond position.
Benefits: Incorporating embeddings can dramatically reduce ID switches and duplicates in multi-target
tracking, as shown in multi-camera tracking research. It provides another channel of association that is
independent of (and complementary to) geometry. Especially in a heterogeneous sensor network, one
agent’s view might be front-facing camera (with color info) and another’s side LiDAR – combining their
evidence through a similarity embedding can yield more confidence that two detections are the same
object. Embeddings can also help filter out false associations: e.g., if a false positive detection has a weird
feature not matching anything, it won’t associate even if by chance it’s nearby spatially.
Cost: The additional cost includes computing and sending the embeddings. The bandwidth impact is
minimal – a feature vector of say 16–64 bytes per track is negligible relative to sending positions,
covariances, etc. (which likely are already on that order or more). The compute cost depends on how we
derive the embedding. If using an existing neural net (like the object detector) to extract features, we reuse
computations. If we design a small separate model for ID features, that could be run on CPU if small (e.g., a
2-layer NN on some inputs), but we must be mindful of the 100 ms budget. Since each agent tracks at most
maybe dozens of objects, even a small inference per object might be okay (tens of microseconds each).
Another approach: use sensor-specific features that are easy to compute (e.g., LiDAR point cluster
statistics).
Integration: In practice, we would update the TrackManager.process_packet logic: when comparing
an incoming packet to existing tracks, after Mahalanobis gating passes, also compare an embedding
similarity. We might maintain a rolling average of the track’s embedding (since an object’s appearance from
one agent might differ from another’s viewpoint, but if they see the same feature at overlap time we can
lock the ID). For clustering, we could perform a joint clustering in the space of [position, feature] by defining
a distance that is a weighted combination of spatial distance and feature distance. Alternatively, do two-
stage association: first cluster by spatial proximity (loose), then within each spatial cluster, check for feature
consistency to either confirm or split if it looks like two different objects accidentally got too close.
Robustness: This approach improves robustness to sensor heterogeneity directly – it leverages whatever
unique info each sensor has. It’s also helpful with delayed packets: if an observation arrives late, pure
geometry might conflict with an updated position, but an appearance embedding tied with a timestamp
could suggest it was that same object a moment ago. However, one must be cautious that embeddings can
themselves be noisy or ambiguous (especially if using simple features). We might treat feature similarity as
a soft vote rather than hard rule: e.g., add a small Mahalanobis distance penalty if features differ, or boost
association probability if features match strongly.
7Illustrative Scenario: Imagine two agents approach an intersection from perpendicular streets. Both see a
pedestrian at the corner. Agent A’s view (camera) captures a red jacket, Agent B’s LiDAR just sees a point
cloud shape. Both report a hazard at roughly the same GPS coordinate. Without feature matching, initial
position might differ by a couple meters (GPS error vs vision calibration), possibly outside gating, leading to
two tracks. With embedding, Agent A attaches an appearance ID “red_jacket_123” (some vector) to the track.
Agent B can’t see color, but perhaps it has no conflicting info – or if Agent B had a library of typical shapes, it
could still allow match. At least Agent A and any other camera-equipped agent could match on that. Over
time as the pedestrian moves, a vision agent might positively ID across agents that it’s the same person,
unifying the track sooner. NVIDIA’s system indeed maintains unique IDs tracked through visual
embeddings for multi-camera tracking 27 . While our mesh is not purely cameras, the principle holds for
any distinctive feature.
In summary, embedding-based association augments the GNN/cluster algorithms by adding another
dimension of matching. It can be seen as a feature-based gating or weighting. It should markedly reduce
both duplicate ghosts (fewer cases of one object being perceived as two because we recognize them as the
same via features) and ID churn (objects keep the same ID across disparate viewpoints because the feature
“fingerprint” travels with them). The approach is compatible with others – e.g., we can use embeddings in
JPDA (to influence association probabilities) or in clustering (to avoid clustering dissimilar objects). It adds
minor overhead and fits well in a decentralized framework (each agent computes its own features and
shares them – no centralized database needed, though one could imagine a learned global embedding
space that all agents use).
Identity Consensus via CRDT Strategies
The Highlander heuristic used in v3 is essentially a CRDT for track identities 6 . Each track’s set of seen
IDs is merged across agents, and by always choosing the minimum UUID as canonical, the network
converges to one ID per object without any explicit negotiation or central authority 16 . This approach is
already quite optimal for a decentralized ID consensus: it’s deterministic, eventually consistent, conflict-
free, and requires zero extra messages (the decision is made locally by each agent upon receiving a track
message with a different ID). The question is: can we do any better or add to this?
Possible Augmentations:
- Faster Convergence / Less Flicker: The main drawback of Highlander is the brief period when different
IDs are in use. In a fast 30 Hz system, this may only last a few frames, but if an object is on the border of two
agents’ views, they might hand off back and forth, causing a flip of IDs (though min-ID ensures a one-way
flip only). One idea is to incorporate a handshake or reservation: e.g., when two tracks associate, instead
of immediately switching to min ID, maybe hold the old ID for one extra cycle until confirmed. However, this
actually adds latency in convergence, so probably not beneficial. Another approach: use a content-based ID
so that both agents might independently generate the same ID for the same object. For instance, a hash of
the object’s initial position (quantized) plus class could serve as an ID. In theory, if two agents see the same
object at the same location, they’d hash to the same ID and the ghost never occurs. In practice, slight
differences in observation would make their hashes differ, and collisions or mistakenly unifying close
objects could happen – not reliable.
• Quality-Based ID Leader Election: Instead of lexicographic min, we could choose the “best” track’s
ID to win. For example, if one agent has a very high confidence detection (or lower covariance) for an
object than others, perhaps that agent’s UUID should become the global ID. That agent could embed
8an indicator of its confidence in the track data it sends. All agents could then use a deterministic rule
like: choose the ID of the track with highest confidence as canonical. This would require all agents to
have a consistent view of the confidence values. If agent A thinks it’s highest and agent B thinks it is,
no conflict; but if two agents both have high confidence on what they initially think are separate
objects (but turns out same object), deciding which ID wins could be ambiguous until they realize it’s
the same object. A tie-break like lexicographic could be combined. The benefit of such scheme is
debatable – it might keep the ID from flapping to a possibly less reliable source’s UUID, but since
Highlander min-ID is arbitrary anyway, it doesn’t heavily impact tracking accuracy, just the aesthetic
of the ID string. Eventually, everyone sees the same data and will converge regardless.
• CRDT Set Union: Currently, each track keeps a set of observed_ids (aliases) 29 30 . This is essentially
a growing set CRDT that accumulates all known aliases for that object across the network. One
augmentation could be to periodically garbage-collect or truncate this if it grows large (in theory, if
an object kept being observed by new agents who each time assign a new UUID before seeing
others, the set could grow – but practically, with min-ID adoption, new aliases should stop appearing
after the first few). Another improvement could be sharing the alias set explicitly so that even if two
tracks haven’t directly met (through an intermediate agent maybe), their alias sets might merge
transitively. However, in a fully connected mesh where everyone broadcasts, this is automatic: if A
and B have two IDs, and C hears both, C will unify and broadcast an update that causes A and B to
see the unified ID. So the alias info is inherently propagated.
Conflict-Free Replica: It is worth emphasizing that the Highlander strategy is already a type of CRDT (LWW
– last writer wins, or here smallest ID wins) merge function 31 . It ensures no oscillation because the
choice function (min) is idempotent and monotonic. Any alternative method must preserve those properties
to be viable in decentralization. For example, a random choice or a cycling choice would be bad. A
consensus algorithm like Raft/Paxos to pick an ID leader is overkill and too slow (plus requires more
bandwidth). The elegance of Highlander is hard to beat.
Integration with Alternatives: Regardless of whether we use GNN, JPDA, or clustering for association, and
whether we fuse or not, we will still need a way to unify track IDs. The current min-UUID approach can
remain as a baseline. If we implemented the clustering approach, essentially we’d be merging IDs of all
tracks in a cluster – again, we could just choose the min ID in that cluster as the unified ID (consistent with
Highlander). So clustering + Highlander are complementary: clustering finds all duplicates at once,
Highlander picks the final ID. If we had a reason to prefer a particular agent’s ID (say the one with lowest
covariance), we could encode that preference by adjusting the UUID generation. For instance, an agent
could encode in its UUID some sort of prefix that correlates with its sensor quality (this is speculative;
normally UUIDs are random or time-based). This quickly becomes hacky and not guaranteed. A simpler
approach: after cluster or association, if multiple IDs are present, instead of immediately dropping to
lexicographic min, let each agent set the ID to the lowest IP address of agents involved or something
deterministic. But that would require knowledge of which agents have tracks in the cluster; that information
might not be globally available without extra comms.
Conclusion on ID consensus: The current heuristic is already near-optimal for decentralized ID resolution.
Our recommendation is to retain the Highlander min-ID CRDT for its proven eventual consistency 16 , but
consider minor augmentations: ensure that track IDs are not recycled too quickly (to avoid confusion if an
object leaves and another appears – though using UUIDs inherently avoids accidental reuse collisions). Also,
we ensure the code (as shown in the design) continues to propagate alias sets so that debugging or
9analysis can see which original IDs were merged 15 . In testing, if any flicker is observed, one could enforce
a short hold such that an agent doesn’t create a new UUID for a detection that is very close to an existing
track’s position in a neighboring cell – essentially predicting that it’s likely the same object and waiting one
cycle for the ID to arrive. That’s less about CRDT and more about spawn logic, though.
In summary, label consensus is effectively solved by the Highlander CRDT approach in the current
system. Alternatives exist in theory (like distributed agreement on IDs or content-based IDs), but none offer
a clear advantage under the constraints. We will stick to CRDT-based merging (min-ID or similar
deterministic rule) as it provides guaranteed ID consistency without extra bandwidth 16 .
Fusion Strategy Alternatives
Covariance Intersection vs. Other Fusion Methods
The Covariance Intersection method is chosen for good reason: it provides a guaranteed consistent fusion
of two state estimates when the correlation between them is unknown 13 . In a decentralized loop, this is
critical to avoid the “rumor” positive feedback where confidence blows up 4 . CI essentially computes:
−1
Pfused
= ωP1−1 + (1 − ω)P2−1 ,
for some weight $0 \le \omega \le 1$, and $x_{\text{fused}} = P_{\text{fused}} [\omega P_1^{-1} x_1 + (1-
\omega) P_2^{-1} x_2]$. By selecting $\omega$ appropriately (often through an iterative minimization of
trace or det of $P_{\text{fused}}$ subject to positive-definiteness), CI yields a fused covariance that never
understates the uncertainty 32 . In fact, if the two inputs are the same info (fully correlated), one possible
solution is $\omega=1$ (or 0), meaning essentially ignore one of them – you get back the original covariance
(no overcounting) 33 . This is a nice property: in the extreme, CI can discount redundant data entirely.
Possible Alternatives:
- Generalized Covariance Intersection (GCI): There are other formulations like Covariance Union (for
worst-case bounding) or GCI (which sometimes refers to arithmetic vs geometric means in track fusion).
These aren’t widely used beyond theoretical circles; CI is already the go-to for unknown correlation. The
current implementation uses a “Fast-CI heuristic” which likely picks $\omega$ in a simple way (maybe
proportional to some confidence score or time freshness) to avoid heavy iteration 14 . We might consider
making $\omega$ adaptive: e.g., if one track’s covariance is much smaller than the other’s, put more weight
on that one (thus approaching simply taking the more accurate source). Conversely, if an agent receives its
own track back from a neighbor (i.e., a looped rumor), perhaps $\omega$ should be 0.5 or equal weight,
which still yields larger covariance than the original, effectively penalizing suspected loops. The design doc
indeed notes that CI ensures even if one source is a copy of the other, the fused covariance is no more
confident than the best single source 32 . This is loop-safe, but possibly over-conservative if we actually
have independent views. Maybe we can detect independence: if two agents have very different sensor
modalities or perspectives, one might argue their data is less correlated. However, quantifying that is hard
without explicit knowledge of common ancestors in the network.
• Weighted Fusion with Discounting: One augmentation could be to apply information
discounting or aging. For example, if a track update is older or second-hand, inflate its covariance
before fusing (giving less weight implicitly). The augmented state EKF (AS-EKF) already projects
tracks to a common timestamp 34 , but still a stale observation could be given a lower weight in
10fusion. This is akin to setting $\omega$ based on the relative age or perceived quality of the two
tracks. We must be careful not to break consistency though; it’s safer to inflate older data’s
covariance (making it less confident) and then do CI or even Kalman fusion if we’re sure the info is
new.
• Hierarchical Fusion / Buffering: Another idea is to prevent immediate fusion of everything. Perhaps
maintain a short buffer of recent states from other agents and fuse in a batch or hierarchical
manner to avoid double-counting. This however increases latency and complexity.
Ultimately, Covariance Intersection is still a very sound choice for decentralization. The main downside is
the conservatism (which can somewhat be mitigated by clever choice of weights). Some literature on
distributed tracking uses consensus algorithms where agents iteratively refine a global state estimate by
averaging (like a decentralized Kalman consensus). Those require many communication rounds (not
feasible under 100 ms and limited bandwidth). There’s also the concept of track-to-track fusion using
Kalman with known cross-covariance – if we could estimate the correlation between two tracks, we could
fuse optimally. In theory, one could send not just state and covariance, but also some indicator of how
much of that info came from which sources (an information matrix decomposition). But tracking that in a
loop is complex and would blow up message size (imagine sending the whole history or source ID list for
each covariance). Covariance Intersection smartly sidesteps needing any of that.
Conclusion: We suggest keeping Covariance Intersection for the fusion step, possibly tuning the weighting
strategy. For example, one might implement Fast-CI such that $\omega$ is chosen based on time or
source: e.g., if one track is the agent’s own fresh observation and the other is a received track, maybe trust
the local more ($\omega$ high) but not fully (still fuse some of the other to incorporate its info). If two
received tracks from different agents are being fused, maybe $\omega=0.5$ (equal) since both could be
correlated or not – equal is a safe neutral choice. This is speculative; the actual Fast-CI might already do
something like this. The performance cost of CI is minimal (just a few matrix ops) 14 , so we’re good there.
One twist: if we incorporate clustering and potentially fuse multiple tracks (>2) at once, we can extend CI
iteratively (fuse two, then fuse the result with the third, etc., each time applying CI). The order could matter;
we might fuse the two most certain first, etc. Alternatively, one can generalize CI via convex combination of
multiple info matrices. Ensuring consistency in multi-fusion still holds as long as each pairwise fusion is
conservative.
Fusion-Free Deduplication (“Tracklet Threading”)
Perhaps the most radical idea is: don’t fuse states at all – at least not in the traditional sense. That is,
handle duplicate tracks by eliminating the duplicates (merging their identities) but not by mathematically
merging their kinematic state estimates. We call this “fusion-free deduplication.” The concept is to maintain
separate local estimates while ensuring they are recognized as the same object in the world model. In
practice, how would that look? A simple approach: when Agent A and Agent B realize they are tracking the
same object (through association/clustering), they agree on one ID (via Highlander). Now each agent has
two estimates for ID_X: its own and the other agent’s. Instead of fusing them into one, the agent can choose
one as the authoritative track and discard or temporarily suspend the other. For example, perhaps Agent A
sees the object clearly (low covariance) while Agent B has a poor view. All agents could then prefer A’s track
data for ID_X and effectively drop B’s contribution. B might even stop broadcasting its track once it knows
it’s duplicated by a better one. The result: no duplicate ghost (only one ID X is displayed) and no
11overconfident fusion (because we never combined info; we simply use one). This is analogous to an
orchestrated hand-off – whichever agent has the best observation “takes ownership” of the object’s state,
and others defer to it.
Advantages: This absolutely prevents rumor propagation because we avoid multi-cycle fusion loops
entirely. If Agent A’s track goes to B and B doesn’t fuse but just notes “okay, that’s the same object I see”, B
doesn’t create a new estimate that goes back to A. Instead, A’s original estimate is used (perhaps updated
by A only). It’s like each object has a single source of truth agent at any given time. When that agent loses
sight (maybe the object moves out of A’s range), another agent that still sees it can take over state
publication. Essentially, we form a thread of tracklets: a sequence of track segments from different agents,
stitched by common ID. Each segment is produced by a single agent’s local tracker, avoiding any
mathematical fusion. This is reminiscent of how multi-camera tracking systems often work – each camera
tracks independently, and an identity reassociation algorithm links the tracks when a person moves from
one camera view to another. During overlap, one camera might be designated primary to avoid double
reporting. We can do the same in multi-agent: during overlap of fields of view, either one agent’s track is
chosen or both report but one is suppressed in the global view. In a decentralized system, we can’t truly
suppress another agent’s broadcast, but we can locally decide not to spawn a new track or not to fuse.
Implementation approach: One approach is a publish/subscribe filtering: when an agent receives a
remote track that it can associate to one of its own, instead of fusing, it could decide “my track is duplicate,
but which is better?”. If the remote track has a smaller covariance or higher confidence, the agent might
drop its own track (stop tracking or mark it as inactive) and simply adopt the remote one (perhaps just
propagate it forward if needed for prediction). Conversely, if the local track is better, the agent could ignore
the remote. If both continue independently, they at least share an ID so the user or higher-level planner
doesn’t get confused by two objects. But for consistency, better to have one active. How to ensure all agents
agree on one active track? This could be done with a rule like: the track with the lexicographically smallest ID is
the primary (that doesn’t necessarily reflect quality though), or the agent with the lowest ID (or some priority) is
the leader for that track. Alternatively, something like whoever first detected it remains owner until they yield.
Perhaps simpler: each track’s metadata could include an “owner” field designating which agent is currently
responsible for state updates. Initially, the owner is the creator (agent who first saw the object). If another
agent later has a much more accurate view (maybe the original agent is far, the new agent is close), we
might switch ownership – perhaps by having the new agent broadcast a message with higher confidence
that effectively “steals” ownership. This begins to sound like a distributed leader election per object. It’s
doable with a rule (e.g., higher confidence or lower covariance wins, tie-break by agent ID). Agents hearing
an ownership change would then refrain from sending updates for that object while it’s owned by another.
This would drastically cut down redundant transmissions – e.g., if 5 agents see the same explosion
hazard, perhaps only one (the closest or first) broadcasts track updates for it, the others either don’t send or
send at lower rate. That also helps bandwidth! It’s like a suppression of duplicate reports.
Trade-offs: Fusion-free means we might not utilize all available information to refine the estimate. In some
cases, two weaker perspectives combined could give a stronger estimate (e.g., triangulating position from
two angles). CI allows that combination safely; fusion-free would choose one perspective and possibly lose
the complementary info. That said, if both perspectives remain separate, each agent still has its local track;
they’re just not merged. So each agent might have a slightly different estimate of the object. The global
model as seen by different agents might then diverge (one thinks position X, another thinks Y, though both
share the same ID). This is a downside: with CI fusion, ideally all agents’ track states converge to the same
fused value for that object (assuming they share all data). With fusion-free, we could have inconsistency in
12state across the network. A way around that is to allow some minimal sharing: maybe the owner’s state is
taken as authoritative and other agents, if they need it, could correct their local estimate to match the
owner (or simply always use the owner’s value for display/planning). Essentially, treat the owner’s track as
the truth. In distributed systems, this introduces a notion of trust/hierarchy which we tried to avoid, but it
can be dynamic per object. It’s not strictly needed that every agent have identical state estimate at all times,
as long as each has at least one estimate with the correct ID. But for coherent situational awareness, it is
nice if all agree roughly where the object is. If not fusing, at least broadcasting the owner’s state to
everyone ensures they all have that info. Those that have their own state could compare and perhaps
decide to update theirs via a standard measurement update (Kalman update) using the owner’s state as a
measurement. That ironically is a form of fusion again (and if repeated, becomes rumor propagation). To
avoid loops, you’d have to ensure only one-way updates (others update towards owner’s state, owner never
takes theirs). That could work if ownership doesn’t oscillate rapidly.
Resilience: This approach shines in eliminating overconfidence and network-induced divergence. If data
loops back, it’s either ignored (since the track is known to be owned by self, so seeing your own ID from
someone else triggers no fusion), or it’s simply recognized as same and dropped. It’s inherently robust to
data incest – the covariance won’t artificially shrink because you’re not combining estimates multiple times.
It also naturally handles delayed data: if an out-of-sequence packet comes in, in a fused system it might
jitter the track; in the fusion-free approach, either that packet is ignored (if the track moved on) or if it’s
relevant, it might temporarily create a duplicate which then merges ID – possibly causing a brief ghost. So
maybe not better for latency issues. The AS-EKF latency compensation and proper timestamp handling is
still needed no matter what. Fusion-free doesn’t solve time ghosts, that’s a separate time alignment issue
solved by prediction.
Bandwidth: As noted, if we implement an ownership suppression, bandwidth use could drop because not
all agents spam the same object. They’d know “Agent 5 is handling ID_X, I don’t need to send my redundant
observations unless I have something significantly new.” This could be implemented via Zenoh keyspace or
a flag in the data (“primary” track vs “aux”). Even without formal ownership messaging, an agent could
decide to reduce its transmit rate or stop after seeing the same ID from another agent with higher
confidence. This might require some careful logic to avoid situations where everyone assumes someone
else will report and then an object isn’t reported at all. Perhaps the first reporter continues until it stops
seeing it; others only step in if the first stops or if they have a much better observation.
Comparison to baseline: The baseline does fusion every time, meaning everyone keeps updating and
forwarding, causing lots of repeated data flow (with CI mitigating the math, but not the bandwidth of
duplicates). Fusion-free is more akin to a distributed track management: it ensures one track per object is
visible, but behind the scenes each agent’s local tracker might still exist or be paused. Conceptually, it’s
separating the concerns of state estimation and identity management: each agent estimates state from
its sensors (no combining of states), and identity management ensures all those states corresponding to
one object share an ID. Then a higher layer could decide which state to use. In the simplest, just present
one (maybe the most certain). This is reminiscent of a multiple hypothesis tracking (MHT) at network
level but simplified: you let multiple tracks (hypotheses) exist but label them as same and eventually pick
one.
Given the complexity, a partial fusion-free strategy might be best: only fuse when necessary. Perhaps use CI
only when two tracks are of comparable quality and both add value; otherwise, just take the better one. This
can be achieved by setting the CI weight $\omega$ towards whichever is more accurate (which is almost
13like not fusing if one is much superior). In effect, CI can approximate a fusion-free selection in extreme
cases (when correlation is high or one covariance is tiny relative to other, CI will heavily weight the better
source, limiting the impact of the other). Fast-CI might already be doing that by design 14 .
Tracklet Threading: The term refers to linking track segments over time. In a multi-agent scenario, an
object might be seen by Agent A, then go out of A’s sight and later appear to Agent B. If IDs were not
unified, that’s a classic ghost: two separate tracks for one object at different times. Highlander would solve
it if there was ever an overlap or communication. But if A had stopped broadcasting after it lost it, B might
start a new ID unaware. With tracklet threading, one could use last known position/time to match that B’s
new track is likely A’s old track (soft metrics on time and space continuity). Embedding features can help
here too (if B’s observation looks similar to what A described before disappearing). Essentially, tracklet
threading extends data association across time gaps. This is a level beyond the current scope (which focuses
on simultaneous observations), but it’s an interesting augmentation for robustness: using slightly older
track info to deduplicate when objects reappear.
In conclusion, fusion-free deduplication is a strategy that prioritizes consistency and avoidance of
overcounting over having a single optimal fused estimate. It is quite radical in a traditional tracking sense,
but given the distributed constraints, it might simplify some issues (no double-counting covariance) at the
cost of a more complex ownership logic and potentially divergent views of the state. It aligns with the idea
of letting each agent do what it’s best at (track locally) and only merging the identities at a higher level (so
the world model contains one marker per object). If implemented carefully (with an agreed selection of
primary track per object), it could reduce both ghosting and rumor significantly, at the expense of possibly
slower improvement in precision for state estimates. A combination approach might yield the best results:
use CI fusion normally, but if a pattern of overconfidence or oscillation is detected, fall back to a fusion-free
mode for that object (i.e., pick one source to trust until situation stabilizes).
Resilience to Heterogeneity, Delays, and Loops
We now evaluate how these proposed alternatives fare against the challenges of sensor heterogeneity,
network issues, and the system constraints, in comparison to the baseline GNN+CI+Highlander approach.
Table 1 provides a high-level summary of trade-offs:
Table 1. Comparative Trade-offs of Track Fusion Approaches (Baseline vs. Proposed Alternatives)
14ApproachAssociation &
ID AccuracyRobustness
(Loops,
Delays)
Baseline GNN
+ CI +
Highlander
<br>(Current v3)– Fast, mostly
correct one-to-
one matching.
<br> – May
miss
associations in
high clutter;
can spawn
duplicate IDs if
gating fails one
cycle. <br> –
Highlander
merges IDs
eventually, but
brief flicker
possible. 2– CI prevents
overconfidence
in loops
(always
consistent)
4 . <br> –
Handles out-
of-sequence
via prediction
(AS-EKF). <br>
– Still may
temporarily
show “ghosts”
until IDs
converge or if
large delays.
15
Compute LoadBandwidth
UseDecentralization
– Low: Gating +
nearest
neighbor is
$O(n \log n)$
(with spatial
index) 35 . CI
fusion is
$O(1)$ 14 .
Easily within
100 ms budget.– Low: Each
agent
broadcasts its
track states
(positions, cov,
ID); minimal
extra
overhead. <br>
– Some
redundancy as
multiple agents
send same
object, but
packets are
small.– Fully
decentralized:
No central
coordinator;
deterministic
CRDT ID merge
6 ensures
consistency
without
negotiation. <br>
– All agents run
same logic and
reach same
conclusions
eventually.ApproachAssociation &
ID AccuracyRobustness
(Loops,
Delays)
JPDAF
(Probabilistic
Association)– Higher
association
accuracy in
dense
scenarios:
accounts for
multiple
feasible
matches; fewer
missed
associations or
wrong
assignments.
<br> – Reduces
duplicate IDs
by merging
observations
probabilistically
when GNN
would split
them. <br> –
Risk of track
coalescence:
closely spaced
distinct objects
might be
merged into
one track 22 ,
hurting ID
accuracy for
those cases.– Moderate
robustness:
Still requires CI
or similar for
fusion to
handle loops
(JPDA
addresses
association,
not fusion).
<br> – No
inherent cure
for delayed
data; out-of-
sequence
handling must
be external
(like baseline’s
prediction
step). <br> – If
distributed,
syncing
probabilities is
difficult 20 ;
agents might
temporarily
disagree on
associations
(but
Highlander can
still fix IDs
eventually).
16
Compute LoadBandwidth
UseDecentralization
– High:
Computational
cost grows
with
ambiguous
pairings
(worst-case
exponential).
With
clustering/
pruning, can
be manageable
for small
groups, but
pushes the
100 ms limit if
many targets.
<br> – Might
need to restrict
JPDA to local
clusters of
tracks to keep
CPU in check.– Potentially
high: A fully
distributed
JPDA would
require sharing
association
probabilities or
global
hypotheses,
which is
infeasible in
1.5 MB/s. <br>
– Practical
use: each
agent runs
JPDA locally on
received data
(no extra
comms beyond
baseline
tracks). That
avoids new
bandwidth, but
sacrifices
perfect global
optimality.– Challenging
but possible:
Without a central
node, each agent
running JPDA
may yield
different results
briefly.
Eventually, via
communication
and ID merging,
they’ll align. <br>
– Fundamentally,
JPDA doesn’t
violate
decentralization,
but its typical
implementations
assume a central
process;
adaptation is
needed (e.g.,
each agent treats
others’ tracks as
inputs).Approach
Clustering
(DBSCAN +
Soft Gating)
Association &
ID AccuracyRobustness
(Loops,
Delays)Compute LoadBandwidth
UseDecentralization
– High
deduplication
accuracy:
Groups all
tracks of same
object,
naturally
solving multi-
ID ghosts in
one step. <br>
– Soft gating
avoids missed
associations: a
slightly out-of-
threshold
observation
can still cluster
via a chain of
nearby points.
<br> – Slight
risk of merging
two distinct
objects if they
come very
close within ε
(needs careful
ε tuning and
possibly
feature
checks).– Strong
robustness to
loops:
Clustering by
itself doesn’t
fuse, it just
identifies
duplicates. If
combined with
CI, we still
handle loops
safely. If used
for ID-only, it
avoids fusion
loops
altogether.
<br> – Handles
sensor
heterogeneity
via
Mahalanobis
distance
(covariance-
scaled
clustering) 10 ,
ensuring fair
association
across GPS vs
LiDAR etc. <br>
– Handles
delayed joins:
once delayed
data arrives,
the cluster
updates (ghost
track merges).
Temporal
ghosts
minimized if
combined with
prediction for
alignment.– Moderate:
Clustering is
efficient with
spatial
indexing
(essentially
$O(n)$ for
relevant
neighborhood).
<br> – DBSCAN
on perhaps
tens of tracks is
negligible
overhead
relative to
100 ms. <br> –
Can leverage
existing octree
(H3) for
neighbor
queries 25 .– Low to
Moderate: No
new data
transmitted,
uses existing
track info. <br>
– Possibly
reduces
redundant
traffic if used
with an
ownership
scheme (one
cluster -> one
broadcast). But
basic clustering
itself doesn’t
change
messages.– Excellent: Each
agent can
independently
cluster tracks it
knows; no
coordinator
needed. <br> –
Deterministic
outcome as long
as all agents
eventually see
the same set of
tracks. Minor
timing
differences
resolve next
cycle. <br> –
Essentially a
decentralized
consensus on
grouping.
17Embedding-
Enhanced
Association
– Very high
accuracy:
Greatly
reduces ID
switches and
duplicate
tracks by using
object
descriptors
27 . Matches
objects across
agents even
when
geometry
alone is
uncertain. <br>
– Helps avoid
false merges:
dissimilar-
looking objects
won’t be paired
even if close
spatially
(reducing
coalescence).
<br> – Requires
that objects
have
distinguishable
features; less
effective if all
objects look
alike or
sensors
provide limited
features.
– Robustness:
Feature
matching adds
resilience to
occlusion and
reappearance
(an object
leaving one
agent’s view
can be
recognized by
another via
feature). <br> –
Doesn’t
directly
address fusion
loops, but by
improving
correct
associations, it
indirectly
prevents some
rumor issues
(fewer
spurious fused
tracks). <br> –
Tolerates
sensor mix:
feature can
encode sensor
modality
differences
(e.g., a camera
provides color,
LiDAR provides
shape – both
can be part of
feature vector).
<br> –
Network jitter:
as long as an
embedding
arrives with
the
observation,
matching can
occur even if
18
– Low to
Moderate:
Computing a
basic
embedding
(e.g., color
histograms,
object
dimensions) is
trivial on CPU.
A deep learned
embedding is
heavier, but
can possibly
reuse existing
model outputs
(e.g., use
detector’s
intermediate
layer). <br> –
Similarity
comparisons
are fast (dot
products).
Memory
overhead per
track is small
(tens of floats).
– Minimal
overhead:
Adds a small
feature vector
to each track’s
broadcast. Well
within
bandwidth
budget (e.g.,
32 bytes per
track). <br> –
No extra
communication
rounds, just
piggyback on
existing
packets.
– Fully
decentralized:
Each agent
generates its own
object features
and uses them in
matching. <br> –
No central
database needed;
embeddings are
shared peer-to-
peer. <br> – All
agents using the
same similarity
metric will agree
when an ID
matches by
feature. <br> –
Requires a
common feature
extraction
method deployed
on all agents
(must be agreed
upon/trained
beforehand, but
then runs
independently).Approach
Association &
ID Accuracy
Robustness
(Loops,
Delays)
Compute Load
Bandwidth
UseDecentralization
– None extra:
Uses
information
already in track
messages (the
UUIDs). <br> –
Doesn’t require
any special
packets or
overhead.– Ideal
decentralization:
Specifically
designed for no
leader, no
additional
comms. <br> – All
agents
independently
apply the same
rule and reach
the same result
(conflict-free
merging). <br> –
Essentially
behaves like a
built-in
consensus
algorithm
(eventual
consistency with
no comm cost).
slightly
delayed (the ID
might be
corrected once
feature is
received).
CRDT-Based ID
Consensus
<br>(Highlander
& variants)
– High ID
accuracy:
Ensures
eventually one
ID per object
16 . No
permanent
ghosts. <br> –
Does not
improve initial
association
accuracy, but
fixes divergent
IDs after the
fact. <br> –
Alternative
CRDT rules
(e.g., quality-
based) could
slightly
improve which
ID is chosen,
but not the
final outcome
(one ID).
– Robust to
loops/delays:
Being CRDT, it
handles out-of-
order
operations.
Even if ID
updates arrive
in any
sequence, all
nodes
converge
without
conflict. <br> –
No infinite
oscillation due
to monotonic
rule (IDs only
change in one
direction to a
stable value).
<br> – Handles
network
partitions
gracefully: if
agents get
split, they
might use
different IDs,
but upon
reconnection,
the rule will
unify them.
19
– Negligible:
Comparing
UUID strings
and updating a
set/variable is
$O(1)$. <br> –
Memory for
alias set is tiny
per track (a few
UUIDs).Fusion-Free
Deduplication
<br>(Tracklet
Threading)
– Association/
ID accuracy:
Duplicate IDs
are eliminated
by choosing
one track per
object. No
flicker once
ownership
decided (one
track is
reported). <br>
– If ownership
handoff is
done well, ID
persists across
agents views
(like a
continuous
track). <br> –
But if two
objects are
close and one
agent drops its
track thinking
the other will
track, there’s
risk of losing
one object
(needs robust
logic to avoid
merging real
distinct objects
– similar
challenge to
JPDA/cluster).
– Loop
robustness:
Highest –
avoids data
incest entirely
by not fusing
loops. No
covariance
blow-up; each
agent’s filter
only uses its
own sensor
data. <br> –
Delay
robustness: If
an update is
delayed, the
owner
continues with
its data; the
late data might
just be ignored
to prevent
jitter. Might
actually handle
delays
smoothly by
effectively
filtering out
stale
duplicates (the
late one might
not override
current
owner’s state).
<br> – On the
flip side, lack
of fusion
means if
owner loses
object and
another had it
but dropped
thinking owner
had it, the
object might
vanish until
20
– Low CPU
load: No fusion
computations,
just logic to
compare
confidences/
IDs. Each agent
runs only its
local Kalman
filter for each
object. <br> –
Slight
overhead in
managing
ownership
state and
possibly
running
multiple
parallel tracks
(if not
dropping non-
owner
immediately).
But those are
small.
– Could reduce
usage: If only
one agent
broadcasts an
object’s state
(others
suppress
theirs),
network traffic
for that object
goes down.
<br> –
However, might
require a small
flag or
heartbeat to
indicate
ownership.
This is minimal
(like a few
bytes or an
occasional
message). <br>
– Risk: if not all
duplicates
broadcast,
need robust
relaying so
information
still reaches all
agents (maybe
owner
broadcasts
suffice if within
range or via
multi-hop).
– Moderate
decentralization:
There is no fixed
central node, but
the per-object
leader concept
introduces a
dynamic
hierarchy. <br> –
Requires
agreement on
how ownership is
transferred – all
agents must
implement the
same rule to
avoid confusion
(e.g., all decide
that lowest UUID
or highest
confidence is
owner). <br> –
Potential for
temporary
disagreement:
two agents might
both think they
should be owner
until one “sees”
the other’s claim.
Highlander could
be leveraged
(e.g., smallest ID
agent becomes
owner by
default). <br> –
Still, overall
remains
decentralized
coordination, just
a bit more
complex than
pure CRDT or
everyone-equal
approach.Approach
Association &
ID Accuracy
Robustness
(Loops,
Delays)
Compute Load
Bandwidth
Use
Decentralization
reacquired
(handoff needs
to be flawless).
Table 1: Comparison of baseline and proposed approaches on key metrics.
From the above comparison, we can make a few observations:
• Accuracy & Robustness: Embedding-enhanced association and clustering stand out as promising
ways to reduce duplicate ghost tracks without heavy cost. JPDAF improves association in theory, but
its complexity and the risk of merging distinct objects make it a less attractive trade-off under our
constraints 20 . Fusion-free deduplication could eliminate ghosts and loops entirely, but must be
implemented carefully to avoid losing track coverage.
• Performance: The baseline is lightweight and safe but could be augmented in specific areas
(embedding, clustering) without significant performance loss. JPDAF, in contrast, could jeopardize
the real-time budget if overused 36 . Clustering algorithms and feature comparisons are well within
the CPU capabilities here, especially with spatial indexing and small data sizes.
• Bandwidth: All approaches except JPDAF do not substantially increase bandwidth and some
(ownership suppression in fusion-free, or simply fewer duplicate IDs being broadcast) could even
lower it. The baseline already was within limits at ~1.5 MB/s for 50 agents 37 ; we ensure none of
these ideas break that.
• Decentralization: Every suggested method has a path to implementation without a central server.
However, some (JPDAF, fusion-free leader election) require more careful distributed algorithm design
to ensure all agents stay in sync. Simpler enhancements like clustering and embeddings are
naturally decentralized and align well with the CRDT philosophy of the current system.
Integration with the Existing Rust + nalgebra + Zenoh Stack
Finally, we consider how to implement these improvements within the current Project GodView v3
architecture and codebase (Rust, nalgebra linear algebra, Zenoh pub-sub mesh). The existing system
already has modular components for gating, data association, fusion, and ID management 34 38 . We can
integrate alternatives as follows:
• JPDAF Integration: We would extend the TrackManager’s association step. Instead of the current
greedy loop that picks the single best match for an incoming packet 25 , we could accumulate all
valid associations within gating and compute assignment probabilities. A simple implementation
could use the Hungarian algorithm on negative likelihoods to get a best assignment, or for JPDA, use
a variant that gives multiple hypothesis weights. There are crates for linear assignment we could
leverage, or implement directly given the small sizes. nalgebra would handle the matrix math for
likelihood computations (e.g., calculating Mahalanobis distances and association probabilities).
21Because we want to possibly run JPDA only on a subset (cluster) of tracks, we might first detect
clusters of mutually close tracks/detections and run a JPDA solver per cluster (keeping complexity
bounded). The results (probabilistic association) would update track states. For the actual filter
update, we’d still rely on the existing Kalman filter in the Augmented EKF. JPDA would basically
provide a weighted measurement for each track (or a no-update if probability of any assignment is
too low). This can be done by augmenting the process_packet logic or as a separate function
that processes a batch of packets together. Because our system is event-driven (packets streaming
in), we might need to accumulate packets in a timestep and solve an assignment before processing
them, to truly do JPDA properly. This is a larger architectural change (moving from purely
asynchronous updates to a synced batch update per tick). If real-time constraints allow, it might be
feasible to buffer, say, 33 ms of data and process in one go at 30 Hz. If not, a simpler approximation
might be to do a limited JPDA: when a new packet arrives, check if it conflicts with an already recently
associated track (e.g., two measurements could belong to two tracks). That might degrade to rule-
based conflict resolution instead of full JPDA, but could still help in edge cases.
• Clustering Integration: This can be added as a post-processing step after all normal associations
and updates in a cycle. Imagine at the end of a frame, each agent has some set of active tracks
(some possibly newly created, some updated via fusion). We can then take all track states (position
vectors) and run a DBSCAN on them (perhaps using a utility function or crate, or even a simple
custom clustering since n is small). Using nalgebra, distance computations are straightforward
(vector subtraction and mahalanobis metric via covariance). We might convert covariance to an
“effective radius” for each track to feed into clustering (or directly compute pairwise Mahalanobis
distance). The clustering would yield groups of tracks. For each group where more than one track
exists, we then perform deduplication: ensure they have one canonical ID. In practice, the system
already does this via Highlander when tracks are associated, but clustering could catch cases where
two tracks didn’t associate (perhaps because each came in as new nearly simultaneously). We could
then manually unify them: pick the smallest UUID in the cluster and call track.canonical_id =
min_uuid for all. We would also likely want to fuse or remove duplicates here: perhaps designate
one track as primary and fuse others into it via CI, or just drop the extras. One strategy: for each
cluster, keep the track with the most confidence (or if equal, the one with smallest UUID to avoid
bias) and remove the others from self.tracks . Before removal, we could optionally fuse their
info into the kept track via fuse_with_ci to not lose information. But if rumor propagation is a
concern, we might skip fusion and simply drop them, relying on the primary track to carry on (this
ties to fusion-free idea). Implementation-wise, this is a relatively contained addition and could reuse
the nalgebra library for matrix ops if we do any fusion.
• Embedding Features: We’d need to extend the GlobalHazardPacket structure to include a
feature vector or some descriptor fields (it already has class_id, which is a simple categorical feature
39 ). We can define, for example, feature: [f32; N] or similar in the packet, and populate it in
the perception module that creates the packet (e.g., if using YOLO, attach the appearance
embedding from the detector). On the receiving end, we incorporate this into association. The
pseudocode snippet in the design can be extended: after computing Mahalanobis distance, also
compute feature distance if both track and packet have feature. For instance:
if dist < thresh {
let feat_sim = cosine_similarity(track.feature, packet.feature);
22if feat_sim < feature_threshold {
continue; // skip association if features mismatch significantly
}
// Otherwise consider it a valid match...
}
We might also store a representative feature in the UniqueTrack . Perhaps maintain an average or a
recent sample of the feature to account for view changes. nalgebra can help with vector operations like dot
products for similarity. The threshold can be tuned (or even dynamic: e.g., require higher similarity if
Mahalanobis distance is large). If multiple feature modalities (color, shape, motion), one could form a
combined feature vector. This integration is local and doesn’t require global changes. All agents just need to
use the same method of computing features for consistency. If a learned model is used, we’d embed that
model in the code or as a compiled module so that Rust can call it (possibly using tract or tch crates for
neural nets if needed). But a simpler approach could avoid heavy dependencies by using basic features.
Testing would be needed to ensure that feature gating doesn’t falsely reject true matches (we’d set it
relatively permissive, only blocking obvious mismatches).
• CRDT and ID Management: The existing Highlander heuristic is implemented and we plan to keep it
40 . If we wanted to experiment with an ownership or quality-based ID leader, we could do it by
piggybacking on the alias set or track metadata. For example, we could add an owner_agent_id
in the track data. Initially,
owner_agent_id = self
(the agent itself for its tracks). When
associating a packet from another agent, we could include logic: if the other agent’s track has lower
covariance, maybe set the owner to that agent’s ID. Then, the agent would refrain from overwriting
certain fields or from sending its track if it’s not the owner. However, implementing this fully in a truly
decentralized way might be complex; all agents would have to do similar and converge. Perhaps
easier is a post-fusion suppression: after fusing tracks, if two tracks merged, only allow the one
with min ID to actually broadcast. We can achieve that by having a rule: if a track’s canonical_id is not
equal to its native UUID and it did not originate locally, do not broadcast it. Essentially, once an agent
knows an object is being tracked by someone with a smaller ID, it stops advertising its redundant
track. This could be implemented by checking track.canonical_id != track.local_uuid
before publishing a track update. This way, eventually only the agent whose UUID ended up as
canonical (the smallest) will broadcast that object. This uses Highlander’s result as a proxy for
ownership. It’s simple: min-ID agent “wins” broadcasting rights. This would implement a form of
fusion-free deduplication automatically: non-min tracks fade out, leaving one source. One must
ensure that if the min-ID agent goes out of range or loses the object, others can resume
broadcasting (which they would once they no longer receive updates; they’d time-out and consider
the object lost, perhaps then treat their local track as a “new” object with a new ID which again will
propagate – a bit of a hiccup but not worse than currently losing an object). This scheme is nicely
decentralized and relies only on the CRDT outcome. We’d need to be careful that every object has at
least one broadcaster at all times – so maybe don’t suppress if the “winning” agent’s data hasn’t been
heard in a while. Perhaps a timeout: if I haven’t heard an update for ID_X from the supposed owner
for T seconds, I resume broadcasting my track to ensure the object isn’t lost. That becomes a guard
against dropouts.
• Testing and Verification: With these changes, we’d test scenarios: multiple agents observing
crossing targets (to test JPDA vs clustering differences), long-range vs short-range sensor mix (test
Mahalanobis + feature gating), and loop scenarios where an observation goes around a loop to see if
23CI or suppression correctly handles it. The uploaded design spec already includes a “Sanity Check”
reference 41 – we would augment those tests to compare new methods.
In terms of architectural diagram, the modified pipeline would look like this: each agent’s TrackManager
now has an enhanced Association module (incorporating clustering/JPDA and feature similarity) feeding
into either the Fusion module (CI as before, unless we choose not to fuse in some cases) followed by an
Identity Resolution module (Highlander CRDT, potentially with an owner suppression twist). The output
tracks are then broadcast over Zenoh to peers. The peers do the same, and via the CRDT and perhaps
suppression rules, they converge on one track per object.
We can summarize the integration in pseudocode form, focusing on the new parts (for clarity we omit
existing EKF prediction which remains unchanged):
// Pseudocode: Enhanced track processing loop (runs per incoming data frame or
batch)
for each incoming measurement packet (could be a local detection or remote
agent's track) {
// Compute Mahalanobis distance to each existing track, gather candidates
let mut associates = [];
for track in self.tracks {
if track.class != packet.class { continue; } // class gating
let dist = mahalanobis(track.state, track.cov, packet.position);
if dist < GATING_THRESHOLD {
let feat_sim = similarity(track.feature, packet.feature);
if feat_sim < FEATURE_SIM_THRESH {
continue; // feature gating: skip if not similar enough
}
associates.push((track.id, dist, feat_sim));
}
}
if associates.is_empty() {
// No association -> initialize new track
new_track = Track::from(packet);
self.tracks.insert(new_track.id, new_track);
} else {
// Potential associations found.
// If more than one track qualifies (rare in GNN, but possible if gates
overlap):
// use a clustering/JPDA strategy to decide uniquely, or even allow
multiple update.
let chosen_track_id;
if associates.length == 1 {
chosen_track_id = associates[0].track_id;
} else {
// e.g., choose the one with smallest Mahalanobis or combine updates
probabilistically
24chosen_track_id = choose_via_JPDA(associates);
}
// Perform update (fusion)
let track = self.tracks[chosen_track_id];
if FUSION_FREE_MODE {
// If opting not to fuse, we might skip state fusion.
// Possibly update timestamp or simple things but not state.
track.last_seen = packet.timestamp;
} else {
// Covariance Intersection fusion with incoming data as a pseudo-
track
track.fuse_with_CI(packet.state, packet.cov);
}
// Update feature (e.g., running average)
track.feature = blend(track.feature, packet.feature);
// Highlander ID merge:
if packet.uuid < track.canonical_id {
track.canonical_id = packet.uuid;
}
track.observed_ids.insert(packet.uuid);
}
}
// After processing all packets, optional clustering to catch any duplicates:
clusters = DBSCAN(self.tracks.values(), epsilon = dynamic_epsilon);
for cluster in clusters {
if cluster.size > 1 {
let canonical = min_uuid(cluster);
for track in cluster {
if track.canonical_id != canonical {
// unify IDs
track.canonical_id = canonical;
}
}
if FUSION_FREE_MODE {
// elect one track to keep
primary = choose_best_track(cluster);
for track in cluster {
if track != primary {
self.tracks.remove(track.id);
}
}
} else {
// fuse states of tracks in cluster into one (using CI sequentially)
primary = cluster.first();
for track in cluster.rest() {
primary.fuse_with_CI(track.state, track.cov);
self.tracks.remove(track.id);
}
25}
}
}
// Before broadcasting tracks, apply suppression rule if using ownership via
IDs:
for track in self.tracks {
if track.canonical_id != track.local_id {
// This track was merged into another ID, so don't broadcast it unless
we are the canonical owner.
if track.local_id != track.canonical_id {
suppress(track);
}
}
else {
broadcast(track);
}
}
This pseudocode is a sketch combining various options (JPDA, clustering, fusion-free flag, etc.). In
implementation, we’d choose a specific combination to deploy (likely: clustering + CI fusion + highlander +
embed features, or fusion-free + highlander + embed).
Rust considerations: Rust’s ownership and real-time performance aligns well with these changes. nalgebra
provides matrix operations for computing Mahalanobis distance and performing CI (in the current code,
fuse_with_ci presumably uses nalgebra for matrix inversion and combination 38 ). We may introduce
new data structures (like a KD-tree or use the existing H3 spatial index) for clustering efficiently. Zenoh
messaging would include any new fields (feature, maybe an “owner” flag if needed). Ensuring thread-safety
and no heavy allocations is important for performance; the design doc noted using stack-allocated matrices
to avoid heap fragmentation 14 , which we will continue doing for any new linear algebra as well.
In summary, integrating these alternatives is feasible within the current stack. The architecture remains
distributed and event-driven, but we enhance the logic inside each agent’s tracking module. The result
should be a more robust global perception: fewer duplicate ghosts, more stable IDs, and no
overconfident covariances even under high network latency or looped communications. Each agent will
contribute to a coherent “HTTP of Reality” model where hazards are consistently identified and tracked
across the swarm of vehicles, fulfilling the vision of Project GodView v3 with greater reliability.
1
2
7
8
9
18
19
20
28
36
37
Distributed Data Association Problem Analysis2 .pdf
file://file_000000004bd4720cad43e1c1b3ee70db
3
4
5
6
10
11
12
13
14
15
16
17
25
29
30
31
Fusion Algorithm Design (1).pdf
file://file_00000000d210722f9b1f4febcdd59b00
21
22
pure.uva.nl
https://pure.uva.nl/ws/files/4318772/59155_thesis.pdf
26
32
33
34
35
38
39
40
41
Decentralized Track23 24 26 Distributed Multi-Target Tracking with D-DBSCAN Clustering - Korea Advanced Institute of
Science and Technology
https://pure.kaist.ac.kr/en/publications/distributed-multi-target-tracking-with-d-dbscan-clustering/
27
The Multi-Camera Tracking AI Workflow | NVIDIA
https://www.nvidia.com/en-us/ai-data-science/ai-workflows/multi-camera-tracking/
27
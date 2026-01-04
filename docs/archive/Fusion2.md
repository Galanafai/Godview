Technical Specification: Distributed Data Association & Fusion for Project GodView v3
To: Project GodView Engineering Team
From: Principal Sensor Fusion Engineer
Subject: Architecture and Implementation of the Decentralized TrackManager Module
Date: October 26, 2025
1. Executive Summary
Project GodView v3 represents a paradigm shift in autonomous perception, moving from centralized data processing to a decentralized, cooperative mesh network. We have successfully established the foundational pillars of this architecture: the Augmented State Extended Kalman Filter (AS-EKF) for latency compensation, the Hierarchical Hybrid Sharding (H3 + Octree) for spatial indexing, and Capability-Based Access Control (CapBAC) for security. However, the system currently exhibits a critical failure mode in the presentation and consensus layer: the "Duplicate Ghost" phenomenon. This artifact arises when spatially overlapping observations from distinct agents fail to coalesce into a single semantic entity, degrading the user experience from a coherent world model to a chaotic cloud of flickering, redundant objects.
This report articulates the mathematical and architectural solution to the Distributed Data Association problem. Unlike centralized systems where a global server assigns unique identifiers (Ground Truth IDs), our decentralized topology requires a consensus mechanism that operates without a single source of truth. The challenge is not merely determining that two observations are the same, but mathematically combining them into a fused estimate while managing the uncertainty correctly in a network prone to data looping.
The proposed solution leverages Mahalanobis Distance for statistically valid gating, Global Nearest Neighbor (GNN) for lightweight association, and Covariance Intersection (CI) for fusion. Crucially, CI is selected over standard Kalman fusion to mitigate "rumor propagation"—the mathematical incest that occurs when agents recirculate fused estimates in a mesh network. Furthermore, we define a deterministic ID Resolution Strategy (The "Highlander" Heuristic) to ensure all peers independently converge on a single stable ID for any given object, eliminating visual flickering without network negotiation.
This document serves as the comprehensive design specification for the TrackManager module, detailing the theoretical basis, the mathematical derivation of fusion algorithms, the logic for identity management, and the concrete Rust implementation required to deploy this solution on consumer-grade hardware (GTX 1050 Ti) constraints.
2. Theoretical Framework: The Physics of Decentralized Fusion
In a centralized architecture, data association is primarily a matching problem. A central server receives all inputs, compares them against a master list, and updates the state. In a decentralized architecture, however, data association becomes a consensus problem. When Agent A and Agent B observe the same pedestrian, they generate independent estimates $\hat{x}_A$ and $\hat{x}_B$ with independent UUIDs. The challenge is not merely determining that $\hat{x}_A \approx \hat{x}_B$, but mathematically combining them into $\hat{x}_{fused}$ while managing the uncertainty $P_{fused}$ correctly.
The fundamental difficulty lies in the fact that Agent A and Agent B are not independent observers in the long run; they share information. If Agent A sends a track to Agent B, and Agent B updates its local state and transmits back to the network, Agent A might receive this "new" information and treat it as an independent confirmation of its own prior observation. This leads to catastrophic overconfidence in the filter.
2.1 The "Rumor Propagation" Problem
A naive approach to decentralized fusion would be to employ a standard Kalman Filter update, often referred to in this context as the Information Filter. The Information Filter sums the information matrices (inverse covariances) of the incoming measurements.

$$P_{new}^{-1} = P_{1}^{-1} + P_{2}^{-1}$$

$$P_{new}^{-1}\hat{x}_{new} = P_{1}^{-1}\hat{x}_{1} + P_{2}^{-1}\hat{x}_{2}$$
This equation is mathematically rigorous if and only if the estimation errors of track 1 and track 2 are mutually independent (uncorrelated). However, in a mesh network like Zenoh, where agents constantly gossip state updates, this independence assumption is violated almost immediately.
Consider a scenario where Agent A observes a hazard at position $P$ with variance $\sigma^2$. It broadcasts this to Agent B. Agent B fuses this into its map. Later, Agent C requests a map from Agent B. Agent B sends the fused track. Agent C then broadcasts this track. Agent A receives Agent C's broadcast. If Agent A uses the standard Information Filter, it will treat this "new" report as an independent validation, effectively squaring the information and halving the variance. If this cycle repeats, the covariance matrix $P$ shrinks asymptotically to zero. The filter becomes convinced it has infinite precision, eventually ignoring all real sensor data because the "rumor" is perceived as overwhelmingly accurate. This phenomenon is known as rumor propagation, data incest, or double counting.1
The Solution: Covariance Intersection (CI)
To solve this, we employ the Covariance Intersection algorithm, originally developed by Julier and Uhlmann. CI provides a consistent estimate even when the correlation between two estimates is completely unknown.2 Unlike the standard Kalman update which sums information matrices, CI computes a weighted convex combination of the information.
The fused covariance $P_{CI}$ and state $\hat{x}_{CI}$ are calculated as:

$$P_{CI}^{-1} = \omega P_A^{-1} + (1 - \omega) P_B^{-1}$$

$$P_{CI}^{-1} \hat{x}_{CI} = \omega P_A^{-1} \hat{x}_A + (1 - \omega) P_B^{-1} \hat{x}_B$$
Where $\omega \in $ is a weighting factor. This formulation guarantees that the resulting covariance ellipsoid encloses the intersection of the two input ellipsoids, regardless of the actual correlation between them. This ensures the fused covariance is a conservative upper bound of the true error, guaranteeing safety and stability in our decentralized loop.4 Even if Agent A fuses its own data fed back to it, CI will simply recognize that the information is redundant and assign an appropriate $\omega$ (likely near 1 or 0), preventing the artificial collapse of uncertainty.
2.2 Gating Logic: Why Euclidean is Insufficient
The first step in data association is "gating"—deciding which tracks are candidates for fusion. Using a simple Euclidean distance threshold (e.g., "merge if < 2 meters") is dangerous and insufficient for dynamic agents with heterogeneous sensors.
Consider two scenarios:
Scenario 1: A GPS sensor with a 5-meter error radius estimates a position.
Scenario 2: A LiDAR sensor with a 0.1-meter error radius estimates a position.
A 2-meter deviation is statistically insignificant for the GPS sensor (it is well within the noise floor), but it is a massive deviation for the LiDAR sensor. Euclidean distance ignores this "shape" of uncertainty. It treats the error space as a sphere, whereas real sensor error is an ellipsoid defined by the covariance matrix $P$.5
We must use the Mahalanobis Distance ($D_M$), which measures how many standard deviations away a point is from a distribution. It effectively normalizes the Euclidean distance by the covariance of the distributions.

$$D_M(x) = \sqrt{(z - Hx)^T S^{-1} (z - Hx)}$$
Where:
$z$ is the measurement (from the incoming packet).
$x$ is the state of the local track.
$H$ is the observation matrix.
$S$ is the residual covariance, typically $H P H^T + R$, where $P$ is the track covariance and $R$ is the measurement noise.
The Mahalanobis distance creates a gating volume that adapts to the uncertainty of the track. If the track is very uncertain, the gate expands; if it is precise, the gate contracts. We accept a match if $D_M^2 < \gamma$, where $\gamma$ is a threshold derived from the Chi-squared distribution. For a 3D position (3 degrees of freedom), a $\gamma$ of approximately 7.815 corresponds to a 95% confidence interval.7 This ensures we fuse data that is statistically likely to belong to the same object, rather than just spatially close.
2.3 Optimization of Fusion Weights
In the Covariance Intersection algorithm, the parameter $\omega$ determines the relative weight of the two estimates. The choice of $\omega$ is an optimization problem. We seek to minimize the size of the resulting covariance $P_{CI}$. There are two common metrics for the "size" of a matrix: the determinant (related to the volume of the error ellipsoid) and the trace (related to the sum of the variances).8
Determinant Minimization: Minimizing the determinant minimizes the volume of the uncertainty ellipsoid. This is information-theoretically rigorous but computationally expensive, often requiring iterative solvers.
Trace Minimization: Minimizing the trace minimizes the sum of the eigenvalues, which corresponds to the total mean squared error. This is generally preferred for tracking applications because it is robust and often has closed-form approximations or simpler convex properties.9
For Project GodView v3, running on constrained hardware (GTX 1050 Ti, but the logic is CPU-bound), we opt for Trace Minimization using a fast, closed-form approximation or a coarse Golden Section Search. This avoids the overhead of a full convex optimizer in the hot loop of the track manager. The "Fast CI" approximation suggests computing $\omega$ based on the relative traces of the input covariances:

$$\omega \approx \frac{\text{tr}(P_B)}{\text{tr}(P_A) + \text{tr}(P_B)}$$
This heuristic assigns higher weight to the covariance with the smaller trace (lower uncertainty), which aligns with intuitive expectations for sensor fusion.10
3. Algorithmic Design: The TrackManager
The TrackManager is the core Rust module responsible for ingesting GlobalHazardPackets and emitting UniqueTracks. It sits between the Zenoh network listener and the Visualization/Path-Planning layer. Its primary duty is to maintain a local, coherent state of the world despite the chaotic, asynchronous nature of the incoming data stream.
3.1 The Processing Pipeline
The lifecycle of a packet within the TrackManager follows a strict sequential pipeline designed for performance and correctness.
Ingest & Transcode: The module receives a GlobalHazardPacket via Zenoh. This packet contains WGS84 coordinates (Lat/Lon/Alt). These must be immediately converted to the local Cartesian system (ECEF or Local Tangent Plane) used by the tracking math. This ensures that all distance calculations are performed in a metric space.
Prediction (Time Alignment): The tracks currently in the local store are likely timestamped at $t_{last}$. The incoming packet is at $t_{now}$. Before association can occur, we must propagate the state of all local tracks forward to $t_{now}$ using the motion model (Constant Velocity). This aligns the "apples to apples" comparison.
Spatial Indexing (Broad Phase): A naive $O(N^2)$ comparison of all tracks to the new packet is inefficient. We utilize the H3 spatial index. The packet's location maps to an H3 cell. We query our local track store for tracks residing in that cell and its immediate neighbors (k-ring 1). This reduces the candidate set significantly.11
Gating & Association (Narrow Phase): For each candidate track, we compute the Mahalanobis distance. If the distance falls within the Chi-squared gate, the track is considered a match candidate.
Assignment Strategy: For low-to-moderate object density, a Greedy Global Nearest Neighbor (GNN) approach is sufficient and highly performant. We sort candidates by Mahalanobis distance and assign the best matches first. While the Hungarian Algorithm (Munkres) offers optimal assignment, its $O(N^3)$ complexity is unnecessary given the high update rate (10Hz+) and spatial sparsity of typical traffic scenes. GNN is $O(N \log N)$ or better with spatial hashing.
Fusion: Once a track is associated with the packet, we perform the Covariance Intersection update. The track's state and covariance are updated to reflect the new information, fusing the high-confidence local data with the potentially lower-confidence remote data (or vice versa).
Identity Resolution: If a match is found, we must decide which UUID to display. This is the critical step for solving the "Flickering Ghost" problem.
Track Management:
Creation: If no match is found, and the packet's confidence is high enough, a new track is initialized.
Pruning: Tracks that have not been updated for a certain duration (e.g., 2 seconds) are deleted. This handles objects leaving the scene.
3.2 ID Management: The "Highlander" Heuristic
In a decentralized system, we cannot rely on a central registry to dispense IDs. When Agent A and Agent B encounter the same object, they will initially assign it different UUIDs (e.g., UUID_A and UUID_B). As their tracks converge spatially, the system recognizes them as the same physical entity, but they carry different metadata. If we simply toggle between them based on the latest packet, the object will flicker on screen.
To solve this, we implement the "Highlander" Heuristic: There can be only one.
The Logic:
Every UniqueTrack structure maintains a set of observed_uuids—a history of all the aliases this object has been known by.
When a fusion occurs between a local track (ID L) and an incoming packet (ID R):
The fusion math updates the physical state.
The ID logic compares L and R.
Deterministic Selection: The system deterministically selects the Canonical ID as the lexicographically smallest UUID among all IDs seen for this track (min(L, R,...)). Alternatively, one could use the UUID with the oldest timestamp (First Discovery), but lexicographical comparison of UUID bytes is faster and strictly deterministic.
Convergence: Even if Agent A and Agent B start with different IDs, as soon as they exchange packets, they will both perform the same comparison. If UUID_A < UUID_B, Agent B will swap its display ID to UUID_A. Agent A will keep UUID_A. Both agents converge to the same identifier without requiring a 3rd party arbitrator.
This strategy effectively treats the UUID space as a CRDT (Conflict-free Replicated Data Type), where the merge operation is defined by the min() function. This guarantees eventual consistency across the entire network.12
4. Rust Implementation Specification
The implementation relies on the nalgebra crate for high-performance linear algebra. nalgebra is preferred over ndarray for this application because it supports statically sized matrices (Matrix6, Vector6), which allows for stack allocation. This is crucial for minimizing memory fragmentation and garbage collection overhead in a real-time system running on limited hardware.14
4.1 Struct Definitions
We define the state space as a 6-dimensional vector: $[x, y, z, v_x, v_y, v_z]$. Consequently, the covariance matrix is $6 \times 6$.

Rust


use nalgebra::{Matrix6, Vector6, U6};
use std::collections::{HashMap, HashSet};
use uuid::Uuid;
use std::time::Instant;

// Type aliases for clarity and ease of refactoring
type StateVector = Vector6<f64>;
type CovarianceMatrix = Matrix6<f64>;

/// The raw packet received from the Zenoh network.
/// Represents a single observation from a remote agent.
#
pub struct GlobalHazardPacket {
    pub entity_id: Uuid,
    pub position: [f64; 3], // ECEF coordinates [x, y, z]
    pub velocity: [f64; 3], // [vx, vy, vz]
    pub class_id: u8,       // e.g., 1=Car, 2=Pedestrian
    pub timestamp: f64,     // Unix timestamp
    
    // Confidence score (0.0 to 1.0).
    // In a real protocol, we might send the full covariance matrix if bandwidth permits.
    // If not, we reconstruct a matrix from this scalar.
    pub confidence_score: f64, 
}

/// The internal representation of a fused object in the local world model.
#
pub struct UniqueTrack {
    pub canonical_id: Uuid,        // The "Winning" ID used for display/rendering
    pub observed_ids: HashSet<Uuid>, // All aliases associated with this track (deduplication)
    
    pub state: StateVector,        // The fused state [x, y, z, vx, vy, vz]
    pub covariance: CovarianceMatrix, // The fused uncertainty matrix
    
    pub last_update: f64,          // Timestamp of the last valid fusion
    pub class_id: u8,
}

/// The main module managing the lifecycle of tracks.
pub struct TrackManager {
    // The local state store. Maps Canonical IDs to Tracks.
    tracks: HashMap<Uuid, UniqueTrack>,
    
    // Tuning parameters
    gating_threshold: f64, // Chi-squared threshold (e.g., 12.59 for 6DOF, 95%)
    base_pos_variance: f64, // Baseline variance for position if not provided
    base_vel_variance: f64, // Baseline variance for velocity
}


4.2 Gating Logic (Mahalanobis Distance)
The gating function is critical for rejecting outliers. We implement the squared Mahalanobis distance calculation. Note that if the incoming packet does not provide a full covariance matrix, we must construct one synthetically based on its confidence_score.
Construction of Measurement Covariance ($R$):
We map the scalar confidence ($C \in $) to variance. A simple heuristic is an inverse relationship: $\sigma^2 \propto (1 - C)$.

Rust


impl TrackManager {
    /// Calculate Mahalanobis distance squared between a track and a measurement.
    /// D^2 = (z - x)^T * S^-1 * (z - x)
    fn mahalanobis_distance(&self, track: &UniqueTrack, meas: &GlobalHazardPacket) -> f64 {
        // 1. State Difference (Residual) z - x
        let meas_vec = Vector6::new(
            meas.position, meas.position, meas.position,
            meas.velocity, meas.velocity, meas.velocity
        );
        let residual = meas_vec - track.state;

        // 2. Innovation Covariance S = P_track + R_meas
        // We assume independence here for the *gating* step to see if they *could* match.
        // The actual fusion (CI) will handle the correlation.
        let r_meas = self.confidence_to_covariance(meas.confidence_score);
        let s_matrix = track.covariance + r_meas;

        // 3. Invert S
        // try_inverse returns None if the matrix is singular (determinant is 0).
        match s_matrix.try_inverse() {
            Some(s_inv) => {
                // d^2 = residual^T * S^-1 * residual
                // The result is a 1x1 matrix (scalar).
                let d_squared = residual.transpose() * s_inv * residual;
                d_squared // Extract scalar
            }
            None => f64::MAX, // Singularity implies infinite distance or invalid state
        }
    }

    /// Helper to convert scalar confidence (0.0-1.0) to a Covariance Matrix.
    /// This allows us to handle packets that don't transmit full 6x6 matrices.
    fn confidence_to_covariance(&self, score: f64) -> CovarianceMatrix {
        // Heuristic: Higher score -> Lower variance.
        // We clamp the score to avoid division by zero or negative variance.
        let inv_score = (1.0 - score).clamp(0.001, 0.999);
        
        let pos_var = self.base_pos_variance * inv_score; 
        let vel_var = self.base_vel_variance * inv_score;
        
        let mut cov = Matrix6::zeros();
        // Diagonal covariance matrix (assuming uncorrelated x,y,z errors for the heuristic)
        cov[(0,0)] = pos_var; // x
        cov[(1,1)] = pos_var; // y
        cov[(2,2)] = pos_var; // z
        cov[(3,3)] = vel_var; // vx
        cov[(4,4)] = vel_var; // vy
        cov[(5,5)] = vel_var; // vz
        cov
    }
}


4.3 Fusion Math: Covariance Intersection (CI)
This implementation uses the Fast Covariance Intersection method approximating the optimal weight $\omega$ by minimizing the trace. This avoids numerical optimization loops.8
The standard covariance update for CI is:
$P_{CI} =^{-1}$
However, inverting three times ($P_A, P_B, P_{sum}$) is expensive. We can rewrite the state update to be more numerically stable.

Rust


impl UniqueTrack {
    /// Fuses this track with a new measurement using Covariance Intersection.
    pub fn fuse_with_ci(&mut self, meas_state: &StateVector, meas_cov: &CovarianceMatrix) {
        let p_a = self.covariance; // Current Track Covariance
        let p_b = *meas_cov;       // Measurement Covariance
        
        // 1. Calculate Omega (Fast Trace Minimization Heuristic)
        // A common approximation for trace minimization is:
        // omega = tr(P_B) / (tr(P_A) + tr(P_B))
        // This weights the "smaller" (more precise) covariance higher.
        let tr_a = p_a.trace();
        let tr_b = p_b.trace();
        let sum_tr = tr_a + tr_b;
        
        // Prevent division by zero
        let omega = if sum_tr > 1e-9 {
            tr_b / sum_tr
        } else {
            0.5 // Default to equal weight if both are singular/zero
        };

        // 2. Compute Fused Covariance
        // P_c^-1 = w * P_a^-1 + (1-w) * P_b^-1
        let p_a_inv = p_a.try_inverse().unwrap_or_else(|| Matrix6::identity()); 
        let p_b_inv = p_b.try_inverse().unwrap_or_else(|| Matrix6::identity());

        let p_c_inv = p_a_inv * omega + p_b_inv * (1.0 - omega);
        
        // We invert back to get the final Covariance P_c
        let p_c = p_c_inv.try_inverse().unwrap_or(p_a); // Fallback to P_a if fusion fails

        // 3. Compute Fused State
        // x_c = P_c * (w * P_a^-1 * x_a + (1-w) * P_b^-1 * x_b)
        // We calculate the weighted information vectors first.
        let info_a = p_a_inv * self.state * omega;
        let info_b = p_b_inv * meas_state * (1.0 - omega);
        
        self.state = p_c * (info_a + info_b);
        self.covariance = p_c;
        
        // Update timestamp to current
        // Note: In a real system, you might interpolate if timestamps differ significantly.
        // Here we assume prediction was already run to align timestamps.
    }
}


4.4 Association & ID Resolution (The Manager Logic)
This function orchestrates the logic: matching, fusing, and resolving IDs.

Rust


impl TrackManager {
    pub fn process_packet(&mut self, packet: GlobalHazardPacket) {
        // 1. Predict all tracks to packet timestamp (Motion Model)
        // (Skipped for brevity: involves F * state, F * P * F^T + Q)
        // self.predict_tracks(packet.timestamp);

        let mut best_match_id: Option<Uuid> = None;
        let mut min_dist = self.gating_threshold;

        // 2. Association (Greedy Nearest Neighbor)
        // In production, query a spatial index (Octree/H3) first to get candidates.
        for (track_id, track) in &self.tracks {
            // Hard Gating: Class ID must match
            if track.class_id!= packet.class_id { continue; }

            // Soft Gating: Mahalanobis Distance
            let dist = self.mahalanobis_distance(track, &packet);
            
            // Check threshold and find the closest valid match
            if dist < min_dist {
                min_dist = dist;
                best_match_id = Some(*track_id);
            }
        }

        match best_match_id {
            Some(id) => {
                // MATCH FOUND
                if let Some(track) = self.tracks.get_mut(&id) {
                    let meas_cov = self.confidence_to_covariance(packet.confidence_score);
                    let meas_state = Vector6::new(
                        packet.position, packet.position, packet.position,
                        packet.velocity, packet.velocity, packet.velocity
                    );

                    // Step 5: Fusion (Covariance Intersection)
                    track.fuse_with_ci(&meas_state, &meas_cov);

                    // Step 6: ID Resolution (Highlander Heuristic)
                    // If the packet's ID is "smaller" (lexicographically) than our current canonical,
                    // we switch to it. This ensures eventual consistency across the network.
                    if packet.entity_id < track.canonical_id {
                         track.canonical_id = packet.entity_id;
                    }
                    // Record that we have seen this ID associated with this track
                    track.observed_ids.insert(packet.entity_id);
                    track.last_update = packet.timestamp;
                }
            },
            None => {
                // NO MATCH: Create new track
                let mut new_track = UniqueTrack {
                    canonical_id: packet.entity_id,
                    observed_ids: HashSet::new(),
                    state: Vector6::new(
                        packet.position, packet.position, packet.position,
                        packet.velocity, packet.velocity, packet.velocity
                    ),
                    covariance: self.confidence_to_covariance(packet.confidence_score),
                    last_update: packet.timestamp,
                    class_id: packet.class_id,
                };
                new_track.observed_ids.insert(packet.entity_id);
                
                // Insert into our map
                self.tracks.insert(packet.entity_id, new_track);
            }
        }
    }
}


5. Insight & Justification
5.1 Why This Solves the "Duplicate Ghost" Problem
The architectural solution specifically targets the two root causes of "ghosts":
Geometric Ambiguity: Euclidean distance fails when mixing high-precision (LiDAR) and low-precision (Camera/GPS) data. A 2-meter gap is "close" for GPS but "far" for LiDAR. Mahalanobis distance uses the covariance matrix to normalize this error. It essentially asks, "Is this point within the probability cloud of the track?" rather than "Is this point within X meters?" This allows the system to correctly associate a "fuzzy" observation with a "sharp" track.7
ID Divergence: In decentralized systems, split-brain ID generation is inevitable. The "Highlander" heuristic (min-UUID wins) acts as a Conflict-free Replicated Data Type (CRDT) merge strategy. It forces all agents observing the same physical object to eventually agree on the same ID without needing a leader election or central server. While Agent A might display ID A for a few milliseconds, as soon as it receives a packet with ID B (where B < A) and associates it with the track, it snaps to B. Agent B, seeing A, ignores it (since B < A). Thus, the network converges.
5.2 Why Covariance Intersection over Kalman Fusion?
Standard Kalman Fusion is optimal only for independent errors. In a Zenoh mesh, data loops are frequent (A -> B -> C -> A). Using standard fusion here creates a positive feedback loop where the covariance shrinks to zero, causing the filter to reject new, valid maneuvers (divergence). CI is conservative. By computing a convex combination, it ensures that even if two sources are fully correlated (one is a copy of the other), the fused result is no more confident than the best single source. This guarantees stability in the mesh network.
5.3 Rust & Performance Considerations
The use of nalgebra's stack-allocated types prevents the heap fragmentation that plagues many linear algebra libraries in embedded contexts. The Fast-CI heuristic is $O(1)$ and involves basic arithmetic operations, avoiding the heavy iterative solvers usually required for CI. This ensures the logic remains lightweight enough for the target GTX 1050 Ti environment (which implies a shared memory budget with rendering). The gating step, while theoretically $O(N^2)$, is effectively $O(N)$ when combined with the existing H3 spatial index, as we only compare tracks within a constant number of hexagonal cells.
6. Recommendations for Deployment
Metric Tuning: The gating_threshold (Chi-square) should be tuned based on the degrees of freedom. For 3D position only, use $\sim7.8$ (95%). For full 6D state (Pos + Vel), use $\sim12.6$.
Hysteresis: Implement a "zombie state" for tracks. When a track stops receiving updates, do not delete it immediately. Coast it for 1-2 seconds using the velocity vector. This handles temporary occlusions (e.g., a truck passing between the sensor and the target).
Visualization Smoothing: The internal fused state updates discretely (e.g., at 10Hz or whenever packets arrive). The rendering engine should not snap the object to the new coordinate immediately. Instead, it should interpolate from the current visual position to the new fused state using the velocity vector to ensure smooth motion at 60fps.
This specification provides a rigorous, mathematically sound foundation for the "Hidden Boss" of Project GodView v3, transforming disjointed decentralized observations into a coherent, single-truth reality.
Comparison of Fusion Architectures:
Feature
Standard Kalman Filter
Covariance Intersection (Proposed)
Assumption
Independent Errors
Unknown Correlation
Math
Sum of Information ($P^{-1}_1 + P^{-1}_2$)
Weighted Sum ($\omega P^{-1}_1 + (1-\omega)P^{-1}_2$)
Network Topology
Star / Tree (No Loops)
Mesh / P2P (Loops Allowed)
Failure Mode
Overconfidence / Divergence
Conservative Estimate
Consistency
Inconsistent in loops
Guaranteed Consistent

End of Report.
Works cited
Track-to-Track Fusion for Automotive Safety Applications - MATLAB & Simulink - MathWorks, accessed December 20, 2025, https://www.mathworks.com/help/driving/ug/track-to-track-fusion-for-automotive-safety-applications.html
Chapter 12: General Decentralized Data Fusion with Covariance Intersection (CI) - DSP-Book, accessed December 20, 2025, https://dsp-book.narod.ru/HMDF/2379ch12.pdf
Inverse Covariance Intersection: New Insights and Properties - KIT - ISAS, accessed December 20, 2025, https://isas.iar.kit.edu/pdf/Fusion17_Noack.pdf
Covariance intersection - Wikipedia, accessed December 20, 2025, https://en.wikipedia.org/wiki/Covariance_intersection
Mahalanobis Distance - Understanding the math with examples (python), accessed December 20, 2025, https://www.machinelearningplus.com/statistics/mahalanobis-distance/
Bottom to top explanation of the Mahalanobis distance? - Cross Validated, accessed December 20, 2025, https://stats.stackexchange.com/questions/62092/bottom-to-top-explanation-of-the-mahalanobis-distance
4.4 - Multivariate Normality and Outliers, accessed December 20, 2025, https://online.stat.psu.edu/stat505/book/export/html/679
Closed-form Optimization of Covariance Intersection for Low-dimensional Matrices - KIT - ISAS, accessed December 20, 2025, https://isas.iar.kit.edu/pdf/Fusion12_Reinhardt-FastCI.pdf
A Fast Covariance Union Algorithm for Inconsistent Sensor Data Fusion - IEEE Xplore, accessed December 20, 2025, https://ieeexplore.ieee.org/iel7/6287639/9312710/09585120.pdf
SANDIA REPORT Analysis of Covariance Intersection For Triangulation - OSTI, accessed December 20, 2025, https://www.osti.gov/servlets/purl/2429882
Sanity Check for Project GodView v3_ Distributed Perception Architecture.pdf
Decentralized Identity: The Ultimate Guide 2025 - Dock Labs, accessed December 20, 2025, https://www.dock.io/post/decentralized-identity
One-Shot Multiple Object Tracking With Robust ID Preservation - ResearchGate, accessed December 20, 2025, https://www.researchgate.net/publication/376252570_One-shot_Multiple_Object_Tracking_with_Robust_ID_Preservation
Mahalanobis Distance - RPubs, accessed December 20, 2025, https://rpubs.com/DragonflyStats/Mahalanobis-Distance
nalgebra - Rust - Varlociraptor, accessed December 20, 2025, https://varlociraptor.github.io/varlociraptor/nalgebra/index.html
Vectors and matrices - nalgebra, accessed December 20, 2025, https://www.nalgebra.rs/docs/user_guide/vectors_and_matrices/
Mahalanobis distances and ecological niche modelling: correcting a chi-squared probability error - Semantic Scholar, accessed December 20, 2025, https://pdfs.semanticscholar.org/3f82/71a9d4c6d1c0b508b243857eda34f17d28a2.pdf

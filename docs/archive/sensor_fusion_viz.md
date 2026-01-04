GodView Deep Inspection Architecture: A Comprehensive Framework for Decentralized Sensor Fusion Observability
1. Introduction: The Observability Crisis in Decentralized Fusion
The "GodView" project represents a paradigm shift in autonomous perception, moving from centralized, monolithic tracking servers to a decentralized, mesh-based sensor fusion engine. By leveraging Rust for memory safety, Covariance Intersection (CI) for consistent distributed estimation, and a "Highlander" Conflict-Free Replicated Data Type (CRDT) heuristic for identity management, the system aims to create a robust, partition-tolerant tracking fabric. However, the architectural decoupling of the fusion logic introduces a critical new challenge: Observability. In centralized systems, a single "God" node dictates truth, making debugging a matter of inspecting a single state vector. In decentralized systems like GodView, "truth" is an emergent property of consensus between disparate agents (cars, drones, infrastructure), making anomalies like "Ghosting"—the persistence of duplicate tracks for a single physical object—exponentially harder to diagnose.1
This report presents a rigorous design for a "Deep Inspection" visualization architecture tailored specifically for GodView. The proposed system goes beyond simple state visualization to provide a forensic toolkit for analyzing the internal dynamics of the fusion engine. We define a dual-interface approach: a high-fidelity 3D/4D spatial-temporal inspector using the Rerun SDK 3 for analyzing track genealogy and geometry, and a real-time engineering dashboard using the Ratatui library 5 for monitoring high-frequency fusion metrics. This architecture is designed to validate the correctness of the Highlander heuristic, expose the hidden "tension" between conflicting sensor observations, and quantify the system's performance using novel metrics like the "Ghost Score" and "Entropy Reduction Rate."
By instrumenting the godview_tracking.rs core with these visualization structures, we transform the fusion engine from a "black box" into a "glass box," enabling the rapid identification and resolution of tracking artifacts in complex environments like nuScenes.6
2. Theoretical Framework: The Anatomy of Ghosting in CI Systems
To design an effective inspection tool, we must first mathematically characterize the failure modes it is intended to detect. "Ghosting" in the context of Covariance Intersection (CI) is not merely a tracking error; it is often a symptom of the algorithm's inherent conservatism or a failure in the consensus mechanism.
2.1 The Covariance Intersection Conservatism
Covariance Intersection is chosen for decentralized fusion because it guarantees consistency even when the correlation between estimates is unknown.7 Given two estimates $(x_A, P_A)$ and $(x_B, P_B)$ with unknown correlation, CI computes a fused estimate $(x_{CI}, P_{CI})$ such that:

$$P_{CI}^{-1} = \omega P_A^{-1} + (1-\omega) P_B^{-1}$$

$$P_{CI}^{-1} x_{CI} = \omega P_A^{-1} x_A + (1-\omega) P_B^{-1} x_B$$
where $\omega \in $ is optimized to minimize the determinant or trace of $P_{CI}$.1
The Ghosting Mechanism: CI is, by definition, conservative. It creates a fused covariance that encloses the intersection of the input covariances. In scenarios with high sensor noise or temporal misalignment (latency), the fused covariance $P_{CI}$ may remain large (inflated) to maintain consistency. This inflation creates a "Gating Paradox":
Inflation: The uncertainty ellipsoids of two redundant tracks swell due to conservative fusion updates.
Rejection: The Mahalanobis distance gating threshold, designed to associate measurements with tracks, becomes ambiguous. A new measurement might fall just outside the gate of an inflated track due to the conservative mean shift.
Bifurcation: The system instantiates a new track (a Ghost) for the measurement. Because the original track is also maintained (supported by other agents), the two persist in parallel.
The inspection architecture must therefore visualize not just the position of tracks, but the evolution of their covariance inflation and the cross-track Mahalanobis distances that lead to these rejection events.8
2.2 The Highlander Heuristic and CRDT Conflicts
The "Highlander" heuristic functions as a conflict resolution strategy: "There Can Be Only One." This implies a merge operation where two IDs ($ID_A, ID_B$) collapse into a single survivor ($ID_{survivor}$). In a CRDT context, this merge must be associative, commutative, and idempotent.
The Debugging Gap: When a merge fails to happen (Ghosting) or happens incorrectly (Hallucination collapse), it is often due to:
Oscillation: Agents A and B continually swap authority, preventing the CRDT from settling on a winner.
Metric Failure: The "score" used to determine the winner (e.g., track length, covariance determinant) is ambiguous.
Identity Partition: Network partitions cause agents to diverge in their local ID registries.
The inspection tool must visualize the "Genealogy" of these merges—a directed acyclic graph (DAG) showing which IDs merged into which, and crucially, why specific merge candidates were rejected.9
3. Mathematical Derivations for Deep Inspection Metrics
We introduce three derived metrics to drive the visualization: the Ghost Score, the Entropy Reduction Rate, and the Tension Metric.
3.1 The Ghost Score ($S_{ghost}$)
A "Ghost" is defined as a track that is spatially redundant but distinct in identity. We derive a continuous score $S_{ghost} \in $ to highlight potential ghosts in the visualizer.
The score is a composite of three components: Spatial Proximity, Kinematic Similarity, and Consensus Weakness.
3.1.1 Spatial Proximity via Robust Mahalanobis Distance
For a target track $T_i$ and a candidate neighbor $T_j$, the Mahalanobis distance $D_M(i, j)$ provides a scale-invariant measure of separation 11:

$$D_M^2(i, j) = (x_i - x_j)^T (P_i + P_j)^{-1} (x_i - x_j)$$
We define a proximity score $p_{ij}$ using a Gaussian kernel over $D_M$, tuned to the gating threshold $\gamma$:

$$p_{ij} = \exp\left(-\frac{D_M^2(i, j)}{2\sigma^2}\right)$$
This score is maximal when tracks are identical and decays as they separate. However, ghosts often hover near the gating threshold. Thus, we modify the kernel to peak at the "Ambiguity Zone" where $D_M \approx \gamma$.8
3.1.2 Consensus Weakness
Legitimate objects in a dense sensor network should be observed by multiple agents. A track supported by a single agent in a field of view shared by $N$ agents is likely a ghost. We define the Support Ratio $R_{supp}$:

$$R_{supp}(T_i) = \frac{N_{supporting\_agents}(T_i)}{N_{capable\_agents}(T_i)}$$
3.1.3 The Composite Formula
The final Ghost Score for track $T_i$ is:

$$S_{ghost}(T_i) = \max_{j \neq i} \left( w_1 \cdot p_{ij} + w_2 \cdot (1 - R_{supp}(T_i)) + w_3 \cdot \frac{\text{Trace}(P_i)}{\text{Trace}(P_{limit})} \right)$$
where the last term penalizes tracks with exploding covariance (a sign of unobservability).14 This metric will drive the "Ghost Hunter" coloring logic in Rerun.
3.2 Entropy Reduction Rate ($H_{\Delta}$)
To monitor the health of the CI fusion process, we use Differential Entropy. For a multivariate Gaussian $x \sim \mathcal{N}(\mu, P)$ of dimension $d$, the entropy is 15:

$$H(P) = \frac{1}{2} \ln((2\pi e)^d \det(P))$$
The Entropy Reduction achieved by a fusion step at time $k$ is the difference between the prior entropy (prediction) and posterior entropy (update):

$$H_{\Delta}(k) = H(P_{k|k-1}) - H(P_{k|k}) = \frac{1}{2} \ln\left(\frac{\det(P_{k|k-1})}{\det(P_{k|k})}\right)$$
Interpretation:
$H_{\Delta} > 0$: The fusion added information.
$H_{\Delta} \approx 0$: The fusion was redundant or the CI update was maximally conservative (effectively rejecting the measurement).
$H_{\Delta} < 0$: System divergence (should not happen with optimal CI).
This metric is critical for the "Entropy Dashboard," distinguishing between active tracking (high information gain) and stale tracking (coasting).
3.3 The Tension Metric ($T_{tension}$)
"Tension" quantifies the contradiction between a local agent's observation and the global fused belief. It helps identify "hallucinating" agents. It is derived from the Normalized Innovation Squared (NIS) 18:

$$T_{tension} = (z_{local} - H x_{global})^T S^{-1} (z_{local} - H x_{global})$$
where $S = H P_{global} H^T + R_{local}$ is the innovation covariance. High tension values indicate that an agent is confidently asserting a position that contradicts the consensus, a prime indicator of sensor faults or spoofing.
4. Deep Inspection Architecture: The "GodView Inspector"
The inspection architecture is designed as a "Sidecar" to the main GodView engine. It minimizes impact on the fusion latency while extracting rich telemetry. We utilize a split-topology design where Rerun handles complex 3D/4D visualizations and Ratatui handles real-time metric dashboards.
4.1 System Topology
The GodviewViz module sits alongside the GodviewTracker. It intercepts three streams of data:
Raw Input Stream: Vec<Detection> from every agent (Lidar clusters, Camera boxes).
Internal State Stream: HashMap<TrackId, LocalTrack> representing each agent's local belief.
Fused Output Stream: HashMap<TrackId, GlobalTrack> representing the Highlander consensus.
Table 1: Data Layers and Visualization Strategy
Data Layer
Source Context
Visualization Tool
Representation Archetype
Layer 0: Raw Data
nuscenes_fusion_demo.rs
Rerun (Spatial3D)
Points3D (Lidar), Image (Camera)
Layer 1: Local Beliefs
GodviewTracker::local
Rerun (Spatial3D)
Boxes3D (Colored by Agent ID)
Layer 2: Fused Truth
GodviewTracker::global
Rerun (Spatial3D)
Ellipsoids3D (Colored by Ghost Score)
Layer 3: Genealogy
Highlander::merge_log
Rerun (GraphView)
GraphNodes (Tracks), GraphEdges (Merges)
Layer 4: Metrics
GodviewStats
Ratatui (TUI)
Sparkline (Entropy), Gauge (Ghost Count)

4.2 Rerun Visualization Topology
We employ a Multi-View Blueprint 19 to organize the visual information logically. Instead of a single cluttered 3D view, we programmatically define a layout that separates concerns.
4.2.1 The "GodView" Blueprint Specification
The blueprint divides the screen into four coordinated panels:
Main View (Global Consensus):
Type: Spatial3DView
Content: The world/fused entity path.
Features: Shows the final consensus. Tracks are rendered as Covariance Ellipsoids. "Ghost Hunter" coloring is active here (Red = Ghost).
Time Range: Shows a trail of the last 2 seconds of history to visualize velocity stability.3
Input Inspection View (Sensor Raw):
Type: Spatial3DView
Content: world/raw/** and world/local/**.
Features: Shows the "Chaos" of raw inputs. Each agent is assigned a distinct color (Agent A = Cyan, Agent B = Magenta). This view allows the user to see if the fusion engine is correctly effectively filtering noise.
Genealogy Lab (The Merge Graph):
Type: GraphView 10
Content: genealogy/tree.
Features: A directed graph where nodes are Track IDs and edges are Merge/Spawn events. This directly addresses the user's requirement to see the "family tree" of a track.
Metric Telemetry:
Type: TimeSeriesView
Content: metrics/**.
Features: Real-time plots of $S_{ghost}$ and $H_{\Delta}$ for the selected track.
4.3 Rust Implementation Structs
We define a Visualizer struct that encapsulates the Rerun RecordingStream and manages the metric calculations.

Rust


// godview_core/src/visualization.rs

use rerun::{RecordingStream, RecordingStreamBuilder};
use rerun::archetypes::{Points3D, Ellipsoids3D, GraphNodes, GraphEdges, TextLog};
use crate::godview_tracking::{Track, TrackId, AgentId};

pub struct GodviewInspector {
    rec: RecordingStream,
    ghost_threshold: f32,
    // Cache for calculating differential metrics
    history: HashMap<TrackId, Vec<TrackState>>,
}

impl GodviewInspector {
    pub fn new(app_id: &str) -> Self {
        let rec = RecordingStreamBuilder::new(app_id)
           .connect_grpc() // Connect to external Rerun Viewer
           .unwrap_or_else(|_| RecordingStreamBuilder::new(app_id).save("godview_debug.rrd").unwrap());
        
        // Initialize the Blueprint immediately upon connection
        self::blueprints::send_default_layout(&rec);
        
        Self {
            rec,
            ghost_threshold: 0.65,
            history: HashMap::new(),
        }
    }
    
    // Core update loop called every fusion step
    pub fn update(&mut self, fused_tracks: &, raw_inputs: &) {
        self.log_raw_inputs(raw_inputs);
        self.log_fused_state(fused_tracks);
        self.detect_and_log_ghosts(fused_tracks);
        self.calculate_and_log_entropy(fused_tracks);
    }
}


5. Feature Implementation: "Ghost Hunter" Mode
The "Ghost Hunter" mode is a specialized visualization filter designed to make the invisible (ambiguity) visible.
5.1 Visualizing the Ghost Score
Instead of standard coloring (by ID), we use a diverging color map driven by the calculate_ghost_score function defined in Section 3.1.
Low Score (< 0.3): Render as Green (Solid Consensus).
Medium Score (0.3 - 0.7): Render as Yellow (Ambiguous).
High Score (> 0.7): Render as Bright Red (Probable Ghost).
Visual Flair: For high-score tracks, we pulse the radius of the rendered point or ellipsoid using a sine wave on the radius component in Rerun, drawing the eye to the problem area.

Rust


fn detect_and_log_ghosts(&self, tracks: &) {
    for track in tracks {
        let score = self::metrics::calculate_ghost_score(track, tracks);
        
        // Map score to color
        let color = if score > 0.8 {
            rerun::Color::from_rgb(255, 0, 0) // Red
        } else if score > 0.4 {
            rerun::Color::from_rgb(255, 165, 0) // Orange
        } else {
            rerun::Color::from_rgb(0, 255, 0) // Green
        };
        
        self.rec.log(
            format!("world/fused/{}", track.id),
            &Ellipsoids3D::from_centers_and_radii(
                [track.position],
                [track.covariance_diagonal]
            ).with_colors([color])
            .with_labels([format!("{} (👻 {:.2})", track.id, score)])
        ).unwrap();
        
        // Log score to TimeSeries for historical analysis
        self.rec.log(
            format!("metrics/ghost_score/{}", track.id),
            &rerun::Scalar::new(score as f64)
        ).unwrap();
    }
}


5.2 The Tension Line (Contradiction Visualizer)
The user requested a visual flag when an agent's observation contradicts the fused belief. We implement this using LineStrips3D.
Logic: For each raw detection $z$, find the closest fused track $x$. Calculate the Tension $T_{tension}$.
Threshold: If $T_{tension} > \chi^2_{threshold}$ (statistical significance, e.g., 95% confidence interval), the contradiction is significant.8
Visualization: Draw a line from $z$ to $x$.
Color: Magenta (Conflict).
Style: Dashed (using rerun's strip features if available, or segmented points).
Entity Path: world/debug/tension/{agent_id}_{track_id}.
This creates a visual "spider web" of tension. If a fused track is surrounded by magenta lines connecting to raw detections, it indicates the fused estimate is rejecting valid data (over-confident) or the sensors are drifting.
6. Feature Implementation: The "Merge Graph" (Genealogy)
Tracking the lineage of an ID is essential for debugging the Highlander heuristic. We use Rerun's GraphView to construct a dynamic family tree.
6.1 Graph Node Taxonomy
The graph consists of three node types:
Seed Node: A new track spawned from a raw detection.
State Node: A standard update to an existing track ID.
Merge Node: A special node representing the CRDT resolution of two IDs.
6.2 Visualizing Genealogy with Rerun
We use the GraphNodes and GraphEdges archetypes.10 Since Rerun's graph layout is force-directed by default 9, we can let the physics engine organize the tree or pin nodes based on time.
Design Choice: We will use Force-Directed Layout but with a "Time Gravity" force. New nodes are spawned at $Y=0$, and older nodes are pushed to $Y+$. This creates a waterfall effect.

Rust


// Rust method to log a merge event
pub fn log_merge_event(&self, winner: TrackId, loser: TrackId, reason: &str) {
    let graph_path = "genealogy/tree";
    
    // Log the nodes
    self.rec.log(
        graph_path,
        &GraphNodes::new([winner.to_string(), loser.to_string()])
           .with_labels()
           .with_colors([rerun::Color::from_rgb(0, 255, 0), rerun::Color::from_rgb(255, 0, 0)])
    ).unwrap();

    // Log the edge (Merge relationship)
    self.rec.log(
        graph_path,
        &GraphEdges::new([(loser.to_string(), winner.to_string())])
           .with_directed(true)
    ).unwrap();
    
    // Log the reason to the text log
    self.rec.log(
        "logs/highlander",
        &TextLog::new(format!("MERGE: {} absorbed {}. Reason: {}", winner, loser, reason))
           .with_level("INFO")
    ).unwrap();
}


This visualization allows the user to click on a "Global Track" in the 3D view and trace its graph back to the two "Local Tracks" that merged to form it, effectively solving the "unexpected splitting/merging" requirement.
7. Debugging the "Highlander" Event
The Highlander event is discrete and instantaneous. To visualize it on a continuous timeline, we need a Transient Marker (a "Pop").
7.1 The Visual "Pop"
When a merge occurs at location $x_{merge}$:
Frame $k$: Log a Points3D at $x_{merge}$ with radius = 5.0 meters and color = Cyan (Alpha 0.5).
Frame $k+1$: Log the same point with radius = 4.0.
Frame $k+5$: Log radius = 0.0 (or Clear archetype).
This creates an implosion effect that draws the user's eye to the merge location in the 3D view.
7.2 Detailed Event Logging
We utilize the TextLog archetype 19 to provide the "Why." Rerun's text logs are synchronized with the timeline.
Log Format:
[Highlander] Conflict Detected at t=12.4s
Candidate A: ID 42 (Conf: 0.9, CovDet: 0.01)
Candidate B: ID 88 (Conf: 0.8, CovDet: 0.05)
Resolution: ID 42 Wins (Lower Determinant)
Action: ID 88 marked for deletion; state fused into ID 42.
By selecting the "Pop" in the 3D view, the user can inspect this log entry in the adjacent TextLog view to confirm if the b2 ID won because it was "canonical" as requested.
8. Enhanced CLI Dashboard: The Ratatui TUI
While Rerun is excellent for post-hoc analysis and spatial debugging, a Terminal User Interface (TUI) provides the low-latency, "at-a-glance" health monitoring required during simulation runs. We design this using Ratatui.5
8.1 TUI Architecture: The Async Sidecar
The TUI runs on a separate thread to avoid blocking the high-frequency fusion loop. We use a crossbeam::channel to send lightweight MetricPacket structs from the fusion engine to the TUI thread.
The Metric Packet:

Rust


struct MetricPacket {
    timestamp: f64,
    active_track_count: usize,
    active_ghost_count: usize,
    entropy_reduction_rate: f64,
    conflicting_associations: usize,
    system_status: SystemStatus, // Enum: Healthy, Degraded, Critical
}


8.2 Widget Design and Layout
The TUI layout is defined using Ratatui's constraint-based grid system.
Top Row: Health Gauges
System Health: A Paragraph widget with green/red background.
Active Ghosts: A Gauge widget.
0-5 Ghosts: Green.
5-10 Ghosts: Yellow.
10+ Ghosts: Red (Critical Warning).
Middle Row: Real-Time Trends
Entropy Reduction Rate: A Sparkline widget.5 This widget plots the last 100 values of $H_{\Delta}$ as a bar chart. A "healthy" fusion process shows a jagged positive line. A "flatline" at zero indicates the CI engine has saturated or stopped converging.
Conflict Histogram: A BarChart showing the distribution of "Associations per Measurement."
Bin 1: Unique (Good).
Bin 2: Double-Associated (Ambiguous).
Bin 3+: Highly Conflicted (Bad).
Bottom Row: The "Ghost Watch" Table
Widget: Table.
Columns: ID, Score, Nearest Neighbor, Velocity Delta.
Logic: This table sorts tracks by $S_{ghost}$ descending. It gives the user the exact IDs to search for in Rerun ("Ghost Hunter Mode").
9. Implementation Guide: Integrating with godview_core
To implement this architecture in the existing code base (godview_tracking.rs), follow this integration plan.
9.1 Step 1: Dependencies
Add the following to Cargo.toml:

Ini, TOML


[dependencies]
rerun = "0.28" # Core visualization
ratatui = "0.29" # TUI
crossterm = "0.28" # Terminal handling
nalgebra = "0.32" # Linear algebra for Ghost Score math
crossbeam = "0.8" # For TUI channel


9.2 Step 2: The Visualizer Trait
Define a trait that abstracts the visualization, allowing for "Headless" runs if needed.

Rust


pub trait FusionVisualizer {
    fn log_step(&mut self, step: usize, tracks: &, inputs: &);
    fn log_merge(&mut self, event: MergeEvent);
}


9.3 Step 3: Wiring the Core Loop
Modify the main simulation loop in nuscenes_fusion_demo.rs:

Rust


// In nuscenes_fusion_demo.rs

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Initialize Rerun Inspector
    let mut inspector = GodviewInspector::new("godview_nuscenes_demo");
    
    // 2. Initialize TUI Channel
    let (tx, rx) = crossbeam::channel::unbounded();
    thread::spawn(move |

| run_ratatui_dashboard(rx));

    // 3. Main Loop
    for (step, frame) in nuscenes_loader.iter().enumerate() {
        //... Prediction Step...
        
        //... Update Step (CI Fusion)...
        
        //... Highlander Resolution...
        if let Some(merge_event) = tracker.resolve_conflicts() {
            inspector.log_merge(merge_event);
        }

        // 4. Log to Inspector (Rerun)
        inspector.update(&tracker.tracks, &frame.detections);
        
        // 5. Send Metrics to Dashboard (Ratatui)
        let metrics = calculate_metrics(&tracker);
        tx.send(metrics).unwrap();
    }
    Ok(())
}


9.4 Step 4: nuScenes Specifics
Since the user is using nuScenes, special attention must be paid to coordinate frames. nuScenes provides data in the ego_vehicle frame, but decentralized fusion often occurs in a map (global) frame.
Rerun Transform: Use rerun::Transform3D to log the ego-to-map transform at every frame. This allows Rerun to visualize tracks in the stable Map frame while correctly positioning the moving Ego vehicle.6
Lidar/Camera Projection: To deeply validate ghosts, project the 3D track ellipsoids onto the 2D camera images using rerun::Pinhole. If a "Ghost" track projects onto an empty road in the camera image, it is confirmed as a sensor artifact.
10. Conclusion
The "GodView" decentralized fusion engine requires a sophisticated observability stack to validate its "Highlander" logic and diagnose "Ghosting" artifacts. By implementing the architecture detailed in this report—combining Rerun for genealogical and spatial inspection with Ratatui for real-time metric monitoring—the development team can gain deep insights into the system's behavior.
The introduction of the Ghost Score, Entropy Reduction Rate, and Tension Metric provides the mathematical rigor needed to quantify errors that were previously subjective. The "Merge Graph" visualizes the complex history of track identities, ensuring that the CRDT logic is operating as intended. This "Deep Inspection" capability is the key to transitioning GodView from a simulation demo to a production-grade decentralized perception system.
Works cited
Inverse Covariance Intersection: New Insights and Properties - KIT - ISAS, accessed December 24, 2025, https://isas.iar.kit.edu/pdf/Fusion17_Noack.pdf
Sensor fusion - Wikipedia, accessed December 24, 2025, https://en.wikipedia.org/wiki/Sensor_fusion
Exploring Rerun — An Open-Source Logging and Visualization Tool_Derek | by Turing Inc., accessed December 24, 2025, https://medium.com/@turingmotors/7-exploring-rerun-an-open-source-logging-and-visualization-tool-derek-4667015dc965
Rerun — Rerun, accessed December 24, 2025, https://www.rerun.io/
Ratatui | Ratatui, accessed December 24, 2025, https://ratatui.rs/
Rerun.io: My Deep Dive into the Go-To Visualizer for Physical AI, accessed December 24, 2025, https://skywork.ai/skypage/en/Rerun.io-My-Deep-Dive-into-the-Go-To-Visualizer-for-Physical-AI/1975249775198138368
Covariance intersection - Wikipedia, accessed December 24, 2025, https://en.wikipedia.org/wiki/Covariance_intersection
Outlier Detection Based on Robust Mahalanobis Distance and Its Application - Scirp.org., accessed December 24, 2025, https://www.scirp.org/journal/paperinformation?paperid=90172
Graphs - Rerun, accessed December 24, 2025, https://rerun.io/examples/feature-showcase/graphs
GraphView - Rerun, accessed December 24, 2025, https://rerun.io/docs/reference/types/views/graph_view
Mahalanobis distance - Wikipedia, accessed December 24, 2025, https://en.wikipedia.org/wiki/Mahalanobis_distance
Understanding Mahalanobis Distance | by amit - Medium, accessed December 24, 2025, https://medium.com/@pamit2235/understanding-mahalanobis-distance-081bd765fcdb
Introduction to Multiple Target Tracking - MATLAB & Simulink - MathWorks, accessed December 24, 2025, https://www.mathworks.com/help/fusion/ug/introduction-to-multiple-target-tracking.html
Detecting Ghost Targets Using Multilayer Perceptron in Multiple-Target Tracking - MDPI, accessed December 24, 2025, https://www.mdpi.com/2073-8994/10/1/16
Estimation of Entropy Reduction and Degrees of Freedom for Signal for Large Variational Analysis Systems - ECMWF, accessed December 24, 2025, https://www.ecmwf.int/sites/default/files/elibrary/2003/9402-estimation-entropy-reduction-and-degrees-freedom-signal-large-variational-analysis-systems.pdf
Insights into Entropy as a Measure of Multivariate Variability - MDPI, accessed December 24, 2025, https://www.mdpi.com/1099-4300/18/5/196
Entropy of the Gaussian - Gregory Gundersen, accessed December 24, 2025, https://gregorygundersen.com/blog/2020/09/01/gaussian-entropy/
trackOSPAMetric - Optimal subpattern assignment (OSPA) metric - MATLAB - MathWorks, accessed December 24, 2025, https://www.mathworks.com/help/fusion/ref/trackospametric-system-object.html
Blueprints - Rerun, accessed December 24, 2025, https://rerun.io/docs/concepts/blueprints
Introducing Ratatui: A Rust library to cook up terminal user interfaces (FOSDEM 2024), accessed December 24, 2025, https://www.reddit.com/r/rust/comments/1ark1xx/introducing_ratatui_a_rust_library_to_cook_up/

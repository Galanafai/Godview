# Godview PowerPoint Slide Content
## Byzantine General's Problem for Autonomous Sensor Networks

---

# Slide 1: The Problem - "The Byzantine Sensor Dilemma"

## Title
**"The Byzantine Sensor Dilemma"**

## Visual Description

```
┌─────────────────────────────────────────────────────────────────┐
│                                                                 │
│    ┌─────────────┐                                              │
│    │   📷        │──────────────────┐                          │
│    │  CAMERA     │   "I see a       │                          │
│    │  (Blue)     │    PERSON!"      │                          │
│    └─────────────┘                  │                          │
│                                     ▼                          │
│    ┌─────────────┐           ┌──────────────┐                  │
│    │   🔦        │──────────▶│     🧠       │                  │
│    │  LiDAR      │   "I see  │    BRAIN     │                  │
│    │  (Green)    │  HYDRANT!"│              │                  │
│    └─────────────┘           │   ⚠️ ERROR   │                  │
│                              │   CONFLICTING│                  │
│    ┌─────────────┐           │   REALITY!   │                  │
│    │   📡        │──────────▶│              │                  │
│    │  RADAR      │   "I see  └──────────────┘                  │
│    │  (Magenta)  │  NOTHING!"                                  │
│    └─────────────┘                                              │
│                                                                 │
│    RED WARNING OVERLAY: Three sensors, three truths.            │
│    Which one is lying?                                          │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Layout Specifications
- **Ego-Vehicle**: Center-right side of the slide, large brain icon
- **3 Sensors**: Left side, vertically stacked with color-coded icons
- **Arrows**: Flowing from each sensor to the brain, each with a speech bubble
- **Warning Overlay**: Red semi-transparent banner across brain with "CONFLICTING REALITY"
- **Colors**: Camera (Blue #3B82F6), LiDAR (Green #22C55E), Radar (Magenta #D946EF)

## Key Text Content

> ### The Byzantine Generals Problem in Robotics
> 
> In distributed computing, the Byzantine Generals Problem asks: 
> **How can independent agents reach consensus when some may be faulty or malicious?**
>
> For autonomous vehicles:
> - **Camera** might hallucinate objects (adversarial patches)
> - **LiDAR** might be spoofed by reflections
> - **Radar** might miss entirely (low cross-section)
>
> **Naive sensor fusion = Catastrophic failure**
>
> *"When your sensors disagree, who do you trust?"*

---

# Slide 2: Solution Engine A - Time & Causality

## Title
**"The Time Engine: Cause Precedes Effect"**

## Visual Description

```
┌─────────────────────────────────────────────────────────────────┐
│ SWIMLANE DIAGRAM                                                │
│                                                                 │
│ ┌─────────────────────────────────────────────────────────────┐ │
│ │ CARLA SENSOR A                                              │ │
│ │      ●───────────●───────────●───────────●────────▶         │ │
│ │    (LTS 95)    (100)       (101)       (102)                │ │
│ │        │         │           │           │                   │ │
│ └────────│─────────│───────────│───────────│──────────────────┘ │
│          │         │           │           │                    │
│          │         ▼           ▼           ▼                    │
│          │     ┌───────┐   ┌───────┐   ┌───────┐               │
│          │     │  ✅   │   │  ✅   │   │  ✅   │               │
│          │     │ACCEPT │   │ACCEPT │   │ACCEPT │               │
│          │     └───────┘   └───────┘   └───────┘               │
│          │                                                      │
│          └─────────────────────────────────────┐                │
│                DELAYED ARRIVAL                 │                │
│          ┌─────────────────────────────────────┼───┐            │
│          │       🧱 CAUSAL WALL 🧱              │   │            │
│          │                                     ▼   │            │
│          │         ┌────────────────────┐          │            │
│          │         │   🚫 REJECTED!     │          │            │
│          │         │  "Time Traveling   │          │            │
│          │         │     Packet"        │          │            │
│          │         │  LTS 95 < T_local  │          │            │
│          │         └────────────────────┘          │            │
│          └─────────────────────────────────────────┘            │
│                                                                 │
│ ┌─────────────────────────────────────────────────────────────┐ │
│ │ GODVIEW CORE                                                │ │
│ │      ●───────────●───────────●───────────●────────▶         │ │
│ │   T=100        T=101       T=102       T=103                │ │
│ │               (now)                                         │ │
│ └─────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

## Layout Specifications
- **Top Lane**: "CARLA Sensor A" with event dots at LTS timestamps
- **Bottom Lane**: "Godview Core" with local Lamport clock
- **Green checkmarks**: Valid messages accepted in order
- **Red rejected box**: Late-arriving packet with LTS 95 blocked by "Causal Wall"
- **Brick wall graphic**: Visual metaphor for temporal boundary

## Key Math & Text

### Lamport Clock Update Rule
```math
T_{new} = max(T_{local}, T_{msg}) + 1
```

### Causal Constraint
```math
REJECT \text{ if } T_{msg} < T_{local}
```

> **Why This Matters:**
> - Out-of-Sequence Measurements (OOSM) corrupt state estimation
> - Replay attacks inject old data to cause confusion
> - The AS-EKF handles legitimate delays via retrodiction
> - The Causal Wall blocks illegitimate temporal violations

---

# Slide 3: Solution Engine B - Space & Trust

## Title
**"Space & Trust: Solving Pancakes and Phantoms"**

## Visual Description

```
┌──────────────────────────────────────────────────────────────────┐
│                         SPLIT SCREEN                              │
│                                                                   │
│ ┌─────────────────────────┐   ┌─────────────────────────────┐    │
│ │   LEFT: SPACE ENGINE    │   │   RIGHT: TRUST ENGINE       │    │
│ │                         │   │                             │    │
│ │      🚁 DRONE           │   │   Bayesian Trust Graph      │    │
│ │      z = 50m            │   │   (Beta Distribution)       │    │
│ │         │               │   │                             │    │
│ │    ┌────┴────┐          │   │    ▲                        │    │
│ │    │ VOXEL 5 │          │   │    │   ╱╲                   │    │
│ │    └─────────┘          │   │    │  ╱  ╲ Trusted Agent    │    │
│ │         │               │   │    │ ╱    ╲ (α=50, β=2)     │    │
│ │    ┌─────────┐          │   │ P  │╱      ╲                │    │
│ │    │ VOXEL 4 │ (empty)  │   │    │        ╲               │    │
│ │    └─────────┘          │   │    │         ╲              │    │
│ │         │               │   │    │──────────────────      │    │
│ │    ┌─────────┐          │   │    │ Unknown Source         │    │
│ │    │ VOXEL 3 │ (empty)  │   │    │ (α=1, β=1) = FLAT     │    │
│ │    └─────────┘          │   │    │                        │    │
│ │         │               │   │    └───────────────────▶    │    │
│ │    ┌─────────┐          │   │           Trust Score       │    │
│ │    │ VOXEL 2 │ (empty)  │   │             0.0 ─── 1.0     │    │
│ │    └─────────┘          │   │                             │    │
│ │         │               │   │   ┌─────────────────────┐   │    │
│ │    ┌─────────┐          │   │   │ Trusted: 96% conf   │   │    │
│ │    │ VOXEL 1 │ (empty)  │   │   │ Unknown: BLOCKED    │   │    │
│ │    └─────────┘          │   │   └─────────────────────┘   │    │
│ │         │               │   │                             │    │
│ │    ┌────┴────┐          │   │   "Phantom hazards from     │    │
│ │    │ VOXEL 0 │          │   │    untrusted sources are    │    │
│ │    └─────────┘          │   │    rejected automatically"  │    │
│ │      🚗 CAR             │   │                             │    │
│ │      z = 0m             │   │                             │    │
│ │                         │   │                             │    │
│ │  "Same Lat/Lon ≠        │   │                             │    │
│ │   Same Location!"       │   │                             │    │
│ └─────────────────────────┘   └─────────────────────────────┘    │
│                                                                   │
└──────────────────────────────────────────────────────────────────┘
```

## Layout Specifications

### Left Panel (Space Engine)
- **Wireframe 3D Voxel Stack**: 6 vertical voxels (10m each)
- **Drone icon**: Top voxel (z=50m, Voxel 5)
- **Car icon**: Bottom voxel (z=0m, Voxel 0)
- **Middle voxels**: Empty/transparent to show separation
- **H3 Cell outline**: Same 2D hex for both, different Z
- **Caption**: "Voxel Conflict Resolution - Same H3 ≠ Same Location"

### Right Panel (Trust Engine)
- **Beta Distribution Graph**: X-axis = Trust Score (0-1), Y-axis = Probability
- **Sharp Peak Curve**: Green, labeled "Trusted Source (α=50, β=2)"
- **Flat Line**: Red/gray, labeled "Unknown Source (α=1, β=1)"
- **Trust Score Callouts**: "96% trusted" vs "BLOCKED"

## Key Math & Text

### Beta Distribution Trust Score
```math
Trust = \frac{\alpha}{\alpha + \beta}
```

Where:
- `α` = successful observations + prior
- `β` = failed/suspicious observations + prior

### Trust Examples
| Source | α | β | Trust Score |
|--------|---|---|-------------|
| Verified Fleet Agent | 50 | 2 | **96.2%** |
| New Unknown Source | 1 | 1 | **50.0%** (uncertain) |
| Suspicious Actor | 1 | 10 | **9.1%** (blocked) |

> **Solving Two Problems:**
> 
> **The Pancake Problem (2D Ambiguity)**
> - H3 provides fast 2D geospatial lookup
> - 3D Voxel grid adds altitude dimension
> - Drone at 50m and Car at 0m are DIFFERENT objects
>
> **The Phantom Hazard Problem (Security)**
> - Untrusted sources can inject fake obstacles
> - Bayesian trust scoring learns from history
> - Low-trust data is downweighted or rejected
> - Uses Ed25519 signatures + Biscuit tokens for provenance

---

# Design Notes for PowerPoint Creation

## Color Palette
| Element | Hex Code | Usage |
|---------|----------|-------|
| Success/Trusted | `#22C55E` | Green highlights, checkmarks |
| Warning/Blocked | `#EF4444` | Red for rejections |
| Caution | `#F59E0B` | Yellow/amber for warnings |
| Primary Accent | `#3B82F6` | Blue for primary callouts |
| Secondary | `#8B5CF6` | Purple for secondary elements |
| Background | `#0F172A` | Dark slate for contrast |

## Typography
- **Titles**: Bold, 44pt, white/light
- **Body**: Regular, 18-24pt
- **Math**: LaTeX-rendered or code font (Fira Code/JetBrains Mono)
- **Captions**: Italic, 14pt, dimmed

## Animation Suggestions
1. **Slide 1**: Sensors fade in one by one, each with conflicting message
2. **Slide 2**: Swimlane arrows animate left-to-right, "Causal Wall" appears with shake effect
3. **Slide 3**: Split-screen wipe, beta curves draw themselves

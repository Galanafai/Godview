# GodView documentation

## Current

| Document | What it covers |
|:--|:--|
| [PERCEPTION_SHARING.md](PERCEPTION_SHARING.md) | The current specification: layers, frames and uncertainty, local tracking, the wire format, the report store, the world model, trust, running nodes over Zenoh, and the deterministic simulation suite with what it does and does not establish. |
| [../README.md](../README.md) | Overview, quick start, and current status. |
| [../mission.md](../mission.md) | Engineering principles for the project (safety, performance budget, security model, code standards). |

## Earlier design

These documents describe GodView before the perception-sharing rewrite: the `TrackManager` tracker with nearest-neighbor association, covariance intersection and min-UUID ("Highlander") identity, the evolutionary parameter-tuning experiments, and the first simulation suite. They are kept for history. Where they disagree with PERCEPTION_SHARING.md, PERCEPTION_SHARING.md is correct. In particular, their test counts, performance figures and "fixed" statuses are from that time and were not re-verified.

| Document | Topic |
|:--|:--|
| [ARCHITECTURE.md](ARCHITECTURE.md) | Architecture of the earlier agent runtime |
| [TECHNICAL_DOCUMENTATION.md](TECHNICAL_DOCUMENTATION.md) | Walkthrough of the earlier engines |
| [PROTOCOL_ANALYSIS.md](PROTOCOL_ANALYSIS.md) | Constraints and safety review of the earlier code |
| [STRATEGIC_ANALYSIS.md](STRATEGIC_ANALYSIS.md) | Use cases and scaling discussion |
| [REAL_WORLD_SCENARIO.md](REAL_WORLD_SCENARIO.md) | Illustrative drone delivery scenario |
| [blind_fitness.md](blind_fitness.md), [v4_imp.md](v4_imp.md) | Evolutionary tuning ("blind fitness") experiments |
| [Planning Multi-Agent DST Simulation Testing.md](Planning%20Multi-Agent%20DST%20Simulation%20Testing.md), [../DST_IMPLEMENTATION.md](../DST_IMPLEMENTATION.md) | Planning notes for the first simulation suite |
| [../godview_sim/DST.md](../godview_sim/DST.md), [../godview_sim/FINAL_REPORT.md](../godview_sim/FINAL_REPORT.md), [../godview_sim/SIM_TO_REAL.md](../godview_sim/SIM_TO_REAL.md), [../godview_sim/VULNERABILITIES.md](../godview_sim/VULNERABILITIES.md) | The legacy scenarios (`--scenario legacy`) and their reports |
| [../RESEARCH_BRIEF.md](../RESEARCH_BRIEF.md) | Research brief for the evolutionary experiments |
| [v0.6.0/](v0.6.0) | Release notes and reports for v0.6.0 |
| [archive/](archive) | Older plans, summaries and visualization research |

//! Deterministic replay of a converted V2X-Real scenario through N `PerceptionNode`s
//! (vehicles and roadside units), dumping every agent's ego view and the shared world.
//!
//! Input: a directory written by `scripts/phase_d/convert_v2xreal.py`: `agent<k>.jsonl`
//! (the `godview_node` input format, parsed by `godview_node::input::parse_line`, plus
//! `agent_yaw_deg` and `agent_kind`), `truth.json` and `attribution.json`.
//!
//! Each frame, for each agent in order: deliver the batches that are due, ingest the
//! frame's detections, maintain, set the self state (vehicles, with `--self-reports`),
//! publish, and queue the batch for every other agent `--delay-ms` later. Then each
//! agent's shared world and ego view are written as one JSON line per frame (the scoring
//! is `scripts/phase_d/v2x_eval.py`). Each world object lists the labels its members'
//! latest detections were attributed to.

use godview_core::frames::{EnuFrame, Geodetic};
use godview_core::local_tracker::ScoreMode;
use godview_core::perception_node::{NodeConfig, PerceptionNode, SelfState};
use godview_core::report::ReportKey;
use godview_core::world_model::WorldSnapshot;
use godview_env::Timestamp;
use godview_node::input::{parse_line, ParsedFrame};
use godview_sim::DeterministicKeyProvider;
use nalgebra::Vector3;
use serde_json::{json, Value};
use std::collections::{BTreeMap, VecDeque};

type Attr = BTreeMap<ReportKey, (usize, u64)>;

struct Args {
    dir: String,
    out: String,
    delay_ms: i64,
    seed: u64,
    fresh_only: bool,
    overlap_iou: f64,
    pub_min_score: f32,
    self_reports: bool,
    same_source: bool,
    compete: bool,
    self_size: [f64; 3],
    self_offset: [f64; 2],
    offset_filter: bool,
    anchor_rsu: bool,
    align: bool,
}

fn parse_args() -> Args {
    let mut a = Args {
        dir: String::new(),
        out: String::new(),
        delay_ms: 100,
        seed: 1,
        fresh_only: false,
        overlap_iou: 0.0,
        pub_min_score: 0.0,
        self_reports: false,
        same_source: false,
        compete: false,
        self_size: [4.57, 2.10, 2.29],
        self_offset: [0.60, 0.12],
        offset_filter: false,
        anchor_rsu: false,
        align: false,
    };
    let v: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < v.len() {
        let flag = |name: &str| v[i] == name;
        if flag("--fresh-only") || flag("--self-reports") || flag("--self-same-source-exact") || flag("--self-compete")
            || flag("--offset-filter") || flag("--anchor-rsu") || flag("--align")
        {
            match v[i].as_str() {
                "--fresh-only" => a.fresh_only = true,
                "--self-reports" => a.self_reports = true,
                "--self-same-source-exact" => a.same_source = true,
                "--self-compete" => a.compete = true,
                "--offset-filter" => a.offset_filter = true,
                "--anchor-rsu" => a.anchor_rsu = true,
                _ => a.align = true,
            }
            i += 1;
            continue;
        }
        let val = v.get(i + 1).cloned().unwrap_or_default();
        match v[i].as_str() {
            "--dir" => a.dir = val,
            "--out" => a.out = val,
            "--delay-ms" => a.delay_ms = val.parse().unwrap(),
            "--seed" => a.seed = val.parse().unwrap(),
            "--overlap-iou" => a.overlap_iou = val.parse().unwrap(),
            "--overlap-keep" => assert_eq!(val, "recent", "only --overlap-keep recent is used"),
            "--pub-min-score" => a.pub_min_score = val.parse().unwrap(),
            "--self-size" => {
                let x: Vec<f64> = val.split(',').map(|s| s.parse().unwrap()).collect();
                a.self_size = [x[0], x[1], x[2]];
            }
            "--self-offset" => {
                let x: Vec<f64> = val.split(',').map(|s| s.parse().unwrap()).collect();
                a.self_offset = [x[0], x[1]];
            }
            other => panic!("unknown argument {other}"),
        }
        i += 2;
    }
    assert!(!a.dir.is_empty() && !a.out.is_empty(), "usage: --dir DIR --out FILE");
    a
}

fn dump_world(
    w: &WorldSnapshot,
    odom_from_ecef: &godview_core::frames::Pose3,
    known: &Attr,
    gids: &[Vec<Option<String>>],
    node_ids: &[godview_core::report::AgentId],
) -> Vec<Value> {
    w.objects
        .iter()
        .map(|o| {
            let p = odom_from_ecef.transform_point(&o.position().0);
            let mut labels: Vec<String> = Vec::new();
            for (k, _) in &o.members {
                if let Some((src, obs)) = known.get(k) {
                    if let Some(g) = &gids[*src][*obs as usize] {
                        if !labels.contains(g) {
                            labels.push(g.clone());
                        }
                    }
                }
            }
            let sources: Vec<usize> = o
                .sources()
                .iter()
                .filter_map(|s| node_ids.iter().position(|n| n == s))
                .collect();
            json!({
                "x": p.x, "y": p.y, "z": p.z, "class_id": o.class_id, "class_prob": o.class_prob,
                "box": o.object_box.map(|b| json!({"l": b.size[0], "w": b.size[1], "h": b.size[2], "yaw_deg": b.yaw.to_degrees()})),
                "self_agent": o.self_agent.and_then(|a| node_ids.iter().position(|n| *n == a)),
                "labels": labels, "sources": sources, "members": o.members.len(),
            })
        })
        .collect()
}

fn main() {
    let args = parse_args();
    let truth: Value =
        serde_json::from_str(&std::fs::read_to_string(format!("{}/truth.json", args.dir)).unwrap()).unwrap();
    let n = truth["agents"].as_array().unwrap().len();
    let kinds: Vec<String> = truth["agents"].as_array().unwrap().iter().map(|a| a["kind"].as_str().unwrap().to_string()).collect();
    let lines: Vec<Vec<String>> = (0..n)
        .map(|a| {
            std::fs::read_to_string(format!("{}/agent{a}.jsonl", args.dir))
                .unwrap()
                .lines()
                .filter(|l| !l.trim().is_empty())
                .map(String::from)
                .collect()
        })
        .collect();
    let frames = truth["frames"].as_array().unwrap();
    // Label of each detection, by observation id (parse_line numbers them from 1).
    let attr_file: Value =
        serde_json::from_str(&std::fs::read_to_string(format!("{}/attribution.json", args.dir)).unwrap()).unwrap();
    let mut gids: Vec<Vec<Option<String>>> = vec![vec![None]; n];
    for f in attr_file["frames"].as_array().unwrap() {
        for a in 0..n {
            for g in f[a].as_array().unwrap() {
                gids[a].push(g.as_str().map(String::from));
            }
        }
    }
    let anchor = &truth["anchor"];
    let _site = EnuFrame::new(
        Geodetic::new(
            anchor["lat_deg"].as_f64().unwrap(),
            anchor["lon_deg"].as_f64().unwrap(),
            anchor["height_m"].as_f64().unwrap(),
        )
        .unwrap(),
    );
    let odom_yaw = anchor["yaw_deg"].as_f64().unwrap();
    let mut keys = DeterministicKeyProvider::new(args.seed);
    let mut config = NodeConfig::default();
    // The setting selected on V2V4Real val; no declared systematic box errors (labels plus noise has none).
    config.tracker.score_mode = ScoreMode::Mean;
    config.tracker.overlap_prune_iou = args.overlap_iou;
    config.tracker.overlap_keep_recent = args.overlap_iou > 0.0;
    config.publish.fresh_only = args.fresh_only;
    config.publish.min_score = args.pub_min_score;
    config.self_reports = args.self_reports;
    config.world.self_same_source_exact = args.same_source;
    config.world.self_compete = args.compete;
    config.ego_view = true;
    config.offset_filter.enabled = args.offset_filter;
    config.world.align.enabled = args.align;
    let mut nodes: Vec<PerceptionNode> = (0..n).map(|a| PerceptionNode::new(keys.agent_key(a as u64), 1, config)).collect();
    let node_ids: Vec<_> = nodes.iter().map(|x| x.id()).collect();
    if args.anchor_rsu {
        let rsus: Vec<_> = (0..n).filter(|a| kinds[*a] == "rsu").map(|a| node_ids[a]).collect();
        for nd in nodes.iter_mut() {
            nd.set_anchors(rsus.clone());
        }
    }
    let mut next_obs = vec![0u64; n];
    let mut queue: VecDeque<(i64, usize, Vec<u8>, Attr)> = VecDeque::new();
    let mut known: Vec<Attr> = vec![Attr::new(); n];
    let mut prev_agent: Vec<Option<(i64, Vector3<f64>)>> = vec![None; n];
    let mut out_lines: Vec<String> = Vec::new();
    for (i, fr) in frames.iter().enumerate() {
        let t = Timestamp(fr["t_ns"].as_i64().unwrap());
        let parsed: Vec<ParsedFrame> = (0..n).map(|a| parse_line(&lines[a][i], &mut next_obs[a]).expect("valid input line")).collect();
        for a in 0..n {
            deliver(&mut queue, &mut nodes, t, &mut known);
            nodes[a].ingest(&parsed[a].detections);
            nodes[a].maintain(t);
            if args.self_reports && kinds[a] == "vehicle" {
                let line: Value = serde_json::from_str(&lines[a][i]).unwrap();
                let yaw = line["agent_yaw_deg"].as_f64().unwrap().to_radians();
                let (c, s) = (yaw.cos(), yaw.sin());
                let pos = parsed[a].agent_odom
                    + Vector3::new(c * args.self_offset[0] - s * args.self_offset[1], s * args.self_offset[0] + c * args.self_offset[1], 0.0);
                let vel = prev_agent[a]
                    .map(|(pt, pp)| (pos - pp) / (t.as_nanos() - pt) as f64 * 1e9)
                    .unwrap_or_else(Vector3::zeros);
                prev_agent[a] = Some((t.as_nanos(), pos));
                nodes[a].set_self_state(Some(SelfState {
                    position: pos,
                    velocity: vel,
                    yaw,
                    size: args.self_size,
                    class_id: 1,
                    sigma_pos_m: 0.1,
                    sigma_vel_mps: 0.5,
                    sigma_yaw_rad: 0.03,
                    sigma_size_m: 0.3,
                }));
            }
            if let Some(bytes) = nodes[a].publish(t, parsed[a].pose.as_ref()) {
                let snap: Attr = nodes[a].tracker().tracks().map(|tr| ((node_ids[a], tr.id), (a, tr.last_obs_id))).collect();
                known[a].extend(snap.iter().map(|(k, v)| (*k, *v)));
                for to in (0..n).filter(|x| *x != a) {
                    queue.push_back((t.as_nanos() + args.delay_ms * 1_000_000, to, bytes.clone(), snap.clone()));
                }
            }
        }
        deliver(&mut queue, &mut nodes, t, &mut known);
        let pose_exact = parsed[0].pose.as_ref().expect("pose").ecef_from_odom.pose;
        let odom_from_ecef = pose_exact.inverse();
        let mut agents_out = Vec::new();
        for a in 0..n {
            let shared = nodes[a].world(t);
            let ego = nodes[a].ego_world(t, parsed[a].pose.as_ref().unwrap()).expect("ego view");
            let alignment: Vec<Value> = ego
                .alignment
                .iter()
                .map(|o| {
                    let tr = odom_from_ecef.rotation * o.translation;
                    json!({"src": node_ids.iter().position(|x| *x == o.source), "t_odom": [tr.x, tr.y, tr.z]})
                })
                .collect();
            agents_out.push(json!({
                "index": a,
                "ego": dump_world(&ego, &odom_from_ecef, &known[a], &gids, &node_ids),
                "shared": dump_world(&shared, &odom_from_ecef, &known[a], &gids, &node_ids),
                "alignment": alignment,
            }));
        }
        out_lines.push(json!({"frame": i, "agents": agents_out}).to_string());
    }
    let _ = odom_yaw;
    std::fs::write(&args.out, out_lines.join("\n")).unwrap();
}

fn deliver(queue: &mut VecDeque<(i64, usize, Vec<u8>, Attr)>, nodes: &mut [PerceptionNode], t: Timestamp, known: &mut [Attr]) {
    let mut keep = VecDeque::new();
    while let Some((at, to, bytes, snap)) = queue.pop_front() {
        if at <= t.as_nanos() {
            if nodes[to].receive(&bytes, t).is_ok() {
                known[to].extend(snap);
            }
        } else {
            keep.push_back((at, to, bytes, snap));
        }
    }
    *queue = keep;
}

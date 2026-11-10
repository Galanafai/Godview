//! Deterministic replay of a converted V2V4Real scenario through two
//! [`PerceptionNode`]s, scored against the union of both cars' labels.
//!
//! Input: a directory written by `scripts/v2v4real/convert.py` with
//! `car0.jsonl`, `car1.jsonl` (the `godview_node` input format, parsed by the
//! same `godview_node::input::parse_line`) and `truth.json`.
//!
//! Each frame, for each car: deliver the peer batches that are due, ingest
//! the frame's detections, maintain, publish with the frame's pose, and
//! queue the batch for the other car `--delay-ms` later. Then each car's
//! world is scored.
//!
//! Scoring (per car, per frame), with Hungarian matching and a 5 m cutoff
//! against every labeled object of the frame:
//! * expected objects: detected (by the converter's rule) by at least one
//!   car in this frame and the previous 3 frames; the scoring car itself is
//!   never expected in its own world;
//! * matched: an expected object with a world estimate; missed: an expected
//!   object without one; an estimate matched to a labeled but not expected
//!   object is neither credited nor penalized;
//! * duplicate: an unmatched estimate within the cutoff of some labeled
//!   object (the object appears twice); false: any other unmatched estimate.
//!
//! With `attribution.json` in the input directory (Phase D, real detections,
//! written by `scripts/phase_d/convert_detections.py`), each detection carries
//! the label it was matched to in its own car's frame. A world object's
//! member is attributed to the label of the latest detection its track fused
//! (as of the batch that delivered it), and per car and frame:
//! * wrong merge: a world object whose members are attributed to two or more
//!   different labels (object-frames);
//! * split: a label to which two or more world objects are attributed
//!   (label-frames);
//! * false from sharing: a false estimate (above) whose sources are only the
//!   other car.

#![recursion_limit = "256"]

use godview_core::extent::{wrap_pi, yaw_residual};
use godview_core::frames::{EnuFrame, Geodetic};
use godview_core::perception_node::{NodeConfig, PerceptionNode};
use godview_core::report::ReportKey;
use godview_env::Timestamp;
use godview_node::input::{parse_line, ParsedFrame};
use godview_sim::perception::metrics::score;
use godview_sim::DeterministicKeyProvider;
use nalgebra::{Matrix2, Matrix3, Vector3};
use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet, VecDeque};

const EXPECT_HISTORY: usize = 4;

struct Args {
    dir: String,
    out: String,
    delay_ms: i64,
    cutoff_m: f64,
    seed: u64,
    ablate: Vec<String>,
    box_size_sys_m: f64,
    box_yaw_sys_deg: f64,
    box_center_sys_frac: f64,
    box_center_sys_z_m: f64,
    diag: String,
    split_diag: String,
    spawn_diag: String,
    dump: String,
    no_sharing: bool,
    align: bool,
}

fn parse_args() -> Args {
    let mut a = Args {
        dir: String::new(),
        out: String::new(),
        delay_ms: 100,
        cutoff_m: 5.0,
        seed: 1,
        ablate: Vec::new(),
        box_size_sys_m: 0.0,
        box_yaw_sys_deg: 0.0,
        box_center_sys_frac: 0.0,
        box_center_sys_z_m: 0.0,
        diag: String::new(),
        split_diag: String::new(),
        spawn_diag: String::new(),
        dump: String::new(),
        no_sharing: false,
        align: false,
    };
    let v: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < v.len() {
        // Each car fuses only its own reports (nothing is delivered between cars).
        if v[i] == "--no-sharing" {
            a.no_sharing = true;
            i += 1;
            continue;
        }
        // Relative alignment of the two cars from shared objects (world model).
        if v[i] == "--align" {
            a.align = true;
            i += 1;
            continue;
        }
        let val = v.get(i + 1).cloned().unwrap_or_default();
        match v[i].as_str() {
            "--dir" => a.dir = val,
            "--out" => a.out = val,
            "--delay-ms" => a.delay_ms = val.parse().expect("--delay-ms"),
            "--cutoff-m" => a.cutoff_m = val.parse().expect("--cutoff-m"),
            "--seed" => a.seed = val.parse().expect("--seed"),
            // Declared systematic box errors of the detector (tracker config).
            "--box-size-sys-m" => a.box_size_sys_m = val.parse().expect("--box-size-sys-m"),
            // Diagnostics only: per frame and car, the gids of missed objects and of
            // objects with a duplicate, written as JSON lines. Does not change scoring.
            "--diag" => a.diag = val,
            // Diagnostics only (with attribution.json): one JSON line per split
            // label-frame, with the tracks behind each of the label's world objects.
            "--split-diag" => a.split_diag = val,
            // Diagnostics only (with attribution.json): one JSON line per track
            // started by a detection of a label that another live track of the
            // same car already follows, with the gate statistics of that
            // detection against the existing track.
            "--spawn-diag" => a.spawn_diag = val,
            // Per frame: each car's world objects and the labels, in the odometry
            // frame, as JSON lines (for visualization). Does not change scoring.
            "--dump" => a.dump = val,
            "--box-yaw-sys-deg" => a.box_yaw_sys_deg = val.parse().expect("--box-yaw-sys-deg"),
            "--box-center-sys-frac" => a.box_center_sys_frac = val.parse().expect("--box-center-sys-frac"),
            "--box-center-sys-z-m" => a.box_center_sys_z_m = val.parse().expect("--box-center-sys-z-m"),
            // Diagnostics only (world model): no-box-gate, no-size-gate, no-yaw-gate,
            // no-vel-gate, no-pos-gate.
            "--ablate" => a.ablate.push(val),
            other => panic!("unknown argument {other}"),
        }
        i += 2;
    }
    assert!(!a.dir.is_empty() && !a.out.is_empty(), "usage: --dir DIR --out FILE [--delay-ms 100]");
    a
}

#[derive(Default)]
struct CarStats {
    frames: usize,
    matched: usize,
    missed: usize,
    duplicates: usize,
    duplicates_split_sources: usize,
    false_estimates: usize,
    false_previously_matched: usize,
    matched_world_ids: BTreeSet<ReportKey>,
    unexpected_matched: usize,
    self_in_world: usize,
    sq_err: f64,
    nees3: Vec<f64>,
    nees3_both_labeled: Vec<f64>,
    sq_err_both_labeled: Vec<f64>,
    nees_h: Vec<f64>,
    nees_v: Vec<f64>,
    size_nees: Vec<f64>,
    heading_err_deg: Vec<f64>,
    both_detected: usize,
    both_detected_fused: usize,
    gained: usize,
    gained_objects: BTreeSet<String>,
    not_detected_now: usize,
    ids: BTreeMap<String, ReportKey>,
    id_changes: BTreeMap<String, usize>,
    per_frame: Vec<Value>,
    /// Frames in which alignment estimated an offset between the two cars,
    /// and the norm of that relative translation (m).
    aligned_frames: usize,
    align_relative_m: Vec<f64>,
    /// Attribution metrics (only with attribution.json).
    wrong_merges: usize,
    splits: usize,
    split_extra_objects: usize,
    false_from_sharing: usize,
    unattributed_members: usize,
    members: usize,
}

fn quantile(v: &[f64], q: f64) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    let mut s = v.to_vec();
    s.sort_by(|a, b| a.total_cmp(b));
    s[((s.len() - 1) as f64 * q).round() as usize]
}

fn mean(v: &[f64]) -> f64 {
    if v.is_empty() {
        f64::NAN
    } else {
        v.iter().sum::<f64>() / v.len() as f64
    }
}

fn arr3(v: &Value) -> Vector3<f64> {
    Vector3::new(
        v[0].as_f64().unwrap(),
        v[1].as_f64().unwrap(),
        v[2].as_f64().unwrap(),
    )
}

fn main() {
    let args = parse_args();
    let truth: Value =
        serde_json::from_str(&std::fs::read_to_string(format!("{}/truth.json", args.dir)).unwrap())
            .unwrap();
    let lines: Vec<Vec<String>> = (0..2)
        .map(|c| {
            std::fs::read_to_string(format!("{}/car{c}.jsonl", args.dir))
                .unwrap()
                .lines()
                .filter(|l| !l.trim().is_empty())
                .map(String::from)
                .collect()
        })
        .collect();
    let frames = truth["frames"].as_array().unwrap();
    // Per car, obs id (as parse_line numbers them, from 1) -> attributed label.
    let attribution: Option<[Vec<Option<String>>; 2]> =
        std::fs::read_to_string(format!("{}/attribution.json", args.dir)).ok().map(|text| {
            let a: Value = serde_json::from_str(&text).unwrap();
            let fr = a["frames"].as_array().unwrap();
            assert_eq!(fr.len(), frames.len(), "attribution frames");
            let mut out: [Vec<Option<String>>; 2] = [vec![None], vec![None]];
            for f in fr {
                for (c, o) in out.iter_mut().enumerate() {
                    for g in f[c].as_array().unwrap() {
                        o.push(g.as_str().map(String::from));
                    }
                }
            }
            out
        });
    assert_eq!(lines[0].len(), frames.len());
    assert_eq!(lines[1].len(), frames.len());

    let anchor = &truth["anchor"];
    let site = EnuFrame::new(
        Geodetic::new(
            anchor["lat_deg"].as_f64().unwrap(),
            anchor["lon_deg"].as_f64().unwrap(),
            anchor["height_m"].as_f64().unwrap(),
        )
        .unwrap(),
    );
    let r_ecef_enu: Matrix3<f64> = site.ecef_from_enu();

    let mut keys = DeterministicKeyProvider::new(args.seed);
    let mut config = NodeConfig::default();
    config.tracker.box_size_sys_m = args.box_size_sys_m;
    config.tracker.box_yaw_sys_rad = args.box_yaw_sys_deg.to_radians();
    config.tracker.box_center_sys_frac = args.box_center_sys_frac;
    config.tracker.box_center_sys_z_m = args.box_center_sys_z_m;
    config.world.align.enabled = args.align;
    for a in &args.ablate {
        match a.as_str() {
            "no-box-gate" => config.world.box_gate = godview_core::extent::BoxGate::disabled(),
            "no-size-gate" => config.world.box_gate.size_chi2 = f64::INFINITY,
            "no-yaw-gate" => config.world.box_gate.yaw_chi2 = f64::INFINITY,
            "no-vel-gate" => config.world.gate_vel_chi2 = f64::INFINITY,
            "no-pos-gate" => config.world.gate_pos_chi2 = f64::INFINITY,
            other => panic!("unknown ablation {other}"),
        }
    }
    let mut nodes: Vec<PerceptionNode> = (0..2)
        .map(|c| PerceptionNode::new(keys.agent_key(c), 1, config))
        .collect();
    let mut next_obs = [0u64; 2];
    let mut queue: VecDeque<(i64, usize, Vec<u8>, Attr)> = VecDeque::new();
    // Per car: report key -> (source car, latest obs id fused), from its own
    // publishes and from the snapshots that came with delivered peer batches.
    let mut known: [Attr; 2] = [Attr::new(), Attr::new()];
    // Per car: track id -> obs id of the detection that started the track.
    let mut first_obs: [BTreeMap<u64, u64>; 2] = [BTreeMap::new(), BTreeMap::new()];
    let mut split_lines: Vec<String> = Vec::new();
    let mut spawn_lines: Vec<String> = Vec::new();
    let mut stats: Vec<CarStats> = (0..2).map(|_| CarStats::default()).collect();
    let mut detected_hist: VecDeque<BTreeSet<String>> = VecDeque::new();
    let mut ever_detected: [BTreeSet<String>; 2] = [BTreeSet::new(), BTreeSet::new()];
    let mut rejected = [0usize; 2];
    let mut diag_lines: Vec<String> = Vec::new();
    let mut dump_lines: Vec<String> = Vec::new();
    let odom_yaw = anchor["yaw_deg"].as_f64().unwrap();
    let node_ids: Vec<godview_core::report::AgentId> = nodes.iter().map(|n| n.id()).collect();

    for (i, fr) in frames.iter().enumerate() {
        let t = Timestamp(fr["t_ns"].as_i64().unwrap());
        let parsed: Vec<ParsedFrame> = (0..2)
            .map(|c| parse_line(&lines[c][i], &mut next_obs[c]).expect("valid input line"))
            .collect();
        assert!(parsed.iter().all(|p| p.t == t), "car files and truth disagree on time");
        for c in 0..2 {
            deliver(&mut queue, &mut nodes, t, &mut rejected, &mut known);
            nodes[c].ingest(&parsed[c].detections);
            if let (Some(attr), false) = (&attribution, args.spawn_diag.is_empty()) {
                for new in nodes[c].tracker().tracks().filter(|tr| !first_obs[c].contains_key(&tr.id.0)) {
                    let Some(g) = attr[c][new.last_obs_id as usize].as_deref() else { continue };
                    let Some(d) = parsed[c].detections.iter().find(|d| d.obs_id == new.last_obs_id) else {
                        continue;
                    };
                    let old = nodes[c]
                        .tracker()
                        .tracks()
                        .filter(|tr| tr.id != new.id && attr[c][tr.last_obs_id as usize].as_deref() == Some(g))
                        .max_by_key(|tr| tr.hits);
                    let Some(old) = old else { continue };
                    let pred = old.predicted(d.t);
                    let innov = d.z - pred.x.fixed_rows::<3>(0).into_owned();
                    let v = pred.x.fixed_rows::<3>(3).into_owned();
                    let (pb, db) = (old.predicted_box(d.t), d.object_box);
                    let boxes = match (pb, db) {
                        (Some(x), Some(y)) => {
                            let dy = yaw_residual(x.yaw, y.yaw, x.yaw_ambiguous || y.yaw_ambiguous);
                            // Innovation along / across the track's box.
                            let (cs, sn) = (x.yaw.cos(), x.yaw.sin());
                            json!({
                                "dsize": [y.size[0] - x.size[0], y.size[1] - x.size[1], y.size[2] - x.size[2]],
                                "size_chi2_terms": (0..3).map(|k| (x.size[k] - y.size[k]).powi(2) / (x.size_var[k] + y.size_var[k])).collect::<Vec<f64>>(),
                                "track_size_sigma": x.size_var.map(f64::sqrt),
                                "dyaw_deg": dy.to_degrees(),
                                "yaw_chi2": dy * dy / (x.yaw_var + y.yaw_var),
                                "track_yaw_sigma_deg": x.yaw_var.sqrt().to_degrees(),
                                "innov_along": cs * innov.x + sn * innov.y,
                                "innov_across": -sn * innov.x + cs * innov.y,
                            })
                        }
                        _ => Value::Null,
                    };
                    spawn_lines.push(
                        json!({
                            "frame": i, "car": c, "gid": g, "old_hits": old.hits,
                            "old_lifecycle": format!("{:?}", old.lifecycle),
                            "frames_since_old_update": ((d.t.as_nanos() - old.last_update.as_nanos()) as f64 / 1e8).round(),
                            "nis": pred.nis(&d.z, &d.r), "innov_m": innov.norm(), "innov_z": innov.z,
                            "track_speed": v.norm(), "det_sigma_diag": [d.r[(0, 0)].sqrt(), d.r[(1, 1)].sqrt(), d.r[(2, 2)].sqrt()],
                            "box": boxes,
                        })
                        .to_string(),
                    );
                }
            }
            for tr in nodes[c].tracker().tracks() {
                first_obs[c].entry(tr.id.0).or_insert(tr.last_obs_id);
            }
            nodes[c].maintain(t);
            if let Some(bytes) = nodes[c].publish(t, parsed[c].pose.as_ref()) {
                let snap: Attr = nodes[c]
                    .tracker()
                    .tracks()
                    .map(|tr| {
                        let info = TrackInfo {
                            src: c,
                            obs: tr.last_obs_id,
                            first_obs: first_obs[c][&tr.id.0],
                            created_ns: tr.created.as_nanos(),
                            last_update_ns: tr.last_update.as_nanos(),
                            hits: tr.hits,
                        };
                        ((node_ids[c], tr.id), info)
                    })
                    .collect();
                known[c].extend(snap.iter().map(|(k, v)| (*k, *v)));
                if args.no_sharing {
                    continue;
                }
                queue.push_back((t.as_nanos() + args.delay_ms * 1_000_000, 1 - c, bytes, snap));
            }
        }
        deliver(&mut queue, &mut nodes, t, &mut rejected, &mut known);

        // Truth for this frame, in ECEF through the shared anchor (no error).
        let pose_exact = parsed[0].pose.as_ref().expect("pose").ecef_from_odom.pose;
        let objs = fr["objects"].as_array().unwrap();
        let detected_now: BTreeSet<String> = objs
            .iter()
            .filter(|o| !o["detected_by"].as_array().unwrap().is_empty())
            .map(|o| o["gid"].as_str().unwrap().to_string())
            .collect();
        detected_hist.push_back(detected_now);
        if detected_hist.len() > EXPECT_HISTORY {
            detected_hist.pop_front();
        }
        for o in objs {
            for c in o["detected_by"].as_array().unwrap() {
                ever_detected[c.as_u64().unwrap() as usize]
                    .insert(o["gid"].as_str().unwrap().to_string());
            }
        }
        let truth_ecef: Vec<godview_core::frames::Ecef> = objs
            .iter()
            .map(|o| godview_core::frames::Ecef(pose_exact.transform_point(&arr3(&o["pos"]))))
            .collect();

        let mut dump_worlds: Vec<Value> = Vec::new();
        for c in 0..2 {
            let world = nodes[c].world(t);
            if world.alignment.len() == 2 {
                stats[c].aligned_frames += 1;
                let rel = world.alignment[0].translation - world.alignment[1].translation;
                stats[c].align_relative_m.push(rel.norm());
            }
            if !args.dump.is_empty() {
                let odom_from_ecef = pose_exact.inverse();
                let objs_dump: Vec<Value> = world
                    .objects
                    .iter()
                    .map(|w| {
                        let p = odom_from_ecef.transform_point(&w.position().0);
                        let sources: Vec<usize> = w
                            .sources()
                            .iter()
                            .filter_map(|s| node_ids.iter().position(|n| n == s))
                            .collect();
                        let (agent, track) = w.id;
                        let id = format!(
                            "{}:{}",
                            node_ids.iter().position(|n| *n == agent).map_or("?".to_string(), |k| k.to_string()),
                            track.0
                        );
                        json!({
                            "id": id,
                            "x": p.x, "y": p.y, "z": p.z,
                            "class_prob": w.class_prob, "p_exist": w.p_exist,
                            "box": w.object_box.map(|b| json!({
                                "l": b.size[0], "w": b.size[1], "h": b.size[2],
                                "yaw_deg": b.yaw.to_degrees() - odom_yaw,
                            })),
                            "members": w.members.len(),
                            "sources": sources,
                        })
                    })
                    .collect();
                dump_worlds.push(Value::Array(objs_dump));
            }
            let est: Vec<_> = world
                .objects
                .iter()
                .map(|o| (o.position(), Some(o.cov.fixed_view::<3, 3>(0, 0).into_owned())))
                .collect();
            let sc = score(&est, &truth_ecef, args.cutoff_m);
            let st = &mut stats[c];
            if let Some(attr) = &attribution {
                let mut per_label: BTreeMap<&str, usize> = BTreeMap::new();
                let mut objs_of: BTreeMap<&str, Vec<usize>> = BTreeMap::new();
                for (wi, w) in world.objects.iter().enumerate() {
                    let mut labels: BTreeSet<&str> = BTreeSet::new();
                    for (key, _) in &w.members {
                        st.members += 1;
                        match known[c].get(key).and_then(|k| attr[k.src][k.obs as usize].as_deref()) {
                            Some(g) => {
                                labels.insert(g);
                            }
                            None => {
                                if !known[c].contains_key(key) {
                                    st.unattributed_members += 1;
                                }
                            }
                        }
                    }
                    if labels.len() >= 2 {
                        st.wrong_merges += 1;
                    }
                    for g in labels {
                        *per_label.entry(g).or_default() += 1;
                        objs_of.entry(g).or_default().push(wi);
                    }
                }
                if !args.split_diag.is_empty() {
                    let odom_from_ecef = pose_exact.inverse();
                    for (g, ws) in objs_of.iter().filter(|(_, ws)| ws.len() >= 2) {
                        let objs_json: Vec<Value> = ws
                            .iter()
                            .map(|wi| {
                                let w = &world.objects[*wi];
                                let p = odom_from_ecef.transform_point(&w.position().0);
                                let members: Vec<Value> = w
                                    .members
                                    .iter()
                                    .filter_map(|(key, _)| known[c].get(key).map(|k| (key, k)))
                                    .map(|(key, k)| {
                                        json!({
                                            "src": k.src, "track": (key.1).0, "hits": k.hits,
                                            "created_ns": k.created_ns, "last_update_ns": k.last_update_ns,
                                            "last_gid": attr[k.src][k.obs as usize],
                                            "first_gid": attr[k.src][k.first_obs as usize],
                                        })
                                    })
                                    .collect();
                                json!({"x": p.x, "y": p.y, "z": p.z, "members": members})
                            })
                            .collect();
                        // Gates between the label's first two world objects, from their
                        // fused states (exact for single-member objects).
                        let (a, b) = (&world.objects[ws[0]], &world.objects[ws[1]]);
                        let d = a.state - b.state;
                        let sm = a.cov + b.cov;
                        let chi = |off: usize| {
                            let dv = d.fixed_rows::<3>(off).into_owned();
                            let sv: Matrix3<f64> = sm.fixed_view::<3, 3>(off, off).into_owned();
                            sv.cholesky().map(|ch| dv.dot(&ch.solve(&dv)))
                        };
                        let (size_chi2, yaw_chi2) = match (a.object_box, b.object_box) {
                            (Some(x), Some(y)) => {
                                let sz: f64 = (0..3)
                                    .map(|k| (x.size[k] - y.size[k]).powi(2) / (x.size_var[k] + y.size_var[k]))
                                    .sum();
                                let dy = yaw_residual(x.yaw, y.yaw, x.yaw_ambiguous || y.yaw_ambiguous);
                                (Some(sz), Some(dy * dy / (x.yaw_var + y.yaw_var)))
                            }
                            _ => (None, None),
                        };
                        let e_enu = r_ecef_enu.transpose() * d.fixed_rows::<3>(0).into_owned();
                        split_lines.push(
                            json!({
                                "frame": i, "t_ns": t.as_nanos(), "car": c, "gid": g, "objects": objs_json,
                                "pos_chi2": chi(0), "vel_chi2": chi(3), "size_chi2": size_chi2, "yaw_chi2": yaw_chi2,
                                "dist_h_m": (e_enu.x * e_enu.x + e_enu.y * e_enu.y).sqrt(), "dz_m": e_enu.z,
                            })
                            .to_string(),
                        );
                    }
                }
                for n in per_label.values().filter(|n| **n >= 2) {
                    st.splits += 1;
                    st.split_extra_objects += n - 1;
                }
            }
            st.frames += 1;
            let mut f_matched = 0;
            let mut f_missed = 0;
            let mut f_dup = 0;
            let mut f_false = 0;
            let mut f_sq = 0.0;
            let mut d_missed: Vec<String> = Vec::new();
            let mut d_dup: Vec<String> = Vec::new();
            let matched_truths: BTreeMap<usize, usize> = sc.pairs.iter().map(|(e, t)| (*t, *e)).collect();
            for (ti, o) in objs.iter().enumerate() {
                let gid = o["gid"].as_str().unwrap().to_string();
                let is_self = o["cav"].as_u64() == Some(c as u64);
                let expected = !is_self
                    && detected_hist.len() == EXPECT_HISTORY
                    && detected_hist.iter().all(|s| s.contains(&gid));
                match matched_truths.get(&ti) {
                    Some(_) if is_self => st.self_in_world += 1,
                    Some(&ei) if expected => {
                        f_matched += 1;
                        let w = &world.objects[ei];
                        let e = w.position().0 - truth_ecef[ti].0;
                        f_sq += e.norm_squared();
                        let p: Matrix3<f64> = w.cov.fixed_view::<3, 3>(0, 0).into_owned();
                        let both_labeled = o["labeled_by"].as_array().unwrap().len() == 2;
                        if let Some(ch) = p.cholesky() {
                            let n = e.dot(&ch.solve(&e));
                            st.nees3.push(n);
                            if both_labeled {
                                st.nees3_both_labeled.push(n);
                            }
                        }
                        if both_labeled {
                            st.sq_err_both_labeled.push(e.norm_squared());
                        }
                        let e_enu = r_ecef_enu.transpose() * e;
                        let p_enu = r_ecef_enu.transpose() * p * r_ecef_enu;
                        let ph: Matrix2<f64> = p_enu.fixed_view::<2, 2>(0, 0).into_owned();
                        let eh = e_enu.fixed_rows::<2>(0).into_owned();
                        if let Some(ch) = ph.cholesky() {
                            st.nees_h.push(eh.dot(&ch.solve(&eh)));
                        }
                        st.nees_v.push(e_enu.z * e_enu.z / p_enu[(2, 2)]);
                        if let Some(b) = w.object_box {
                            let size = arr3(&o["size"]);
                            st.size_nees.push((0..3).map(|k| (b.size[k] - size[k]).powi(2) / b.size_var[k]).sum());
                            let ty = wrap_pi(o["yaw_enu_deg"].as_f64().unwrap().to_radians());
                            st.heading_err_deg.push(yaw_residual(b.yaw, ty, b.yaw_ambiguous).abs().to_degrees());
                        }
                        let by: Vec<u64> = o["detected_by"].as_array().unwrap().iter().map(|x| x.as_u64().unwrap()).collect();
                        if by.len() == 2 {
                            st.both_detected += 1;
                            if w.sources().len() == 2 {
                                st.both_detected_fused += 1;
                            }
                        }
                        if !ever_detected[c].contains(&gid) {
                            st.gained += 1;
                            st.gained_objects.insert(gid.clone());
                        }
                        if !by.contains(&(c as u64)) {
                            st.not_detected_now += 1;
                        }
                        st.matched_world_ids.insert(w.id);
                        if let Some(prev) = st.ids.insert(gid.clone(), w.id) {
                            if prev != w.id {
                                *st.id_changes.entry(gid.clone()).or_default() += 1;
                            }
                        } else {
                            st.id_changes.entry(gid.clone()).or_default();
                        }
                    }
                    Some(_) => st.unexpected_matched += 1,
                    None if expected => {
                        f_missed += 1;
                        d_missed.push(gid.clone());
                    }
                    None => {}
                }
            }
            for &ei in &sc.unmatched_estimates {
                let p = est[ei].0;
                let near = truth_ecef
                    .iter()
                    .enumerate()
                    .filter(|(_, t)| t.distance(&p) <= args.cutoff_m)
                    .min_by(|a, b| a.1.distance(&p).total_cmp(&b.1.distance(&p)));
                match near {
                    Some((ti, _)) => {
                        f_dup += 1;
                        d_dup.push(objs[ti]["gid"].as_str().unwrap().to_string());
                        // Split across cars: the matched estimate of that object has other sources.
                        if let Some(&em) = matched_truths.get(&ti) {
                            if world.objects[em].sources() != world.objects[ei].sources() {
                                st.duplicates_split_sources += 1;
                            }
                        }
                    }
                    None => {
                        f_false += 1;
                        let srcs = world.objects[ei].sources();
                        if srcs.len() == 1 && srcs.contains(&node_ids[1 - c]) {
                            st.false_from_sharing += 1;
                        }
                        if st.matched_world_ids.contains(&world.objects[ei].id) {
                            st.false_previously_matched += 1;
                        }
                    }
                }
            }
            st.matched += f_matched;
            st.missed += f_missed;
            st.duplicates += f_dup;
            st.false_estimates += f_false;
            st.sq_err += f_sq;
            st.per_frame.push(json!([i, f_matched, f_missed, f_dup, f_false]));
            if !args.dump.is_empty() && c == 1 {
                let labels: Vec<Value> = objs
                    .iter()
                    .map(|o| {
                        let p = arr3(&o["pos"]);
                        json!({
                            "gid": o["gid"], "x": p.x, "y": p.y,
                            "l": o["size"][0], "w": o["size"][1],
                            "yaw_deg": o["yaw_enu_deg"].as_f64().unwrap() - odom_yaw,
                            "detected_by": o["detected_by"], "cav": o["cav"],
                        })
                    })
                    .collect();
                dump_lines.push(
                    json!({"frame": i, "ego": fr["ego_odom"], "labels": labels, "worlds": dump_worlds}).to_string(),
                );
            }
            if !args.diag.is_empty() && (!d_missed.is_empty() || !d_dup.is_empty()) {
                diag_lines.push(json!({"frame": i, "car": c, "missed": d_missed, "dup": d_dup}).to_string());
            }
        }
    }

    let mut cars = Vec::new();
    for (c, st) in stats.iter().enumerate() {
        let changes: Vec<usize> = st.id_changes.values().copied().collect();
        let objects = changes.len();
        let total_changes: usize = changes.iter().sum();
        cars.push(json!({
            "car": c,
            "frames": st.frames,
            "matched": st.matched,
            "missed": st.missed,
            "duplicates": st.duplicates,
            "duplicates_split_across_cars": st.duplicates_split_sources,
            "false_estimates": st.false_estimates,
            "false_estimates_with_id_of_a_previously_matched_object": st.false_previously_matched,
            "matched_not_expected": st.unexpected_matched,
            "self_in_own_world": st.self_in_world,
            "rms_m": if st.matched > 0 { (st.sq_err / st.matched as f64).sqrt() } else { f64::NAN },
            "n_nees": st.nees3.len(),
            "n_box": st.size_nees.len(),
            "nees_pos_3d_mean": mean(&st.nees3),
            "n_both_labeled": st.sq_err_both_labeled.len(),
            "nees_pos_3d_mean_both_labeled": mean(&st.nees3_both_labeled),
            "rms_m_both_labeled": mean(&st.sq_err_both_labeled).sqrt(),
            "nees_pos_h_2d_mean": mean(&st.nees_h),
            "nees_pos_v_1d_mean": mean(&st.nees_v),
            "nees_pos_3d_p95": quantile(&st.nees3, 0.95),
            "size_nees_3d_mean": mean(&st.size_nees),
            "heading_err_deg_median": quantile(&st.heading_err_deg, 0.5),
            "heading_err_deg_p95": quantile(&st.heading_err_deg, 0.95),
            "heading_err_deg_max": quantile(&st.heading_err_deg, 1.0),
            "detected_by_both_matched": st.both_detected,
            "detected_by_both_fused_into_one": st.both_detected_fused,
            "matched_never_detected_by_own_sensor": st.gained,
            "objects_gained_only_through_sharing": st.gained_objects.len(),
            "aligned_frames": st.aligned_frames,
            "align_relative_offset_m_median": quantile(&st.align_relative_m, 0.5),
            "align_relative_offset_m": st.align_relative_m,
            "matched_not_detected_by_own_sensor_this_frame": st.not_detected_now,
            "objects_with_id": objects,
            "id_changes_total": total_changes,
            "id_changes_per_object": if objects > 0 { total_changes as f64 / objects as f64 } else { f64::NAN },
            "objects_with_id_change": changes.iter().filter(|c| **c > 0).count(),
            "attribution": attribution.is_some(),
            "wrong_merges_object_frames": st.wrong_merges,
            "split_label_frames": st.splits,
            "split_extra_objects": st.split_extra_objects,
            "false_from_sharing_object_frames": st.false_from_sharing,
            "false_from_sharing_per_frame": st.false_from_sharing as f64 / st.frames.max(1) as f64,
            "gained_object_frames_per_frame": st.gained as f64 / st.frames.max(1) as f64,
            "members_seen": st.members,
            "members_without_attribution_snapshot": st.unattributed_members,
            "per_frame_matched_missed_dup_false": st.per_frame,
        }));
    }
    let out = json!({
        "scenario": truth["scenario"],
        "declared": truth["declared"],
        "delay_ms": args.delay_ms,
        "ablations": args.ablate,
        "no_sharing": args.no_sharing,
        "align": args.align,
        "box_size_sys_m": args.box_size_sys_m,
        "box_yaw_sys_deg": args.box_yaw_sys_deg,
        "box_center_sys_frac": args.box_center_sys_frac,
        "box_center_sys_z_m": args.box_center_sys_z_m,
        "cutoff_m": args.cutoff_m,
        "expected_rule": format!("detected by at least one car in this and the previous {} frames", EXPECT_HISTORY - 1),
        "batches_rejected": rejected,
        "cars": cars,
    });
    std::fs::write(&args.out, serde_json::to_string_pretty(&out).unwrap()).unwrap();
    if !args.diag.is_empty() {
        std::fs::write(&args.diag, diag_lines.join("\n")).unwrap();
    }
    if !args.dump.is_empty() {
        std::fs::write(&args.dump, dump_lines.join("\n")).unwrap();
    }
    if !args.split_diag.is_empty() {
        std::fs::write(&args.split_diag, split_lines.join("\n")).unwrap();
    }
    if !args.spawn_diag.is_empty() {
        std::fs::write(&args.spawn_diag, spawn_lines.join("\n")).unwrap();
    }
    for c in out["cars"].as_array().unwrap() {
        println!(
            "{} car{}: matched {} missed {} dup {} (split {}) false {} rms {:.2} nees3 {:.2} (h {:.2} v {:.2}) fused {}/{} gained {} idchg {}/{} heading p95 {:.1}",
            truth["scenario"].as_str().unwrap(),
            c["car"], c["matched"], c["missed"], c["duplicates"], c["duplicates_split_across_cars"], c["false_estimates"],
            c["rms_m"].as_f64().unwrap_or(f64::NAN), c["nees_pos_3d_mean"].as_f64().unwrap_or(f64::NAN),
            c["nees_pos_h_2d_mean"].as_f64().unwrap_or(f64::NAN), c["nees_pos_v_1d_mean"].as_f64().unwrap_or(f64::NAN),
            c["detected_by_both_fused_into_one"], c["detected_by_both_matched"], c["objects_gained_only_through_sharing"],
            c["id_changes_total"], c["objects_with_id"], c["heading_err_deg_p95"].as_f64().unwrap_or(f64::NAN),
        );
        if c["attribution"].as_bool() == Some(true) {
            println!(
                "    wrong merges {} splits {} false from sharing {} (per frame {:.2}) gained per frame {:.2}",
                c["wrong_merges_object_frames"], c["split_label_frames"], c["false_from_sharing_object_frames"],
                c["false_from_sharing_per_frame"].as_f64().unwrap(), c["gained_object_frames_per_frame"].as_f64().unwrap(),
            );
        }
    }
}

/// Report key -> (source car, obs id of the latest detection its track fused).
type Attr = BTreeMap<ReportKey, TrackInfo>;

/// What the scorer knows about a track as of a publish (never fed to GodView).
#[derive(Clone, Copy)]
struct TrackInfo {
    /// Source car.
    src: usize,
    /// Obs id of the latest detection fused, and of the one that started the track.
    obs: u64,
    first_obs: u64,
    created_ns: i64,
    last_update_ns: i64,
    hits: u32,
}

fn deliver(
    queue: &mut VecDeque<(i64, usize, Vec<u8>, Attr)>,
    nodes: &mut [PerceptionNode],
    t: Timestamp,
    rejected: &mut [usize; 2],
    known: &mut [Attr; 2],
) {
    let mut keep = VecDeque::new();
    while let Some((at, to, bytes, snap)) = queue.pop_front() {
        if at <= t.as_nanos() {
            if nodes[to].receive(&bytes, t).is_err() {
                rejected[to] += 1;
            } else {
                known[to].extend(snap);
            }
        } else {
            keep.push_back((at, to, bytes, snap));
        }
    }
    *queue = keep;
}

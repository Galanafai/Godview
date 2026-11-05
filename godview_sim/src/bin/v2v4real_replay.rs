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
    };
    let v: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < v.len() {
        let val = v.get(i + 1).cloned().unwrap_or_default();
        match v[i].as_str() {
            "--dir" => a.dir = val,
            "--out" => a.out = val,
            "--delay-ms" => a.delay_ms = val.parse().expect("--delay-ms"),
            "--cutoff-m" => a.cutoff_m = val.parse().expect("--cutoff-m"),
            "--seed" => a.seed = val.parse().expect("--seed"),
            // Declared systematic box errors of the detector (tracker config).
            "--box-size-sys-m" => a.box_size_sys_m = val.parse().expect("--box-size-sys-m"),
            "--box-yaw-sys-deg" => a.box_yaw_sys_deg = val.parse().expect("--box-yaw-sys-deg"),
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
    let mut queue: VecDeque<(i64, usize, Vec<u8>)> = VecDeque::new();
    let mut stats: Vec<CarStats> = (0..2).map(|_| CarStats::default()).collect();
    let mut detected_hist: VecDeque<BTreeSet<String>> = VecDeque::new();
    let mut ever_detected: [BTreeSet<String>; 2] = [BTreeSet::new(), BTreeSet::new()];
    let mut rejected = [0usize; 2];

    for (i, fr) in frames.iter().enumerate() {
        let t = Timestamp(fr["t_ns"].as_i64().unwrap());
        let parsed: Vec<ParsedFrame> = (0..2)
            .map(|c| parse_line(&lines[c][i], &mut next_obs[c]).expect("valid input line"))
            .collect();
        assert!(parsed.iter().all(|p| p.t == t), "car files and truth disagree on time");
        for c in 0..2 {
            deliver(&mut queue, &mut nodes, t, &mut rejected);
            nodes[c].ingest(&parsed[c].detections);
            nodes[c].maintain(t);
            if let Some(bytes) = nodes[c].publish(t, parsed[c].pose.as_ref()) {
                queue.push_back((t.as_nanos() + args.delay_ms * 1_000_000, 1 - c, bytes));
            }
        }
        deliver(&mut queue, &mut nodes, t, &mut rejected);

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

        for c in 0..2 {
            let world = nodes[c].world(t);
            let est: Vec<_> = world
                .objects
                .iter()
                .map(|o| (o.position(), Some(o.cov.fixed_view::<3, 3>(0, 0).into_owned())))
                .collect();
            let sc = score(&est, &truth_ecef, args.cutoff_m);
            let st = &mut stats[c];
            st.frames += 1;
            let mut f_matched = 0;
            let mut f_missed = 0;
            let mut f_dup = 0;
            let mut f_false = 0;
            let mut f_sq = 0.0;
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
                    None if expected => f_missed += 1,
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
                        // Split across cars: the matched estimate of that object has other sources.
                        if let Some(&em) = matched_truths.get(&ti) {
                            if world.objects[em].sources() != world.objects[ei].sources() {
                                st.duplicates_split_sources += 1;
                            }
                        }
                    }
                    None => {
                        f_false += 1;
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
            "matched_not_detected_by_own_sensor_this_frame": st.not_detected_now,
            "objects_with_id": objects,
            "id_changes_total": total_changes,
            "id_changes_per_object": if objects > 0 { total_changes as f64 / objects as f64 } else { f64::NAN },
            "objects_with_id_change": changes.iter().filter(|c| **c > 0).count(),
            "per_frame_matched_missed_dup_false": st.per_frame,
        }));
    }
    let out = json!({
        "scenario": truth["scenario"],
        "declared": truth["declared"],
        "delay_ms": args.delay_ms,
        "ablations": args.ablate,
        "box_size_sys_m": args.box_size_sys_m,
        "box_yaw_sys_deg": args.box_yaw_sys_deg,
        "cutoff_m": args.cutoff_m,
        "expected_rule": format!("detected by at least one car in this and the previous {} frames", EXPECT_HISTORY - 1),
        "batches_rejected": rejected,
        "cars": cars,
    });
    std::fs::write(&args.out, serde_json::to_string_pretty(&out).unwrap()).unwrap();
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
    }
}

fn deliver(
    queue: &mut VecDeque<(i64, usize, Vec<u8>)>,
    nodes: &mut [PerceptionNode],
    t: Timestamp,
    rejected: &mut [usize; 2],
) {
    let mut keep = VecDeque::new();
    while let Some((at, to, bytes)) = queue.pop_front() {
        if at <= t.as_nanos() {
            if nodes[to].receive(&bytes, t).is_err() {
                rejected[to] += 1;
            }
        } else {
            keep.push_back((at, to, bytes));
        }
    }
    *queue = keep;
}

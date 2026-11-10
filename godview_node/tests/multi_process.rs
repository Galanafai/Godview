//! Multi-process tests: real `godview-node` processes talking over Zenoh.
//!
//! Each test starts several node processes on localhost, connected as a
//! Zenoh peer mesh, with a common start time. Every process rebuilds the
//! same synthetic scene from the seed and senses it with its own noise and
//! localization error; nodes share only signed bytes over Zenoh.
//!
//! What these tests can establish: the end-to-end path (per-cell publish,
//! Zenoh transport, subscription scoping, verification, fusion) works
//! between separate processes, under injected loss, duplication, delay and
//! reordering, for these runs. Real-time scheduling makes runs
//! non-deterministic, so per-tick tracking is checked with a stated
//! tolerance; the final world after the fault-free final round is checked
//! exactly. The deterministic, many-seed guarantees come from the DST suite
//! in `godview_sim`.

use godview_node::report::NodeReport;
use std::net::TcpListener;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

fn free_port() -> u16 {
    TcpListener::bind("127.0.0.1:0")
        .and_then(|l| l.local_addr())
        .map(|a| a.port())
        .expect("a free localhost port")
}

fn now_ms() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("clock after 1970")
        .as_millis() as i64
}

/// Tick rate for these tests. Debug builds verify signatures and run the
/// filters about a hundred times slower than release builds; at 10 Hz a
/// debug node on a two-core machine skips most ticks. The protocol checks
/// below are the same at either rate.
const RATE_HZ: f64 = if cfg!(debug_assertions) { 2.0 } else { 10.0 };

fn scratch(name: &str) -> PathBuf {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR"))
        .join(format!("godview_node_{name}_{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("temp dir");
    dir
}

/// Starts node `i` of a mesh: it listens on `ports[i]` and connects to every earlier node.
fn spawn(ports: &[u16], i: usize, args: &[String], report: &Path) -> Child {
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_godview-node"));
    cmd.arg("--listen")
        .arg(format!("tcp/127.0.0.1:{}", ports[i]));
    for p in &ports[..i] {
        cmd.arg("--connect").arg(format!("tcp/127.0.0.1:{p}"));
    }
    cmd.args(args)
        .arg("--rate-hz")
        .arg(RATE_HZ.to_string())
        .arg("--report")
        .arg(report)
        .stdout(Stdio::null())
        .stderr(Stdio::piped());
    cmd.spawn().expect("spawn godview-node")
}

fn wait_all(children: Vec<Child>, timeout: Duration) -> Vec<String> {
    let deadline = Instant::now() + timeout;
    let mut errors = Vec::new();
    for mut c in children {
        loop {
            match c.try_wait().expect("wait on node") {
                Some(status) => {
                    if !status.success() {
                        let out = c.wait_with_output().expect("node output");
                        errors.push(format!(
                            "node exited with {status}: {}",
                            String::from_utf8_lossy(&out.stderr)
                        ));
                    }
                    break;
                }
                None if Instant::now() > deadline => {
                    let _ = c.kill();
                    errors.push("node timed out".into());
                    break;
                }
                None => std::thread::sleep(Duration::from_millis(50)),
            }
        }
    }
    errors
}

fn read(report: &Path) -> NodeReport {
    let text = std::fs::read_to_string(report).expect("node wrote its report");
    serde_json::from_str(&text).expect("valid node report")
}

#[test]
fn four_nodes_under_network_faults_share_one_world() {
    let dir = scratch("four");
    let ports: Vec<u16> = (0..4).map(|_| free_port()).collect();
    let start = now_ms() + 4000;
    let mut children = Vec::new();
    let mut reports = Vec::new();
    for i in 0..4 {
        let report = dir.join(format!("node{i}.json"));
        let args: Vec<String> = [
            "--synthetic-index",
            &i.to_string(),
            "--ring-agents",
            "4",
            "--objects",
            "4",
            "--seed",
            "7",
            "--key-seed",
            &(i + 1).to_string(),
            "--start-unix-ms",
            &start.to_string(),
            "--duration-s",
            "8",
            "--settle-s",
            "1.5",
            "--loss",
            "0.2",
            "--duplicate",
            "0.2",
            "--delay-ms",
            "10",
            "--jitter-ms",
            "60",
            "--fault-seed",
            &(100 + i).to_string(),
        ]
        .iter()
        .map(|s| s.to_string())
        .collect();
        children.push(spawn(&ports, i, &args, &report));
        reports.push(report);
    }
    let errors = wait_all(children, Duration::from_secs(90));
    assert!(errors.is_empty(), "{errors:#?}");
    let r: Vec<NodeReport> = reports.iter().map(|p| read(p)).collect();

    for (i, n) in r.iter().enumerate() {
        // Every peer's batches arrived and verified; nothing was rejected.
        assert_eq!(
            n.accepted_from.len(),
            3,
            "node {i} heard from {:?}",
            n.accepted_from.keys()
        );
        assert!(n.rejected.is_empty(), "node {i} rejected {:?}", n.rejected);
        // Final world: every object, fused from all four sources, no ghosts.
        let f = n.tracking.final_score.expect("synthetic run is scored");
        assert!(f.exact(), "node {i} final: {f:?}");
        // Gross-error bound only (0.3 m sensors, 0.1 m localization); accuracy
        // and covariance calibration are checked statistically in the DST suite.
        assert!(f.rms_m < 1.0, "node {i} final rms {} m", f.rms_m);
        assert_eq!(n.final_world.objects.len(), 4, "node {i}");
        assert!(
            n.final_world.objects.iter().all(|o| o.sources == 4),
            "node {i}: {:?}",
            n.final_world.objects
        );
        // Every object carries a fused box whose size matches one of the scene's vehicles.
        for o in &n.final_world.objects {
            let b = o.object_box.as_ref().expect("fused box");
            let nearest = godview_node::synthetic::SIZES
                .iter()
                .map(|s| {
                    (0..3)
                        .map(|k| (s[k] - b.size_m[k]).abs())
                        .fold(0.0, f64::max)
                })
                .fold(f64::INFINITY, f64::min);
            assert!(
                nearest < 0.5,
                "node {i}: box {:?} matches no vehicle",
                b.size_m
            );
        }
        // Motion resolves front from back for objects faster than 1 m/s once
        // their tracks are at least 1 s old (checked with margin at 1.5 m/s);
        // slower ones may stay ambiguous. A resolved heading must point the
        // way some object of that size moves, never backwards (the truck and
        // the bus have nearly equal sizes, so both are candidates).
        let truth = godview_node::synthetic::Truth::new(&godview_node::synthetic::SyntheticSpec {
            seed: 7,
            ring_agents: 4,
            objects: 4,
            duration_s: 9.5,
            ..Default::default()
        });
        let fast = truth.objects.iter().filter(|(_, v)| v.norm() > 1.5).count();
        let mut resolved = 0;
        for o in &n.final_world.objects {
            let b = o.object_box.as_ref().expect("fused box");
            if b.heading_ambiguous {
                continue;
            }
            resolved += 1;
            let err = (0..truth.objects.len())
                .filter(|&k| {
                    truth.sizes[k].is_some_and(|s| (0..3).all(|d| (s[d] - b.size_m[d]).abs() < 0.5))
                })
                .map(|k| {
                    godview_core::extent::wrap_pi(b.heading_deg.to_radians() - truth.heading_enu(k))
                        .abs()
                })
                .fold(f64::INFINITY, f64::min);
            assert!(
                err < 30f64.to_radians(),
                "node {i}: a resolved heading is {:.0} deg from every matching object's direction of travel",
                err.to_degrees()
            );
        }
        assert!(
            resolved >= fast,
            "node {i}: {resolved} headings resolved, {fast} objects move faster than 1.5 m/s"
        );
        // During the run (real time, 20% loss): at most 5% of ticks with an
        // error after warm-up, never more than 3 in a row.
        let t = &n.tracking;
        // 8 s of sensing minus 2.5 s of warm-up; allow some skipped ticks.
        let scheduled = (5.5 * RATE_HZ) as u64;
        assert!(
            t.ticks_scored * 10 >= scheduled * 7,
            "node {i} scored {} of {scheduled} ticks: {:?}",
            t.ticks_scored,
            n.timing
        );
        assert!(t.error_ticks * 20 <= t.ticks_scored, "node {i}: {t:?}");
        assert!(t.max_consecutive_error_ticks <= 3, "node {i}: {t:?}");
    }
    // The network really was hostile (counted over the whole run: at 2 Hz in
    // debug builds one node may offer too few batches to see every fault).
    let total = |f: fn(&godview_node::faults::FaultStats) -> u64| {
        r.iter().map(|n| f(&n.faults)).sum::<u64>()
    };
    assert!(total(|f| f.dropped) > 0, "no batch was dropped");
    assert!(total(|f| f.duplicated) > 0, "no batch was duplicated");
    assert!(total(|f| f.delayed) > 0, "no batch was delayed");
    // After the fault-free final round every node computes the same world.
    let digests: Vec<&str> = r.iter().map(|n| n.final_world.digest.as_str()).collect();
    assert!(digests.iter().all(|d| *d == digests[0]), "{digests:?}");
    let _ = std::fs::remove_dir_all(dir);
}

#[test]
fn a_distant_node_receives_nothing_from_other_cells() {
    // Nodes 0 and 1 share a scene; node 2 is 5 km east, in the same Zenoh mesh
    // (it is directly connected to both). Cell topics must keep it out of their
    // traffic entirely.
    let dir = scratch("far");
    let ports: Vec<u16> = (0..3).map(|_| free_port()).collect();
    let start = now_ms() + 3500;
    let mut children = Vec::new();
    let mut reports = Vec::new();
    for i in 0..3 {
        let report = dir.join(format!("node{i}.json"));
        let (index, offset) = if i == 2 { (0, "5000") } else { (i, "0") };
        let args: Vec<String> = [
            "--synthetic-index",
            &index.to_string(),
            "--offset-east-m",
            offset,
            "--ring-agents",
            "2",
            "--objects",
            "3",
            "--seed",
            "11",
            "--key-seed",
            &(i + 10).to_string(),
            "--start-unix-ms",
            &start.to_string(),
            "--duration-s",
            "5",
            "--settle-s",
            "1.0",
        ]
        .iter()
        .map(|s| s.to_string())
        .collect();
        children.push(spawn(&ports, i, &args, &report));
        reports.push(report);
    }
    let errors = wait_all(children, Duration::from_secs(60));
    assert!(errors.is_empty(), "{errors:#?}");
    let r: Vec<NodeReport> = reports.iter().map(|p| read(p)).collect();

    assert_eq!(
        r[2].batches_received, 0,
        "far node received {} batches",
        r[2].batches_received
    );
    assert!(r[2].final_world.objects.is_empty());
    let near: std::collections::BTreeSet<&String> = r[0]
        .subscribed_cells
        .iter()
        .chain(&r[1].subscribed_cells)
        .collect();
    assert!(
        r[2].subscribed_cells.iter().all(|c| !near.contains(c)),
        "far node shares no topic cell"
    );
    for (i, n) in r.iter().enumerate().take(2) {
        assert_eq!(
            n.accepted_from.len(),
            1,
            "node {i} hears exactly its neighbor"
        );
        assert!(n.tracking.final_score.expect("scored").exact(), "node {i}");
        assert!(
            n.final_world.objects.iter().all(|o| o.sources == 2),
            "node {i}"
        );
    }
    assert_eq!(r[0].final_world.digest, r[1].final_world.digest);
    let _ = std::fs::remove_dir_all(dir);
}

#[test]
fn a_node_fed_json_lines_on_stdin_fuses_with_a_synthetic_peer() {
    // The external-input path: the test plays an outside perception stack and
    // writes JSON lines into a node's stdin, while a synthetic node watches the
    // same scene. Each must end up fusing the other's reports.
    use godview_core::frames::{EnuFrame, Pose3};
    use godview_node::input::{InputBox, InputDetection, InputFrame, InputPose};
    use godview_node::synthetic::{site, SyntheticSpec, Truth};
    use nalgebra::Vector3;
    use std::io::Write;

    let dir = scratch("stdin");
    let ports: Vec<u16> = (0..2).map(|_| free_port()).collect();
    let start = now_ms() + 3000;
    let seed = 21u64;
    let duration_s = 6.0;
    let synth_report = dir.join("synthetic.json");
    let stdin_report = dir.join("stdin.json");
    let synth_args: Vec<String> = [
        "--synthetic-index",
        "0",
        "--ring-agents",
        "2",
        "--objects",
        "3",
        "--seed",
        &seed.to_string(),
        "--key-seed",
        "31",
        "--start-unix-ms",
        &start.to_string(),
        "--duration-s",
        &duration_s.to_string(),
        "--settle-s",
        "1.0",
    ]
    .iter()
    .map(|s| s.to_string())
    .collect();
    let synth = spawn(&ports, 0, &synth_args, &synth_report);
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_godview-node"));
    cmd.arg("--listen")
        .arg(format!("tcp/127.0.0.1:{}", ports[1]))
        .arg("--connect")
        .arg(format!("tcp/127.0.0.1:{}", ports[0]))
        .args(["--key-seed", "32", "--start-unix-ms", &start.to_string()])
        .arg("--rate-hz")
        .arg(RATE_HZ.to_string())
        .arg("--report")
        .arg(&stdin_report)
        .stdin(Stdio::piped())
        .stdout(Stdio::null())
        .stderr(Stdio::piped());
    let mut external = cmd.spawn().expect("spawn stdin node");
    let mut pipe = external.stdin.take().expect("stdin pipe");

    // The outside stack: an agent 45 m south of the site, odometry frame
    // rotated 30 degrees, seeing the same truth objects with small noise.
    let spec = SyntheticSpec {
        seed,
        ring_agents: 2,
        objects: 3,
        duration_s: duration_s + 1.0,
        ..SyntheticSpec::default()
    };
    let truth = Truth::new(&spec);
    let origin = site().to_ecef(&Vector3::new(0.0, -45.0, 2.0)).to_geodetic();
    let yaw_deg = 30.0;
    let ecef_from_odom = EnuFrame::new(origin)
        .ecef_from_enu_pose()
        .compose(&Pose3::from_yaw(f64::to_radians(yaw_deg), Vector3::zeros()));
    let odom_from_ecef = ecef_from_odom.inverse();
    while now_ms() < start {
        std::thread::sleep(Duration::from_millis(10));
    }
    let mut k = 0u64;
    while (now_ms() - start) as f64 / 1e3 < duration_s {
        let t_ms = now_ms();
        let t_s = (t_ms - start) as f64 / 1e3;
        let detections = truth
            .positions_enu(t_s)
            .iter()
            .enumerate()
            .map(|(j, p)| {
                let z = odom_from_ecef.transform_point(&site().to_ecef(p).0);
                // Small deterministic jitter in place of sensor noise.
                let w = 0.05 * (((k * 7 + j as u64 * 3) % 5) as f64 - 2.0);
                InputDetection {
                    x: z.x + w,
                    y: z.y - w,
                    z: z.z,
                    sigma_m: 0.3,
                    sigma_along_m: None,
                    sigma_across_m: None,
                    sigma_z_m: None,
                    class_id: 1,
                    class_prob: 0.9,
                    // Boxes the same as the synthetic peer's objects, heading in this
                    // agent's odometry frame (rotated 30 degrees from East).
                    object_box: truth.sizes[j].map(|size| InputBox {
                        length_m: size[0],
                        width_m: size[1],
                        height_m: size[2],
                        yaw_deg: truth.heading_enu(j).to_degrees() - yaw_deg,
                        sigma_size_m: 0.2,
                        sigma_yaw_deg: 3.0,
                        yaw_ambiguous: false,
                    }),
                }
            })
            .collect();
        let frame = InputFrame {
            t_unix_ns: t_ms * 1_000_000,
            pose: Some(InputPose {
                lat_deg: origin.lat_deg,
                lon_deg: origin.lon_deg,
                height_m: origin.height_m,
                yaw_deg,
                sigma_h_m: 0.1,
                sigma_v_m: 0.2,
                sigma_yaw_deg: 0.2,
            }),
            detections,
            agent_xyz: [0.0, 0.0, 0.0],
        };
        writeln!(
            pipe,
            "{}",
            serde_json::to_string(&frame).expect("frame json")
        )
        .expect("write frame");
        if k % 7 == 3 {
            writeln!(pipe, "{{this is not json}}").expect("write junk");
        }
        k += 1;
        std::thread::sleep(Duration::from_millis(100));
    }
    drop(pipe); // EOF ends the external node

    let errors = wait_all(vec![synth, external], Duration::from_secs(60));
    assert!(errors.is_empty(), "{errors:#?}");
    let synthetic = read(&synth_report);
    let fed = read(&stdin_report);
    assert!(fed.pose_known);
    assert!(
        fed.tracking.input_errors > 0,
        "junk lines are counted, not fatal"
    );
    assert_eq!(
        synthetic.accepted_from.len(),
        1,
        "synthetic node heard the stdin node"
    );
    assert_eq!(
        fed.accepted_from.len(),
        1,
        "stdin node heard the synthetic node"
    );
    for (name, r) in [("synthetic", &synthetic), ("stdin", &fed)] {
        assert_eq!(
            r.final_world.objects.len(),
            3,
            "{name}: {:?}",
            r.final_world.objects
        );
        assert!(
            r.final_world.objects.iter().all(|o| o.sources == 2),
            "{name}: {:?}",
            r.final_world.objects
        );
        // Boxes arrive through both inputs and over the wire, and fuse.
        for o in &r.final_world.objects {
            let b = o.object_box.as_ref().expect("fused box");
            let nearest = godview_node::synthetic::SIZES
                .iter()
                .map(|s| (s[0] - b.size_m[0]).abs())
                .fold(f64::INFINITY, f64::min);
            assert!(
                nearest < 0.5,
                "{name}: box length {} matches no object",
                b.size_m[0]
            );
        }
    }
    let _ = std::fs::remove_dir_all(dir);
}

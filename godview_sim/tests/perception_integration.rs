//! Integration tests across the perception-sharing modules.
//!
//! These exercise the boundaries between crates and modules: a `GodViewAgent`
//! (godview_core) receiving signed envelopes (godview_env) produced by a peer's
//! perception node, shared world state flowing into the spatial engine, and
//! the DST harness (godview_sim) end to end.

use ed25519_dalek::SigningKey;
use godview_core::frames::{EnuFrame, Geodetic, Pose3, PoseWithCov};
use godview_core::godview_space::SpatialEngine;
use godview_core::local_tracker::Detection;
use godview_core::localization::{LocalizationStatus, PoseEstimate};
use godview_core::perception_node::{NodeConfig, PerceptionNode};
use godview_core::{AgentConfig, GodViewAgent};
use godview_env::{GodViewContext, NodeId, SignedPacketEnvelope, Timestamp};
use godview_sim::perception::catalog;
use godview_sim::perception::harness::{run, AgentSpec, ObjectSpec, ScenarioSpec};
use godview_sim::{DeterministicKeyProvider, SimContext, SimNetwork};
use nalgebra::{Matrix3, Vector3};
use std::sync::Arc;
use std::time::Duration;

fn site() -> EnuFrame {
    EnuFrame::new(Geodetic::new(37.7749, -122.4194, 10.0).unwrap())
}

/// A peer node at `origin` (ENU) with odometry yaw `yaw`, tracking `target` for 1 s.
fn peer_batches(
    key: SigningKey,
    origin: Vector3<f64>,
    yaw: f64,
    target: Vector3<f64>,
    t0: Timestamp,
) -> Vec<(Timestamp, Vec<u8>)> {
    let mut node = PerceptionNode::new(key, 1, NodeConfig::default());
    let odom = Pose3::from_yaw(yaw, origin);
    let link = site().ecef_from_enu_pose().compose(&odom);
    let mut out = Vec::new();
    for k in 0..10 {
        let t = t0.add_millis(k * 100);
        let z = odom.inverse().transform_point(&target);
        node.ingest(&[Detection {
            obs_id: k as u64 + 1,
            t,
            z,
            r: Matrix3::identity() * 0.04,
            class_id: 1,
            class_prob: 0.9,
            object_box: None,
        }]);
        node.maintain(t);
        let pose = PoseEstimate {
            t,
            ecef_from_odom: PoseWithCov::exact(link),
            status: LocalizationStatus::Global,
        };
        if let Some(bytes) = node.publish(t, Some(&pose)) {
            out.push((t, bytes));
        }
    }
    out
}

#[test]
fn agent_runtime_receives_peer_reports_over_envelopes() {
    let ctx = Arc::new(SimContext::new(5));
    let net = Arc::new(SimNetwork::new_stub(NodeId::from_seed(1)));
    let root = DeterministicKeyProvider::new(5).biscuit_root_key().public();
    let mut agent = GodViewAgent::new(ctx.clone(), net, AgentConfig::default(), root);

    let target = Vector3::new(30.0, -12.0, 1.0);
    let t0 = ctx.timestamp();
    let batches = peer_batches(
        SigningKey::from_bytes(&[9; 32]),
        Vector3::new(-20.0, 5.0, 2.0),
        0.9,
        target,
        t0,
    );
    assert!(
        !batches.is_empty(),
        "peer confirmed and published its track"
    );

    for (t, bytes) in &batches {
        ctx.set_time((t.as_nanos() - t0.as_nanos()) as u64);
        let env = SignedPacketEnvelope::new(bytes.clone(), t.as_millis() as u64);
        agent
            .process_packet(NodeId::from_seed(9), env)
            .expect("valid peer batch accepted");
    }
    let now = ctx.timestamp();
    let world = agent.perception.world_neutral(now);
    assert_eq!(world.objects.len(), 1);
    let err = world.objects[0]
        .position()
        .distance(&site().to_ecef(&target));
    assert!(err < 0.5, "peer's object placed {err} m from truth");

    // Tampered payloads and stale envelopes are rejected with errors, not ignored.
    let (_, last) = batches.last().unwrap();
    let mut bad = last.clone();
    bad[70] ^= 0x01;
    assert!(agent
        .process_packet(
            NodeId::from_seed(9),
            SignedPacketEnvelope::new(bad, now.as_millis() as u64)
        )
        .is_err());
    ctx.advance_time(Duration::from_secs(20));
    let stale = SignedPacketEnvelope::new(last.clone(), now.as_millis() as u64);
    assert!(agent.process_packet(NodeId::from_seed(9), stale).is_err());
    assert!(agent.perception.stats().rejected.values().sum::<u64>() >= 1);
}

#[test]
fn shared_world_state_flows_into_the_spatial_engine_with_uncertainty() {
    let t0 = Timestamp::from_millis(1_704_067_200_000);
    let target = Vector3::new(12.0, 40.0, 1.0);
    let mut receiver =
        PerceptionNode::new(SigningKey::from_bytes(&[1; 32]), 1, NodeConfig::default());
    for (key, origin, yaw) in [
        ([2u8; 32], Vector3::new(-30.0, 0.0, 2.0), 0.2),
        ([3u8; 32], Vector3::new(50.0, 60.0, 2.0), -2.5),
    ] {
        for (t, bytes) in peer_batches(SigningKey::from_bytes(&key), origin, yaw, target, t0) {
            receiver.receive(&bytes, t).unwrap();
        }
    }
    let t = t0.add_millis(900);
    let world = receiver.world_neutral(t);
    assert_eq!(world.objects.len(), 1);
    assert_eq!(
        world.objects[0].members.len(),
        2,
        "two peers' reports fused"
    );

    let mut engine = SpatialEngine::new(h3o::Resolution::Ten);
    for o in &world.objects {
        engine.update_entity(o.to_entity()).unwrap();
    }
    let target_geo = site().to_ecef(&target).to_geodetic();
    let hits = engine.query_radius(
        [target_geo.lat_deg, target_geo.lon_deg, target_geo.height_m],
        1.0,
    );
    assert_eq!(hits.len(), 1);
    let cov = hits[0]
        .position_cov_enu
        .expect("uncertainty carried into the engine");
    assert!(cov[0] > 0.0 && cov[3] > 0.0 && cov[5] > 0.0);
    let back = Geodetic::new(
        hits[0].position[0],
        hits[0].position[1],
        hits[0].position[2],
    )
    .unwrap()
    .to_ecef();
    assert!(
        back.distance(&world.objects[0].position()) < 1e-6,
        "continuous position preserved"
    );
}

fn small_spec(seed: u64) -> ScenarioSpec {
    let mut spec = ScenarioSpec::new("integration_small", seed);
    spec.duration_s = 3.0;
    let mut a = AgentSpec::at(Vector3::new(-30.0, 0.0, 2.0));
    a.odom_yaw_rad = 0.4;
    let mut b = AgentSpec::at(Vector3::new(30.0, 10.0, 2.0));
    b.odom_yaw_rad = -1.9;
    spec.agents = vec![a, b];
    spec.objects = vec![
        ObjectSpec::moving(Vector3::new(0.0, 0.0, 1.0), Vector3::new(2.0, 1.0, 0.0)),
        ObjectSpec::moving(Vector3::new(10.0, -15.0, 1.0), Vector3::new(-1.0, 2.0, 0.0)),
    ];
    spec
}

#[test]
fn harness_runs_are_reproducible_from_the_seed() {
    let a = run(&small_spec(11));
    let b = run(&small_spec(11));
    let c = run(&small_spec(12));
    assert_eq!(a.digest(), b.digest(), "same seed, same run");
    assert_ne!(a.digest(), c.digest(), "different seed, different noise");
    assert_eq!(a.samples.len(), b.samples.len());
}

#[test]
fn harness_end_to_end_two_agents_agree() {
    let rec = run(&small_spec(3));
    let last = rec.samples.last().unwrap();
    for a in 0..2 {
        let s = rec.score(last, a, 5.0, |t| !t.global_observers.is_empty());
        assert_eq!((s.missed, s.false_estimates), (0, 0), "agent {a}");
    }
    let reference = rec.source_only_world.as_ref().unwrap().digest();
    assert!(
        rec.final_worlds.iter().all(|w| w.digest() == reference),
        "agents converged"
    );
}

#[test]
fn quick_catalog_scenarios_pass() {
    // The full catalog runs in release via `godview-sim --scenario all`; this keeps a
    // fast cross-section in the default test run.
    for name in ["two_views", "crossing_targets"] {
        let entry = catalog::find(name).expect("catalog entry");
        let out = (entry.run)(42);
        assert!(out.passed(), "{name}: {:#?}", out.failures);
    }
}

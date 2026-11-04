//! L2: an agent's own multi-object tracker.
//!
//! The local tracker turns the agent's own detections into tracks, one
//! [`RetrodictingFilter`] per object, all in the agent's odometry frame.
//! It never sees peers' information (the source-purity rule), so every
//! report published from it carries only the agent's own evidence.
//!
//! Per scan: predict each live track to the scan time, gate with the
//! position NIS against `chi²(3)` (99% by default), assign with the
//! Hungarian algorithm, update with a Kalman step (replaying the window if
//! the scan is late), and start tentative tracks from unassigned detections.
//! Lifecycle is driven by elapsed time, not ticks.

use crate::assignment::assign;
use crate::extent::{BoxGate, BoxModel, ObjectBox};
use crate::filter::{BufferedObs, CvModel, CvState, FilterError, RetrodictingFilter};
use crate::report::{LocalTrackId, TrackLifecycle};
use godview_env::Timestamp;
use nalgebra::{Matrix3, Vector3};
use std::collections::{BTreeMap, VecDeque};

/// A detection from one of the agent's own sensors, in the odometry frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Detection {
    /// Agent-local unique observation id (used for deduplication).
    pub obs_id: u64,
    /// Measurement time in the shared timebase.
    pub t: Timestamp,
    /// Measured position in the odometry frame (m).
    pub z: Vector3<f64>,
    /// Measurement covariance (m²), from the sensor model.
    pub r: Matrix3<f64>,
    /// Class id reported by the detector.
    pub class_id: u8,
    /// Detector confidence for `class_id` in `[0, 1]`.
    pub class_prob: f32,
    /// Oriented box, heading in the odometry frame, if the detector reports extent.
    pub object_box: Option<ObjectBox>,
}

/// Local tracker parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalTrackerConfig {
    /// Motion model.
    pub model: CvModel,
    /// NIS gate for 3-D position innovations (11.34 = 99% for 3 DOF).
    pub gate_chi2: f64,
    /// A detection this close (NIS, 3 DOF) to a published track never starts a
    /// new track, even if it fell outside `gate_chi2` (21.11 = 99.99%). Tail
    /// noise near a known object is not a new object.
    pub spawn_suppress_chi2: f64,
    /// Two published tracks whose difference passes this position gate and
    /// `merge_vel_chi2` are duplicates of one object; the weaker is terminated.
    pub merge_pos_chi2: f64,
    /// Velocity part of the duplicate test (16.27 = 99.9% for 3 DOF).
    pub merge_vel_chi2: f64,
    /// How late an observation may be and still be fused exactly (s).
    pub retro_window_s: f64,
    /// Updates needed to confirm a tentative track.
    pub confirm_hits: u32,
    /// Tentative tracks not confirmed within this time are dropped silently (s).
    pub tentative_timeout_s: f64,
    /// Longest gap between a tentative track's observations (s).
    ///
    /// A tentative track has a velocity known only from a few close-together
    /// observations. Left unobserved, its gate grows with that uncertainty
    /// until it can capture another object's detection and confirm with a
    /// confident, wrong velocity (found by DST, `time_tornado`). A tentative
    /// track not re-observed within this gap is dropped, and it never gates a
    /// detection further than this from its last observation.
    pub tentative_max_gap_s: f64,
    /// A confirmed track without updates for this long starts coasting (s).
    pub coast_after_s: f64,
    /// A track without updates for this long is terminated (s).
    pub terminate_after_s: f64,
    /// Box consistency required to associate a detection with a track, to
    /// suppress a spawn near a track, and to call two tracks duplicates.
    pub box_gate: BoxGate,
    /// Process noise of track boxes.
    pub box_model: BoxModel,
    /// Systematic center error of the detector, as a standard deviation in
    /// fractions of the box's half-length and half-width (0 trusts the
    /// detector's covariance). Detectors that see only part of an object
    /// shift the center toward the visible side on every scan; that error
    /// does not average out, so it is added to published covariances (see
    /// [`crate::extent::center_sys_cov`]) rather than to the measurement noise.
    pub box_center_sys_frac: f64,
}

impl Default for LocalTrackerConfig {
    fn default() -> Self {
        Self {
            model: CvModel::default(),
            gate_chi2: 11.34,
            spawn_suppress_chi2: 21.11,
            merge_pos_chi2: 11.34,
            merge_vel_chi2: 16.27,
            retro_window_s: 0.5,
            confirm_hits: 3,
            tentative_timeout_s: 1.0,
            tentative_max_gap_s: 0.5,
            coast_after_s: 0.3,
            terminate_after_s: 1.5,
            box_gate: BoxGate::default(),
            box_model: BoxModel::default(),
            box_center_sys_frac: 0.0,
        }
    }
}

/// One of the agent's own tracks.
#[derive(Debug, Clone)]
pub struct LocalTrack {
    /// Track id, never reused.
    pub id: LocalTrackId,
    /// Current lifecycle state.
    pub lifecycle: TrackLifecycle,
    filter: RetrodictingFilter,
    /// Class id.
    pub class_id: u8,
    /// Running mean of detector confidence for `class_id`.
    pub class_prob: f32,
    /// Number of fused observations.
    pub hits: u32,
    /// Creation time (first observation).
    pub created: Timestamp,
    /// Latest observation time fused.
    pub last_update: Timestamp,
    /// Box estimate (odometry frame), if any detection carried one.
    object_box: Option<ObjectBox>,
    /// Time `object_box` refers to.
    box_t: Timestamp,
    box_model: BoxModel,
}

impl LocalTrack {
    /// Current estimate (after all fused observations).
    pub fn state(&self) -> &CvState {
        self.filter.state()
    }

    /// Estimate predicted to `t`.
    pub fn predicted(&self, t: Timestamp) -> CvState {
        self.filter.predicted(t)
    }

    /// Box estimate (odometry frame) predicted to `t`, if the track has one.
    pub fn predicted_box(&self, t: Timestamp) -> Option<ObjectBox> {
        self.object_box
            .map(|b| self.box_model.predict(&b, t.secs_since(self.box_t)))
    }

    /// Fuses a detection's box into the track's box, then lets a well-determined
    /// velocity pick front from back.
    fn update_box(&mut self, d: &Detection) {
        let Some(m) = d.object_box else {
            return;
        };
        let updated = match self.predicted_box(d.t) {
            Some(est) => self.box_model.update(&est, &m),
            None => m,
        };
        let st = self.filter.state();
        let v = st.x.fixed_rows::<3>(3);
        let speed = (v[0] * v[0] + v[1] * v[1]).sqrt();
        let speed_var = if speed > 0.0 {
            let u = Vector3::new(v[0] / speed, v[1] / speed, 0.0);
            let pv: Matrix3<f64> = st.p.fixed_view::<3, 3>(3, 3).into_owned();
            u.dot(&(pv * u))
        } else {
            f64::INFINITY
        };
        self.object_box = Some(
            self.box_model
                .resolve_with_velocity(&updated, v[0], v[1], speed_var),
        );
        self.box_t = self.box_t.max(d.t);
    }

    /// Whether this track is published.
    pub fn is_published(&self) -> bool {
        matches!(
            self.lifecycle,
            TrackLifecycle::Confirmed | TrackLifecycle::Coasting
        )
    }
}

/// What one scan did.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ScanOutcome {
    /// Detections fused into existing tracks.
    pub updated: usize,
    /// Tentative tracks started.
    pub created: usize,
    /// Detections rejected as older than the retrodiction window.
    pub stale: usize,
    /// Detections rejected as already processed.
    pub duplicate: usize,
    /// Detections near a published track that were not allowed to start a track.
    pub suppressed: usize,
    /// Duplicate tracks terminated by pruning.
    pub pruned: usize,
    /// Tentative tracks discarded because they duplicate a published track
    /// (never published, so no tombstone).
    pub discarded: usize,
}

/// The agent's local multi-object tracker.
#[derive(Debug, Clone)]
pub struct LocalTracker {
    config: LocalTrackerConfig,
    tracks: BTreeMap<LocalTrackId, LocalTrack>,
    next_id: u64,
    /// Observation ids already processed, with their times.
    seen: BTreeMap<u64, Timestamp>,
    /// Latest detection time seen.
    latest: Option<Timestamp>,
    /// Terminated tracks awaiting a tombstone report.
    terminated: Vec<LocalTrack>,
    /// Recent NIS values of fused observations (consistency diagnostic).
    nis: VecDeque<f64>,
}

impl LocalTracker {
    /// Creates an empty tracker.
    pub fn new(config: LocalTrackerConfig) -> Self {
        Self {
            config,
            tracks: BTreeMap::new(),
            next_id: 1,
            seen: BTreeMap::new(),
            latest: None,
            terminated: Vec::new(),
            nis: VecDeque::with_capacity(256),
        }
    }

    /// Configuration in use.
    pub fn config(&self) -> &LocalTrackerConfig {
        &self.config
    }

    /// All live tracks in id order.
    pub fn tracks(&self) -> impl Iterator<Item = &LocalTrack> {
        self.tracks.values()
    }

    /// A live track by id.
    pub fn track(&self, id: LocalTrackId) -> Option<&LocalTrack> {
        self.tracks.get(&id)
    }

    /// Mean NIS over the most recent fused observations (3 when consistent).
    pub fn mean_nis(&self) -> Option<f64> {
        (!self.nis.is_empty()).then(|| self.nis.iter().sum::<f64>() / self.nis.len() as f64)
    }

    /// Processes the detections of one scan (they may be late, out of order or repeated).
    pub fn process_scan(&mut self, dets: &[Detection]) -> ScanOutcome {
        let mut out = ScanOutcome::default();
        let window_ns = (self.config.retro_window_s * 1e9) as i64;

        // L1 deduplication by observation id.
        let mut fresh: Vec<Detection> = Vec::with_capacity(dets.len());
        for d in dets {
            if self.seen.contains_key(&d.obs_id) || fresh.iter().any(|f| f.obs_id == d.obs_id) {
                out.duplicate += 1;
            } else {
                fresh.push(*d);
            }
        }
        if let Some(newest) = fresh.iter().map(|d| d.t).max() {
            self.latest = Some(self.latest.map_or(newest, |l| l.max(newest)));
        }
        let latest = match self.latest {
            Some(l) => l,
            None => return out,
        };
        let horizon = Timestamp::from_nanos(latest.as_nanos() - window_ns);

        // Gate every (track, detection) pair at the detection's own time with the
        // NIS, and rank permitted pairs by negative log-likelihood, NIS + ln det S.
        // Ranking by NIS alone favors uncertain tracks (a large S shrinks the NIS),
        // so a fresh tentative track with unknown velocity would steal detections
        // from the established track of the same object and both would survive as
        // duplicates. The ln det S term penalizes that uncertainty.
        let ids: Vec<LocalTrackId> = self.tracks.keys().copied().collect();
        let gate = self.config.gate_chi2;
        let mut cost: Vec<Vec<f64>> = ids
            .iter()
            .map(|id| {
                let tr = &self.tracks[id];
                fresh
                    .iter()
                    .map(|d| {
                        if d.class_id != tr.class_id {
                            return f64::INFINITY;
                        }
                        if tr.lifecycle == TrackLifecycle::Tentative
                            && d.t.secs_since(tr.last_update).abs()
                                > self.config.tentative_max_gap_s
                        {
                            return f64::INFINITY;
                        }
                        if !self.box_ok(tr, d) {
                            return f64::INFINITY;
                        }
                        let pred = tr.predicted(d.t);
                        let (y, s) = pred.innovation(&d.z, &d.r);
                        match s.cholesky() {
                            Some(c) => {
                                let nis = y.dot(&c.solve(&y));
                                if nis < gate {
                                    nis + c.l().diagonal().iter().map(|v| 2.0 * v.ln()).sum::<f64>()
                                } else {
                                    f64::INFINITY
                                }
                            }
                            None => f64::INFINITY,
                        }
                    })
                    .collect()
            })
            .collect();
        // Shift permitted costs to be non-negative (ln det S can be negative).
        let min = cost
            .iter()
            .flatten()
            .copied()
            .filter(|c| c.is_finite())
            .fold(f64::INFINITY, f64::min);
        if min.is_finite() {
            for c in cost.iter_mut().flatten() {
                *c -= min;
            }
        }
        let mut pairs = if ids.is_empty() || fresh.is_empty() {
            Vec::new()
        } else {
            assign(&cost, f64::MAX)
        };

        // Second pass: published tracks left without a detection and detections
        // left without a track, gated at the spawn-suppression level. Without it
        // a detection between the two gates is neither associated (outside the
        // 99% gate) nor allowed to start a track (inside the 99.99% one) and is
        // lost; a young track with a poor velocity estimate can fall into that
        // band for many scans and starve while the object goes untracked (found
        // by DST, object_expiry). The first pass decides contested cases; this
        // pass only pairs leftovers.
        let free_tracks: Vec<usize> = (0..ids.len())
            .filter(|ti| !pairs.iter().any(|(t, _)| t == ti))
            .filter(|ti| self.tracks[&ids[*ti]].is_published())
            .collect();
        let free_dets: Vec<usize> = (0..fresh.len())
            .filter(|di| !pairs.iter().any(|(_, d)| d == di))
            .collect();
        if !free_tracks.is_empty() && !free_dets.is_empty() {
            let wide = self.config.spawn_suppress_chi2;
            let second: Vec<Vec<f64>> = free_tracks
                .iter()
                .map(|&ti| {
                    let tr = &self.tracks[&ids[ti]];
                    free_dets
                        .iter()
                        .map(|&di| {
                            let d = &fresh[di];
                            if d.class_id != tr.class_id || !self.box_ok(tr, d) {
                                return f64::INFINITY;
                            }
                            match tr.predicted(d.t).nis(&d.z, &d.r) {
                                Some(nis) if nis < wide => nis,
                                _ => f64::INFINITY,
                            }
                        })
                        .collect()
                })
                .collect();
            for (a, b) in assign(&second, f64::MAX) {
                pairs.push((free_tracks[a], free_dets[b]));
            }
        }

        let mut used = vec![false; fresh.len()];
        for (ti, di) in pairs {
            let d = fresh[di];
            used[di] = true;
            let tr = self
                .tracks
                .get_mut(&ids[ti])
                .expect("assigned track exists");
            match tr.filter.apply(BufferedObs {
                obs_id: d.obs_id,
                t: d.t,
                z: d.z,
                r: d.r,
            }) {
                Ok(nis) => {
                    tr.update_box(&d);
                    out.updated += 1;
                    tr.hits += 1;
                    tr.last_update = tr.last_update.max(d.t);
                    tr.class_prob += (d.class_prob - tr.class_prob) / tr.hits as f32;
                    if tr.lifecycle == TrackLifecycle::Coasting
                        || (tr.lifecycle == TrackLifecycle::Tentative
                            && tr.hits >= self.config.confirm_hits)
                    {
                        tr.lifecycle = TrackLifecycle::Confirmed;
                    }
                    if self.nis.len() == 256 {
                        self.nis.pop_front();
                    }
                    self.nis.push_back(nis);
                }
                Err(FilterError::TooOld) => out.stale += 1,
                Err(FilterError::Duplicate) => out.duplicate += 1,
                Err(FilterError::Numerical) => {}
            }
        }

        // Unassigned detections start tentative tracks, unless too old to trust:
        // a stale detection must not become a ghost track.
        for (d, _) in fresh.iter().zip(used.iter()).filter(|(_, u)| !**u) {
            if d.t < horizon {
                out.stale += 1;
                continue;
            }
            let near_known = self.tracks.values().any(|tr| {
                tr.is_published()
                    && tr.class_id == d.class_id
                    && self.box_ok(tr, d)
                    && tr
                        .predicted(d.t)
                        .nis(&d.z, &d.r)
                        .is_some_and(|n| n < self.config.spawn_suppress_chi2)
            });
            if near_known {
                out.suppressed += 1;
                continue;
            }
            let id = LocalTrackId(self.next_id);
            self.next_id += 1;
            let filter = RetrodictingFilter::new(
                BufferedObs {
                    obs_id: d.obs_id,
                    t: d.t,
                    z: d.z,
                    r: d.r,
                },
                self.config.model,
                self.config.retro_window_s,
            );
            let lifecycle = if self.config.confirm_hits <= 1 {
                TrackLifecycle::Confirmed
            } else {
                TrackLifecycle::Tentative
            };
            self.tracks.insert(
                id,
                LocalTrack {
                    id,
                    lifecycle,
                    filter,
                    class_id: d.class_id,
                    class_prob: d.class_prob,
                    hits: 1,
                    created: d.t,
                    last_update: d.t,
                    object_box: d.object_box,
                    box_t: d.t,
                    box_model: self.config.box_model,
                },
            );
            out.created += 1;
        }

        for d in &fresh {
            self.seen.insert(d.obs_id, d.t);
        }
        self.seen.retain(|_, t| *t >= horizon);
        out.discarded = self.discard_tentative_duplicates(latest);
        out.pruned = self.prune_duplicates(latest);
        out
    }

    /// Whether a detection's box is consistent with a track's predicted box.
    fn box_ok(&self, tr: &LocalTrack, d: &Detection) -> bool {
        self.config
            .box_gate
            .passes(tr.predicted_box(d.t).as_ref(), d.object_box.as_ref())
    }

    /// Drops tentative tracks that duplicate a published track of the same class.
    ///
    /// A detection in the far tail of an established track's gate can start a
    /// tentative track next to it, and later tail detections (outside the
    /// established gate, inside the newcomer's wide one) can feed it until it
    /// confirms. Such a track is statistically indistinguishable from the
    /// established one, so it is a split of one object, not a new object.
    /// Dropping it before confirmation keeps it from ever being published.
    /// Spawn suppression already refuses new tracks this close to a published
    /// one, so this does not hide genuinely new objects that spawn suppression
    /// would admit.
    fn discard_tentative_duplicates(&mut self, t: Timestamp) -> usize {
        let published: Vec<(u8, CvState, Option<ObjectBox>)> = self
            .tracks
            .values()
            .filter(|tr| tr.is_published())
            .map(|tr| (tr.class_id, tr.predicted(t), tr.predicted_box(t)))
            .collect();
        let doomed: Vec<LocalTrackId> = self
            .tracks
            .values()
            .filter(|tr| tr.lifecycle == TrackLifecycle::Tentative)
            .filter(|tr| {
                let x = tr.predicted(t);
                let xb = tr.predicted_box(t);
                published.iter().any(|(class, p, pb)| {
                    *class == tr.class_id
                        && self.config.box_gate.passes(xb.as_ref(), pb.as_ref())
                        && duplicate(
                            &x,
                            p,
                            self.config.merge_pos_chi2,
                            self.config.merge_vel_chi2,
                        )
                })
            })
            .map(|tr| tr.id)
            .collect();
        for id in &doomed {
            self.tracks.remove(id);
        }
        doomed.len()
    }

    /// Terminates published tracks that duplicate a stronger track of the same object.
    ///
    /// Two published tracks of one class are duplicates when their predicted
    /// difference at `t` passes both the position and the velocity gate. The
    /// track with fewer hits (then the higher id) is terminated, so a tombstone
    /// removes it from peers' world states at once rather than after it expires.
    fn prune_duplicates(&mut self, t: Timestamp) -> usize {
        let published: Vec<(LocalTrackId, u32, u8, CvState, Option<ObjectBox>)> = self
            .tracks
            .values()
            .filter(|tr| tr.is_published())
            .map(|tr| {
                (
                    tr.id,
                    tr.hits,
                    tr.class_id,
                    tr.predicted(t),
                    tr.predicted_box(t),
                )
            })
            .collect();
        let mut doomed = std::collections::BTreeSet::new();
        for (i, a) in published.iter().enumerate() {
            for b in &published[i + 1..] {
                if a.2 != b.2
                    || doomed.contains(&a.0)
                    || doomed.contains(&b.0)
                    || !self.config.box_gate.passes(a.4.as_ref(), b.4.as_ref())
                {
                    continue;
                }
                if duplicate(
                    &a.3,
                    &b.3,
                    self.config.merge_pos_chi2,
                    self.config.merge_vel_chi2,
                ) {
                    let weaker = if (a.1, std::cmp::Reverse(a.0)) >= (b.1, std::cmp::Reverse(b.0)) {
                        b.0
                    } else {
                        a.0
                    };
                    doomed.insert(weaker);
                }
            }
        }
        for id in &doomed {
            if let Some(mut tr) = self.tracks.remove(id) {
                tr.lifecycle = TrackLifecycle::Terminated;
                self.terminated.push(tr);
            }
        }
        doomed.len()
    }

    /// Advances lifecycles to time `now`.
    ///
    /// Tentative tracks that fail to confirm are dropped silently (they were
    /// never published). Published tracks coast, then terminate; terminated
    /// tracks are queued so the publisher can emit a tombstone.
    pub fn maintain(&mut self, now: Timestamp) {
        let cfg = self.config;
        let mut drop = Vec::new();
        for (id, tr) in self.tracks.iter_mut() {
            let idle = now.secs_since(tr.last_update);
            match tr.lifecycle {
                TrackLifecycle::Tentative => {
                    if now.secs_since(tr.created) > cfg.tentative_timeout_s
                        || now.secs_since(tr.last_update) > cfg.tentative_max_gap_s
                    {
                        drop.push((*id, false));
                    }
                }
                TrackLifecycle::Confirmed | TrackLifecycle::Coasting => {
                    if idle > cfg.terminate_after_s {
                        drop.push((*id, true));
                    } else if idle > cfg.coast_after_s {
                        tr.lifecycle = TrackLifecycle::Coasting;
                    }
                }
                TrackLifecycle::Terminated => drop.push((*id, true)),
            }
        }
        for (id, tombstone) in drop {
            if let Some(mut tr) = self.tracks.remove(&id) {
                if tombstone {
                    tr.lifecycle = TrackLifecycle::Terminated;
                    self.terminated.push(tr);
                }
            }
        }
    }

    /// Takes the tracks terminated since the last call.
    pub fn drain_terminated(&mut self) -> Vec<LocalTrack> {
        std::mem::take(&mut self.terminated)
    }
}

/// Whether two state estimates of one object are statistically indistinguishable.
fn duplicate(a: &CvState, b: &CvState, pos_chi2: f64, vel_chi2: f64) -> bool {
    let d = a.x - b.x;
    let s = a.p + b.p;
    let test = |off: usize, gate: f64| {
        let dv = d.fixed_rows::<3>(off).into_owned();
        let sv: Matrix3<f64> = s.fixed_view::<3, 3>(off, off).into_owned();
        sv.cholesky().is_some_and(|c| dv.dot(&c.solve(&dv)) < gate)
    };
    test(0, pos_chi2) && test(3, vel_chi2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::{Rng, SeedableRng};

    fn det(id: u64, t_ms: i64, x: f64, y: f64) -> Detection {
        Detection {
            obs_id: id,
            t: Timestamp::from_millis(t_ms),
            z: Vector3::new(x, y, 1.0),
            r: Matrix3::identity() * 0.09,
            class_id: 1,
            class_prob: 0.9,
            object_box: None,
        }
    }

    #[test]
    fn confirms_two_objects_and_keeps_them_apart() {
        let mut lt = LocalTracker::new(LocalTrackerConfig::default());
        let mut rng = rand::rngs::StdRng::seed_from_u64(1);
        let mut id = 0;
        for k in 0..30 {
            let t = k * 100;
            let s = k as f64 * 0.1;
            let mut scan = Vec::new();
            for (x0, vx) in [(0.0, 5.0), (20.0, -5.0)] {
                id += 1;
                scan.push(det(
                    id,
                    t,
                    x0 + vx * s + rng.gen_range(-0.2..0.2),
                    rng.gen_range(-0.2..0.2),
                ));
            }
            lt.process_scan(&scan);
            lt.maintain(Timestamp::from_millis(t));
        }
        let live: Vec<_> = lt.tracks().filter(|t| t.is_published()).collect();
        assert_eq!(live.len(), 2, "two crossing objects, two confirmed tracks");
        let mut xs: Vec<f64> = live.iter().map(|t| t.state().x[3]).collect();
        xs.sort_by(f64::total_cmp);
        assert!(
            (xs[0] + 5.0).abs() < 1.0 && (xs[1] - 5.0).abs() < 1.0,
            "velocities {xs:?}"
        );
    }

    #[test]
    fn duplicate_and_stale_detections_do_not_change_tracks() {
        let mut lt = LocalTracker::new(LocalTrackerConfig::default());
        for k in 0..10 {
            lt.process_scan(&[det(k, k as i64 * 100, k as f64, 0.0)]);
        }
        let before = *lt.tracks().next().unwrap().state();
        let out = lt.process_scan(&[det(5, 500, 5.0, 0.0), det(100, 100, 50.0, 50.0)]);
        assert_eq!(out.duplicate, 1);
        assert_eq!(
            out.stale, 1,
            "a 0.8 s old detection must not start a ghost track"
        );
        assert_eq!(out.created, 0);
        assert_eq!(*lt.tracks().next().unwrap().state(), before);
    }

    #[test]
    fn late_detection_within_window_is_fused_exactly() {
        let cfg = LocalTrackerConfig::default();
        let mut in_order = LocalTracker::new(cfg);
        let mut late = LocalTracker::new(cfg);
        let dets: Vec<_> = (0..12)
            .map(|k| det(k, k as i64 * 100, k as f64 * 0.5, 0.0))
            .collect();
        for d in &dets {
            in_order.process_scan(&[*d]);
        }
        // Detection 9 arrives after 10 and 11 (0.2 s late).
        for d in dets.iter().filter(|d| d.obs_id != 9) {
            late.process_scan(&[*d]);
        }
        late.process_scan(&[dets[9]]);
        let a = in_order.tracks().next().unwrap().state();
        let b = late.tracks().next().unwrap().state();
        assert!((a.x - b.x).norm() < 1e-9);
        assert_eq!(late.tracks().count(), 1);
    }

    #[test]
    fn lifecycle_coasts_then_terminates_by_time() {
        let mut lt = LocalTracker::new(LocalTrackerConfig::default());
        for k in 0..5 {
            lt.process_scan(&[det(k, k as i64 * 100, 0.0, 0.0)]);
        }
        lt.maintain(Timestamp::from_millis(400));
        assert_eq!(
            lt.tracks().next().unwrap().lifecycle,
            TrackLifecycle::Confirmed
        );
        lt.maintain(Timestamp::from_millis(1000));
        assert_eq!(
            lt.tracks().next().unwrap().lifecycle,
            TrackLifecycle::Coasting
        );
        lt.maintain(Timestamp::from_millis(2000));
        assert_eq!(lt.tracks().count(), 0);
        let ended = lt.drain_terminated();
        assert_eq!(ended.len(), 1);
        assert_eq!(ended[0].lifecycle, TrackLifecycle::Terminated);
        assert!(lt.drain_terminated().is_empty());
    }

    #[test]
    fn unconfirmed_tentative_tracks_vanish_without_tombstone() {
        let mut lt = LocalTracker::new(LocalTrackerConfig::default());
        lt.process_scan(&[det(1, 0, 0.0, 0.0)]);
        lt.maintain(Timestamp::from_millis(1500));
        assert_eq!(lt.tracks().count(), 0);
        assert!(lt.drain_terminated().is_empty());
    }

    #[test]
    fn established_track_wins_over_uncertain_newcomer() {
        // Regression: with NIS-only costs a fresh tentative track (huge velocity
        // covariance, so a large S and a small NIS) stole detections from the
        // established track of the same object, and both survived as duplicates.
        let mut rng = rand::rngs::StdRng::seed_from_u64(12);
        let mut noise = || {
            let u1: f64 = rng.gen_range(1e-12..1.0);
            let u2: f64 = rng.gen();
            (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos() * 0.3
        };
        let mut lt = LocalTracker::new(LocalTrackerConfig::default());
        let mut id = 0;
        let mut scan = |lt: &mut LocalTracker, k: u64, offset: f64, n: (f64, f64)| {
            id += 1;
            let mut d = det(id, k as i64 * 50, k as f64 * 0.15 + offset + n.0, n.1);
            d.r = Matrix3::identity() * 0.09;
            lt.process_scan(&[d]);
            lt.maintain(Timestamp::from_millis(k as i64 * 50));
        };
        for k in 0..20u64 {
            scan(&mut lt, k, 0.0, (noise(), noise()));
        }
        // An outlier 1.5 m off spawns a tentative track next to the object.
        scan(&mut lt, 20, 1.5, (0.0, 0.0));
        assert_eq!(lt.tracks().count(), 2);
        for k in 21..80u64 {
            scan(&mut lt, k, 0.0, (noise(), noise()));
            let published: Vec<_> = lt
                .tracks()
                .filter(|t| t.is_published())
                .map(|t| t.id)
                .collect();
            assert_eq!(
                published,
                vec![LocalTrackId(1)],
                "k={k}: only the established track may be published"
            );
        }
        // Later 1% outliers may start new tentative tracks; the first newcomer must be gone.
        assert!(
            lt.track(LocalTrackId(2)).is_none(),
            "the newcomer must starve and be dropped"
        );
    }

    fn with_box(mut d: Detection, length: f64, yaw: f64) -> Detection {
        d.object_box = Some(ObjectBox {
            size: [length, 2.0, 1.6],
            size_var: [0.04; 3],
            yaw,
            yaw_var: 0.0025,
            yaw_ambiguous: true,
        });
        d
    }

    #[test]
    fn boxes_refine_and_motion_resolves_front_from_back() {
        // A car moving +x at 3 m/s, detected with ambiguous, randomly flipped headings.
        let mut lt = LocalTracker::new(LocalTrackerConfig::default());
        for k in 0..30u64 {
            let flip = if k % 3 == 0 {
                std::f64::consts::PI
            } else {
                0.0
            };
            let d = with_box(
                det(k + 1, k as i64 * 100, 0.3 * k as f64, 0.0),
                4.5 + 0.1 * ((k % 5) as f64 - 2.0),
                crate::extent::wrap_pi(0.02 + flip),
            );
            lt.process_scan(&[d]);
        }
        let tr = lt.tracks().next().unwrap();
        let b = tr.predicted_box(Timestamp::from_millis(2900)).unwrap();
        assert!((b.size[0] - 4.5).abs() < 0.1, "length {}", b.size[0]);
        assert!(!b.yaw_ambiguous, "motion resolved the heading");
        assert!(
            crate::extent::wrap_pi(b.yaw - 0.02).abs() < 0.05,
            "points along travel: {}",
            b.yaw
        );
    }

    #[test]
    fn a_truck_beside_a_car_track_starts_its_own_track() {
        // A truck detection inside the car track's position gate does not associate
        // with it and is not suppressed as tail noise: the sizes disagree.
        let mut lt = LocalTracker::new(LocalTrackerConfig::default());
        for k in 0..10u64 {
            lt.process_scan(&[with_box(det(k + 1, k as i64 * 100, 0.0, 0.0), 4.5, 0.0)]);
        }
        let truck = with_box(det(100, 1000, 0.0, 0.8), 12.0, 0.0);
        let out = lt.process_scan(&[truck]);
        assert_eq!((out.updated, out.created, out.suppressed), (0, 1, 0));
        // The same detection without a box would have been associated (or suppressed).
        let mut lt2 = LocalTracker::new(LocalTrackerConfig::default());
        for k in 0..10u64 {
            lt2.process_scan(&[with_box(det(k + 1, k as i64 * 100, 0.0, 0.0), 4.5, 0.0)]);
        }
        let mut plain = truck;
        plain.object_box = None;
        let out = lt2.process_scan(&[plain]);
        assert_eq!(out.created, 0, "without a box, position decides");
    }

    #[test]
    fn a_stale_tentative_track_cannot_capture_a_distant_detection() {
        // Regression (found by DST time_tornado): two detections 50 ms apart
        // start a tentative track with a velocity known to about 10 m/s. Almost
        // a second later its gate spans tens of meters, and it captured another
        // object's detection 25 m away, confirming with a confident wrong
        // velocity. It must neither gate across the gap nor survive it.
        let mut lt = LocalTracker::new(LocalTrackerConfig::default());
        let mut a = det(1, 0, 0.0, 0.0);
        a.r = Matrix3::identity() * 0.09;
        let mut b = det(2, 50, 1.0, 0.0);
        b.r = Matrix3::identity() * 0.09;
        lt.process_scan(&[a]);
        lt.process_scan(&[b]);
        assert_eq!(lt.track(LocalTrackId(1)).unwrap().hits, 2);
        let mut far = det(3, 1000, 22.0, 4.0);
        far.r = Matrix3::identity() * 0.09;
        let nis = lt
            .track(LocalTrackId(1))
            .unwrap()
            .predicted(far.t)
            .nis(&far.z, &far.r)
            .unwrap();
        assert!(
            nis < 11.34,
            "test premise: the stale track's gate covers the detection (NIS {nis})"
        );
        let out = lt.process_scan(&[far]);
        assert_eq!(
            (out.updated, out.created),
            (0, 1),
            "the detection starts its own track"
        );
        lt.maintain(Timestamp::from_millis(1000));
        assert!(
            lt.track(LocalTrackId(1)).is_none(),
            "the stale tentative track is dropped"
        );
        assert!(
            lt.drain_terminated().is_empty(),
            "it was never published, so no tombstone"
        );
    }

    #[test]
    fn detection_between_the_gates_updates_the_unassociated_track() {
        // Regression (found by DST object_expiry): a detection outside the 99%
        // association gate but inside the spawn-suppression gate of a track that
        // has no other detection was dropped. A young track with a poor velocity
        // estimate starved this way while its object went untracked.
        let mut lt = LocalTracker::new(LocalTrackerConfig::default());
        for k in 0..40u64 {
            let mut d = det(k + 1, k as i64 * 50, 0.0, 0.0);
            d.r = Matrix3::identity() * 0.09;
            lt.process_scan(&[d]);
        }
        let mut d = det(100, 40 * 50, 1.3, 0.0);
        d.r = Matrix3::identity() * 0.09;
        let nis = lt
            .tracks()
            .next()
            .unwrap()
            .predicted(d.t)
            .nis(&d.z, &d.r)
            .unwrap();
        let cfg = LocalTrackerConfig::default();
        assert!(
            nis > cfg.gate_chi2 && nis < cfg.spawn_suppress_chi2,
            "test premise: NIS {nis}"
        );
        let out = lt.process_scan(&[d]);
        assert_eq!((out.updated, out.created, out.suppressed), (1, 0, 0));
        assert_eq!(lt.tracks().count(), 1);
    }

    #[test]
    fn tail_detections_never_confirm_a_split_track() {
        // Regression (found by DST slow_loris, seed 33): a detection in the far
        // tail of the established gate starts a tentative track; two later tail
        // detections fail the established gate but pass the newcomer's wide one,
        // so it reached three hits and was published as a duplicate.
        let mut lt = LocalTracker::new(LocalTrackerConfig::default());
        let mut id = 0;
        let mut scan = |lt: &mut LocalTracker, k: u64, offset: f64| {
            id += 1;
            let mut d = det(id, k as i64 * 50, offset, 0.0);
            d.r = Matrix3::identity() * 0.09;
            let out = lt.process_scan(&[d]);
            lt.maintain(Timestamp::from_millis(k as i64 * 50));
            out
        };
        for k in 0..40u64 {
            scan(&mut lt, k, 0.0);
        }
        let mut discarded = 0;
        for (k, offset) in [(40u64, 1.6), (41, 0.0), (42, 1.3), (43, 0.0), (44, 1.3)] {
            discarded += scan(&mut lt, k, offset).discarded;
            let published: Vec<_> = lt
                .tracks()
                .filter(|t| t.is_published())
                .map(|t| t.id)
                .collect();
            assert_eq!(published, vec![LocalTrackId(1)], "k={k}");
        }
        assert!(discarded >= 1, "the split track was discarded, not starved");
        assert!(
            lt.drain_terminated().is_empty(),
            "nothing published was terminated"
        );
    }

    #[test]
    fn duplicate_tracks_of_one_object_are_pruned_with_a_tombstone() {
        // Over many noisy runs a single object must never keep two published tracks
        // for longer than one scan, and every pruned track leaves a tombstone.
        for seed in 0..60u64 {
            let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
            let mut n = || {
                let u1: f64 = rng.gen_range(1e-12..1.0);
                let u2: f64 = rng.gen();
                (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos() * 0.3
            };
            let mut lt = LocalTracker::new(LocalTrackerConfig::default());
            let mut pruned = 0;
            for k in 0..60u64 {
                let p = Vector3::new(-1.4, -0.1, 0.0) * (k as f64 * 0.05);
                let mut d = det(k + 1, k as i64 * 50, 0.0, 0.0);
                d.z = p + Vector3::new(n(), n(), n());
                pruned += lt.process_scan(&[d]).pruned;
                lt.maintain(Timestamp::from_millis(k as i64 * 50));
                let published = lt.tracks().filter(|t| t.is_published()).count();
                assert!(
                    published <= 1,
                    "seed {seed} k={k}: {published} published tracks for one object"
                );
            }
            assert_eq!(
                lt.drain_terminated().len(),
                pruned,
                "seed {seed}: every pruned track is tombstoned"
            );
        }
    }

    #[test]
    fn close_but_distinct_objects_are_not_pruned() {
        // Two objects 1.5 m apart moving perpendicular: never duplicates.
        let mut lt = LocalTracker::new(LocalTrackerConfig::default());
        let mut id = 0;
        for k in 0..40u64 {
            let s = k as f64 * 0.05;
            id += 2;
            let a = det(id, k as i64 * 50, -3.0 + 6.0 * s, 0.0);
            let b = det(id + 1, k as i64 * 50, 1.5, -3.0 + 6.0 * s);
            let out = lt.process_scan(&[a, b]);
            assert_eq!(out.pruned, 0, "k={k}");
            lt.maintain(Timestamp::from_millis(k as i64 * 50));
        }
        assert_eq!(lt.tracks().filter(|t| t.is_published()).count(), 2);
    }

    #[test]
    fn class_mismatch_never_associates() {
        let mut lt = LocalTracker::new(LocalTrackerConfig::default());
        lt.process_scan(&[det(1, 0, 0.0, 0.0)]);
        let mut other = det(2, 100, 0.0, 0.0);
        other.class_id = 2;
        let out = lt.process_scan(&[other]);
        assert_eq!(out.created, 1);
        assert_eq!(lt.tracks().count(), 2);
    }

    #[test]
    fn filter_consistency_over_monte_carlo() {
        // I2 (local): mean NIS of fused observations is near 3 for a correctly modeled target.
        let mut rng = rand::rngs::StdRng::seed_from_u64(9);
        let mut total = 0.0;
        let mut n = 0;
        for trial in 0..40 {
            let mut lt = LocalTracker::new(LocalTrackerConfig::default());
            let mut v = Vector3::new(4.0, -2.0, 0.0);
            let mut p = Vector3::new(0.0, 0.0, 1.0);
            for k in 0..60u64 {
                // Truth with white acceleration matching the filter's PSD (q = 1, 1, 0.25).
                let dt = 0.1;
                let a = Vector3::new(rng.gen::<f64>() - 0.5, rng.gen::<f64>() - 0.5, 0.0)
                    * (12.0f64 / dt).sqrt();
                p += v * dt + a * dt * dt / 2.0;
                v += a * dt;
                let noise = Vector3::from_fn(|_, _| {
                    let u1: f64 = rng.gen_range(1e-12..1.0);
                    let u2: f64 = rng.gen();
                    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos() * 0.3
                });
                let mut d = det(trial * 1000 + k, k as i64 * 100, 0.0, 0.0);
                d.z = p + noise;
                lt.process_scan(&[d]);
            }
            if let Some(m) = lt.mean_nis() {
                total += m;
                n += 1;
            }
        }
        let mean = total / n as f64;
        assert!((2.0..4.0).contains(&mean), "mean NIS {mean}");
    }
}

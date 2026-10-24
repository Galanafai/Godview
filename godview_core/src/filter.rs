//! Per-track estimation: a constant-velocity Kalman filter with exact
//! out-of-sequence handling by retrodiction (rollback and replay).
//!
//! Every local track owns one [`RetrodictingFilter`]. Measurements may arrive
//! late (sensor pipelines, batching) and out of order. Within a bounded
//! window the filter keeps the not-yet-folded observations, sorted by time,
//! together with the state *before* the first of them. A late observation is
//! inserted in time order and the window is replayed from that base state,
//! so the result is identical to having received every observation in order
//! (the replay performs the same floating point operations in the same
//! order). Observations older than the window are rejected, never silently
//! applied to the wrong time (audit finding F-16).
//!
//! The filter is frame-agnostic: callers keep it in one Cartesian frame
//! (the agent's odometry frame for local tracks).

use crate::frames::{cv_process_noise, cv_transition, symmetrize};
use godview_env::Timestamp;
use nalgebra::{Matrix3, Matrix6, Vector3, Vector6};
use std::collections::BTreeSet;

/// Configuration for constant-velocity tracking.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CvModel {
    /// Acceleration power spectral density along x, y, z of the filter frame (m²/s³).
    pub accel_psd: Vector3<f64>,
    /// Standard deviation of the initial velocity (m/s).
    pub init_vel_sigma: f64,
}

impl Default for CvModel {
    fn default() -> Self {
        Self {
            accel_psd: Vector3::new(1.0, 1.0, 0.25),
            init_vel_sigma: 10.0,
        }
    }
}

/// A Gaussian state estimate at a point in time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CvState {
    /// `[px, py, pz, vx, vy, vz]` in the filter frame.
    pub x: Vector6<f64>,
    /// State covariance.
    pub p: Matrix6<f64>,
    /// Time the estimate refers to.
    pub t: Timestamp,
}

impl CvState {
    /// Initializes a state from a single position measurement.
    pub fn from_measurement(
        t: Timestamp,
        z: &Vector3<f64>,
        r: &Matrix3<f64>,
        model: &CvModel,
    ) -> Self {
        let mut p = Matrix6::zeros();
        p.fixed_view_mut::<3, 3>(0, 0).copy_from(r);
        let vv = model.init_vel_sigma * model.init_vel_sigma;
        p[(3, 3)] = vv;
        p[(4, 4)] = vv;
        p[(5, 5)] = vv;
        Self {
            x: Vector6::new(z.x, z.y, z.z, 0.0, 0.0, 0.0),
            p,
            t,
        }
    }

    /// Predicts the state to time `t` (forward or backward) without mutating `self`.
    ///
    /// Process noise uses `|dt|`, so a backward prediction is conservative.
    pub fn predicted(&self, t: Timestamp, model: &CvModel) -> CvState {
        let dt = t.secs_since(self.t);
        if dt == 0.0 {
            return *self;
        }
        let f = cv_transition(dt);
        let q = cv_process_noise(dt, &model.accel_psd, &Matrix3::identity());
        CvState {
            x: f * self.x,
            p: symmetrize(&(f * self.p * f.transpose() + q)),
            t,
        }
    }

    /// Position innovation and its covariance for a measurement at this state's time.
    pub fn innovation(&self, z: &Vector3<f64>, r: &Matrix3<f64>) -> (Vector3<f64>, Matrix3<f64>) {
        let y = z - self.x.fixed_rows::<3>(0);
        let s = self.p.fixed_view::<3, 3>(0, 0) + r;
        (y, s)
    }

    /// Squared Mahalanobis distance of a position measurement (NIS), or `None` if `S` is singular.
    pub fn nis(&self, z: &Vector3<f64>, r: &Matrix3<f64>) -> Option<f64> {
        let (y, s) = self.innovation(z, r);
        let chol = s.cholesky()?;
        Some(y.dot(&chol.solve(&y)))
    }

    /// Kalman update with a position measurement at this state's time (Joseph form).
    ///
    /// Returns the normalized innovation squared, or `None` if `S` is singular
    /// (the state is left unchanged in that case).
    pub fn update(&mut self, z: &Vector3<f64>, r: &Matrix3<f64>) -> Option<f64> {
        let (y, s) = self.innovation(z, r);
        let chol = s.cholesky()?;
        let nis = y.dot(&chol.solve(&y));
        // K = P Hᵀ S⁻¹ with H = [I 0].
        let pht = self.p.fixed_view::<6, 3>(0, 0).into_owned();
        let k = pht * chol.inverse();
        self.x += k * y;
        let mut ikh = Matrix6::identity();
        ikh.fixed_view_mut::<6, 3>(0, 0)
            .copy_from(&(Matrix6::identity().fixed_view::<6, 3>(0, 0) - k));
        self.p = symmetrize(&(ikh * self.p * ikh.transpose() + k * r * k.transpose()));
        Some(nis)
    }
}

/// A position observation retained for out-of-sequence replay.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BufferedObs {
    /// Agent-local observation id (unique per agent).
    pub obs_id: u64,
    /// Measurement time.
    pub t: Timestamp,
    /// Measured position in the filter frame.
    pub z: Vector3<f64>,
    /// Measurement covariance.
    pub r: Matrix3<f64>,
}

/// Why an observation was not applied.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum FilterError {
    /// The observation predates the retrodiction window.
    #[error("observation older than the retrodiction window")]
    TooOld,
    /// The observation id was already applied.
    #[error("duplicate observation")]
    Duplicate,
    /// The innovation covariance was singular or non-finite.
    #[error("numerical failure in update")]
    Numerical,
}

/// Constant-velocity filter with exact out-of-sequence retrodiction.
///
/// The track's very first observation is part of the replay window too, so a
/// late detection that predates the track's birth (common when the first
/// detections of a new object arrive out of order) re-initializes the filter
/// from the earliest observation instead of being rejected.
#[derive(Debug, Clone)]
pub struct RetrodictingFilter {
    model: CvModel,
    window_ns: i64,
    /// State after all folded observations; `None` until the first fold, in
    /// which case replay initializes from `pending[0]`.
    base: Option<CvState>,
    /// Observations not yet folded, sorted by `(t, obs_id)`. Never empty.
    pending: Vec<BufferedObs>,
    /// State after applying `pending` to `base`.
    current: CvState,
    /// Ids already applied, with their times; pruned below the oldest accepted time.
    seen: BTreeSet<(Timestamp, u64)>,
    /// NIS of the most recently applied observation.
    last_nis: Option<f64>,
}

impl RetrodictingFilter {
    /// Starts a filter from its first observation.
    pub fn new(first: BufferedObs, model: CvModel, window_s: f64) -> Self {
        let init = CvState::from_measurement(first.t, &first.z, &first.r, &model);
        let mut seen = BTreeSet::new();
        seen.insert((first.t, first.obs_id));
        Self {
            model,
            window_ns: (window_s * 1e9).round() as i64,
            base: None,
            pending: vec![first],
            current: init,
            seen,
            last_nis: None,
        }
    }

    /// The motion model.
    pub fn model(&self) -> &CvModel {
        &self.model
    }

    /// Estimate after every applied observation.
    pub fn state(&self) -> &CvState {
        &self.current
    }

    /// Estimate predicted to `t` (does not change the filter).
    pub fn predicted(&self, t: Timestamp) -> CvState {
        self.current.predicted(t, &self.model)
    }

    /// Earliest measurement time still accepted.
    pub fn oldest_accepted(&self) -> Timestamp {
        let window_start = Timestamp::from_nanos(self.current.t.as_nanos() - self.window_ns);
        match self.base {
            Some(b) => b.t.max(window_start),
            None => window_start,
        }
    }

    /// NIS of the most recently applied observation.
    pub fn last_nis(&self) -> Option<f64> {
        self.last_nis
    }

    /// Applies an observation, replaying the window if it arrived out of order.
    ///
    /// Returns the observation's NIS against the prediction it was fused with
    /// (0 if it became the track's initializing observation).
    pub fn apply(&mut self, obs: BufferedObs) -> Result<f64, FilterError> {
        if obs.t < self.oldest_accepted() {
            return Err(FilterError::TooOld);
        }
        if self.seen.iter().any(|(_, id)| *id == obs.obs_id) {
            return Err(FilterError::Duplicate);
        }
        let in_order = self
            .pending
            .last()
            .is_none_or(|l| (l.t, l.obs_id) <= (obs.t, obs.obs_id))
            && obs.t >= self.current.t;
        let nis = if in_order {
            let mut next = self.current.predicted(obs.t, &self.model);
            let nis = next.update(&obs.z, &obs.r).ok_or(FilterError::Numerical)?;
            self.current = next;
            self.pending.push(obs);
            nis
        } else {
            let idx = self
                .pending
                .partition_point(|p| (p.t, p.obs_id) < (obs.t, obs.obs_id));
            self.pending.insert(idx, obs);
            let (state, nis_of_new) = self.replay(Some(idx))?;
            self.current = state;
            nis_of_new
        };
        self.seen.insert((obs.t, obs.obs_id));
        self.last_nis = Some(nis);
        self.fold();
        Ok(nis)
    }

    /// Re-runs `pending` from `base`, returning the final state and the NIS of
    /// the observation at `mark`.
    fn replay(&self, mark: Option<usize>) -> Result<(CvState, f64), FilterError> {
        let mut nis_at_mark = 0.0;
        let (mut state, start) = match self.base {
            Some(b) => (b, 0),
            None => {
                let f = &self.pending[0];
                (CvState::from_measurement(f.t, &f.z, &f.r, &self.model), 1)
            }
        };
        for (i, p) in self.pending.iter().enumerate().skip(start) {
            state = state.predicted(p.t, &self.model);
            let n = state.update(&p.z, &p.r).ok_or(FilterError::Numerical)?;
            if Some(i) == mark {
                nis_at_mark = n;
            }
        }
        Ok((state, nis_at_mark))
    }

    /// Folds observations that fell out of the window into the base state.
    ///
    /// Folding performs exactly the operations a replay would, in the same
    /// order, so it never changes `current`.
    fn fold(&mut self) {
        let cutoff = Timestamp::from_nanos(self.current.t.as_nanos() - self.window_ns);
        let mut folded = 0;
        for p in &self.pending {
            if p.t >= cutoff || folded + 1 == self.pending.len() {
                break;
            }
            self.base = Some(match self.base {
                None => CvState::from_measurement(p.t, &p.z, &p.r, &self.model),
                Some(b) => {
                    let mut s = b.predicted(p.t, &self.model);
                    if s.update(&p.z, &p.r).is_none() {
                        b
                    } else {
                        s
                    }
                }
            });
            folded += 1;
        }
        if folded > 0 {
            self.pending.drain(..folded);
            let oldest = self.oldest_accepted();
            self.seen.retain(|(t, _)| *t >= oldest);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::{Rng, SeedableRng};

    fn obs(id: u64, t_ms: i64, x: f64) -> BufferedObs {
        BufferedObs {
            obs_id: id,
            t: Timestamp::from_millis(t_ms),
            z: Vector3::new(x, 0.5 * x, 0.0),
            r: Matrix3::identity() * 0.25,
        }
    }

    fn truth_track(rng: &mut impl Rng, n: usize) -> Vec<BufferedObs> {
        (0..n)
            .map(|i| {
                let t = i as f64 * 0.1;
                let noise: f64 = rng.gen_range(-0.5..0.5);
                obs(i as u64, (t * 1000.0) as i64, 3.0 * t + noise)
            })
            .collect()
    }

    #[test]
    fn retrodiction_equals_in_order_filtering() {
        let mut rng = rand::rngs::StdRng::seed_from_u64(3);
        let all = truth_track(&mut rng, 40);
        let model = CvModel::default();

        let mut in_order = RetrodictingFilter::new(all[0], model, 0.5);
        for o in &all[1..] {
            in_order.apply(*o).unwrap();
        }

        // Deliver with bounded jitter (each obs at most 3 steps = 0.3 s late).
        let mut delivery: Vec<(usize, BufferedObs)> = all[1..]
            .iter()
            .enumerate()
            .map(|(i, o)| (i + rng.gen_range(0..4), *o))
            .collect();
        delivery.sort_by_key(|(k, o)| (*k, o.obs_id));
        let mut shuffled = RetrodictingFilter::new(all[0], model, 0.5);
        let mut replays = 0;
        for (_, o) in delivery {
            let before = shuffled.state().t;
            shuffled.apply(o).unwrap();
            if o.t < before {
                replays += 1;
            }
        }
        assert!(replays > 5, "test must exercise out-of-order delivery");
        let a = in_order.state();
        let b = shuffled.state();
        assert_eq!(a.t, b.t);
        assert!(
            (a.x - b.x).norm() < 1e-9,
            "state differs by {}",
            (a.x - b.x).norm()
        );
        assert!((a.p - b.p).norm() < 1e-9);
    }

    #[test]
    fn rejects_observations_older_than_window() {
        let model = CvModel::default();
        let mut f = RetrodictingFilter::new(obs(0, 0, 0.0), model, 0.5);
        for i in 1..20 {
            f.apply(obs(i, i as i64 * 100, i as f64 * 0.3)).unwrap();
        }
        let before = *f.state();
        assert_eq!(f.apply(obs(99, 200, 50.0)), Err(FilterError::TooOld));
        assert_eq!(
            *f.state(),
            before,
            "a rejected observation must not change the estimate"
        );
    }

    #[test]
    fn rejects_duplicates_in_and_out_of_order() {
        let model = CvModel::default();
        let mut f = RetrodictingFilter::new(obs(0, 0, 0.0), model, 1.0);
        f.apply(obs(1, 100, 0.3)).unwrap();
        f.apply(obs(2, 200, 0.6)).unwrap();
        let before = *f.state();
        assert_eq!(f.apply(obs(2, 200, 0.6)), Err(FilterError::Duplicate));
        assert_eq!(f.apply(obs(1, 100, 0.3)), Err(FilterError::Duplicate));
        assert_eq!(*f.state(), before);
    }

    #[test]
    fn kalman_update_gains_information_unlike_ci() {
        // Regression for audit finding F-07: CI on raw measurements never reduced variance.
        let model = CvModel {
            accel_psd: Vector3::zeros(),
            init_vel_sigma: 10.0,
        };
        let mut f = RetrodictingFilter::new(obs(0, 0, 0.0), model, 10.0);
        for i in 1..31 {
            f.apply(BufferedObs {
                t: Timestamp::from_millis(0),
                ..obs(i, 0, 0.0)
            })
            .unwrap();
        }
        // 31 measurements of a static point, R = 0.25: variance 0.25 / 31.
        assert!((f.state().p[(0, 0)] - 0.25 / 31.0).abs() < 1e-9);
    }

    #[test]
    fn observation_before_the_first_one_reinitializes_exactly() {
        // A track born from a detection at t = 200 ms receives an older detection
        // (t = 100 ms) late: it must be fused as if it had arrived first.
        let model = CvModel::default();
        let a = obs(1, 100, 0.3);
        let b = obs(2, 200, 0.6);
        let c = obs(3, 300, 0.9);
        let mut in_order = RetrodictingFilter::new(a, model, 0.5);
        in_order.apply(b).unwrap();
        in_order.apply(c).unwrap();
        let mut late = RetrodictingFilter::new(b, model, 0.5);
        late.apply(c).unwrap();
        late.apply(a).unwrap();
        assert!((in_order.state().x - late.state().x).norm() < 1e-12);
        assert!((in_order.state().p - late.state().p).norm() < 1e-12);
    }

    #[test]
    fn folding_does_not_change_the_estimate() {
        let model = CvModel::default();
        let mut long = RetrodictingFilter::new(obs(0, 0, 0.0), model, 100.0);
        let mut short = RetrodictingFilter::new(obs(0, 0, 0.0), model, 0.25);
        for i in 1..30 {
            long.apply(obs(i, i as i64 * 100, i as f64 * 0.3)).unwrap();
            short.apply(obs(i, i as i64 * 100, i as f64 * 0.3)).unwrap();
        }
        assert!((long.state().x - short.state().x).norm() < 1e-9);
        assert!(short.oldest_accepted() > Timestamp::from_millis(2000));
    }

    #[test]
    fn prediction_grows_uncertainty_with_time() {
        let s = CvState::from_measurement(
            Timestamp::ZERO,
            &Vector3::zeros(),
            &Matrix3::identity(),
            &CvModel::default(),
        );
        let fwd = s.predicted(Timestamp::from_millis(1000), &CvModel::default());
        let back = s.predicted(Timestamp::from_millis(-1000), &CvModel::default());
        assert!(fwd.p.trace() > s.p.trace());
        assert!(back.p.trace() > s.p.trace());
    }
}

//! The perception-sharing wire contract: [`TrackReport`] and signed [`ReportBatch`]es.
//!
//! # What is shared, and why
//!
//! Agents publish **source-pure local tracks**: each report describes one of
//! the source's own L2 tracks, estimated only from the source's own sensors,
//! transformed into the shared frame at publish time with the source's pose
//! uncertainty folded into the covariance. Receivers never re-publish fused
//! (L3) estimates as reports. This rule, plus last-writer-wins replacement by
//! `(epoch, seq)`, is what prevents double counting (see
//! `docs/PERCEPTION_SHARING.md`).
//!
//! # Encoding
//!
//! A batch is a canonical little-endian byte string followed by an Ed25519
//! signature over all preceding bytes. There is no self-describing format; the
//! one optional field (the box) has a tag byte, so equal content always means
//! equal bytes, and every decoded value is exactly what the signer signed. The
//! layout is pinned by a golden-bytes test; changing it requires bumping
//! [`SCHEMA_VERSION`]. Schemas 1 (no box, no birth time) and 2 (no birth
//! time) are still decoded; their reports get `born = t_valid`.
//!
//! ```text
//! batch   := "GVR" schema:u8 source:[u8;32] epoch:u32 sent_at:i64 count:u16 report* signature:[u8;64]
//! report  := seq:u64 track:u64 t_valid:i64 frame pos:f64×3 vel:f32×3 cov:f32×21
//!            class:u8 class_prob:f32 p_exist:f32 lifecycle:u8 valid_for_ms:u32 born box
//! frame   := 0u8 (ECEF) | 1u8 frame_id:u64 (source-local frame)
//! box     := 0u8 (none) | 1u8 size:f32×3 size_var:f32×3 yaw:f32 yaw_var:f32 flags:u8
//!            (schema 2 and later; flags bit 0 = heading ambiguous modulo π)
//! born    := i64 ns, when the source's track was created (schema 3 and later)
//! ```
//!
//! A box's heading is counterclockwise from local East at the object for
//! ECEF reports, and from the source frame's x axis for local-frame reports.

use crate::extent::ObjectBox;
use crate::frames::{is_spd, unpack_upper6};
use ed25519_dalek::{Signature, Signer, SigningKey, Verifier, VerifyingKey};
use godview_env::Timestamp;
use nalgebra::{Matrix6, Vector3, Vector6};

/// Current wire schema.
pub const SCHEMA_VERSION: u8 = 3;
/// Schema 2 (no birth time), still accepted when decoding.
pub const SCHEMA_VERSION_V2: u8 = 2;
/// Schema 1 (no box, no birth time), still accepted when decoding.
pub const SCHEMA_VERSION_V1: u8 = 1;
const MAGIC: &[u8; 3] = b"GVR";
/// Maximum reports in one batch.
pub const MAX_REPORTS_PER_BATCH: usize = 4096;
/// Smallest accepted position variance (m²): a 1 cm standard deviation.
///
/// Claims of more precision than this are rejected as implausible, which
/// bounds how much a single report can dominate a fused estimate.
pub const MIN_POS_VARIANCE: f64 = 1e-4;
/// Smallest accepted velocity variance ((m/s)²).
pub const MIN_VEL_VARIANCE: f64 = 1e-6;
/// Largest accepted variance on any axis (m² or (m/s)²).
pub const MAX_VARIANCE: f64 = 1e12;

/// Identity of a publishing agent: its Ed25519 verifying key.
///
/// Using the key itself (rather than a separately asserted name) binds every
/// report's provenance to the signature that carries it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct AgentId(pub [u8; 32]);

impl AgentId {
    /// The agent id for a signing key.
    pub fn from_signing_key(key: &SigningKey) -> Self {
        Self(key.verifying_key().to_bytes())
    }

    /// Short hex form for logs.
    pub fn short(&self) -> String {
        self.0[..4].iter().map(|b| format!("{b:02x}")).collect()
    }
}

impl std::fmt::Display for AgentId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.short())
    }
}

/// A local track id, unique within one source and epoch, never reused.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct LocalTrackId(pub u64);

/// Globally unique key of a shared track: `(source, local track id)`.
pub type ReportKey = (AgentId, LocalTrackId);

/// Track lifecycle as seen by its source.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum TrackLifecycle {
    /// Not yet confirmed; never published.
    Tentative,
    /// Updated recently.
    Confirmed,
    /// No recent update; prediction only.
    Coasting,
    /// Ended; a published report with this state is a tombstone.
    Terminated,
}

impl TrackLifecycle {
    fn to_u8(self) -> u8 {
        match self {
            TrackLifecycle::Tentative => 0,
            TrackLifecycle::Confirmed => 1,
            TrackLifecycle::Coasting => 2,
            TrackLifecycle::Terminated => 3,
        }
    }

    fn from_u8(v: u8) -> Result<Self, ReportError> {
        Ok(match v {
            0 => TrackLifecycle::Tentative,
            1 => TrackLifecycle::Confirmed,
            2 => TrackLifecycle::Coasting,
            3 => TrackLifecycle::Terminated,
            other => return Err(ReportError::BadLifecycle(other)),
        })
    }
}

/// The frame a report's state is expressed in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FrameRef {
    /// WGS84 ECEF: globally referenced, fusable by every receiver.
    Ecef,
    /// A source-local frame (the source had no valid global pose).
    ///
    /// Receivers replicate these reports but exclude them from global fusion.
    Local {
        /// Source-chosen frame id (unique per source).
        frame_id: u64,
    },
}

/// One shared estimate of one of the source's local tracks.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrackReport {
    /// Publishing agent (from the batch).
    pub source: AgentId,
    /// Source boot epoch (from the batch); `(epoch, seq)` orders versions.
    pub epoch: u32,
    /// Per-source sequence number, strictly increasing within an epoch.
    pub seq: u64,
    /// The source's local track id.
    pub track: LocalTrackId,
    /// Time the estimate refers to.
    pub t_valid: Timestamp,
    /// Frame of `pos`, `vel` and `cov`.
    pub frame: FrameRef,
    /// Position (m).
    pub pos: [f64; 3],
    /// Velocity (m/s).
    pub vel: [f32; 3],
    /// Packed upper triangle of the 6x6 state covariance; includes pose and clock error.
    pub cov: [f32; 21],
    /// Object class id.
    pub class_id: u8,
    /// Probability of `class_id` in `[0, 1]`.
    pub class_prob: f32,
    /// Probability the object exists in `[0, 1]`.
    pub p_exist: f32,
    /// Source-side lifecycle; `Terminated` acts as a tombstone.
    pub lifecycle: TrackLifecycle,
    /// How long after `t_valid` the source vouches for this estimate (ms).
    pub valid_for_ms: u32,
    /// When the source created this track (source clock). Constant over the
    /// track's life; the world model names a fused object after its oldest
    /// member. Never after `t_valid`.
    pub born: Timestamp,
    /// Oriented box, if the source's sensors report extent (see [`crate::extent`]).
    pub object_box: Option<ObjectBox>,
}

impl TrackReport {
    /// The report's key.
    pub fn key(&self) -> ReportKey {
        (self.source, self.track)
    }

    /// The version used for last-writer-wins ordering.
    pub fn version(&self) -> (u32, u64) {
        (self.epoch, self.seq)
    }

    /// State vector `[p, v]` in `self.frame`.
    pub fn state(&self) -> Vector6<f64> {
        Vector6::new(
            self.pos[0],
            self.pos[1],
            self.pos[2],
            self.vel[0] as f64,
            self.vel[1] as f64,
            self.vel[2] as f64,
        )
    }

    /// Position in `self.frame`.
    pub fn position(&self) -> Vector3<f64> {
        Vector3::new(self.pos[0], self.pos[1], self.pos[2])
    }

    /// The 6x6 covariance as `f64`.
    pub fn covariance(&self) -> Matrix6<f64> {
        let mut packed = [0.0f64; 21];
        for (o, v) in packed.iter_mut().zip(self.cov.iter()) {
            *o = *v as f64;
        }
        unpack_upper6(&packed)
    }

    /// Time after which this report no longer describes the world.
    pub fn expires_at(&self) -> Timestamp {
        self.t_valid.add_millis(self.valid_for_ms as i64)
    }

    /// Checks every value-level invariant a receiver relies on.
    pub fn validate(&self) -> Result<(), ReportError> {
        let finite = self.pos.iter().all(|v| v.is_finite())
            && self.vel.iter().all(|v| v.is_finite())
            && self.cov.iter().all(|v| v.is_finite())
            && self.class_prob.is_finite()
            && self.p_exist.is_finite();
        if !finite {
            return Err(ReportError::NonFinite);
        }
        if !(0.0..=1.0).contains(&self.class_prob) || !(0.0..=1.0).contains(&self.p_exist) {
            return Err(ReportError::ProbabilityOutOfRange);
        }
        let c = self.covariance();
        for i in 0..6 {
            let min = if i < 3 {
                MIN_POS_VARIANCE
            } else {
                MIN_VEL_VARIANCE
            };
            if c[(i, i)] < min || c[(i, i)] > MAX_VARIANCE {
                return Err(ReportError::ImplausibleCovariance);
            }
        }
        if !is_spd(&c) {
            return Err(ReportError::CovarianceNotPositiveDefinite);
        }
        if self.object_box.is_some_and(|b| !b.is_valid()) {
            return Err(ReportError::ImplausibleBox);
        }
        if self.born > self.t_valid {
            return Err(ReportError::BornAfterEstimate);
        }
        Ok(())
    }
}

/// A set of reports from one source, signed once.
#[derive(Debug, Clone, PartialEq)]
pub struct ReportBatch {
    /// Publishing agent.
    pub source: AgentId,
    /// Source boot epoch.
    pub epoch: u32,
    /// Publication time (source clock).
    pub sent_at: Timestamp,
    /// Reports with strictly increasing `seq` and distinct tracks.
    pub reports: Vec<TrackReport>,
}

/// Reasons a batch is rejected. Nothing from a rejected batch is used.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, thiserror::Error)]
pub enum ReportError {
    /// Input ended early.
    #[error("truncated batch")]
    Truncated,
    /// Bytes after the signature.
    #[error("trailing bytes after signature")]
    TrailingBytes,
    /// Not a GodView report batch.
    #[error("bad magic")]
    BadMagic,
    /// Schema version this build does not understand.
    #[error("unsupported schema {0}")]
    UnsupportedSchema(u8),
    /// Unknown frame tag.
    #[error("bad frame tag {0}")]
    BadFrameTag(u8),
    /// Unknown lifecycle tag.
    #[error("bad lifecycle tag {0}")]
    BadLifecycle(u8),
    /// NaN or infinite value.
    #[error("non-finite value")]
    NonFinite,
    /// Probability outside `[0, 1]`.
    #[error("probability out of range")]
    ProbabilityOutOfRange,
    /// Variance below the plausibility floor or above the ceiling.
    #[error("implausible covariance")]
    ImplausibleCovariance,
    /// Covariance not symmetric positive definite.
    #[error("covariance not positive definite")]
    CovarianceNotPositiveDefinite,
    /// A box with a non-finite, non-positive or implausible value.
    #[error("implausible box")]
    ImplausibleBox,
    /// Unknown box tag.
    #[error("bad box tag {0}")]
    BadBoxTag(u8),
    /// A track created after the time of its own estimate.
    #[error("track born after its estimate")]
    BornAfterEstimate,
    /// More reports than allowed.
    #[error("too many reports")]
    TooManyReports,
    /// Sequence numbers not strictly increasing, or a track repeated.
    #[error("inconsistent sequence or duplicate track within batch")]
    InconsistentBatch,
    /// The source field is not a valid Ed25519 key.
    #[error("invalid source key")]
    BadKey,
    /// Signature does not verify.
    #[error("bad signature")]
    BadSignature,
}

impl ReportBatch {
    /// Encodes and signs the batch. The signer's key must match `source`.
    ///
    /// # Panics
    /// Panics if the key does not match `source` or a report fails
    /// [`TrackReport::validate`]; publishing invalid data is a programming error.
    pub fn sign(&self, key: &SigningKey) -> Vec<u8> {
        assert_eq!(
            AgentId::from_signing_key(key),
            self.source,
            "signing key does not match source"
        );
        self.check_structure()
            .expect("refusing to sign an invalid batch");
        let mut out = self.encode_unsigned();
        let sig = key.sign(&out);
        out.extend_from_slice(&sig.to_bytes());
        out
    }

    fn check_structure(&self) -> Result<(), ReportError> {
        if self.reports.len() > MAX_REPORTS_PER_BATCH {
            return Err(ReportError::TooManyReports);
        }
        let mut tracks = std::collections::BTreeSet::new();
        let mut last_seq = None;
        for r in &self.reports {
            if r.source != self.source || r.epoch != self.epoch {
                return Err(ReportError::InconsistentBatch);
            }
            if last_seq.is_some_and(|s| r.seq <= s) || !tracks.insert(r.track) {
                return Err(ReportError::InconsistentBatch);
            }
            last_seq = Some(r.seq);
            r.validate()?;
        }
        Ok(())
    }

    fn encode_unsigned(&self) -> Vec<u8> {
        let mut b = Vec::with_capacity(50 + self.reports.len() * 209 + 64);
        b.extend_from_slice(MAGIC);
        b.push(SCHEMA_VERSION);
        b.extend_from_slice(&self.source.0);
        b.extend_from_slice(&self.epoch.to_le_bytes());
        b.extend_from_slice(&self.sent_at.as_nanos().to_le_bytes());
        b.extend_from_slice(&(self.reports.len() as u16).to_le_bytes());
        for r in &self.reports {
            b.extend_from_slice(&r.seq.to_le_bytes());
            b.extend_from_slice(&r.track.0.to_le_bytes());
            b.extend_from_slice(&r.t_valid.as_nanos().to_le_bytes());
            match r.frame {
                FrameRef::Ecef => b.push(0),
                FrameRef::Local { frame_id } => {
                    b.push(1);
                    b.extend_from_slice(&frame_id.to_le_bytes());
                }
            }
            for v in r.pos {
                b.extend_from_slice(&v.to_le_bytes());
            }
            for v in r.vel {
                b.extend_from_slice(&v.to_le_bytes());
            }
            for v in r.cov {
                b.extend_from_slice(&v.to_le_bytes());
            }
            b.push(r.class_id);
            b.extend_from_slice(&r.class_prob.to_le_bytes());
            b.extend_from_slice(&r.p_exist.to_le_bytes());
            b.push(r.lifecycle.to_u8());
            b.extend_from_slice(&r.valid_for_ms.to_le_bytes());
            b.extend_from_slice(&r.born.as_nanos().to_le_bytes());
            match r.object_box {
                None => b.push(0),
                Some(x) => {
                    b.push(1);
                    for v in x
                        .size
                        .iter()
                        .chain(x.size_var.iter())
                        .chain([x.yaw, x.yaw_var].iter())
                    {
                        b.extend_from_slice(&(*v as f32).to_le_bytes());
                    }
                    b.push(u8::from(x.yaw_ambiguous));
                }
            }
        }
        b
    }

    /// Verifies the signature and decodes the batch, validating every report.
    ///
    /// The signature is checked before any field is interpreted beyond the
    /// header, so a forged or corrupted batch costs one verification.
    pub fn verify_and_decode(bytes: &[u8]) -> Result<ReportBatch, ReportError> {
        if bytes.len() < 4 + 32 + 4 + 8 + 2 + 64 {
            return Err(ReportError::Truncated);
        }
        if &bytes[..3] != MAGIC {
            return Err(ReportError::BadMagic);
        }
        let schema = bytes[3];
        if !(SCHEMA_VERSION_V1..=SCHEMA_VERSION).contains(&schema) {
            return Err(ReportError::UnsupportedSchema(schema));
        }
        let (body, sig_bytes) = bytes.split_at(bytes.len() - 64);
        let mut source = [0u8; 32];
        source.copy_from_slice(&body[4..36]);
        let key = VerifyingKey::from_bytes(&source).map_err(|_| ReportError::BadKey)?;
        let sig = Signature::from_bytes(sig_bytes.try_into().map_err(|_| ReportError::Truncated)?);
        key.verify(body, &sig)
            .map_err(|_| ReportError::BadSignature)?;

        let mut r = Reader { b: body, at: 36 };
        let source = AgentId(source);
        let epoch = r.u32()?;
        let sent_at = Timestamp::from_nanos(r.i64()?);
        let count = r.u16()? as usize;
        if count > MAX_REPORTS_PER_BATCH {
            return Err(ReportError::TooManyReports);
        }
        let mut reports = Vec::with_capacity(count);
        for _ in 0..count {
            let seq = r.u64()?;
            let track = LocalTrackId(r.u64()?);
            let t_valid = Timestamp::from_nanos(r.i64()?);
            let frame = match r.u8()? {
                0 => FrameRef::Ecef,
                1 => FrameRef::Local { frame_id: r.u64()? },
                other => return Err(ReportError::BadFrameTag(other)),
            };
            let pos = [r.f64()?, r.f64()?, r.f64()?];
            let vel = [r.f32()?, r.f32()?, r.f32()?];
            let mut cov = [0f32; 21];
            for c in cov.iter_mut() {
                *c = r.f32()?;
            }
            let class_id = r.u8()?;
            let class_prob = r.f32()?;
            let p_exist = r.f32()?;
            let lifecycle = TrackLifecycle::from_u8(r.u8()?)?;
            let valid_for_ms = r.u32()?;
            let born = if schema >= 3 {
                Timestamp::from_nanos(r.i64()?)
            } else {
                t_valid
            };
            let object_box = if schema == SCHEMA_VERSION_V1 {
                None
            } else {
                match r.u8()? {
                    0 => None,
                    1 => {
                        let mut v = [0f64; 8];
                        for x in v.iter_mut() {
                            *x = r.f32()? as f64;
                        }
                        let flags = r.u8()?;
                        if flags > 1 {
                            return Err(ReportError::BadBoxTag(flags));
                        }
                        Some(ObjectBox {
                            size: [v[0], v[1], v[2]],
                            size_var: [v[3], v[4], v[5]],
                            yaw: v[6],
                            yaw_var: v[7],
                            yaw_ambiguous: flags == 1,
                        })
                    }
                    other => return Err(ReportError::BadBoxTag(other)),
                }
            };
            reports.push(TrackReport {
                source,
                epoch,
                seq,
                track,
                t_valid,
                frame,
                pos,
                vel,
                cov,
                class_id,
                class_prob,
                p_exist,
                lifecycle,
                valid_for_ms,
                born,
                object_box,
            });
        }
        if r.at != body.len() {
            return Err(ReportError::TrailingBytes);
        }
        let batch = ReportBatch {
            source,
            epoch,
            sent_at,
            reports,
        };
        batch.check_structure()?;
        Ok(batch)
    }
}

struct Reader<'a> {
    b: &'a [u8],
    at: usize,
}

impl Reader<'_> {
    fn take<const N: usize>(&mut self) -> Result<[u8; N], ReportError> {
        let end = self.at.checked_add(N).ok_or(ReportError::Truncated)?;
        let s = self.b.get(self.at..end).ok_or(ReportError::Truncated)?;
        self.at = end;
        let mut out = [0u8; N];
        out.copy_from_slice(s);
        Ok(out)
    }
    fn u8(&mut self) -> Result<u8, ReportError> {
        Ok(self.take::<1>()?[0])
    }
    fn u16(&mut self) -> Result<u16, ReportError> {
        Ok(u16::from_le_bytes(self.take()?))
    }
    fn u32(&mut self) -> Result<u32, ReportError> {
        Ok(u32::from_le_bytes(self.take()?))
    }
    fn u64(&mut self) -> Result<u64, ReportError> {
        Ok(u64::from_le_bytes(self.take()?))
    }
    fn i64(&mut self) -> Result<i64, ReportError> {
        Ok(i64::from_le_bytes(self.take()?))
    }
    fn f32(&mut self) -> Result<f32, ReportError> {
        Ok(f32::from_le_bytes(self.take()?))
    }
    fn f64(&mut self) -> Result<f64, ReportError> {
        Ok(f64::from_le_bytes(self.take()?))
    }
}

/// Converts a 6x6 `f64` covariance into the packed `f32` wire form.
///
/// Rounding to `f32` can break positive definiteness for nearly singular
/// matrices, and tiny variances are rejected by receivers. This floors the
/// diagonal at the plausibility limits and, if needed, adds diagonal loading
/// until the rounded matrix is positive definite again, so a source never
/// publishes a covariance smaller than the one it computed.
pub fn quantize_covariance(p: &Matrix6<f64>) -> [f32; 21] {
    let mut m = crate::frames::symmetrize(p);
    for i in 0..6 {
        let floor = if i < 3 {
            MIN_POS_VARIANCE
        } else {
            MIN_VEL_VARIANCE
        } * 1.01;
        if m[(i, i)] < floor {
            m[(i, i)] = floor;
        }
    }
    let mut load = 0.0;
    for _ in 0..40 {
        let mut packed = [0f32; 21];
        let mut k = 0;
        for i in 0..6 {
            for j in i..6 {
                let extra = if i == j { load } else { 0.0 };
                packed[k] = (m[(i, j)] + extra) as f32;
                k += 1;
            }
        }
        let mut back = [0.0f64; 21];
        for (o, v) in back.iter_mut().zip(packed.iter()) {
            *o = *v as f64;
        }
        if is_spd(&unpack_upper6(&back)) {
            return packed;
        }
        load = if load == 0.0 {
            1e-6 * m.trace().max(1e-6)
        } else {
            load * 4.0
        };
    }
    // Unreachable for finite input; fall back to a diagonal that is safe.
    let mut packed = [0f32; 21];
    let mut k = 0;
    for i in 0..6 {
        for j in i..6 {
            packed[k] = if i == j {
                (m[(i, i)].abs() + 1.0) as f32
            } else {
                0.0
            };
            k += 1;
        }
    }
    packed
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frames::pack_upper6;

    fn key(seed: u8) -> SigningKey {
        SigningKey::from_bytes(&[seed; 32])
    }

    fn report(src: AgentId, seq: u64, track: u64) -> TrackReport {
        let p = Matrix6::from_diagonal(&Vector6::new(0.5, 0.5, 0.2, 1.0, 1.0, 0.3));
        TrackReport {
            source: src,
            epoch: 7,
            seq,
            track: LocalTrackId(track),
            t_valid: Timestamp::from_millis(1_704_067_200_000 + seq as i64),
            frame: FrameRef::Ecef,
            pos: [-2_706_000.25, -4_261_000.5, 3_885_000.75],
            vel: [1.5, -0.25, 0.0],
            cov: quantize_covariance(&p),
            class_id: 1,
            class_prob: 0.9,
            p_exist: 0.95,
            lifecycle: TrackLifecycle::Confirmed,
            valid_for_ms: 1500,
            born: Timestamp::from_millis(1_704_067_190_000),
            object_box: None,
        }
    }

    fn a_box() -> ObjectBox {
        ObjectBox {
            size: [4.5, 1.875, 1.5],
            size_var: [0.0625, 0.015625, 0.03125],
            yaw: 0.5,
            yaw_var: 0.0078125,
            yaw_ambiguous: true,
        }
    }

    fn batch(k: &SigningKey) -> ReportBatch {
        let src = AgentId::from_signing_key(k);
        let mut local = report(src, 11, 4);
        local.frame = FrameRef::Local { frame_id: 9 };
        local.lifecycle = TrackLifecycle::Terminated;
        let mut boxed = report(src, 10, 3);
        boxed.object_box = Some(a_box());
        ReportBatch {
            source: src,
            epoch: 7,
            sent_at: Timestamp::from_millis(1_704_067_200_050),
            reports: vec![boxed, local],
        }
    }

    #[test]
    fn round_trips_exactly() {
        let k = key(1);
        let b = batch(&k);
        let bytes = b.sign(&k);
        assert_eq!(ReportBatch::verify_and_decode(&bytes).unwrap(), b);
    }

    #[test]
    fn encoding_is_canonical() {
        let k = key(1);
        assert_eq!(batch(&k).sign(&k), batch(&k).sign(&k));
    }

    #[test]
    fn golden_layout_is_pinned() {
        // Changing the wire layout must be deliberate: bump SCHEMA_VERSION and this test.
        let k = key(1);
        let bytes = batch(&k).sign(&k);
        // header 50 + ECEF report with birth and box (159 + 8 + 1 + 33) +
        // local-frame report with birth, without box (167 + 8 + 1) + signature 64
        assert_eq!(bytes.len(), 50 + 201 + 176 + 64);
        assert_eq!(&bytes[..4], b"GVR\x03");
        let crc: u32 = bytes[..bytes.len() - 64]
            .iter()
            .fold(0u32, |a, b| a.rotate_left(5) ^ (*b as u32));
        assert_eq!(crc, reference_encoding_checksum(), "wire layout changed");
    }

    /// An independent encoder written from the layout in the module header.
    fn reference_encoding_checksum() -> u32 {
        let v = reference_encoding(&batch(&key(1)), 3);
        v.iter().fold(0u32, |a, b| a.rotate_left(5) ^ (*b as u32))
    }

    /// Encodes `b` (unsigned) in the given schema from the documented layout.
    fn reference_encoding(b: &ReportBatch, schema: u8) -> Vec<u8> {
        let mut v: Vec<u8> = Vec::new();
        v.extend_from_slice(b"GVR");
        v.push(schema);
        v.extend_from_slice(&b.source.0);
        v.extend_from_slice(&7u32.to_le_bytes());
        v.extend_from_slice(&b.sent_at.as_nanos().to_le_bytes());
        v.extend_from_slice(&2u16.to_le_bytes());
        for r in &b.reports {
            v.extend_from_slice(&r.seq.to_le_bytes());
            v.extend_from_slice(&r.track.0.to_le_bytes());
            v.extend_from_slice(&r.t_valid.as_nanos().to_le_bytes());
            match r.frame {
                FrameRef::Ecef => v.push(0),
                FrameRef::Local { frame_id } => {
                    v.push(1);
                    v.extend_from_slice(&frame_id.to_le_bytes());
                }
            }
            r.pos
                .iter()
                .for_each(|x| v.extend_from_slice(&x.to_le_bytes()));
            r.vel
                .iter()
                .for_each(|x| v.extend_from_slice(&x.to_le_bytes()));
            r.cov
                .iter()
                .for_each(|x| v.extend_from_slice(&x.to_le_bytes()));
            v.push(r.class_id);
            v.extend_from_slice(&r.class_prob.to_le_bytes());
            v.extend_from_slice(&r.p_exist.to_le_bytes());
            v.push(match r.lifecycle {
                TrackLifecycle::Tentative => 0,
                TrackLifecycle::Confirmed => 1,
                TrackLifecycle::Coasting => 2,
                TrackLifecycle::Terminated => 3,
            });
            v.extend_from_slice(&r.valid_for_ms.to_le_bytes());
            if schema >= 3 {
                v.extend_from_slice(&r.born.as_nanos().to_le_bytes());
            }
            if schema >= 2 {
                match r.object_box {
                    None => v.push(0),
                    Some(x) => {
                        v.push(1);
                        for f in [
                            x.size[0],
                            x.size[1],
                            x.size[2],
                            x.size_var[0],
                            x.size_var[1],
                            x.size_var[2],
                            x.yaw,
                            x.yaw_var,
                        ] {
                            v.extend_from_slice(&(f as f32).to_le_bytes());
                        }
                        v.push(x.yaw_ambiguous as u8);
                    }
                }
            }
        }
        v
    }

    #[test]
    fn schema_1_batches_still_decode_without_boxes() {
        let k = key(1);
        let mut b = batch(&k);
        for r in &mut b.reports {
            r.object_box = None;
        }
        let mut bytes = reference_encoding(&b, 1);
        let sig = k.sign(&bytes);
        bytes.extend_from_slice(&sig.to_bytes());
        // Without a birth time on the wire, a report is taken as born at t_valid.
        for r in &mut b.reports {
            r.born = r.t_valid;
        }
        assert_eq!(ReportBatch::verify_and_decode(&bytes).unwrap(), b);
    }

    #[test]
    fn schema_2_batches_still_decode_with_boxes() {
        let k = key(1);
        let mut b = batch(&k);
        let mut bytes = reference_encoding(&b, 2);
        let sig = k.sign(&bytes);
        bytes.extend_from_slice(&sig.to_bytes());
        for r in &mut b.reports {
            r.born = r.t_valid;
        }
        let decoded = ReportBatch::verify_and_decode(&bytes).unwrap();
        assert!(decoded.reports[0].object_box.is_some());
        assert_eq!(decoded, b);
    }

    #[test]
    fn a_track_born_after_its_estimate_is_rejected() {
        let k = key(1);
        let mut b = batch(&k);
        b.reports[0].born = b.reports[0].t_valid.add_millis(1);
        assert_eq!(b.reports[0].validate(), Err(ReportError::BornAfterEstimate));
        b.reports[0].born = b.reports[0].t_valid;
        assert_eq!(b.reports[0].validate(), Ok(()));
    }

    #[test]
    fn implausible_boxes_are_rejected() {
        let k = key(1);
        let mut b = batch(&k);
        let mut bad = a_box();
        bad.size_var[1] = 0.0;
        b.reports[0].object_box = Some(bad);
        assert_eq!(b.reports[0].validate(), Err(ReportError::ImplausibleBox));
        // A valid batch whose box tag is corrupted (and re-signed) is rejected too.
        let good = batch(&k);
        let mut bytes = reference_encoding(&good, 3);
        let tag_at = 50 + 159 + 8; // first report's box tag
        assert_eq!(bytes[tag_at], 1);
        bytes[tag_at] = 7;
        let sig = k.sign(&bytes);
        bytes.extend_from_slice(&sig.to_bytes());
        assert_eq!(
            ReportBatch::verify_and_decode(&bytes),
            Err(ReportError::BadBoxTag(7))
        );
    }

    #[test]
    fn any_flipped_byte_is_rejected() {
        // I8: tampering anywhere (header, payload or signature) is detected.
        let k = key(2);
        let bytes = batch(&k).sign(&k);
        for i in 0..bytes.len() {
            let mut t = bytes.clone();
            t[i] ^= 0x10;
            assert!(
                ReportBatch::verify_and_decode(&t).is_err(),
                "byte {i} flip accepted"
            );
        }
    }

    #[test]
    fn forged_source_is_rejected() {
        // Signing with key A while claiming to be B fails verification.
        let a = key(3);
        let b = key(4);
        let mut forged = batch(&a);
        let victim = AgentId::from_signing_key(&b);
        let mut bytes = forged.sign(&a);
        bytes[4..36].copy_from_slice(&victim.0);
        assert_eq!(
            ReportBatch::verify_and_decode(&bytes),
            Err(ReportError::BadSignature)
        );
        forged.source = victim;
        let res = std::panic::catch_unwind(|| forged.sign(&a));
        assert!(res.is_err(), "signing for another source must panic");
    }

    #[test]
    fn truncated_and_trailing_input_rejected() {
        let k = key(5);
        let bytes = batch(&k).sign(&k);
        assert_eq!(
            ReportBatch::verify_and_decode(&bytes[..20]),
            Err(ReportError::Truncated)
        );
        let mut extra = bytes.clone();
        extra.push(0);
        assert!(ReportBatch::verify_and_decode(&extra).is_err());
    }

    #[test]
    fn validation_rejects_bad_values() {
        let src = AgentId::from_signing_key(&key(6));
        let mut r = report(src, 1, 1);
        r.pos[0] = f64::NAN;
        assert_eq!(r.validate(), Err(ReportError::NonFinite));

        let mut r = report(src, 1, 1);
        r.p_exist = 1.5;
        assert_eq!(r.validate(), Err(ReportError::ProbabilityOutOfRange));

        let mut r = report(src, 1, 1);
        let tiny = Matrix6::identity() * 1e-9;
        let mut packed = [0f32; 21];
        for (o, v) in packed.iter_mut().zip(pack_upper6(&tiny).iter()) {
            *o = *v as f32;
        }
        r.cov = packed;
        assert_eq!(r.validate(), Err(ReportError::ImplausibleCovariance));

        let mut r = report(src, 1, 1);
        // Off-diagonal larger than the diagonal: indefinite.
        r.cov[1] = 10.0;
        assert_eq!(
            r.validate(),
            Err(ReportError::CovarianceNotPositiveDefinite)
        );
    }

    #[test]
    fn batch_structure_enforced() {
        let k = key(7);
        let src = AgentId::from_signing_key(&k);
        let dup = ReportBatch {
            source: src,
            epoch: 7,
            sent_at: Timestamp::ZERO,
            reports: vec![report(src, 2, 1), report(src, 3, 1)],
        };
        assert_eq!(dup.check_structure(), Err(ReportError::InconsistentBatch));
        let backwards = ReportBatch {
            source: src,
            epoch: 7,
            sent_at: Timestamp::ZERO,
            reports: vec![report(src, 3, 1), report(src, 2, 2)],
        };
        assert_eq!(
            backwards.check_structure(),
            Err(ReportError::InconsistentBatch)
        );
    }

    #[test]
    fn quantization_never_shrinks_and_stays_positive_definite() {
        // A nearly singular covariance (strong position-velocity correlation).
        let mut p = Matrix6::from_diagonal(&Vector6::new(1.0, 1.0, 1.0, 1.0, 1.0, 1.0));
        p[(0, 3)] = 0.9999999;
        p[(3, 0)] = 0.9999999;
        let q = quantize_covariance(&p);
        let mut back = [0.0; 21];
        for (o, v) in back.iter_mut().zip(q.iter()) {
            *o = *v as f64;
        }
        let m = unpack_upper6(&back);
        assert!(is_spd(&m));
        for i in 0..6 {
            assert!(m[(i, i)] >= p[(i, i)] - 1e-6);
        }
    }
}

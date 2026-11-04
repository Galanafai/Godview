//! The replicated state: a last-writer-wins map of track reports.
//!
//! Each key `(source, local track id)` has exactly one writer, its source,
//! and versions are totally ordered by `(epoch, seq)`. Merging two replicas
//! takes the higher version per key, which is commutative, associative and
//! idempotent, so replicas that have seen the same reports hold the same
//! state regardless of delivery order, duplication or relaying. There is no
//! conflict to resolve because no two writers share a key.
//!
//! The store replicates **inputs** to fusion, never fused estimates. The
//! shared world state is recomputed from it by `WorldModel`, so convergence
//! of this store plus a deterministic fusion function gives convergence of
//! the world state. Convergence is not accuracy: accuracy comes from the
//! estimation math, not from this layer.
//!
//! # Replay protection after garbage collection
//!
//! Once an expired entry is collected, a replay of an old report for that
//! key could resurrect it. The store therefore keeps, per source, a floor:
//! the highest version among collected entries. Reports at or below a
//! source's floor are rejected.

use crate::report::{AgentId, ReportKey, TrackReport};
use godview_env::Timestamp;
use std::collections::BTreeMap;

/// Result of inserting a report.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum InsertOutcome {
    /// New key, or newer version replaced the stored one.
    Applied,
    /// Same version already stored (duplicate delivery or relay echo).
    Duplicate,
    /// Older than the stored version (out-of-order delivery).
    Stale,
    /// At or below the source's garbage-collection floor (replay).
    BelowFloor,
}

/// Last-writer-wins map of the latest report per key.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ReportStore {
    entries: BTreeMap<ReportKey, TrackReport>,
    floors: BTreeMap<AgentId, (u32, u64)>,
}

impl ReportStore {
    /// Creates an empty store.
    pub fn new() -> Self {
        Self::default()
    }

    /// Number of stored reports (including tombstones).
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// True if empty.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Stored report for a key.
    pub fn get(&self, key: &ReportKey) -> Option<&TrackReport> {
        self.entries.get(key)
    }

    /// All stored reports in key order.
    pub fn iter(&self) -> impl Iterator<Item = &TrackReport> {
        self.entries.values()
    }

    /// Inserts a report if it is newer than what is stored for its key.
    pub fn insert(&mut self, report: TrackReport) -> InsertOutcome {
        if let Some(floor) = self.floors.get(&report.source) {
            if report.version() <= *floor {
                return InsertOutcome::BelowFloor;
            }
        }
        match self.entries.get(&report.key()) {
            Some(existing) if existing.version() == report.version() => InsertOutcome::Duplicate,
            Some(existing) if existing.version() > report.version() => InsertOutcome::Stale,
            _ => {
                self.entries.insert(report.key(), report);
                InsertOutcome::Applied
            }
        }
    }

    /// Merges another replica into this one (state-based CRDT join).
    pub fn merge(&mut self, other: &ReportStore) {
        for (src, floor) in &other.floors {
            let f = self.floors.entry(*src).or_insert(*floor);
            if *floor > *f {
                *f = *floor;
            }
        }
        for r in other.entries.values() {
            self.insert(*r);
        }
        // A raised floor may now cover entries we already held.
        let floors = &self.floors;
        self.entries
            .retain(|(src, _), r| floors.get(src).is_none_or(|f| r.version() > *f));
    }

    /// Removes every report from a source (for example after revocation).
    ///
    /// Raises the source's floor to the highest removed version, so replays of
    /// the removed reports stay rejected.
    pub fn purge_source(&mut self, source: &AgentId) -> usize {
        let keys: Vec<ReportKey> = self
            .entries
            .range((*source, crate::report::LocalTrackId(0))..)
            .take_while(|(k, _)| k.0 == *source)
            .map(|(k, _)| *k)
            .collect();
        for k in &keys {
            if let Some(r) = self.entries.remove(k) {
                self.raise_floor(r.source, r.version());
            }
        }
        keys.len()
    }

    /// Drops reports that expired more than `grace_s` before `now`.
    ///
    /// Returns the number of entries removed. Not part of the CRDT join:
    /// collection depends on the local clock, so replicas converge in content
    /// only while they agree on what is live.
    pub fn collect_garbage(&mut self, now: Timestamp, grace_s: f64) -> usize {
        let cutoff = now.add_secs_f64(-grace_s);
        let expired: Vec<ReportKey> = self
            .entries
            .iter()
            .filter(|(_, r)| r.expires_at() < cutoff)
            .map(|(k, _)| *k)
            .collect();
        for k in &expired {
            if let Some(r) = self.entries.remove(k) {
                self.raise_floor(r.source, r.version());
            }
        }
        expired.len()
    }

    fn raise_floor(&mut self, source: AgentId, version: (u32, u64)) {
        let f = self.floors.entry(source).or_insert(version);
        if version > *f {
            *f = version;
        }
    }

    /// A deterministic digest of the stored content (FNV-1a over key, version and values).
    pub fn digest(&self) -> u64 {
        let mut h = Fnv::new();
        for r in self.entries.values() {
            h.bytes(&r.source.0);
            h.u64(r.track.0);
            h.u64(r.epoch as u64);
            h.u64(r.seq);
            h.u64(r.t_valid.as_nanos() as u64);
            for p in r.pos {
                h.u64(p.to_bits());
            }
        }
        h.finish()
    }
}

/// Minimal FNV-1a hasher with a fixed seed (stable across runs and platforms).
#[derive(Debug, Clone, Copy)]
pub(crate) struct Fnv(u64);

impl Fnv {
    pub(crate) fn new() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }
    pub(crate) fn bytes(&mut self, b: &[u8]) {
        for x in b {
            self.0 ^= *x as u64;
            self.0 = self.0.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    pub(crate) fn u64(&mut self, v: u64) {
        self.bytes(&v.to_le_bytes());
    }
    pub(crate) fn finish(self) -> u64 {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::report::{quantize_covariance, FrameRef, LocalTrackId, TrackLifecycle};
    use nalgebra::Matrix6;
    use rand::seq::SliceRandom;
    use rand::{Rng, SeedableRng};

    fn rep(src: u8, track: u64, epoch: u32, seq: u64, t_ms: i64) -> TrackReport {
        TrackReport {
            source: AgentId([src; 32]),
            epoch,
            seq,
            track: LocalTrackId(track),
            t_valid: Timestamp::from_millis(t_ms),
            frame: FrameRef::Ecef,
            pos: [seq as f64, track as f64, src as f64],
            vel: [0.0; 3],
            cov: quantize_covariance(&Matrix6::identity()),
            class_id: 1,
            class_prob: 1.0,
            p_exist: 1.0,
            lifecycle: TrackLifecycle::Confirmed,
            valid_for_ms: 1000,
            object_box: None,
        }
    }

    fn random_reports(rng: &mut impl Rng, n: usize) -> Vec<TrackReport> {
        (0..n)
            .map(|_| {
                // A source never signs two different reports with one version, so content
                // (here t_valid) is a function of (source, track, epoch, seq).
                let (src, track, epoch, seq) = (
                    rng.gen_range(1..4),
                    rng.gen_range(1..5),
                    rng.gen_range(0..2),
                    rng.gen_range(0..20),
                );
                rep(
                    src,
                    track,
                    epoch,
                    seq,
                    epoch as i64 * 10_000 + seq as i64 * 100,
                )
            })
            .collect()
    }

    fn store_of(rs: &[TrackReport]) -> ReportStore {
        let mut s = ReportStore::new();
        for r in rs {
            s.insert(*r);
        }
        s
    }

    #[test]
    fn last_writer_wins_by_epoch_then_seq() {
        let mut s = ReportStore::new();
        assert_eq!(s.insert(rep(1, 1, 0, 5, 0)), InsertOutcome::Applied);
        assert_eq!(s.insert(rep(1, 1, 0, 5, 0)), InsertOutcome::Duplicate);
        assert_eq!(s.insert(rep(1, 1, 0, 4, 0)), InsertOutcome::Stale);
        assert_eq!(
            s.insert(rep(1, 1, 1, 0, 0)),
            InsertOutcome::Applied,
            "new epoch wins after reboot"
        );
        assert_eq!(
            s.get(&(AgentId([1; 32]), LocalTrackId(1))).unwrap().epoch,
            1
        );
    }

    #[test]
    fn insertion_order_and_duplication_do_not_matter() {
        // I5 at the replication layer: any delivery order, with duplicates, gives one state.
        let mut rng = rand::rngs::StdRng::seed_from_u64(21);
        for _ in 0..200 {
            let mut rs = random_reports(&mut rng, 30);
            let reference = store_of(&rs);
            let dupes: Vec<_> = rs.iter().take(10).copied().collect();
            rs.extend(dupes);
            rs.shuffle(&mut rng);
            assert_eq!(store_of(&rs), reference);
        }
    }

    #[test]
    fn merge_is_commutative_associative_idempotent() {
        let mut rng = rand::rngs::StdRng::seed_from_u64(22);
        for _ in 0..200 {
            let a = store_of(&random_reports(&mut rng, 12));
            let b = store_of(&random_reports(&mut rng, 12));
            let c = store_of(&random_reports(&mut rng, 12));
            let join = |x: &ReportStore, y: &ReportStore| {
                let mut z = x.clone();
                z.merge(y);
                z
            };
            assert_eq!(join(&a, &b), join(&b, &a), "commutative");
            assert_eq!(
                join(&join(&a, &b), &c),
                join(&a, &join(&b, &c)),
                "associative"
            );
            assert_eq!(join(&a, &a), a, "idempotent");
        }
    }

    #[test]
    fn garbage_collection_blocks_replays() {
        let mut s = ReportStore::new();
        s.insert(rep(1, 1, 0, 3, 0));
        assert_eq!(s.collect_garbage(Timestamp::from_millis(5000), 1.0), 1);
        assert!(s.is_empty());
        assert_eq!(
            s.insert(rep(1, 1, 0, 3, 0)),
            InsertOutcome::BelowFloor,
            "replay after GC"
        );
        assert_eq!(s.insert(rep(1, 2, 0, 2, 0)), InsertOutcome::BelowFloor);
        assert_eq!(s.insert(rep(1, 2, 0, 4, 4900)), InsertOutcome::Applied);
    }

    #[test]
    fn purge_removes_only_that_source_and_blocks_its_replays() {
        let mut s = store_of(&[rep(1, 1, 0, 1, 0), rep(1, 2, 0, 2, 0), rep(2, 1, 0, 1, 0)]);
        assert_eq!(s.purge_source(&AgentId([1; 32])), 2);
        assert_eq!(s.len(), 1);
        assert_eq!(s.insert(rep(1, 1, 0, 1, 0)), InsertOutcome::BelowFloor);
    }

    #[test]
    fn digest_tracks_content() {
        let a = store_of(&[rep(1, 1, 0, 1, 0), rep(2, 1, 0, 1, 0)]);
        let b = store_of(&[rep(2, 1, 0, 1, 0), rep(1, 1, 0, 1, 0)]);
        assert_eq!(a.digest(), b.digest());
        let c = store_of(&[rep(2, 1, 0, 2, 0), rep(1, 1, 0, 1, 0)]);
        assert_ne!(a.digest(), c.digest());
    }
}

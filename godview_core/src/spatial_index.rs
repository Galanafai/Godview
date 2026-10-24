//! An H3-partitioned index of object ids with exact continuous positions.
//!
//! H3 is used **only** as a partition key: each id is filed under the cell
//! containing its mean position, and the continuous ECEF position is stored
//! alongside. Queries enumerate every cell that could contain a point within
//! the query radius and then filter by exact 3D Euclidean distance, so the
//! answer equals a brute-force scan regardless of cell boundaries, altitude
//! or pentagon distortion. The cell never stands in for the position
//! (audit findings F-10 and F-24).

use crate::frames::{Ecef, FrameError};
use h3o::{CellIndex, Resolution};
use std::collections::{BTreeMap, BTreeSet, HashMap};

/// Spatial index from ids to continuous positions, bucketed by H3 cell.
#[derive(Debug, Clone)]
pub struct SpatialIndex<K: Ord + Copy> {
    resolution: Resolution,
    entries: BTreeMap<K, (Ecef, CellIndex)>,
    cells: HashMap<CellIndex, BTreeSet<K>>,
}

impl<K: Ord + Copy> SpatialIndex<K> {
    /// Creates an empty index at the given H3 resolution.
    pub fn new(resolution: Resolution) -> Self {
        Self {
            resolution,
            entries: BTreeMap::new(),
            cells: HashMap::new(),
        }
    }

    /// H3 resolution used for partitioning.
    pub fn resolution(&self) -> Resolution {
        self.resolution
    }

    /// Number of indexed ids.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// True if nothing is indexed.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// The H3 cell holding a position at this index's resolution.
    pub fn cell_of(&self, p: &Ecef) -> Result<CellIndex, FrameError> {
        Ok(p.to_geodetic().to_latlng()?.to_cell(self.resolution))
    }

    /// Inserts or moves an id. Returns the cell it is filed under.
    pub fn upsert(&mut self, key: K, position: Ecef) -> Result<CellIndex, FrameError> {
        if !(position.0.iter().all(|v| v.is_finite())) {
            return Err(FrameError::NonFinite);
        }
        let cell = self.cell_of(&position)?;
        if let Some((_, old_cell)) = self.entries.insert(key, (position, cell)) {
            if old_cell != cell {
                self.detach(old_cell, &key);
            }
        }
        self.cells.entry(cell).or_default().insert(key);
        Ok(cell)
    }

    /// Removes an id. Returns its last position if it was present.
    pub fn remove(&mut self, key: &K) -> Option<Ecef> {
        let (pos, cell) = self.entries.remove(key)?;
        self.detach(cell, key);
        Some(pos)
    }

    /// The stored position of an id.
    pub fn position(&self, key: &K) -> Option<Ecef> {
        self.entries.get(key).map(|(p, _)| *p)
    }

    /// Iterates all ids with their positions in key order.
    pub fn iter(&self) -> impl Iterator<Item = (&K, &Ecef)> {
        self.entries.iter().map(|(k, (p, _))| (k, p))
    }

    /// All ids within `radius_m` (3D Euclidean) of `center`, in key order.
    pub fn query_radius(&self, center: &Ecef, radius_m: f64) -> Vec<K> {
        let mut out = Vec::new();
        let Ok(center_cell) = self.cell_of(center) else {
            return out;
        };
        for cell in center_cell.grid_disk_safe(self.ring_count(radius_m)) {
            if let Some(keys) = self.cells.get(&cell) {
                for k in keys {
                    if let Some((p, _)) = self.entries.get(k) {
                        if p.distance(center) <= radius_m {
                            out.push(*k);
                        }
                    }
                }
            }
        }
        out.sort();
        out
    }

    /// Number of H3 rings needed so that every point within `radius_m` of any
    /// point in the center cell lies in the searched disk.
    ///
    /// Adjacent hexagon centers are `√3 e` apart for edge length `e`, so ring
    /// `k` reaches at least `(k - 1) √3 e` from the center cell. Cell sizes
    /// vary across the globe (pentagons and icosahedron distortion), so the
    /// average edge length is scaled down by a factor of two before dividing.
    pub fn ring_count(&self, radius_m: f64) -> u32 {
        ring_count(self.resolution, radius_m)
    }

    fn detach(&mut self, cell: CellIndex, key: &K) {
        if let Some(set) = self.cells.get_mut(&cell) {
            set.remove(key);
            if set.is_empty() {
                self.cells.remove(&cell);
            }
        }
    }

    /// Checks internal consistency (every entry filed under exactly its cell).
    pub fn check_invariants(&self) -> bool {
        let filed: usize = self.cells.values().map(|s| s.len()).sum();
        filed == self.entries.len()
            && self
                .entries
                .iter()
                .all(|(k, (_, c))| self.cells.get(c).is_some_and(|s| s.contains(k)))
    }
}

/// Number of H3 rings around a point's cell at `resolution` such that every
/// point within `radius_m` of it lies in the disk. See
/// [`SpatialIndex::ring_count`] for the bound.
pub fn ring_count(resolution: Resolution, radius_m: f64) -> u32 {
    let min_edge = resolution.edge_length_m() * 0.5;
    ((radius_m.max(0.0) / (min_edge * 3f64.sqrt())).ceil() as u32) + 1
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frames::{EnuFrame, Geodetic};
    use nalgebra::Vector3;
    use rand::{Rng, SeedableRng};

    fn brute(points: &BTreeMap<u32, Ecef>, c: &Ecef, r: f64) -> Vec<u32> {
        points
            .iter()
            .filter(|(_, p)| p.distance(c) <= r)
            .map(|(k, _)| *k)
            .collect()
    }

    #[test]
    fn matches_brute_force_under_random_operations() {
        // I9: index answers equal a brute-force scan after inserts, moves and removals.
        for (lat, lon) in [
            (37.7749, -122.4194),
            (64.7, 10.5),
            (-0.3, 179.99),
            (58.0, 11.0),
        ] {
            let site = EnuFrame::new(Geodetic::new(lat, lon, 0.0).unwrap());
            let mut rng = rand::rngs::StdRng::seed_from_u64((lat * 1000.0) as u64);
            let mut idx = SpatialIndex::new(Resolution::Ten);
            let mut model: BTreeMap<u32, Ecef> = BTreeMap::new();
            for step in 0..1500 {
                let key = rng.gen_range(0..200u32);
                match rng.gen_range(0..10) {
                    0..=6 => {
                        let enu = Vector3::new(
                            rng.gen_range(-400.0..400.0),
                            rng.gen_range(-400.0..400.0),
                            rng.gen_range(-5.0..150.0),
                        );
                        let p = site.to_ecef(&enu);
                        idx.upsert(key, p).unwrap();
                        model.insert(key, p);
                    }
                    _ => {
                        assert_eq!(idx.remove(&key).is_some(), model.remove(&key).is_some());
                    }
                }
                if step % 50 == 0 {
                    assert!(idx.check_invariants());
                    let c = site.to_ecef(&Vector3::new(
                        rng.gen_range(-300.0..300.0),
                        rng.gen_range(-300.0..300.0),
                        rng.gen_range(0.0..100.0),
                    ));
                    let r = rng.gen_range(1.0..250.0);
                    assert_eq!(
                        idx.query_radius(&c, r),
                        brute(&model, &c, r),
                        "at ({lat},{lon}) step {step}"
                    );
                }
            }
        }
    }

    #[test]
    fn finds_neighbors_across_a_cell_edge() {
        // Two points 0.2 m apart on opposite sides of an H3 edge must find each other.
        let res = Resolution::Ten;
        let cell = h3o::LatLng::new(37.7749, -122.4194).unwrap().to_cell(res);
        let boundary = cell.boundary();
        let (a, b) = (boundary[0], boundary[1]);
        let mid = Geodetic::new((a.lat() + b.lat()) / 2.0, (a.lng() + b.lng()) / 2.0, 5.0).unwrap();
        let center: h3o::LatLng = cell.into();
        let inward = EnuFrame::new(mid);
        let c_enu = inward.from_ecef(
            &Geodetic::new(center.lat(), center.lng(), 5.0)
                .unwrap()
                .to_ecef(),
        );
        let dir = Vector3::new(c_enu.x, c_enu.y, 0.0).normalize();
        let inside = inward.to_ecef(&(dir * 0.1));
        let outside = inward.to_ecef(&(-dir * 0.1));
        let mut idx = SpatialIndex::new(res);
        let ci = idx.upsert(1u32, inside).unwrap();
        let co = idx.upsert(2u32, outside).unwrap();
        assert_ne!(ci, co, "test points must straddle the edge");
        assert_eq!(idx.query_radius(&inside, 0.5), vec![1, 2]);
        assert_eq!(idx.query_radius(&outside, 0.5), vec![1, 2]);
    }

    #[test]
    fn preserves_exact_positions_and_altitude() {
        let site = EnuFrame::new(Geodetic::new(37.7749, -122.4194, 0.0).unwrap());
        let mut idx = SpatialIndex::new(Resolution::Ten);
        let ground = site.to_ecef(&Vector3::new(3.25, -7.5, 0.0));
        let drone = site.to_ecef(&Vector3::new(3.25, -7.5, 300.0));
        idx.upsert(1u32, ground).unwrap();
        idx.upsert(2u32, drone).unwrap();
        assert_eq!(
            idx.position(&1),
            Some(ground),
            "positions are stored, not quantized"
        );
        assert_eq!(
            idx.query_radius(&ground, 50.0),
            vec![1],
            "altitude separates"
        );
    }

    #[test]
    fn moving_an_id_leaves_no_stale_entry() {
        // Regression for audit finding F-10: re-insertion left stale index entries.
        let site = EnuFrame::new(Geodetic::new(37.7749, -122.4194, 0.0).unwrap());
        let mut idx = SpatialIndex::new(Resolution::Ten);
        for i in 0..300 {
            idx.upsert(7u32, site.to_ecef(&Vector3::new(i as f64 * 3.0, 0.0, 0.0)))
                .unwrap();
        }
        assert_eq!(idx.len(), 1);
        assert!(idx.check_invariants());
        let near_start = site.to_ecef(&Vector3::zeros());
        assert!(idx.query_radius(&near_start, 10.0).is_empty());
    }

    #[test]
    fn rejects_non_finite_positions() {
        let mut idx = SpatialIndex::new(Resolution::Ten);
        assert!(idx.upsert(1u32, Ecef::new(f64::NAN, 0.0, 0.0)).is_err());
        assert!(idx.is_empty());
    }
}

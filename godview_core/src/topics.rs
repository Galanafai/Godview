//! Spatial topics: the H3 cell a report is published under, and the cells a
//! node subscribes to so that it hears every report near it.
//!
//! A transport (for example Zenoh) carries report batches on one topic per
//! H3 cell at a coarse "topic resolution". A publisher files each report
//! under the cell containing its position; a subscriber listens to every
//! cell that could contain a point within its radius of interest. Traffic a
//! node receives then grows with the density of objects around it, not with
//! the size of the fleet.
//!
//! This only decides what a node hears. Association and fusion never use the
//! cell as a position (see [`crate::spatial_index`]).

use crate::frames::{Ecef, FrameError};
use crate::spatial_index::ring_count;
use h3o::{CellIndex, Resolution};
use std::collections::BTreeSet;

/// Default topic resolution: H3 resolution 8 (average edge about 530 m).
pub const DEFAULT_TOPIC_RESOLUTION: Resolution = Resolution::Eight;

/// The topic cell for a report at `position`.
pub fn topic_cell(position: &Ecef, resolution: Resolution) -> Result<CellIndex, FrameError> {
    if !position.0.iter().all(|v| v.is_finite()) {
        return Err(FrameError::NonFinite);
    }
    Ok(position.to_geodetic().to_latlng()?.to_cell(resolution))
}

/// Every topic cell that can contain a point within `radius_m` of `center`.
///
/// Sorted, so subscription sets compare cheaply when a node moves.
pub fn cover_cells(
    center: &Ecef,
    radius_m: f64,
    resolution: Resolution,
) -> Result<BTreeSet<CellIndex>, FrameError> {
    let cell = topic_cell(center, resolution)?;
    Ok(cell
        .grid_disk_safe(ring_count(resolution, radius_m))
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frames::{EnuFrame, Geodetic};
    use nalgebra::Vector3;
    use rand::{Rng, SeedableRng};

    #[test]
    fn cover_contains_every_cell_within_the_radius() {
        // Property: for random centers at several latitudes and random points
        // within the radius, the point's topic cell is always subscribed.
        let mut rng = rand::rngs::StdRng::seed_from_u64(7);
        for lat in [0.0, 37.77, 64.1, -45.0] {
            for res in [Resolution::Seven, Resolution::Eight, Resolution::Nine] {
                for _ in 0..40 {
                    let site = EnuFrame::new(
                        Geodetic::new(
                            lat + rng.gen_range(-0.5..0.5),
                            rng.gen_range(-179.0..179.0),
                            10.0,
                        )
                        .unwrap(),
                    );
                    let radius = rng.gen_range(20.0..900.0);
                    let center = site.to_ecef(&Vector3::zeros());
                    let cover = cover_cells(&center, radius, res).unwrap();
                    for _ in 0..200 {
                        let th = rng.gen_range(0.0..std::f64::consts::TAU);
                        let r = radius * rng.gen::<f64>().sqrt();
                        let h = rng.gen_range(-30.0..120.0);
                        let p = site.to_ecef(&Vector3::new(r * th.cos(), r * th.sin(), h));
                        if p.distance(&center) > radius {
                            continue;
                        }
                        let cell = topic_cell(&p, res).unwrap();
                        assert!(
                            cover.contains(&cell),
                            "lat {lat} res {res:?} r {radius:.0}: {cell} missing"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn far_cells_are_not_covered() {
        // Scoping works: a point 5 km away is outside a 150 m cover at res 8.
        let site = EnuFrame::new(Geodetic::new(37.7749, -122.4194, 10.0).unwrap());
        let cover = cover_cells(
            &site.to_ecef(&Vector3::zeros()),
            150.0,
            DEFAULT_TOPIC_RESOLUTION,
        )
        .unwrap();
        let far = topic_cell(
            &site.to_ecef(&Vector3::new(5000.0, 0.0, 0.0)),
            DEFAULT_TOPIC_RESOLUTION,
        )
        .unwrap();
        assert!(!cover.contains(&far));
        assert!(cover.len() <= 37, "cover stays small: {}", cover.len());
    }

    #[test]
    fn non_finite_positions_are_rejected() {
        assert!(topic_cell(&Ecef::new(f64::NAN, 0.0, 0.0), DEFAULT_TOPIC_RESOLUTION).is_err());
    }
}

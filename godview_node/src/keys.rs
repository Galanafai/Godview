//! Zenoh key expressions for GodView traffic.
//!
//! `<prefix>/reports/<resolution>/<cell>` carries signed report batches for
//! one H3 topic cell (cell in H3's hex form). The prefix defaults to
//! `godview/v1`, so separate deployments or protocol versions do not mix.

use h3o::{CellIndex, Resolution};

/// Default key prefix.
pub const DEFAULT_PREFIX: &str = "godview/v1";

/// The key a batch for `cell` is published on.
pub fn report_key(prefix: &str, cell: CellIndex) -> String {
    format!("{prefix}/reports/{}/{cell}", u8::from(cell.resolution()))
}

/// The cell a report key names, if it is one at `resolution`.
pub fn parse_report_key(prefix: &str, key: &str, resolution: Resolution) -> Option<CellIndex> {
    let rest = key.strip_prefix(prefix)?.strip_prefix("/reports/")?;
    let (res, cell) = rest.split_once('/')?;
    if res.parse::<u8>().ok()? != u8::from(resolution) {
        return None;
    }
    let cell: CellIndex = cell.parse().ok()?;
    (cell.resolution() == resolution).then_some(cell)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keys_round_trip() {
        let cell = h3o::LatLng::new(37.7749, -122.4194)
            .unwrap()
            .to_cell(Resolution::Eight);
        let key = report_key(DEFAULT_PREFIX, cell);
        assert!(key.starts_with("godview/v1/reports/8/"));
        assert_eq!(
            parse_report_key(DEFAULT_PREFIX, &key, Resolution::Eight),
            Some(cell)
        );
        assert_eq!(
            parse_report_key(DEFAULT_PREFIX, &key, Resolution::Nine),
            None
        );
        assert_eq!(parse_report_key("other", &key, Resolution::Eight), None);
    }
}

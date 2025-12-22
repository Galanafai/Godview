# GodView Core - Known Issues & Bugs

**Date:** 2025-12-22
**Status:** ✅ All Critical Bugs Fixed

---

## Fixed Issues

### 1. Time Engine (`godview_time.rs`)

#### ✅ Covariance Matrix Desynchronization - FIXED
- **Original Issue:** Filter shifted state vector but not covariance matrix in `augment_state()`.
- **Fix Applied:** Added full block-shifting for covariance matrix to maintain proper correlations between current and historical states.

---

### 2. Space Engine (`godview_space.rs`)

#### ✅ Linear Scan in Spatial Query - FIXED
- **Original Issue:** `query_sphere()` iterated over all entities (O(N) per shard).
- **Fix Applied:** Implemented 3D grid-based spatial index with hash-based cell lookup.
- **New Complexity:** O(k³ × avg_entities_per_cell) where k = ceil(radius / cell_size).

#### ✅ Hardcoded Hexagon Size - FIXED
- **Original Issue:** k-ring calculation used hardcoded 66.0m divisor for all resolutions.
- **Fix Applied:** Added `H3_EDGE_LENGTH_M` lookup table and `edge_length_meters()` function for resolution-aware calculation.

---

### 3. Trust Engine (`godview_trust.rs`)

#### ✅ O(N) Revocation Check - FIXED
- **Original Issue:** `revoked_keys` stored as `Vec<VerifyingKey>`, requiring linear scan.
- **Fix Applied:** Changed to `HashSet<[u8; 32]>` storing key bytes for O(1) lookup.
- **New Methods Added:** `is_revoked()`, `revoked_count()`.

---

### 4. Tracking Engine (`godview_tracking.rs`)

#### ✅ Track Aging Logic - FIXED
- **Original Issue:** `age_tracks()` incremented age after checking for removal, causing off-by-one in track expiration.
- **Fix Applied:** Increment age before checking for removal.

#### ✅ Track Rekeying on ID Change - FIXED
- **Original Issue:** When Highlander merge changed canonical_id, track remained stored under old key in HashMap.
- **Fix Applied:** Added rekeying logic in `fuse_track()` to move track to new canonical_id when it changes.

---

## Verification

```
$ cargo test

running 22 tests
test godview_space::tests::test_spatial_engine_creation ... ok
test godview_space::tests::test_entity_insertion ... ok
test godview_space::tests::test_vertical_separation ... ok
test godview_time::tests::test_filter_initialization ... ok
test godview_time::tests::test_prediction_step ... ok
test godview_tracking::tests::test_covariance_intersection_basic ... ok
test godview_tracking::tests::test_covariance_intersection_rumor_safety ... ok
test godview_tracking::tests::test_covariance_intersection_weights_precise ... ok
test godview_tracking::tests::test_confidence_to_covariance ... ok
test godview_tracking::tests::test_create_track ... ok
test godview_tracking::tests::test_highlander_merge_id ... ok
test godview_tracking::tests::test_mahalanobis_gating ... ok
test godview_tracking::tests::test_process_packet_associates_with_existing ... ok
test godview_tracking::tests::test_process_packet_creates_new_track ... ok
test godview_tracking::tests::test_process_packet_no_association_different_class ... ok
test godview_tracking::tests::test_spatial_query_kring ... ok
test godview_tracking::tests::test_track_aging ... ok
test godview_tracking::tests::test_track_manager_creation ... ok
test godview_trust::tests::test_biscuit_authorization ... ok
test godview_trust::tests::test_signature_verification_fails_on_tampering ... ok
test godview_trust::tests::test_signed_packet_creation ... ok
test godview_trust::tests::test_unauthorized_access_denied ... ok

test result: ok. 22 passed; 0 failed
```

---

## Remaining Notes

- **Warnings (14):** Variable naming uses capital letters for matrix variables (F, P, Q, R, etc.) following mathematical convention. These are intentional for readability.
- **Dead Code (1):** `reindex_track` method is defined but not used. Retained for future use.

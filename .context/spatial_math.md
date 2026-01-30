# Spatial Mathematics Reference

> Domain knowledge for GodView spatial operations. Reference this when modifying `godview_space.rs`.

## H3 Hierarchical Spatial Index

GodView uses [H3](https://h3geo.org/) for global surface sharding.

### Resolution Reference

| Resolution | Edge Length | Area | Use Case |
|------------|-------------|------|----------|
| 9 | ~174 m | ~0.1 km² | City sectors |
| 10 | ~66 m | ~0.015 km² | Block-level |
| **11** | **~25 m** | ~5,000 m² | **Default for drones** |
| 12 | ~9.4 m | ~700 m² | High precision |

**GodView Default**: Resolution 11 (~25m edge length)

### H3 Cell Properties
- Hexagonal shape (mostly) with 12 pentagons globally
- Hierarchical: each cell contains 7 children at next resolution
- Global uniqueness: 64-bit cell index

---

## Gnomonic Projection

### Why Not Equirectangular?

The equirectangular (Plate Carrée) projection assumes:
```
x = R × (λ - λ₀) × cos(φ₀)
y = R × (φ - φ₀)
```

**Failure mode**: At high latitudes (> 60°), longitudinal distortion exceeds our 10m grid cell size, causing entities to hash into wrong cells.

### Gnomonic Projection Formulas

The Gnomonic projection projects the sphere onto a tangent plane at the cell center. All great circles appear as straight lines.

**Forward projection** (Global → Local):
```
cos(c) = sin(φ₀)sin(φ) + cos(φ₀)cos(φ)cos(λ - λ₀)

x = R × cos(φ)sin(λ - λ₀) / cos(c)
y = R × [cos(φ₀)sin(φ) - sin(φ₀)cos(φ)cos(λ - λ₀)] / cos(c)
```

Where:
- `(φ, λ)` = point latitude/longitude in radians
- `(φ₀, λ₀)` = cell center latitude/longitude in radians  
- `R` = Earth radius (6,378,137 m for WGS84)
- `c` = angular distance from center

**Inverse projection** (Local → Global):
```
ρ = √(x² + y²)
c = arctan(ρ / R)

φ = arcsin(cos(c)sin(φ₀) + y×sin(c)cos(φ₀)/ρ)
λ = λ₀ + arctan(x×sin(c) / (ρ×cos(φ₀)cos(c) - y×sin(φ₀)sin(c)))
```

### Implementation in Rust

```rust
/// Gnomonic projection: projects lat/lon onto tangent plane at center
fn gnomonic_project(lat: f64, lon: f64, center_lat: f64, center_lon: f64) -> (f32, f32) {
    const EARTH_RADIUS: f64 = 6378137.0;
    let (lat, lon) = (lat.to_radians(), lon.to_radians());
    let (clat, clon) = (center_lat.to_radians(), center_lon.to_radians());
    
    let cos_c = clat.sin() * lat.sin() + clat.cos() * lat.cos() * (lon - clon).cos();
    let x = EARTH_RADIUS * (lat.cos() * (lon - clon).sin()) / cos_c;
    let y = EARTH_RADIUS * (clat.cos() * lat.sin() - clat.sin() * lat.cos() * (lon - clon).cos()) / cos_c;
    
    (x as f32, -y as f32) // Negate Y for right-handed coordinate system
}
```

---

## Coordinate Round-Trip Validation

When modifying projection code, always validate with property tests:

```rust
proptest! {
    #[test]
    fn test_roundtrip(lat in -85.0f64..85.0, lon in -180.0f64..180.0) {
        let local = gnomonic_project(lat, lon, center_lat, center_lon);
        let (back_lat, back_lon) = gnomonic_inverse(local.0, local.1, center_lat, center_lon);
        
        prop_assert!((back_lat - lat).abs() < 1e-6);
        prop_assert!((back_lon - lon).abs() < 1e-6);
    }
}
```

---

## 3D Grid (Spatial Hash)

Within each H3 shard, GodView uses a 3D spatial hash:

- **Cell size**: 10m × 10m × 10m (configurable)
- **Index**: `GridCell { x: i32, y: i32, z: i32 }`
- **Hash complexity**: O(1) for point lookup

### Grid Cell Calculation
```rust
fn grid_cell(local_coords: [f32; 3], cell_size: f32) -> GridCell {
    GridCell {
        x: (local_coords[0] / cell_size).floor() as i32,
        y: (local_coords[1] / cell_size).floor() as i32,
        z: (local_coords[2] / cell_size).floor() as i32,
    }
}
```

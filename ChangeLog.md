### 0.20.0

- Added: Smallest enclosing cap (Welzl's algorithm)
- Added: Cap::height
- Fixed: ChordLength::Add to add equivalent angles

### 0.19.0

- Fixed: README

### 0.18.0

- Added: MinorArc::relate and Loop::relate
- Added: Rectangle::intersects
- Added: Cap:intersects
- Added: Cap:expand
- Added: direct/inverse rotation matrix and origin of local frame
- Fixed: Cap::contains_cap

### 0.17.0

- Added: geo-types and geo-traits (behind a feature flag)
- Improved: Local frames API

### 0.16.0

- Added: uom (behind a feature flag)

### 0.15.0

- Fixed: intersection at shared vertex of both minor arcs
- Added: serde
- Added: spherical::ChordLength
- Added: MinorArc::distance_to and Loop::distance_to_boundary
- Removed: Loop::is_pos_within_distance_to_boundary (replaced by Loop::distance_to_boundary)

### 0.14.0

- Performance improvements

### 0.13.0

- Fixed: Sphere::distance_to_angle returns always a angle in [0, 180]
- Tests

### 0.12.0

- Is position within distance to the boundary of loop?
- Renamed Cap::from_points to Cap::from_triangle

### 0.11.0

- Spherical cap boundary
- Tests

### 0.10.0

- Spherical caps

### 0.9.0

- Tests
- Fixed r2zyx

### 0.8.0

- Tests

### 0.7.0

- Doc
- Tests

### 0.6.0

- Geocentric Radius

### 0.5.0

- Ellispoid: Radius of Curvature (Meridian & prime) + Radius at latitude
- Doc & Typos

### 0.4.0

- Performance improvements
- Added tests

### 0.3.0

- Performance improvements
- Doc improvements
- Updated spherical::Rectangle: compare_by_latitude/longitude & union
- Added tests

### 0.2.0

- Performance improvements

### 0.1.0

- Initial version

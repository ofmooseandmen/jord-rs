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

# Jord - Geographical Position Calculations

[![crates.io](https://img.shields.io/crates/v/jord.svg?color=brightgreen)](https://crates.io/crates/jord)
[![build](https://github.com/ofmooseandmen/jord-rs/workflows/CI/badge.svg)](https://github.com/ofmooseandmen/jord-rs/actions)
[![coverage](https://codecov.io/gh/ofmooseandmen/jord-rs/graph/badge.svg?token=MEKNYZRK3V)](https://codecov.io/gh/ofmooseandmen/jord-rs)
[![license](https://img.shields.io/badge/license-MIT-lightgray.svg)](https://opensource.org/license/mit)

> __Jord__ (_Swedish_) is __Earth__ (_English_)

The `jord` crate implements various geographical position calculations, featuring:

- Conversions between ECEF (earth-centred, earth-fixed), latitude/longitude and [n-vector](http://www.navlab.net/Publications/A_Nonsingular_Horizontal_Position_Representation.pdf) positions for [spherical](crate::spherical::Sphere) and [ellipsoidal](crate::ellipsoidal::Ellipsoid) models,
- [Local frame](crate::LocalFrame)s - body; local level, wander azimuth; north, east, down; east, north, up: delta between positions, target position from reference position and delta,
- [Great circle](https://en.wikipedia.org/wiki/Great_circle) ([spherical](crate::spherical::Sphere)) navigation: surface distance, initial & final bearing, interpolated position, [minor arc](crate::spherical::MinorArc) intersection, cross track distance, angle turned, side of position...,
- Kinematics ([spherical](crate::spherical::Sphere)): closest point of approach between tracks, minimum speed for intercept and time to intercept,
- [Spherical Loop](crate::spherical::Loop)s ('simple polygons'): convex/concave, clockwise/anti-clockwise, contains position, [minimum bounding rectangle](crate::spherical::Rectangle), triangulation, spherical excess...,
- [Spherical Cap](crate::spherical::Cap)s and [Rectangular Region](crate::spherical::Rectangle)s
- Location-dependent radii of [ellispoid](crate::ellipsoidal::Ellipsoid)s.

## Literature

The following references provide the theoretical basis of most of the algorithms:

- [Non-singular Horizontal Position Representation; Gade, K.; 2010](https://www.navlab.net/Publications/A_Nonsingular_Horizontal_Position_Representation.pdf)
- [Some Tactical Algorithms for Spherical Geometry](https://calhoun.nps.edu/bitstream/handle/10945/29516/sometacticalalgo00shud.pdf)
- [Triangulation by Ear Clipping](https://www.geometrictools.com/Documentation/TriangulationByEarClipping.pdf)

## Solutions to the 10 examples from [NavLab](https://www.navlab.net/nvector)

### Example 1: A and B to delta
Given two positions A and B. Find the exact vector from A to B in meters north, east and down, and find the direction (azimuth/bearing) to B, relative to north. Use WGS-84 ellipsoid.

```
use jord::{Angle, Cartesian3DVector, GeodeticPosition, Length, LocalFrame, NVector};
use jord::ellipsoidal::Ellipsoid;

let a = GeodeticPosition::new(
    NVector::from_lat_long_degrees(1.0, 2.0),
    Length::from_metres(3.0)
);

let b = GeodeticPosition::new(
    NVector::from_lat_long_degrees(4.0, 5.0),
    Length::from_metres(6.0)
);

let ned = LocalFrame::ned(a, Ellipsoid::WGS84);
let delta = ned.geodetic_to_local_position(b);

assert_eq!(Length::from_metres(331730.863), delta.x().round_mm()); // north
assert_eq!(Length::from_metres(332998.501), delta.y().round_mm()); // east
assert_eq!(Length::from_metres(17398.304), delta.z().round_mm()); // down
assert_eq!(Length::from_metres(470357.384), delta.slant_range().round_mm());
assert_eq!(Angle::from_degrees(45.10926), delta.azimuth().round_d5());
assert_eq!(Angle::from_degrees(-2.11983), delta.elevation().round_d5());
```

### Example 2: B and delta to C
Given the position of vehicle B and a bearing and distance to an object C. Find the exact position of C. Use WGS-72 ellipsoid.

```
use jord::{
    Angle, Cartesian3DVector, GeodeticPosition, LatLong, Length, LocalFrame, LocalPosition,
    NVector, Vec3,
};
use jord::ellipsoidal::Ellipsoid;

let b = GeodeticPosition::new(
    NVector::new(Vec3::new_unit(1.0, 2.0, 3.0)),
    Length::from_metres(400.0)
);

let yaw = Angle::from_degrees(10.0);
let pitch = Angle::from_degrees(20.0);
let roll = Angle::from_degrees(30.0);
let body = LocalFrame::body(yaw, pitch, roll, b, Ellipsoid::WGS72);
let delta = LocalPosition::from_metres(3000.0, 2000.0, 100.0);

let c = body.local_to_geodetic_position(delta);
let c_ll = LatLong::from_nvector(c.horizontal_position());

assert_eq!(Angle::from_degrees(53.32638), c_ll.latitude().round_d5());
assert_eq!(Angle::from_degrees(63.46812), c_ll.longitude().round_d5());
assert_eq!(Length::from_metres(406.007), c.height().round_mm());
```

### Example 3: ECEF-vector to geodetic latitude
Given an ECEF-vector of a position. Find geodetic latitude, longitude and height (using WGS-84 ellipsoid).

```
use jord::{Angle, GeocentricPosition, LatLong, Length, Surface};
use jord::ellipsoidal::Ellipsoid;

let c = GeocentricPosition::from_metres(0.9*6371e3, -1.0*6371e3, 1.1*6371e3);
let p = Ellipsoid::WGS84.geocentric_to_geodetic_position(c);

let p_ll = LatLong::from_nvector(p.horizontal_position());
assert_eq!(Angle::from_degrees(39.37875), p_ll.latitude().round_d5());
assert_eq!(Angle::from_degrees(-48.01279), p_ll.longitude().round_d5());
assert_eq!(Length::from_metres(4702059.834), p.height().round_mm());
```

### Example 4: Geodetic latitude to ECEF-vector
Given geodetic latitude, longitude and height. Find the ECEF-vector (using WGS-84 ellipsoid).

```
use jord::{Cartesian3DVector, GeocentricPosition, GeodeticPosition, Length, NVector, Surface};
use jord::ellipsoidal::Ellipsoid;

let p = GeodeticPosition::new(
    NVector::from_lat_long_degrees(1.0, 2.0),
    Length::from_metres(3.0)
);

let c = Ellipsoid::WGS84.geodetic_to_geocentric_position(p);

assert_eq!(
    GeocentricPosition::from_metres(6_373_290.277, 222_560.201, 110_568.827),
    c.round_mm(),
);
```

**The following examples assume a spherical Earth model**

### Example 5: Surface distance
Given position A and B. Find the surface distance (i.e. great circle distance).

```
use jord::{Length, NVector};
use jord::spherical::Sphere;

let a = NVector::from_lat_long_degrees(88.0, 0.0);
let b = NVector::from_lat_long_degrees(89.0, -170.0);

assert_eq!(
    Length::from_kilometres(332.456),
    Sphere::EARTH.distance(a, b).round_m()
);
```

### Example 6: Interpolated position
Given the position of B at time t0 and t1. Find an interpolated position at time ti.

```
use jord::{LatLong, NVector};
use jord::spherical::Sphere;

let a = NVector::from_lat_long_degrees(89.9, -150.0);
let b = NVector::from_lat_long_degrees(89.9, 150.0);

let t0 = 10.0;
let t1 = 20.0;
let ti = 16.0;

let f = (ti - t0) / (t1 - t0);
let pi = Sphere::interpolated_position(a, b, f);

assert!(pi.is_some());
assert_eq!(
    LatLong::from_degrees(89.91282, 173.41323),
    LatLong::from_nvector(pi.unwrap()).round_d5()
);
```

### Example 7: Mean position/center
Given three positions A, B, and C. Find the mean position (center/midpoint).

```
use jord::{LatLong, NVector};
use jord::spherical::Sphere;

let ps = vec![
    NVector::from_lat_long_degrees(90.0, 0.0),
    NVector::from_lat_long_degrees(60.0, 10.0),
    NVector::from_lat_long_degrees(50.0, -20.0)
];

let m = Sphere::mean_position(&ps);

assert!(m.is_some());
assert_eq!(
    LatLong::from_degrees(67.23615, -6.91751),
    LatLong::from_nvector(m.unwrap()).round_d5()
);
```

### Example 8: A and azimuth/distance to B
Given position A and an azimuth/bearing and a (great circle) distance. Find the destination point B.

```
use jord::{Angle, LatLong, Length, NVector};
use jord::spherical::Sphere;

let p =  NVector::from_lat_long_degrees(80.0, -90.0);
let azimuth = Angle::from_degrees(200.0);
let distance = Length::from_metres(1000.0);

let d = Sphere::EARTH.destination_position(p, azimuth, distance);

assert_eq!(
    LatLong::from_degrees(79.99155, -90.01770),
    LatLong::from_nvector(d).round_d5()
);
```

### Example 9: Intersection of two paths / triangulation
Given path A going through A1 and A2, and path B going through B1 and B2. Find the intersection of the two paths.

```
use jord::{LatLong, NVector};
use jord::spherical::MinorArc;

let a = MinorArc::new(
    NVector::from_lat_long_degrees(50.0, 180.0),
    NVector::from_lat_long_degrees(90.0, 180.0)
);

let b = MinorArc::new(
    NVector::from_lat_long_degrees(60.0, 160.0),
    NVector::from_lat_long_degrees(80.0, -140.0)
);

let i = a.intersection(b);

assert!(i.is_some());
assert_eq!(
    LatLong::from_degrees(74.16345, 180.0),
    LatLong::from_nvector(i.unwrap()).round_d5()
);
```

### Example 10: Cross track distance (cross track error)
Given path A going through A1 and A2, and a point B. Find the cross track distance/cross track error between B and the path.

```
use jord::{LatLong, Length, NVector};
use jord::spherical::{GreatCircle, Sphere};

let a = GreatCircle::new(
    NVector::from_lat_long_degrees(0.0, 0.0),
    NVector::from_lat_long_degrees(10.0, 0.0)
);

let b =  NVector::from_lat_long_degrees(1.0, 0.1);

let d = Sphere::EARTH.cross_track_distance(b, a);

assert_eq!(Length::from_metres(11117.8), d.round_dm());
```

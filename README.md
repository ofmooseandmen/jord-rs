# Jord - Geographical Position Calculations

[![GitHub CI](https://github.com/ofmooseandmen/jord-rs/workflows/CI/badge.svg)](https://github.com/ofmooseandmen/jord-rs/actions)
[![license](https://img.shields.io/badge/license-BSD3-lightgray.svg)](https://opensource.org/licenses/BSD-3-Clause)

> __Jord__ (_Swedish_) is __Earth__ (_English_)

The `jord` crate implements various geographical position calculations.

## Literature

The following reference provide the theoretical basis of most of the algorithms:

- [Non-singular Horizontal Position Representation; Gade, K.; 2010](http://www.navlab.net/Publications/A_Nonsingular_Horizontal_Position_Representation.pdf)
- [Some Tactical Algorithms for Spherical Geometry](https://calhoun.nps.edu/bitstream/handle/10945/29516/sometacticalalgo00shud.pdf)
- [Triangulation by Ear Clipping](https://www.geometrictools.com/Documentation/TriangulationByEarClipping.pdf)
- [STR: A Simple and Efficient Algorithm for R-Tree Packing](https://apps.dtic.mil/sti/pdfs/ADA324493.pdf)

## Solutions to the 10 examples from [NavLab](http:://www.navlab.net/nvector)

### Example 1: A and B to delta
Given two positions A and B. Find the exact vector from A to B in meters north, east and down, and find the direction (azimuth/bearing) to B, relative to north. Use WGS-84 ellipsoid.

```
use jord::{Angle, Cartesian3DVector, GeodeticPos, Length, LocalFrame, NVector};
use jord::ellipsoidal::Ellipsoid;

let a = GeodeticPos::new(
    NVector::from_lat_long_degrees(1.0, 2.0),
    Length::from_metres(3.0)
);

let b = GeodeticPos::new(
    NVector::from_lat_long_degrees(4.0, 5.0),
    Length::from_metres(6.0)
);

let ned = LocalFrame::ned(a, Ellipsoid::WGS84);
let local = ned.geodetic_to_local_pos(b);

assert_eq!(Length::from_metres(331730.863), local.x().round_mm()); // north
assert_eq!(Length::from_metres(332998.501), local.y().round_mm()); // east
assert_eq!(Length::from_metres(17398.304), local.z().round_mm()); // down
assert_eq!(Length::from_metres(470357.384), local.slant_range().round_mm());
assert_eq!(Angle::from_degrees(45.10926), local.azimuth().round_d5());
assert_eq!(Angle::from_degrees(-2.11983), local.elevation().round_d5());
```

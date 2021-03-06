use std::f64::consts::PI;

use crate::{Angle, Error, HorizontalPos, Length, Spherical, Surface, Vec3};

// FIXME Display
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GreatCircle<S: Spherical> {
    position: HorizontalPos<S>,
    normal: Vec3,
}

impl<S: Spherical> GreatCircle<S> {
    pub fn new(position: HorizontalPos<S>, bearing: Angle) -> GreatCircle<S> {
        GreatCircle {
            position,
            normal: arc_normal_bearing(position, bearing),
        }
    }

    pub fn from_position_bearing(position: HorizontalPos<S>, bearing: Angle) -> GreatCircle<S> {
        GreatCircle::new(position, bearing)
    }

    pub fn from_positions(
        position_1: HorizontalPos<S>,
        position_2: HorizontalPos<S>,
    ) -> Result<GreatCircle<S>, Error> {
        let normal = arc_normal(position_1, position_2)?;
        Ok(GreatCircle {
            position: position_1,
            normal,
        })
    }

    pub fn from_minor_arc(minor_arc: MinorArc<S>) -> GreatCircle<S> {
        GreatCircle {
            position: minor_arc.start_pos,
            normal: minor_arc.normal,
        }
    }

    pub fn intersections_with(
        &self,
        other: Self,
    ) -> Result<(HorizontalPos<S>, HorizontalPos<S>), Error> {
        let i = normal_intersection(self.normal, other.normal)?;
        let hpi = HorizontalPos::new(i, self.position.model());
        Ok((hpi, hpi.antipode()))
    }
}

// FIXME Display
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MinorArc<S: Spherical> {
    start_pos: HorizontalPos<S>,
    end_pos: HorizontalPos<S>,
    normal: Vec3,
}

impl<S: Spherical> MinorArc<S> {
    pub fn new(
        start_pos: HorizontalPos<S>,
        end_pos: HorizontalPos<S>,
    ) -> Result<MinorArc<S>, Error> {
        let normal = arc_normal(start_pos, end_pos)?;
        Ok(MinorArc {
            start_pos,
            end_pos,
            normal,
        })
    }

    pub fn from_positions(
        start_pos: HorizontalPos<S>,
        end_pos: HorizontalPos<S>,
    ) -> Result<MinorArc<S>, Error> {
        MinorArc::new(start_pos, end_pos)
    }

    pub fn start_pos(&self) -> HorizontalPos<S> {
        self.start_pos
    }

    pub fn end_pos(&self) -> HorizontalPos<S> {
        self.end_pos
    }

    pub fn intersection_with(&self, other: Self) -> Result<HorizontalPos<S>, Error> {
        let mas = self.start_pos.nvector();
        let mae = self.end_pos.nvector();
        let mbs = other.start_pos.nvector();
        let mbe = other.end_pos.nvector();
        let i = normal_intersection(self.normal, other.normal)?;
        if mas == mbs || mas == mbe {
            Ok(self.start_pos)
        } else if mae == mbs || mae == mbe {
            Ok(self.end_pos)
        } else {
            let mid = unchecked_mean(vec![mas, mae, mbs, mbe]);
            let pot;
            if i.dot(mid) > 0.0 {
                pot = i;
            } else {
                pot = -1.0 * i
            }
            if is_on_minor_arc(pot, mas, mae) && is_on_minor_arc(pot, mbs, mbe) {
                Ok(HorizontalPos::new(pot, self.start_pos.model()))
            } else {
                Err(Error::NoIntersection)
            }
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Side {
    LeftOf,
    RightOf,
    None,
}

impl<S: Spherical> HorizontalPos<S> {
    pub fn from_mean(positions: &[HorizontalPos<S>]) -> Result<Self, Error> {
        if positions.is_empty() {
            Err(Error::NotEnoughPositions)
        } else if positions.len() == 1 {
            Ok(*positions.first().unwrap())
        } else if positions
            .iter()
            .map(|p| (*p).antipode())
            .any(|p| positions.contains(&p))
        {
            Err(Error::AntipodalPositions)
        } else {
            let pos = *positions.first().unwrap();
            let mean = unchecked_mean(positions.iter().map(|p| (*p).nvector()).collect());
            Ok(HorizontalPos::new(mean, pos.model()))
        }
    }

    pub fn along_track_distance_to(&self, minor_arc: MinorArc<S>) -> Length {
        let normal = minor_arc.normal;
        let v = self.nvector();
        let o = normal.cross(v).cross(normal);
        let a = signed_radians_between(minor_arc.start_pos.nvector(), o, Some(normal));
        arc_length(a, earth_radius(self))
    }

    pub fn cross_track_distance_to(&self, great_circle: GreatCircle<S>) -> Length {
        let a = signed_radians_between(great_circle.normal, self.nvector(), None) - (PI / 2.0);
        arc_length(a, earth_radius(self))
    }

    pub fn destination_pos(&self, bearing: Angle, distance: Length) -> Self {
        if distance.metres() == 0.0 {
            *self
        } else {
            let v0 = self.nvector();
            // east direction vector at p0
            let ed = easting(v0);
            // north direction vector at p0
            let nd = v0.cross(ed);
            // central angle
            let ca = distance / earth_radius(self);
            // unit vector in the direction of the azimuth
            let bearing_radians = bearing.decimal_degrees().to_radians();
            let de = nd * bearing_radians.cos() + ed * bearing_radians.sin();
            let nv = v0 * ca.cos() + de * ca.sin();
            HorizontalPos::new(nv, self.model())
        }
    }

    pub fn distance_to(&self, to: Self) -> Length {
        let a = signed_radians_between(self.nvector(), to.nvector(), None);
        arc_length(a, earth_radius(self))
    }

    pub fn final_bearing_to(&self, to: Self) -> Result<Angle, Error> {
        let ib = to.initial_bearing_to(*self)?;
        Ok(Angle::from_decimal_degrees(normalise(
            ib.decimal_degrees(),
            180.0,
        )))
    }

    pub fn initial_bearing_to(&self, to: Self) -> Result<Angle, Error> {
        if *self == to {
            Err(Error::CoincidentalPositions)
        } else {
            let v1 = self.nvector();
            let v2 = to.nvector();
            // great circle through p1 & p2
            let gc1 = v1.cross(v2);
            let gc2;
            if v1 == NORTH_POLE || v1 == SOUTH_POLE {
                // select y axis (west)
                gc2 = Vec3::neg_unit_y()
            } else {
                // great circle through p1 & north pole
                gc2 = v1.cross(NORTH_POLE)
            }
            let a = signed_radians_between(gc1, gc2, Some(v1)).to_degrees();
            Ok(Angle::from_decimal_degrees(normalise(a, 360.0)))
        }
    }

    pub fn intermediate_pos_to(&self, to: Self, fraction: f64) -> Result<Self, Error> {
        if fraction < 0.0 || fraction > 1.0 {
            Err(Error::OutOfRange)
        } else if fraction == 0.0 {
            Ok(*self)
        } else if eq_one(fraction) {
            Ok(to)
        } else {
            let v1 = self.nvector();
            let v2 = to.nvector();
            let v = (v1 + fraction * (v2 - v1)).unit();
            Ok(HorizontalPos::new(v, self.model()))
        }
    }

    pub fn is_enclosed_by(&self, vertices: &[HorizontalPos<S>]) -> bool {
        if vertices.len() < 3 {
            false
        } else {
            let head = vertices.first().unwrap();
            let end;
            if head == vertices.last().unwrap() {
                end = vertices.len() - 1;
            } else {
                end = vertices.len();
            }
            if end < 3 {
                false
            } else {
                let nv = self.nvector();
                let mut sum = 0.0;
                let mut is_vertex = false;
                for i in 0..(end - 1) {
                    let current_vertex = vertices[i];
                    if current_vertex == *self {
                        is_vertex = true;
                        break;
                    }
                    let next_vertex = vertices[i + 1];
                    let vv = nv - current_vertex.nvector();
                    let vn = nv - next_vertex.nvector();
                    sum += signed_radians_between(vv, vn, Some(nv));
                }
                if is_vertex {
                    false
                } else {
                    let end_vertex = vertices[end - 1];
                    if end_vertex == *self {
                        false
                    } else {
                        let vv = nv - end_vertex.nvector();
                        let vn = nv - head.nvector();
                        sum += signed_radians_between(vv, vn, Some(nv));
                        sum.abs() > PI
                    }
                }
            }
        }
    }

    pub fn projection_onto(&self, minor_arc: MinorArc<S>) -> Result<HorizontalPos<S>, Error> {
        if *self == minor_arc.start_pos {
            Ok(minor_arc.start_pos)
        } else if *self == minor_arc.end_pos {
            Ok(minor_arc.end_pos)
        } else {
            let atd = self.along_track_distance_to(minor_arc);
            let d = minor_arc.start_pos.distance_to(minor_arc.end_pos);
            let f = atd / d;
            minor_arc
                .start_pos
                .intermediate_pos_to(minor_arc.end_pos, f)
        }
    }

    pub fn side_of(&self, great_circle: GreatCircle<S>) -> Side {
        let side = self.nvector().dot(great_circle.normal);
        if side < 0.0 {
            Side::RightOf
        } else if side > 0.0 {
            Side::LeftOf
        } else {
            Side::None
        }
    }

    pub fn turn(&self, from: Self, to: Self) -> Result<Angle, Error> {
        let nfa = arc_normal(from, *self)?;
        let nat = arc_normal(*self, to)?;
        let a = signed_radians_between(nfa.unit(), nat.unit(), Some(self.nvector())).to_degrees();
        Ok(Angle::from_decimal_degrees(a))
    }
}

const NORTH_POLE: Vec3 = Vec3::unit_z();

const SOUTH_POLE: Vec3 = Vec3::neg_unit_z();

fn arc_normal<S: Spherical>(p1: HorizontalPos<S>, p2: HorizontalPos<S>) -> Result<Vec3, Error> {
    if p1 == p2 {
        Err(Error::CoincidentalPositions)
    } else if p1.antipode() == p2 {
        Err(Error::AntipodalPositions)
    } else {
        Ok(p1.nvector().cross(p2.nvector()))
    }
}

fn arc_normal_bearing<S: Spherical>(pos: HorizontalPos<S>, bearing: Angle) -> Vec3 {
    let v = pos.nvector();
    // easting
    let e = easting(v);
    // northing
    let n = v.cross(e);
    let bearing_radians = bearing.decimal_degrees().to_radians();
    let se = e * (bearing_radians.cos() / e.norm());
    let sn = n * (bearing_radians.sin() / n.norm());
    sn - se
}

fn easting(v: Vec3) -> Vec3 {
    if v == NORTH_POLE || v == SOUTH_POLE {
        Vec3::unit_y()
    } else {
        NORTH_POLE.cross(v).unit()
    }
}

fn signed_radians_between(v1: Vec3, v2: Vec3, vn: Option<Vec3>) -> f64 {
    let sign = vn.map_or(1.0, |n| n.dot(v1.cross(v2)).signum());
    let sin_o = sign * v1.cross(v2).norm();
    let cos_o = v1.dot(v2);
    sin_o.atan2(cos_o)
}

fn unchecked_mean(vs: Vec<Vec3>) -> Vec3 {
    vs.iter().fold(Vec3::zero(), |sum, v| sum + *v).unit()
}

#[inline]
fn arc_length(radians: f64, radius: Length) -> Length {
    radians * radius
}

#[inline]
fn earth_radius<S: Spherical>(position: &HorizontalPos<S>) -> Length {
    position.model().surface().mean_radius()
}

fn normalise(a: f64, b: f64) -> f64 {
    (a + b) % 360.0
}

fn normal_intersection(n1: Vec3, n2: Vec3) -> Result<Vec3, Error> {
    let i = n1.cross(n2);
    if i == Vec3::zero() {
        // same or opposite normals
        Err(Error::CoincidentalPath)
    } else {
        Ok(i.unit())
    }
}

fn is_on_minor_arc(v: Vec3, mas: Vec3, mae: Vec3) -> bool {
    let l = mas.square_distance_to(mae);
    v.square_distance_to(mas) <= l && v.square_distance_to(mae) <= l
}

fn eq_one(f: f64) -> bool {
    f == 1.0
}

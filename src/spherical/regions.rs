use std::{cmp::Ordering, f64::consts::PI};

use crate::{numbers::eq_zero, Angle, NVector, Vec3};

use super::{
    base::{angle_radians_between, are_ordered},
    MinorArc, Sphere,
};

/// Determines whether the given vertices are given in clockwise order.
///
/// - the loop can be explicity close (first == last) or not (first != last)
/// - returns false if less than 3 positions are given
///
/// # Examples
///
/// ```
/// use jord::NVector;
/// use jord::spherical::is_loop_clockwise;
///
/// let vs = vec![
///     NVector::from_lat_long_degrees(40.0, 40.0),
///     NVector::from_lat_long_degrees(10.0, 30.0),
///     NVector::from_lat_long_degrees(20.0, 20.0),
///     NVector::from_lat_long_degrees(40.0, 40.0)
/// ];
///
/// assert!(is_loop_clockwise(&vs));
/// // same if loop is not closed
/// assert!(is_loop_clockwise(vs.split_last().unwrap().1));
///
/// let mut rvs = vs.to_vec();
/// rvs.reverse();
/// // reverse loop
/// assert!(!is_loop_clockwise(&rvs));
/// ```
pub fn is_loop_clockwise(vs: &[NVector]) -> bool {
    let ovs = opened(vs);
    if ovs.len() < 3 {
        false
    } else {
        is_clockwise(ovs)
    }
}

/// Determines whether the given vertices define a convex [loop](Loop).
///
/// Notes:
/// - array of vertices can be opened (first != last) or closed (first == last)
/// - returns false if less than 3 vertices are given
/// - returns true if all vertices are collinear
///
/// # Examples
///
/// ```
/// use jord::NVector;
/// use jord::spherical::is_loop_convex;
///
/// let vs = vec![
///     NVector::from_lat_long_degrees(40.0, 40.0),
///     NVector::from_lat_long_degrees(10.0, 30.0),
///     NVector::from_lat_long_degrees(20.0, 20.0),
///     NVector::from_lat_long_degrees(40.0, 40.0)
/// ];
///
/// assert!(is_loop_convex(&vs));
/// // same if loop is not closed
/// assert!(is_loop_convex(vs.split_last().unwrap().1));
///
/// let mut rvs = vs.to_vec();
/// rvs.reverse();
/// // reverse loop
/// assert!(is_loop_convex(&rvs));
/// ```
pub fn is_loop_convex(vs: &[NVector]) -> bool {
    let ovs = opened(vs);
    is_convex(ovs)
}

/// Determines whether the given vertices define a simple [loop](Loop).
/// - All edges are defined [minor arc](crate::spherical::MinorArc)s: consecutive vertices cannot be coincidental or the antipode of one another, and,
/// - No 2 edges are intersecting each other.
///
/// # Examples
///
/// ```
/// use jord::NVector;
/// use jord::spherical::is_loop_simple;
///
/// // consectutive coincidental vertices:
/// let vs1 = vec![
///     NVector::from_lat_long_degrees(-2.0, -2.0),
///     NVector::from_lat_long_degrees(-2.0, -2.0),
///     NVector::from_lat_long_degrees(3.0, 0.0),
/// ];
/// assert!(!is_loop_simple(&vs1));
///
/// // consectutive antipodal vertices:
/// let vs2 = vec![
///     NVector::from_lat_long_degrees(-2.0, -2.0),
///     NVector::from_lat_long_degrees(-2.0, -2.0).antipode(),
///     NVector::from_lat_long_degrees(3.0, 0.0),
/// ];
/// assert!(!is_loop_simple(&vs2));
///
/// // self-intersecting loop:
/// let vs3 = vec![
///     NVector::from_lat_long_degrees(-2.0, -2.0),
///     NVector::from_lat_long_degrees(2.0, -2.0),
///     NVector::from_lat_long_degrees(3.0, 0.0),
///     NVector::from_lat_long_degrees(-2.0, 2.0),
///     NVector::from_lat_long_degrees(2.0, 2.0)
/// ];
/// assert!(!is_loop_simple(&vs3));
///
/// // simple loop:
/// let vs4 = vec![
///     NVector::from_lat_long_degrees(-2.0, -2.0),
///     NVector::from_lat_long_degrees(2.0, -2.0),
///     NVector::from_lat_long_degrees(3.0, 0.0),
///     NVector::from_lat_long_degrees(2.0, 2.0),
///     NVector::from_lat_long_degrees(-2.0, 2.0)
/// ];
/// assert!(is_loop_simple(&vs4));
/// ```
pub fn is_loop_simple(vs: &[NVector]) -> bool {
    let len: usize = vs.len();
    let mut es: Vec<MinorArc> = Vec::with_capacity(len - 1);
    for i in 0..len {
        let cur = vs[i];
        let next = vs[(i + 1) % len];
        if Sphere::is_great_circle(cur, next) {
            es.push(MinorArc::new(cur, next));
        } else {
            return false;
        }
    }
    !is_self_intersecting(&es)
}

/// A single chain of vertices where the first vertex is implicitly connected to the last.
///
/// ## Semantics:
/// - Vertices are stored in clockwise order, regardless of the order supplied at creation.
/// - If less than 3 vertices are supplied at [construction](crate::spherical::Loop::new), the loop is considered as empty.
/// - An edge (i.e. the segment connecting 2 consecutive vertices) is always a [minor arc](crate::spherical::MinorArc).
/// - Consecutive vertices cannot be coincidental or the antipode of one another (see [is_great_circle](crate::spherical::Sphere::is_great_circle)).
/// - No 2 edges are intersecting each other.
///
/// The 2 last points are not enforced at runtime, therefore operations are undefined on invalid loops. The following functions can be used to validate a loop:
/// - [is_loop_simple](crate::spherical::is_loop_simple) prior to construction, or,
/// - [is_valid](crate::spherical::Loop::is_valid) post construction.
#[derive(PartialEq, Clone, Debug, Default)]
pub struct Loop {
    /// vertices in clockwise order.
    vertices: Vec<Vertex>,
    /// 2 positions that are inside the loop (none for empty loops and triangles).
    insides: Option<(NVector, NVector)>,
    /// edges in clockwise order.
    edges: Vec<MinorArc>,
}

impl Loop {
    /// an empty [Loop].
    pub const EMPTY: Self = Self {
        vertices: Vec::new(),
        insides: None,
        edges: Vec::new(),
    };

    /// Creates a new loop from the given vertices.
    pub fn new(vs: &[NVector]) -> Self {
        let opened = opened(vs);
        let len = opened.len();

        match len.cmp(&3) {
            // less than 3 vertices: empty loop.
            Ordering::Less => Self::EMPTY,
            // 3 vertices: triangle.
            Ordering::Equal => {
                let clockwise = is_clockwise(opened);
                let vertices: Vec<Vertex> = if clockwise {
                    vec![
                        Vertex(vs[0], Classification::Reflex),
                        Vertex(vs[1], Classification::Reflex),
                        Vertex(vs[2], Classification::Reflex),
                    ]
                } else {
                    vec![
                        Vertex(vs[2], Classification::Reflex),
                        Vertex(vs[1], Classification::Reflex),
                        Vertex(vs[0], Classification::Reflex),
                    ]
                };
                let edges = to_edges(&vertices);
                let insides = None;
                Self {
                    vertices,
                    insides,
                    edges,
                }
            }
            // more than 3 vertices: general case.
            Ordering::Greater => {
                let clockwise = is_clockwise(opened);
                let vertices = if clockwise {
                    in_order_vertices(vs)
                } else {
                    reverse_vertices(vs)
                };
                let edges = to_edges(&vertices);
                let insides = find_insides(&vertices);
                Self {
                    vertices,
                    insides,
                    edges,
                }
            }
        }
    }

    /// Determines whether this loop is convex.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::NVector;
    /// use jord::spherical::{is_loop_convex, Loop};
    ///
    /// let vs = vec![
    ///     NVector::from_lat_long_degrees(40.0, 40.0),
    ///     NVector::from_lat_long_degrees(10.0, 30.0),
    ///     NVector::from_lat_long_degrees(20.0, 20.0),
    ///     NVector::from_lat_long_degrees(40.0, 40.0)
    /// ];
    ///
    /// let l = Loop::new(&vs);
    ///
    /// assert!(l.is_convex());
    /// ```
    pub fn is_convex(&self) -> bool {
        let vs = self
            .vertices
            .iter()
            .map(|v: &Vertex| v.0)
            .collect::<Vec<_>>();
        is_convex(&vs)
    }

    /// Determines whether this loop is valid:
    /// - All edges are defined [minor arc](crate::spherical::MinorArc)s: consecutive vertices cannot be coincidental or the antipode of one another, and,
    /// - No 2 edges are intersecting each other.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::NVector;
    /// use jord::spherical::Loop;
    ///
    /// // consectutive coincidental vertices:
    /// let l1 = Loop::new(&vec![
    ///     NVector::from_lat_long_degrees(-2.0, -2.0),
    ///     NVector::from_lat_long_degrees(-2.0, -2.0),
    ///     NVector::from_lat_long_degrees(3.0, 0.0),
    /// ]);
    /// assert!(!l1.is_valid());
    ///
    /// // consectutive antipodal vertices:
    /// let l2 = Loop::new(&vec![
    ///     NVector::from_lat_long_degrees(-2.0, -2.0),
    ///     NVector::from_lat_long_degrees(-2.0, -2.0).antipode(),
    ///     NVector::from_lat_long_degrees(3.0, 0.0),
    /// ]);
    /// assert!(!l2.is_valid());
    ///
    /// // self-intersecting loop:
    /// let l3 = Loop::new(&vec![
    ///     NVector::from_lat_long_degrees(-2.0, -2.0),
    ///     NVector::from_lat_long_degrees(2.0, -2.0),
    ///     NVector::from_lat_long_degrees(3.0, 0.0),
    ///     NVector::from_lat_long_degrees(-2.0, 2.0),
    ///     NVector::from_lat_long_degrees(2.0, 2.0)
    /// ]);
    /// assert!(!l3.is_valid());
    ///
    /// // valid loop:
    /// let l4 = Loop::new(&vec![
    ///     NVector::from_lat_long_degrees(-2.0, -2.0),
    ///     NVector::from_lat_long_degrees(2.0, -2.0),
    ///     NVector::from_lat_long_degrees(3.0, 0.0),
    ///     NVector::from_lat_long_degrees(2.0, 2.0),
    ///     NVector::from_lat_long_degrees(-2.0, 2.0)
    /// ]);
    /// assert!(l4.is_valid());
    /// ```
    pub fn is_valid(&self) -> bool {
        let v_len = self.vertices.len();
        for i in 0..v_len {
            if !Sphere::is_great_circle(self.vertices[i].0, self.vertices[(i + 1) % v_len].0) {
                return false;
            }
        }
        !is_self_intersecting(&self.edges)
    }

    /// Determines whether this loop is empty (i.e. less than 3 vertices where given at construction).
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::NVector;
    /// use jord::spherical::Loop;
    ///
    /// assert!(Loop::new(&[]).is_empty());
    ///
    /// assert!(Loop::new(&[NVector::from_lat_long_degrees(0.0, 0.0)]).is_empty());
    ///
    /// assert!(Loop::new(&[
    ///     NVector::from_lat_long_degrees(0.0, 0.0),
    ///     NVector::from_lat_long_degrees(1.0, 0.0)
    /// ]).is_empty());
    ///
    /// assert!(Loop::new(&[
    ///     NVector::from_lat_long_degrees(0.0, 0.0),
    ///     NVector::from_lat_long_degrees(1.0, 0.0),
    ///     NVector::from_lat_long_degrees(0.0, 0.0)
    /// ]).is_empty());
    /// ```
    pub fn is_empty(&self) -> bool {
        // see new(): if less than 3 vertices are supplied, self.vertices is empty.
        self.vertices.is_empty()
    }

    /// Determines if the given position is a vertex of this loop.
    pub fn is_vertex(&self, p: NVector) -> bool {
        self.vertices.iter().any(|v| v.0 == p)
    }

    /// Determines whether the given position is on an edge of this loop.
    ///
    /// # Examples
    /// ```
    /// use jord::NVector;
    /// use jord::spherical::Loop;
    ///
    /// let l = Loop::new(&vec![
    ///     NVector::from_lat_long_degrees(0.0, 0.0),
    ///     NVector::from_lat_long_degrees(0.0, 10.0),
    ///     NVector::from_lat_long_degrees(10.0, 10.0),
    ///     NVector::from_lat_long_degrees(10.0, 0.0),
    /// ]);
    ///
    /// assert!(l.is_on_edge(NVector::from_lat_long_degrees(0.0, 5.0)));
    /// assert!(!l.is_on_edge(NVector::from_lat_long_degrees(0.0, 11.0)));
    /// ```
    pub fn is_on_edge(&self, p: NVector) -> bool {
        self.edges.iter().any(|e| {
            Sphere::side(p, e.start(), e.end()) == 0
                && are_ordered(e.start().as_vec3(), p.as_vec3(), e.end().as_vec3())
        })
    }

    /// Returns the number of vertices of this loop.
    pub fn num_vertices(&self) -> usize {
        self.vertices.len()
    }

    /// Returns the vertex at the given index (panics if the given index is invalid).
    pub fn vertex(&self, i: usize) -> NVector {
        self.vertices[i].0
    }

    /// Determines whether the **interior** of this loop contains the given point (i.e. excluding points which are
    /// vertices or on one of the edge of this loop).
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::NVector;
    /// use jord::spherical::Loop;
    ///
    /// let vs = vec![
    ///     NVector::from_lat_long_degrees(0.0, 0.0),
    ///     NVector::from_lat_long_degrees(0.0, 10.0),
    ///     NVector::from_lat_long_degrees(10.0, 10.0),
    ///     NVector::from_lat_long_degrees(10.0, 0.0)
    /// ];
    ///
    /// let l = Loop::new(&vs);
    ///
    /// assert!(l.contains_point(NVector::from_lat_long_degrees(5.0, 5.0)));
    /// assert!(!l.contains_point(NVector::from_lat_long_degrees(11.0, 11.0)));
    /// ```
    pub fn contains_point(&self, p: NVector) -> bool {
        match self.insides {
            Some((a, b)) => {
                let i = if a.is_antipode_of(p) { b } else { a };
                let ma = MinorArc::new(i, p);
                let mut count_i: usize = 0;
                // if ma intersect e on either start or end, then the same
                // intersection will be detected with next edge:
                // assuming the following edges: [e1, e2, e3, e4]
                // - intersection with e2 could be the same as with e1
                // - intersection with e3 could be the same as with e2
                // - intersection with e4 could be the same as with e3 or e1
                let mut first_i_vec3 = Vec3::ZERO;
                let mut prev_i_vec3 = Vec3::ZERO;
                let n = self.edges.len();
                for i in 0..n {
                    let e = self.edges[i];
                    if let Some(iv) = ma.intersection(e) {
                        if i == 0 {
                            count_i += 1;
                            first_i_vec3 = iv.as_vec3();
                        } else if i == n - 1 {
                            let iv_vec3 = iv.as_vec3();
                            // last edge, check diff with first and prev.
                            if eq(first_i_vec3, iv_vec3) || eq(prev_i_vec3, iv_vec3) {
                                // skip this intersection (already found on previous or first edge).
                            } else {
                                count_i += 1;
                            }
                        } else {
                            let iv_vec3 = iv.as_vec3();
                            // check diff with prev.
                            if eq(prev_i_vec3, iv_vec3) {
                                // skip this intersection (already found on previous or first edge).
                            } else {
                                count_i += 1;
                            }
                            prev_i_vec3 = iv_vec3;
                        }
                    } else {
                        // no intersection reset prev_i_vec3
                        prev_i_vec3 = Vec3::ZERO;
                    }
                }

                // inside if number of intersections is even (since start is inside).
                count_i % 2 == 0
            }
            None => {
                if self.vertices.len() == 3 {
                    let loc = locate_with_orientation(
                        p,
                        self.vertices[0].0,
                        self.vertices[1].0,
                        self.vertices[2].0,
                        // vertices are in clockwise order.
                        -1,
                    );
                    loc == PosLocation::Inside
                } else {
                    false
                }
            }
        }
    }

    /// Triangulates this loop - using the [Ear Clipping](https://www.geometrictools.com/Documentation/TriangulationByEarClipping.pdf) method.
    ///  
    /// This method returns either [loop number vertices](crate::spherical::Loop::num_vertices) - 2 triangle (triple of [NVector]s) if the triangulation succeeds
    /// or an empty vec if triangulation fails (should only occur if the loop is not valid - such as self-intersecting).
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::NVector;
    /// use jord::spherical::Loop;
    ///
    /// let v0 = NVector::from_lat_long_degrees(0.0, 0.0);
    /// let v1 = NVector::from_lat_long_degrees(1.0, 0.0);
    /// let v2 = NVector::from_lat_long_degrees(1.0, 1.0);
    /// let v3 = NVector::from_lat_long_degrees(0.0, 1.0);
    ///
    /// let l = Loop::new(&vec![v0, v1, v2, v3]);
    ///
    /// assert_eq!(vec![
    ///     (v3, v0, v1),
    ///     (v1, v2, v3)
    /// ], l.triangulate());
    /// ```
    pub fn triangulate(&self) -> Vec<(NVector, NVector, NVector)> {
        if self.is_empty() {
            Vec::new()
        } else if self.vertices.len() == 3 {
            vec![(self.vertices[0].0, self.vertices[1].0, self.vertices[2].0)]
        } else {
            ear_clipping(&self.vertices)
        }
    }

    /// Calculates the [spherical excess](https://en.wikipedia.org/wiki/Spherical_trigonometry#Area_and_spherical_excess) of this loop.
    ///
    /// The area of this loop can be obtained by multiplying the spherical excess by the sphere radius squared.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{Angle, NVector};
    /// use jord::spherical::{Loop, Sphere};
    ///
    /// let l = Loop::new(&vec![
    ///     NVector::from_lat_long_degrees(0.0, 0.0),
    ///     NVector::from_lat_long_degrees(1.0, 0.0),
    ///     NVector::from_lat_long_degrees(0.0, 1.0),
    /// ]);
    ///
    /// let se = l.spherical_excess();
    ///
    /// assert_eq!(Angle::from_degrees(0.0087271), se.round_d7());
    ///
    /// // area in km^2 (on Earth):
    /// let r = Sphere::EARTH.radius().as_kilometres();
    /// assert_eq!(6_182.0, (se.as_radians() * r * r).round());
    /// ```
    pub fn spherical_excess(&self) -> Angle {
        if self.is_empty() {
            Angle::ZERO
        } else {
            // normal to each edge.
            let ns = self.edges.iter().map(|e| e.normal()).collect::<Vec<_>>();

            // sum interior angles; depending on whether polygon is cw or ccw, angle between edges is PI - a or PI
            // + a, where a is angle between great-circle vectors; so sum a, then take n * PI - abs(sum(a)) (cannot
            // use sum(PI - abs(a)) as concave polygons would fail); use vector to 1st point as plane normal for
            // sign of a.
            let n1 = Some(self.vertices[0].0.as_vec3());
            let mut interior = 0.0;
            let len = ns.len();
            for i in 0..len {
                interior += angle_radians_between(ns[i], ns[(i + 1) % len], n1);
            }

            let n = len as f64;
            let sum = n * PI - interior.abs();

            // spherical excess.
            Angle::from_radians(sum - (n - 2.0) * PI)
        }
    }
}

#[derive(PartialEq, Clone, Copy, Debug)]
enum Classification {
    Convex,
    Reflex,
    // both convex and reflex: co-linear with previous and next vertex.
    Both,
}

/// A vertex of a loop: position + classification.
#[derive(PartialEq, Clone, Copy, Debug)]
struct Vertex(NVector, Classification);

#[derive(PartialEq, Clone, Copy, Debug)]
enum PosLocation {
    Inside,
    Outside,
    Edge,
    Vertex,
}

/// if first == last, returns [first ... last - 1] otherwise returns given array.
fn opened(vs: &[NVector]) -> &[NVector] {
    if vs.is_empty() {
        vs
    } else if vs.first() == vs.last() {
        // unwrap is safe, vs is not empty
        vs.split_last().unwrap().1
    } else {
        vs
    }
}

/// Determines whether given positions are in clockwise order, assuming that:
/// - the loop is opened (first /= last)
/// - the loop contains at least 3 vertices
fn is_clockwise(vs: &[NVector]) -> bool {
    let len: usize = vs.len();
    if len == 3 {
        Sphere::side(vs[0], vs[1], vs[2]) < 0
    } else {
        let mut turn = Angle::ZERO;
        for i in 0..len {
            let prev: NVector = vs[(i + len - 1) % len];
            let cur = vs[i];
            let next = vs[(i + 1) % len];
            turn = turn + Sphere::turn(prev, cur, next);
        }
        turn.as_radians() < 0.0
    }
}

/// Determines whether given positions define a convex loop, assuming that:
/// - the loop is opened (first /= last)
fn is_convex(vs: &[NVector]) -> bool {
    match vs.len().cmp(&3) {
        Ordering::Less => false,
        Ordering::Equal => true,
        Ordering::Greater => {
            let mut cur_side: i8 = i8::MIN;
            let mut found_left_right: bool = false;
            let len: usize = vs.len();
            for i in 0..len {
                let prev: NVector = vs[(i + len - 1) % len];
                let cur: NVector = vs[i];
                let next = vs[(i + 1) % len];
                let side = Sphere::side(prev, cur, next);
                if side != 0 {
                    if !found_left_right {
                        cur_side = side;
                    } else if cur_side != side {
                        // side changed -> concave
                        return false;
                    } else {
                        // still same side.
                    }
                    found_left_right = true;
                }
            }
            true
        }
    }
}

/// Builds vertices by iterating the given array of horizontal
/// positions in order (i.e. positions are given in clockwise order).
fn in_order_vertices(vs: &[NVector]) -> Vec<Vertex> {
    let len: usize = vs.len();
    let mut res: Vec<Vertex> = Vec::with_capacity(len);
    for i in 0..len {
        let prev: NVector = vs[(i + len - 1) % len];
        let cur = vs[i];
        let next = vs[(i + 1) % len];
        let side = Sphere::side(prev, cur, next);
        let vertex = match side.cmp(&0) {
            Ordering::Greater => Vertex(cur, Classification::Reflex),
            Ordering::Less => Vertex(cur, Classification::Convex),
            Ordering::Equal => Vertex(cur, Classification::Both),
        };
        res.push(vertex);
    }
    res
}

/// Builds vertices by iterating the given array of horizontal
/// positions in reverse order (i.e. positions are given in anti-clockwise order).
fn reverse_vertices(vs: &[NVector]) -> Vec<Vertex> {
    let len: usize = vs.len();
    let mut res: Vec<Vertex> = Vec::with_capacity(len);
    for i in (0..len).rev() {
        let prev: NVector = vs[(i + 1) % len];
        let cur = vs[i];
        let next = vs[(i + len - 1) % len];
        let side = Sphere::side(prev, cur, next);
        let vertex = match side.cmp(&0) {
            Ordering::Greater => Vertex(cur, Classification::Reflex),
            Ordering::Less => Vertex(cur, Classification::Convex),
            Ordering::Equal => Vertex(cur, Classification::Both),
        };
        res.push(vertex);
    }
    res
}

/// vertices to edges: last edge connect last vertex to first vertex.
fn to_edges(vs: &[Vertex]) -> Vec<MinorArc> {
    let len: usize = vs.len();
    let mut res: Vec<MinorArc> = Vec::with_capacity(len - 1);
    for i in 0..len {
        let cur = vs[i];
        let next = vs[(i + 1) % len];
        res.push(MinorArc::new(cur.0, next.0));
    }
    res
}

fn ear_clipping(vs: &[Vertex]) -> Vec<(NVector, NVector, NVector)> {
    let mut remaining = vs.to_vec();
    let mut res: Vec<(NVector, NVector, NVector)> = Vec::with_capacity(2);

    loop {
        if remaining.len() == 3 {
            res.push((remaining[0].0, remaining[1].0, remaining[2].0));
            break;
        }

        if let Some(ear) = next_ear(&mut remaining) {
            res.push((ear.0, ear.1, ear.2));
        } else {
            res.clear();
            // could not find an ear, yet more than 3 vertices remain.
            break;
        }
    }
    res
}

/// Finds two positions which are inside the loop defined by the given vertices.
///
/// This works finding the 2 first ears of the loop and then calculating the
/// mid point of the resulting triangle (same principle as the triangulation by ear-clipping).
fn find_insides(vs: &[Vertex]) -> Option<(NVector, NVector)> {
    let mut remaining = vs.to_vec();
    let mut res: Vec<NVector> = Vec::with_capacity(2);

    loop {
        if remaining.len() == 3 {
            let t = vec![remaining[0].0, remaining[1].0, remaining[2].0];
            let inside = Sphere::mean_position(&t);
            if let Some(p) = inside {
                res.push(p);
            }
            break;
        }

        if let Some(ear) = next_ear(&mut remaining) {
            let t: Vec<NVector> = vec![ear.0, ear.1, ear.2];
            let inside = Sphere::mean_position(&t);
            if let Some(p) = inside {
                res.push(p);
                if res.len() == 2 {
                    // found 2 position insides, we're done.
                    break;
                }
            }
        } else {
            // could not find an ear, yet more than 3 vertices remain.
            break;
        }
    }
    if res.len() == 2 {
        Some((res[0], res[1]))
    } else {
        None
    }
}

/// Searches for the next ear in the given list of remaining vertices, returning None if no ear can be found.
fn next_ear(remaining: &mut Vec<Vertex>) -> Option<(NVector, NVector, NVector)> {
    let len = remaining.len();
    for i in 0..len {
        let cur = remaining[i];
        if cur.1 == Classification::Convex {
            // cur is a convex vertex: i is an ear if triangle cur - 1, i, cur + 1 contains no reflex.
            let prev: NVector = remaining[(i + len - 1) % len].0;
            let next = remaining[(i + 1) % len].0;
            let ear = all_outside(prev, cur.0, next, remaining);
            if ear {
                remaining.remove(i);
                // re-classify adjacent vertices if more than 3 vertices.
                if remaining.len() > 3 {
                    re_classify(remaining, i);
                }
                return Some((prev, cur.0, next));
            }
        }
    }
    None
}

/// Re-classifies the vertices adjacent to the removed ear.
/// - a vertex is a reflex if left of [previous, next],
/// - a vertex is a convex if right of [previous, next],
/// - otherwise it's both
fn re_classify(vertices: &mut Vec<Vertex>, ear_index: usize) {
    let len = vertices.len();
    let last = len - 1;
    if ear_index == 0 || ear_index == len {
        let mut v_prev = vertices[last].0;
        let mut v_cur = vertices[0].0;
        let mut v_next = vertices[1].0;
        classify(&mut vertices[0], Sphere::side(v_prev, v_cur, v_next));

        v_next = v_cur;
        v_cur = v_prev;
        v_prev = vertices[last - 1].0;
        classify(&mut vertices[last], Sphere::side(v_prev, v_cur, v_next));
    } else {
        let mut v_prev = vertices[ear_index - 1].0;
        let mut v_cur = vertices[ear_index].0;
        let mut v_next = if ear_index == last {
            vertices[0]
        } else {
            vertices[ear_index + 1]
        }
        .0;
        classify(
            &mut vertices[ear_index],
            Sphere::side(v_prev, v_cur, v_next),
        );

        v_next = v_cur;
        v_cur = v_prev;
        v_prev = if ear_index == 1 {
            vertices[last]
        } else {
            vertices[ear_index - 2]
        }
        .0;
        classify(
            &mut vertices[ear_index - 1],
            Sphere::side(v_prev, v_cur, v_next),
        );
    }
}

fn classify(v: &mut Vertex, side: i8) {
    match side.cmp(&0) {
        Ordering::Greater => v.1 = Classification::Reflex,
        Ordering::Less => v.1 = Classification::Convex,
        Ordering::Equal => v.1 = Classification::Both,
    }
}

/// Tests that all given vertices are outside the triangle defined by [v1, v2, v3].
fn all_outside(v1: NVector, v2: NVector, v3: NVector, vertices: &[Vertex]) -> bool {
    for v in vertices {
        // skip convex vertices.
        if v.1 != Classification::Convex {
            let loc = locate(v.0, v1, v2, v3);
            if loc == PosLocation::Inside || loc == PosLocation::Edge {
                return false;
            }
        }
    }
    true
}

fn locate(p: NVector, v1: NVector, v2: NVector, v3: NVector) -> PosLocation {
    if p == v1 || p == v2 || p == v3 {
        PosLocation::Vertex
    } else {
        let sign = if is_clockwise(&[v1, v2, v3]) { -1 } else { 1 };
        locate_with_orientation(p, v1, v2, v3, sign)
    }
}

/// locate with sign: -1 for clockwise, 1 for anti-clockwise.
fn locate_with_orientation(
    p: NVector,
    v1: NVector,
    v2: NVector,
    v3: NVector,
    sign: i32,
) -> PosLocation {
    let s = sign as f64;
    let side_edge1 = Sphere::side_exact(p, v1, v2) * s;
    let side_edge2 = Sphere::side_exact(p, v2, v3) * s;
    let side_edge3 = Sphere::side_exact(p, v3, v1) * s;

    let on_edge1 = eq_zero(side_edge1);
    let on_edge2 = eq_zero(side_edge2);
    let on_edge3 = eq_zero(side_edge3);

    let mut pending_location = PosLocation::Outside;
    if on_edge1 && side_edge2 > 0.0 && side_edge3 > 0.0 {
        pending_location = PosLocation::Edge;
    }

    if on_edge2 && side_edge1 > 0.0 && side_edge3 > 0.0 {
        if pending_location == PosLocation::Edge {
            // position is detected on (vertex1, vertex2) and (vertex2, vertex3), assume it is vertex2.
            return PosLocation::Vertex;
        }
        pending_location = PosLocation::Edge;
    }

    if on_edge3 && side_edge1 > 0.0 && side_edge2 > 0.0 {
        if pending_location == PosLocation::Edge {
            // position is detected on (vertex1, vertex2) or (vertex2, vertex3)
            // and (vertex3, vertex1), assume it is vertex3 or vertex1.
            return PosLocation::Vertex;
        }
        pending_location = PosLocation::Edge;
    }

    if pending_location == PosLocation::Edge {
        return pending_location;
    }

    if side_edge1 > 0.0 && side_edge2 > 0.0 && side_edge3 > 0.0 {
        return PosLocation::Inside;
    }
    PosLocation::Outside
}

fn eq(a: Vec3, b: Vec3) -> bool {
    let d = a - b;
    eq_zero(d.x()) && eq_zero(d.y()) && eq_zero(d.z())
}

fn is_self_intersecting(es: &[MinorArc]) -> bool {
    let len = es.len();
    if len <= 3 {
        false
    } else {
        // check that no pair of non-contiguous edges intersects.
        for i in 0..len - 1 {
            let e1 = es[i];
            let last = if i == 0 { len - 1 } else { len };
            for e2 in es.iter().take(last).skip(i + 2) {
                if e1.intersection(*e2).is_some() {
                    return true;
                }
            }
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use crate::{spherical::Loop, Angle, NVector, Vec3};

    fn antananrivo() -> NVector {
        NVector::from_lat_long_degrees(-18.8792, 47.5079)
    }

    fn bangui() -> NVector {
        NVector::from_lat_long_degrees(4.3947, 18.5582)
    }

    fn dar_es_salaam() -> NVector {
        NVector::from_lat_long_degrees(-6.7924, 39.2083)
    }

    fn djibouti() -> NVector {
        NVector::from_lat_long_degrees(11.8251, 42.5903)
    }

    fn harare() -> NVector {
        NVector::from_lat_long_degrees(-17.8252, 31.0335)
    }

    fn helsingborg() -> NVector {
        NVector::from_lat_long_degrees(56.0465, 12.6945)
    }

    fn hoor() -> NVector {
        NVector::from_lat_long_degrees(55.9349, 13.5396)
    }

    fn juba() -> NVector {
        NVector::from_lat_long_degrees(4.8594, 31.5713)
    }

    fn kinshasa() -> NVector {
        NVector::from_lat_long_degrees(-4.4419, 15.2663)
    }

    fn kristianstad() -> NVector {
        NVector::from_lat_long_degrees(56.0294, 14.1567)
    }

    fn lund() -> NVector {
        NVector::from_lat_long_degrees(55.7047, 13.191)
    }

    fn malmo() -> NVector {
        NVector::from_lat_long_degrees(55.605, 13.0038)
    }

    fn narobi() -> NVector {
        NVector::from_lat_long_degrees(-1.2921, 36.8219)
    }

    fn ystad() -> NVector {
        NVector::from_lat_long_degrees(55.4295, 13.82)
    }

    // is_convex

    #[test]
    fn is_convex_concave() {
        assert_convex(false, &vec![ystad(), hoor(), helsingborg(), kristianstad()]);
    }

    #[test]
    fn is_convex_concave_collinear_vertices() {
        assert_convex(
            false,
            &vec![
                NVector::from_lat_long_degrees(10.0, 10.0),
                NVector::from_lat_long_degrees(11.0, 10.0),
                NVector::from_lat_long_degrees(12.0, 10.0),
                NVector::from_lat_long_degrees(12.0, 15.0),
                NVector::from_lat_long_degrees(11.0, 12.5),
                NVector::from_lat_long_degrees(10.0, 15.0),
            ],
        );
    }

    #[test]
    fn is_convex() {
        assert_convex(true, &vec![ystad(), malmo(), helsingborg(), kristianstad()]);
    }

    fn assert_convex(e: bool, vs: &[NVector]) {
        assert_eq!(e, Loop::new(vs).is_convex());
        let mut rvs = vs.to_vec();
        rvs.reverse();
        assert_eq!(e, Loop::new(&rvs).is_convex());
    }

    // contains_point.

    #[test]
    fn contains_point_north_pole_cap() {
        let vertices: Vec<NVector> = vec![
            NVector::from_lat_long_degrees(85.0, 10.0),
            NVector::from_lat_long_degrees(85.0, 170.0),
            NVector::from_lat_long_degrees(85.0, -170.0),
            NVector::from_lat_long_degrees(85.0, -10.0),
        ];
        let l = Loop::new(&vertices);
        assert!(l.contains_point(NVector::from_lat_long_degrees(90.0, 0.0)));
        assert!(l.contains_point(NVector::from_lat_long_degrees(89.0, 160.0)));
        assert!(!l.contains_point(NVector::from_lat_long_degrees(84.0, 160.0)));
        assert!(!l.contains_point(NVector::from_lat_long_degrees(-90.0, 0.0)));
    }

    #[test]
    fn contains_point_south_pole_cap() {
        let vertices: Vec<NVector> = vec![
            NVector::from_lat_long_degrees(-85.0, 10.0),
            NVector::from_lat_long_degrees(-85.0, 170.0),
            NVector::from_lat_long_degrees(-85.0, -170.0),
            NVector::from_lat_long_degrees(-85.0, -10.0),
        ];
        let l = Loop::new(&vertices);
        assert!(l.contains_point(NVector::from_lat_long_degrees(-90.0, 0.0)));
        assert!(l.contains_point(NVector::from_lat_long_degrees(-89.0, 160.0)));
        assert!(!l.contains_point(NVector::from_lat_long_degrees(-84.0, 160.0)));
        assert!(!l.contains_point(NVector::from_lat_long_degrees(90.0, 0.0)));
    }

    #[test]
    fn contains_point_concave_polygon() {
        let vertices: Vec<NVector> = vec![malmo(), ystad(), kristianstad(), helsingborg(), lund()];
        let l = Loop::new(&vertices);
        let hoor = NVector::from_lat_long_degrees(55.9295, 13.5297);
        let hassleholm = NVector::from_lat_long_degrees(56.1589, 13.7668);
        assert!(l.contains_point(hoor));
        assert!(!l.contains_point(hassleholm));
        for v in vertices {
            assert!(!l.contains_point(v));
        }
    }

    #[test]
    fn does_not_contain_point_on_edge() {
        let vertices = vec![
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(0.0, 10.0),
            NVector::from_lat_long_degrees(10.0, 10.0),
            NVector::from_lat_long_degrees(10.0, 0.0),
        ];

        let l = Loop::new(&vertices);

        // (0.0, 5.0) is on the (0.0, 0.0) -> (0.0, 10.0)
        let p = NVector::from_lat_long_degrees(0.0, 5.0);
        assert!(!l.contains_point(p));
    }

    // see: https://github.com/spacetelescope/spherical_geometry/blob/master/spherical_geometry/tests/test_basic.py
    #[test]
    fn does_not_contain_point_outside() {
        let p = NVector::new(Vec3::new_unit(-0.27475449, 0.47588873, -0.83548781));
        let vs = vec![
            NVector::new(Vec3::new_unit(0.04821217, -0.29877206, 0.95310589)),
            NVector::new(Vec3::new_unit(0.04451801, -0.47274119, 0.88007608)),
            NVector::new(Vec3::new_unit(-0.14916503, -0.46369786, 0.87334649)),
            NVector::new(Vec3::new_unit(-0.14916503, -0.46369786, 0.87334649)),
            NVector::new(Vec3::new_unit(0.04821217, -0.29877206, 0.95310589)),
        ];
        let l = Loop::new(&vs);
        assert!(!l.contains_point(p));
    }

    // triangulate.

    #[test]
    fn triangulate_collinear_during_triangulation_1() {
        let v0 = NVector::from_lat_long_degrees(35.0, 10.0);
        let v1 = NVector::from_lat_long_degrees(35.0, 20.0);
        let v2 = NVector::from_lat_long_degrees(30.0, 20.0);
        let v3 = NVector::from_lat_long_degrees(25.0, 25.0);
        let v4 = NVector::from_lat_long_degrees(20.0, 20.0);

        let expected = vec![(v0, v1, v2), (v4, v0, v2), (v2, v3, v4)];
        assert_loop_triangulation(&expected, &vec![v0, v1, v2, v3, v4]);
    }

    #[test]
    fn triangulate_collinear_during_triangulation_2() {
        let v0 = NVector::from_lat_long_degrees(17.0, 100.0);
        let v1 = NVector::from_lat_long_degrees(16.0, 105.0);
        let v2 = NVector::from_lat_long_degrees(15.0, 100.0);
        let v3 = NVector::from_lat_long_degrees(10.0, 100.0);
        let v4 = NVector::from_lat_long_degrees(10.0, 90.0);
        let v5 = NVector::from_lat_long_degrees(20.0, 90.0);
        let v6 = NVector::from_lat_long_degrees(20.0, 100.0);

        let expected = vec![
            (v0, v1, v2),
            (v2, v3, v4),
            (v0, v2, v4),
            (v6, v0, v4),
            (v4, v5, v6),
        ];
        assert_loop_triangulation(&expected, &vec![v0, v1, v2, v3, v4, v5, v6]);
    }

    #[test]
    fn triangulate_convex_6() {
        let vs = &vec![
            bangui(),
            juba(),
            narobi(),
            dar_es_salaam(),
            harare(),
            kinshasa(),
        ];

        let expected = vec![
            (kinshasa(), bangui(), juba()),
            (kinshasa(), juba(), narobi()),
            (kinshasa(), narobi(), dar_es_salaam()),
            (dar_es_salaam(), harare(), kinshasa()),
        ];
        assert_loop_triangulation(&expected, &vs);
    }

    #[test]
    fn triangulate_concave_7() {
        let vs = vec![
            bangui(),
            juba(),
            djibouti(),
            antananrivo(),
            dar_es_salaam(),
            kinshasa(),
            narobi(),
        ];

        let expected = vec![
            (narobi(), bangui(), juba()),
            (narobi(), juba(), djibouti()),
            (narobi(), djibouti(), antananrivo()),
            (narobi(), antananrivo(), dar_es_salaam()),
            (dar_es_salaam(), kinshasa(), narobi()),
        ];

        assert_loop_triangulation(&expected, &vs);
    }

    #[test]
    fn triangulate_quadrangle_with_many_on_meridian() {
        let v0 = NVector::from_lat_long_degrees(-85.0, 53.0);
        let v1 = NVector::from_lat_long_degrees(-45.0, 53.0);
        let v2 = NVector::from_lat_long_degrees(-45.0, 75.0);
        let v3 = NVector::from_lat_long_degrees(-55.0, 75.0);
        let v4 = NVector::from_lat_long_degrees(-58.0, 75.0);
        let v5 = NVector::from_lat_long_degrees(-65.0, 75.0);
        let v6 = NVector::from_lat_long_degrees(-75.0, 75.0);
        let v7 = NVector::from_lat_long_degrees(-76.0, 75.0);
        let v8 = NVector::from_lat_long_degrees(-78.0, 75.0);
        let v9 = NVector::from_lat_long_degrees(-85.0, 75.0);

        let expected = vec![
            (v9, v0, v1),
            (v1, v2, v3),
            (v1, v3, v4),
            (v1, v4, v5),
            (v1, v5, v6),
            (v1, v6, v7),
            (v1, v7, v8),
            (v1, v8, v9),
        ];
        assert_loop_triangulation(&expected, &vec![v0, v1, v2, v3, v4, v5, v6, v7, v8, v9]);
    }

    #[test]
    fn triangulate_several_collinear_during_triangulation() {
        let v0 = NVector::from_lat_long_degrees(15.0, 20.0);
        let v1 = NVector::from_lat_long_degrees(10.0, 25.0);
        let v2 = NVector::from_lat_long_degrees(5.0, 20.0);
        let v3 = NVector::from_lat_long_degrees(0.0, 20.0);
        let v4 = NVector::from_lat_long_degrees(0.0, 10.0);
        let v5 = NVector::from_lat_long_degrees(35.0, 10.0);
        let v6 = NVector::from_lat_long_degrees(35.0, 20.0);
        let v7 = NVector::from_lat_long_degrees(30.0, 20.0);
        let v8 = NVector::from_lat_long_degrees(25.0, 25.0);
        let v9 = NVector::from_lat_long_degrees(20.0, 20.0);

        let expected = vec![
            (v0, v1, v2),
            (v2, v3, v4),
            (v0, v2, v4),
            (v9, v0, v4),
            (v9, v4, v5),
            (v5, v6, v7),
            (v9, v5, v7),
            (v7, v8, v9),
        ];
        assert_loop_triangulation(&expected, &vec![v0, v1, v2, v3, v4, v5, v6, v7, v8, v9]);
    }

    #[test]
    fn triangulate_self_intersecting() {
        let l = Loop::new(&vec![
            NVector::from_lat_long_degrees(-2.0, -2.0),
            NVector::from_lat_long_degrees(2.0, -2.0),
            NVector::from_lat_long_degrees(3.0, 0.0),
            NVector::from_lat_long_degrees(-2.0, 2.0),
            NVector::from_lat_long_degrees(2.0, 2.0),
        ]);
        assert!(l.triangulate().is_empty());
    }

    // spherical_excess

    // see: https://github.com/chrisveness/geodesy/blob/master/test/latlon-nvector-spherical-tests.js
    #[test]
    fn spherical_excess() {
        let vs = vec![
            NVector::from_lat_long_degrees(1.0, 1.0),
            NVector::from_lat_long_degrees(5.0, 1.0),
            NVector::from_lat_long_degrees(5.0, 3.0),
            NVector::from_lat_long_degrees(1.0, 3.0),
            NVector::from_lat_long_degrees(3.0, 2.0),
        ];
        let l = Loop::new(&vs);
        assert_eq!(
            Angle::from_radians(0.0018241779916116775),
            l.spherical_excess().round_d7()
        );
    }

    fn assert_loop_triangulation(e: &[(NVector, NVector, NVector)], vs: &[NVector]) {
        assert_triangulation(e, &Loop::new(&vs));
        let mut rvs = vs.to_vec();
        rvs.reverse();
        assert_triangulation(e, &Loop::new(&rvs));
    }

    fn assert_triangulation(e: &[(NVector, NVector, NVector)], l: &Loop) {
        let a = l.triangulate();
        assert_eq!(e, a);
        // invariant: spherical excess
        let mut e_spherical_excess = Angle::ZERO;
        for t in e {
            e_spherical_excess =
                e_spherical_excess + Loop::new(&[t.0, t.1, t.2]).spherical_excess();
        }
        assert_eq!(
            e_spherical_excess.round_d7(),
            l.spherical_excess().round_d7()
        );

        // invariant: number of triangles = number of vertices - 2
        assert_eq!(e.len(), l.num_vertices() - 2);
    }
}

use std::{cmp::Ordering, f64::consts::PI};

use crate::{numbers::eq, numbers::eq_zero, Angle, NVector, Vec3};

use super::{
    base::{angle_radians_between, exact_side},
    MinorArc, Rectangle, Sphere,
};

/// A single chain of vertices where the first vertex is implicitly connected to the last.
///
/// Loops are either:
/// - [simple](crate::spherical::Loop::is_simple) - this property is not enforced at runtime, therefore operations are undefined on non-simple loops
/// - or, [empty](crate::spherical::Loop::is_empty).
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
    /// an empty [Loop]: 0 vertex and edge.
    pub const EMPTY: Self = Self {
        vertices: Vec::new(),
        insides: None,
        edges: Vec::new(),
    };

    /// Creates a new loop from the given vertices.
    ///
    /// The vertices can:
    /// - be given in clockwise or anti-clockwise order,
    /// - define a loop explicity closed (first == last) or opened (first != last)
    ///
    /// An [empty](crate::spherical::Loop::EMPTY) loop is returned if the given vertices are [empty](crate::spherical::Loop::is_empty).
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::NVector;
    /// use jord::spherical::Loop;
    ///
    /// // clockwise or anti-clockwise order:
    /// assert_eq!(
    ///     Loop::new(&vec![
    ///         NVector::from_lat_long_degrees(40.0, 40.0),
    ///         NVector::from_lat_long_degrees(10.0, 30.0),
    ///         NVector::from_lat_long_degrees(20.0, 20.0),
    ///         NVector::from_lat_long_degrees(50.0, 50.0),
    ///     ]),
    ///     Loop::new(&vec![
    ///         NVector::from_lat_long_degrees(50.0, 50.0),
    ///         NVector::from_lat_long_degrees(20.0, 20.0),
    ///         NVector::from_lat_long_degrees(10.0, 30.0),
    ///         NVector::from_lat_long_degrees(40.0, 40.0),
    ///     ])
    /// );
    ///
    /// // open or closed:
    /// assert_eq!(
    ///     Loop::new(&vec![
    ///         NVector::from_lat_long_degrees(40.0, 40.0),
    ///         NVector::from_lat_long_degrees(10.0, 30.0),
    ///         NVector::from_lat_long_degrees(20.0, 20.0),
    ///     ]),
    ///     Loop::new(&vec![
    ///         NVector::from_lat_long_degrees(40.0, 40.0),
    ///         NVector::from_lat_long_degrees(10.0, 30.0),
    ///         NVector::from_lat_long_degrees(20.0, 20.0),
    ///         NVector::from_lat_long_degrees(40.0, 40.0),
    ///     ])
    /// );
    /// ```
    pub fn new(vs: &[NVector]) -> Self {
        let opened = opened(vs);
        let len = opened.len();
        if len < 3 {
            Self::EMPTY
        } else {
            let (edges, clockwise) = to_edges(opened);
            let clockwise_edges = if clockwise {
                edges
            } else {
                reverse_edges(&edges)
            };
            let vertices = clockwise_edges_to_vertices(&clockwise_edges);
            if vertices.iter().all(|v| v.1 == Classification::Both) {
                // only collinear vertices.
                Self::EMPTY
            } else {
                let insides = if len > 3 {
                    find_insides(&vertices)
                } else {
                    None
                };
                Self {
                    vertices,
                    insides,
                    edges: clockwise_edges,
                }
            }
        }
    }

    /// Determines whether this loop is convex.
    ///
    /// This function always returns false for [empty](crate::spherical::Loop::is_empty) loops, undefined for [non simple](crate::spherical::Loop::is_simple) loops.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::NVector;
    /// use jord::spherical::Loop;
    ///
    /// let vs = vec![
    ///     NVector::from_lat_long_degrees(40.0, 40.0),
    ///     NVector::from_lat_long_degrees(10.0, 30.0),
    ///     NVector::from_lat_long_degrees(20.0, 20.0),
    ///     NVector::from_lat_long_degrees(40.0, 40.0),
    /// ];
    ///
    /// let l = Loop::new(&vs);
    ///
    /// assert!(l.is_convex());
    /// ```
    pub fn is_convex(&self) -> bool {
        match self.vertices.len().cmp(&3) {
            Ordering::Less => false,
            Ordering::Equal => true,
            Ordering::Greater => {
                let mut cur_side: i8 = i8::MIN;
                let mut found_left_right: bool = false;
                let len: usize = self.vertices.len();
                for i in 0..len {
                    let prev: NVector = self.vertices[(i + len - 1) % len].0;
                    let cur: NVector = self.vertices[i].0;
                    let next = self.vertices[(i + 1) % len].0;
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

    /// Determines whether this loop is simple:
    /// - All edges are [valid](crate::spherical::Sphere::is_great_circle) [minor arc](crate::spherical::MinorArc)s: consecutive vertices cannot be coincidental or the antipode of one another, and,
    /// - The loop is not self-intersecting: no pair of non-contiguous edges intersect.
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
    /// assert!(!l1.is_simple());
    ///
    /// // consectutive antipodal vertices:
    /// let l2 = Loop::new(&vec![
    ///     NVector::from_lat_long_degrees(-2.0, -2.0),
    ///     NVector::from_lat_long_degrees(-2.0, -2.0).antipode(),
    ///     NVector::from_lat_long_degrees(3.0, 0.0),
    /// ]);
    /// assert!(!l2.is_simple());
    ///
    /// // self-intersecting loop:
    /// let l3 = Loop::new(&vec![
    ///     NVector::from_lat_long_degrees(-2.0, -2.0),
    ///     NVector::from_lat_long_degrees(2.0, -2.0),
    ///     NVector::from_lat_long_degrees(3.0, 0.0),
    ///     NVector::from_lat_long_degrees(-2.0, 2.0),
    ///     NVector::from_lat_long_degrees(2.0, 2.0),
    /// ]);
    /// assert!(!l3.is_simple());
    ///
    /// // simple loop:
    /// let l4 = Loop::new(&vec![
    ///     NVector::from_lat_long_degrees(-2.0, -2.0),
    ///     NVector::from_lat_long_degrees(2.0, -2.0),
    ///     NVector::from_lat_long_degrees(3.0, 0.0),
    ///     NVector::from_lat_long_degrees(2.0, 2.0),
    ///     NVector::from_lat_long_degrees(-2.0, 2.0),
    /// ]);
    /// assert!(l4.is_simple());
    /// ```
    pub fn is_simple(&self) -> bool {
        let v_len = self.vertices.len();
        for i in 0..v_len {
            if !Sphere::is_great_circle(self.vertices[i].0, self.vertices[(i + 1) % v_len].0) {
                return false;
            }
        }
        let es_len = self.edges.len();
        if es_len <= 3 {
            true
        } else {
            // check that no pair of non-contiguous edges intersects.
            for i in 0..es_len - 1 {
                let e1 = self.edges[i];
                let last = if i == 0 { es_len - 1 } else { es_len };
                for e2 in self.edges.iter().take(last).skip(i + 2) {
                    if e1.intersection(*e2).is_some() {
                        return false;
                    }
                }
            }
            true
        }
    }

    /// Determines whether this loop is empty. An loop is empty if less than 3 non-collinear vertices were supplied at construction.
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
    ///     NVector::from_lat_long_degrees(1.0, 0.0),
    /// ]).is_empty());
    ///
    /// assert!(Loop::new(&[
    ///     NVector::from_lat_long_degrees(0.0, 0.0),
    ///     NVector::from_lat_long_degrees(1.0, 0.0),
    ///     NVector::from_lat_long_degrees(0.0, 0.0),
    /// ]).is_empty());
    ///
    /// assert!(Loop::new(&[
    ///     NVector::from_lat_long_degrees(0.0, 0.0),
    ///     NVector::from_lat_long_degrees(0.0, 1.0),
    ///     NVector::from_lat_long_degrees(0.0, 2.0),
    /// ]).is_empty());
    /// ```
    pub fn is_empty(&self) -> bool {
        // see new(): if less than 3 non-collinear vertices were supplied then self.vertices is empty.
        self.vertices.is_empty()
    }

    /// Determines if the given position is a vertex of this loop.
    pub fn has_vertex(&self, p: NVector) -> bool {
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
    /// assert!(l.any_edge_contains_point(NVector::from_lat_long_degrees(0.0, 5.0)));
    /// assert!(!l.any_edge_contains_point(NVector::from_lat_long_degrees(0.0, 11.0)));
    /// ```
    pub fn any_edge_contains_point(&self, p: NVector) -> bool {
        self.edges.iter().any(|e| e.contains_point(p))
    }

    /// Returns the number of vertices of this loop.
    pub fn num_vertices(&self) -> usize {
        self.vertices.len()
    }

    /// Returns the vertex at the given index (panics if the given index is invalid).
    pub fn vertex(&self, i: usize) -> NVector {
        self.vertices[i].0
    }

    /// Returns a iterator over the vertices of this loop in clockwise order.
    pub fn iter_vertices(&self) -> impl Iterator<Item = &NVector> {
        self.vertices.iter().map(|v| &v.0)
    }

    /// Returns a iterator over the edges of this loop in clockwise order.
    pub fn iter_edges(&self) -> impl Iterator<Item = &MinorArc> {
        self.edges.iter()
    }

    /// Calculates the [minimum bounding rectangle](crate::spherical::Rectangle) of this loop. The returned bound is
    /// conservative in that if this loop [contains](crate::spherical::Loop::contains_point) a point P,
    /// then the bound also [contains](crate::spherical::Rectangle::contains_point) P.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{LatLong, NVector};
    /// use jord::spherical::Loop;
    ///
    /// let vs = vec![
    ///     NVector::from_lat_long_degrees(55.605, 13.0038),
    ///     NVector::from_lat_long_degrees(55.4295, 13.82),
    ///     NVector::from_lat_long_degrees(56.0294, 14.1567),
    ///     NVector::from_lat_long_degrees(56.0465, 12.6945),
    ///     NVector::from_lat_long_degrees(55.7047, 13.191),
    /// ];
    ///
    /// let l = Loop::new(&vs);
    /// let b = l.bound();
    /// for v in vs.iter() {
    ///     let ll = LatLong::from_nvector(*v);
    ///     assert!(b.contains_point(ll));
    /// }
    /// ```
    pub fn bound(&self) -> Rectangle {
        let all: Vec<Rectangle> = self
            .edges
            .iter()
            .map(|e| Rectangle::from_minor_arc(*e))
            .collect();
        let mut mbr = Rectangle::from_union(&all);

        // expand by 1e-7 degrees which is about 11.1 millimetres at the equator and
        // is the near limit of GPS-based technique - this is to make sure that floating-point
        // error introduced when converting NVector <-> LatLong does not break the bounds
        // invariant: loop.contains_point(p) -> loop.bounds().contains_point(LatLong::from_nvector(p))
        mbr = mbr.expand(Angle::from_degrees(1.0e-7));

        // expand the longitude interval to full if the latitude interval includes any of the 2 poles.
        mbr = mbr.polar_closure();

        static NP: NVector = NVector::new(Vec3::UNIT_Z);
        static SP: NVector = NVector::new(Vec3::NEG_UNIT_Z);

        if self.contains_point(NP) {
            mbr = mbr.expand_to_north_pole();
        }

        // If a loop contains the south pole, then either it wraps entirely around the sphere (full longitude
        // range), or it also contains the north pole in which case bounds#is_longitude_full() is true due to the
        // test above. Either way, we only need to do the south pole containment test if bounds#is_longitude_full().
        if mbr.is_longitude_full() && self.contains_point(SP) {
            mbr = mbr.expand_to_south_pole();
        }
        mbr
    }

    /// Determines whether the **interior** of this loop contains the given point (i.e. excluding points which are
    /// vertices or on an edge of this loop).
    ///
    /// This function always returns false for [empty](crate::spherical::Loop::is_empty) loops, undefined for [non simple](crate::spherical::Loop::is_simple) loops.
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
                if p == a || p == b {
                    return true;
                }
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
                for (i, e) in self.edges.iter().enumerate() {
                    if let Some(iv) = ma.intersection(*e) {
                        if i == 0 {
                            count_i += 1;
                            first_i_vec3 = iv.as_vec3();
                        } else if i == n - 1 {
                            let iv_vec3 = iv.as_vec3();
                            // last edge, check diff with first and prev.
                            if vec3_eq(first_i_vec3, iv_vec3) || vec3_eq(prev_i_vec3, iv_vec3) {
                                // skip this intersection (already found on previous or first edge).
                            } else {
                                count_i += 1;
                            }
                        } else {
                            let iv_vec3 = iv.as_vec3();
                            // check diff with prev.
                            if vec3_eq(prev_i_vec3, iv_vec3) {
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
                    if p == self.vertices[0].0 || p == self.vertices[1].0 || p == self.vertices[2].0
                    {
                        return false;
                    }
                    // vertices are in clockwise order, so negate dot product for side.
                    let v = p.as_vec3();
                    let side_edge1 = -v.dot_prod(self.edges[0].normal());
                    let side_edge2 = -v.dot_prod(self.edges[1].normal());
                    let side_edge3 = -v.dot_prod(self.edges[2].normal());

                    let on_edge1 = eq_zero(side_edge1);
                    if on_edge1 && side_edge2 > 0.0 && side_edge3 > 0.0 {
                        return false;
                    }

                    let on_edge2 = eq_zero(side_edge2);
                    if on_edge2 && side_edge1 > 0.0 && side_edge3 > 0.0 {
                        return false;
                    }

                    let on_edge3 = eq_zero(side_edge3);
                    if on_edge3 && side_edge1 > 0.0 && side_edge2 > 0.0 {
                        return false;
                    }

                    side_edge1 > 0.0 && side_edge2 > 0.0 && side_edge3 > 0.0
                } else {
                    false
                }
            }
        }
    }

    /// Triangulates this loop using the [Ear Clipping](https://www.geometrictools.com/Documentation/TriangulationByEarClipping.pdf) method.
    ///  
    /// This method returns either ([loop number vertices](crate::spherical::Loop::num_vertices) - 2) triangles - as triples of [NVector]s, if
    /// the triangulation succeeds, or [empty](Vec::new) if the triangulation fails - which should only occur for [non simple](crate::spherical::Loop::is_simple) loops.
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

/// Determines whether the given vertices are given in clockwise order.
///
/// - the loop can be explicity closed (first == last) or opened (first != last)
/// - returns false if less than 3 vertices are given
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
    let len = ovs.len();
    match len.cmp(&3) {
        Ordering::Less => false,
        Ordering::Equal => Sphere::side(ovs[0], ovs[1], ovs[2]) < 0,
        Ordering::Greater => {
            let mut turn: Angle = Angle::ZERO;
            for i in 0..len {
                let prev: NVector = ovs[(i + len - 1) % len];
                let cur = ovs[i];
                let next = ovs[(i + 1) % len];
                turn = turn + Sphere::turn(prev, cur, next);
            }
            turn.as_radians() < 0.0
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

/// Builds vertices by iterating the given array of edges in order (i.e. edges are given in clockwise order).
fn clockwise_edges_to_vertices(es: &[MinorArc]) -> Vec<Vertex> {
    let len: usize = es.len();
    let mut res: Vec<Vertex> = Vec::with_capacity(len);
    for i in 0..len {
        let prev = es[(i + len - 1) % len];
        let cur = es[i];
        let side = cur.side_of(prev.start());
        let vertex = match side.cmp(&0) {
            Ordering::Greater => Vertex(cur.start(), Classification::Reflex),
            Ordering::Less => Vertex(cur.start(), Classification::Convex),
            Ordering::Equal => Vertex(cur.start(), Classification::Both),
        };
        res.push(vertex);
    }
    res
}

/// Reveres the given edges.
fn reverse_edges(es: &[MinorArc]) -> Vec<MinorArc> {
    let len = es.len();
    let last = len - 1;
    let mut res: Vec<MinorArc> = Vec::with_capacity(len);
    for i in (0..last).rev() {
        res.push(es[i].opposite());
    }
    res.push(es[last].opposite());
    res
}

/// vertices to edges: last edge connect last vertex to first vertex + are vertices given in clockwise order?.
fn to_edges(vs: &[NVector]) -> (Vec<MinorArc>, bool) {
    let len: usize = vs.len();
    let mut edges: Vec<MinorArc> = Vec::with_capacity(len);
    let mut turn = Angle::ZERO;
    for i in 0..len {
        let cur = vs[i];
        let next = vs[(i + 1) % len];
        let e = MinorArc::new(cur, next);
        edges.push(e);
        if i > 0 {
            turn = turn + edges[i - 1].turn(e);
        }
    }
    // turn from last edge to first edge.
    turn = turn + edges[len - 1].turn(edges[0]);
    let clockwise = turn.as_radians() < 0.0;
    (edges, clockwise)
}

/// Triangulates given loop using ear-clipping method.
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
            let inside =
                Sphere::triangle_mean_position(remaining[0].0, remaining[1].0, remaining[2].0);
            if let Some(p) = inside {
                res.push(p);
            }
            break;
        }

        if let Some(ear) = next_ear(&mut remaining) {
            let inside = Sphere::triangle_mean_position(ear.0, ear.1, ear.2);
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
        if v.1 != Classification::Convex && inside_or_edge(v.0, v1, v2, v3) {
            return false;
        }
    }
    true
}

/// if p inside triangle (v1, v2, v3) or on any edge of that triangle.
fn inside_or_edge(p: NVector, v1: NVector, v2: NVector, v3: NVector) -> bool {
    if p == v1 || p == v2 || p == v3 {
        return false;
    }
    let clockwise = Sphere::side(v1, v2, v3) < 0;
    let sign = if clockwise { -1.0 } else { 1.0 };
    let side_edge1 = exact_side(p.as_vec3(), v1.as_vec3(), v2.as_vec3()) * sign;
    let side_edge2 = exact_side(p.as_vec3(), v2.as_vec3(), v3.as_vec3()) * sign;
    let side_edge3 = exact_side(p.as_vec3(), v3.as_vec3(), v1.as_vec3()) * sign;

    let on_edge1 = eq_zero(side_edge1);
    let on_edge2 = eq_zero(side_edge2);
    let on_edge3 = eq_zero(side_edge3);

    let mut on_edge = false;
    if on_edge1 && side_edge2 > 0.0 && side_edge3 > 0.0 {
        on_edge = true;
    }

    if on_edge2 && side_edge1 > 0.0 && side_edge3 > 0.0 {
        if on_edge {
            // position is detected on (vertex1, vertex2) and (vertex2, vertex3), assume it is vertex2.
            return false;
        }
        on_edge = true;
    }

    if on_edge3 && side_edge1 > 0.0 && side_edge2 > 0.0 {
        if on_edge {
            // position is detected on (vertex1, vertex2) or (vertex2, vertex3)
            // and (vertex3, vertex1), assume it is vertex3 or vertex1.
            return false;
        }
        on_edge = true;
    }

    if on_edge {
        return true;
    }

    side_edge1 > 0.0 && side_edge2 > 0.0 && side_edge3 > 0.0
}

fn vec3_eq(a: Vec3, b: Vec3) -> bool {
    eq(a.x(), b.x()) && eq(a.y(), b.y()) && eq(a.z(), b.z())
}

#[cfg(test)]
mod tests {
    use crate::{
        spherical::{is_loop_clockwise, Loop},
        Angle, LatLong, NVector, Vec3,
    };

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

    // empty loop
    #[test]
    fn empty() {
        assert!(!Loop::EMPTY.is_convex());
        assert!(Loop::EMPTY.is_simple());
        assert!(Loop::EMPTY.is_empty());
        assert_eq!(0, Loop::EMPTY.num_vertices());
    }

    // new

    #[test]
    fn new_triangle() {
        assert_loop_invariants(&vec![
            NVector::from_lat_long_degrees(20.0, 20.0),
            NVector::from_lat_long_degrees(10.0, 30.0),
            NVector::from_lat_long_degrees(40.0, 40.0),
        ]);
    }

    #[test]
    fn new_loop() {
        assert_loop_invariants(&vec![
            NVector::from_lat_long_degrees(-85.0, 10.0),
            NVector::from_lat_long_degrees(-85.0, 170.0),
            NVector::from_lat_long_degrees(-85.0, -170.0),
            NVector::from_lat_long_degrees(-85.0, -10.0),
        ]);
    }

    #[test]
    fn new_empty() {
        assert!(Loop::new(&[]).is_empty());
        assert!(Loop::new(&[NVector::from_lat_long_degrees(0.0, 0.0)]).is_empty());
        assert!(Loop::new(&[
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(1.0, 0.0),
        ])
        .is_empty());
        assert!(Loop::new(&[
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(1.0, 0.0),
            NVector::from_lat_long_degrees(0.0, 0.0),
        ])
        .is_empty());
        assert!(Loop::new(&[
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(0.0, 1.0),
            NVector::from_lat_long_degrees(0.0, 2.0),
        ])
        .is_empty());
    }

    // asserts [v0, v1, .. , vn] = [vn, .., v1, v0] == [v0, v1, .. , vn, v0].
    // asserts num_vertices = opened.len
    // asserts iter_vertices = clockwise(opened).iter
    fn assert_loop_invariants(opened: &[NVector]) {
        let l1 = Loop::new(opened);

        let mut rvs = opened.to_vec();
        rvs.reverse();
        let l2 = Loop::new(&rvs);

        let mut closed = opened.to_vec();
        closed.push(closed[0]);
        let l3 = Loop::new(&closed);

        assert_eq!(l1, l2);
        assert_eq!(l1, l3);
        assert_eq!(l2, l3);

        assert_eq!(opened.len(), l1.num_vertices());
        assert_eq!(opened.len(), l2.num_vertices());
        assert_eq!(opened.len(), l3.num_vertices());

        let e_it = if is_loop_clockwise(&opened) {
            opened.iter()
        } else {
            rvs.iter()
        };

        assert!(e_it.clone().eq(l1.iter_vertices()));
        assert!(e_it.clone().eq(l2.iter_vertices()));
        assert!(e_it.clone().eq(l3.iter_vertices()));
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

    // bound

    #[test]
    fn bound_expansion() {
        let vs = vec![
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(0.0, 10.0),
            NVector::from_lat_long_degrees(5.0, 0.0),
        ];

        let l = Loop::new(&vs);
        let b = l.bound();

        let ne = b.north_east();
        assert_eq!(5.0000001, ne.latitude().as_degrees());
        assert_eq!(10.0000001, ne.longitude().as_degrees());

        let sw: LatLong = b.south_west();
        assert_eq!(-0.0000001, sw.latitude().as_degrees());
        assert_eq!(-0.0000001, sw.longitude().as_degrees());

        for v in vs.iter() {
            let ll = LatLong::from_nvector(*v);
            assert!(b.contains_point(ll));
        }
    }

    // contains_point.

    #[test]
    fn triangle_does_not_contain_antipode() {
        let inside = NVector::from_lat_long_degrees(15.0, 30.0);
        let antipode = inside.antipode();
        let v1 = NVector::from_lat_long_degrees(20.0, 20.0);
        let v2 = NVector::from_lat_long_degrees(10.0, 30.0);
        let v3 = NVector::from_lat_long_degrees(40.0, 40.0);
        let l = Loop::new(&vec![v1, v2, v3]);
        assert!(l.contains_point(inside));
        assert!(!l.contains_point(antipode));
    }

    #[test]
    fn large_triangle_does_not_contain_far_away() {
        let position = NVector::from_lat_long_degrees(-10.0, 29.0);
        let v1 = NVector::from_lat_long_degrees(10.0, 179.0);
        let v2 = NVector::from_lat_long_degrees(10.0, -150.0);
        let v3 = NVector::from_lat_long_degrees(-85.0, -150.0);
        let l = Loop::new(&vec![v1, v2, v3]);
        assert!(!l.contains_point(position));
    }

    #[test]
    fn contains_insides() {
        let vertices: Vec<NVector> = vec![
            NVector::from_lat_long_degrees(55.605, 13.0038),
            NVector::from_lat_long_degrees(55.4295, 13.82),
            NVector::from_lat_long_degrees(56.0294, 14.1567),
            NVector::from_lat_long_degrees(56.0465, 12.6945),
            NVector::from_lat_long_degrees(55.7047, 13.191),
        ];
        let l = Loop::new(&vertices);
        let i1: NVector = l.insides.unwrap().0;
        let i2: NVector = l.insides.unwrap().1;
        assert!(l.contains_point(i1));
        assert!(l.contains_point(i2));
    }

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
        assert!(l.any_edge_contains_point(p));
    }

    #[test]
    fn does_not_contain_point_on_edge_2() {
        let v2 = NVector::from_lat_long_degrees(0.0, 0.0);
        let one_mas: f64 = 1.0 / 3_600_000_000.0;
        let two_mas = 2.0 * one_mas;
        let v1: NVector = NVector::from_lat_long_degrees(-two_mas, 179.0);
        let v3 = NVector::from_lat_long_degrees(two_mas, 179.0);
        // p is one arc microsecond east of v2: detected on both (v1, v2) and (v2, v3).
        let p = NVector::from_lat_long_degrees(0.0, one_mas);

        let l = Loop::new(&vec![v1, v2, v3]);

        assert!(!l.contains_point(p));
        assert!(l.any_edge_contains_point(p));
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

    #[test]
    fn contains_one_mas_inside() {
        let vs = vec![
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(0.0, 10.0),
            NVector::from_lat_long_degrees(10.0, 10.0),
            NVector::from_lat_long_degrees(10.0, 0.0),
        ];

        let l = Loop::new(&vs);

        let one_mas = 1.0 / 3_600_000_000.0;
        let p = NVector::from_lat_long_degrees(one_mas, one_mas);
        assert!(l.contains_point(p));
    }

    #[test]
    fn does_not_contain_one_mas_outside() {
        let vs = vec![
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(0.0, 10.0),
            NVector::from_lat_long_degrees(10.0, 10.0),
            NVector::from_lat_long_degrees(10.0, 0.0),
        ];

        let l = Loop::new(&vs);

        let one_mas: f64 = 1.0 / 3_600_000_000.0;
        let p = NVector::from_lat_long_degrees(-one_mas, 0.0);
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

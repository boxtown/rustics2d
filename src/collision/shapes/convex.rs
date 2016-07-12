use std::cmp::Ordering;
use std::f64;
use std::result::Result;
use std::vec::Vec;
use collision::{Aabb, CollidesWith, HasAabb};
use common::{Transform, Vec2d};
use util;

/// Convex represents a convex polygon.
/// It contains the necessary information to be used
/// within collision detection algorithms
pub struct Convex {
    vertices: Vec<Vec2d>,
    normals: Vec<Vec2d>,
}

impl Convex {
    /// Creates a convex polygon from the given vertices or returns
    /// an error if a convex polygon could not be created from the vertices.
    /// Only the vertices on the minimal convex hull will be saved within
    /// the returned polygon
    pub fn new(vertices: &[Vec2d]) -> Result<Convex, ()> {
        // TODO: meld edges according to some line slop like in Box2d
        // TODO: switch graham scan out for gift-wrapping algorithm and test for speed
        graham_scan(&vertices).map(|hull| {
            let mut normals = Vec::new();
            for i in 0..hull.len() {
                let i2 = if i + 1 < hull.len() {
                    i + 1
                } else {
                    0
                };

                let edge = hull[i2] - hull[i];
                // TODO: assert non-0 length edge
                normals.push(Vec2d::new(1.0 * edge.y, -1.0 * edge.x).normalize());
            }

            Convex {
                vertices: hull,
                normals: normals,
            }
        })
    }

    /// Returns a reference to the slice of vertices
    /// making up the convex hull of this polygon
    pub fn vertices(&self) -> &[Vec2d] {
        &self.vertices
    }

    /// Returns a reference to the slice of edge normals
    /// for this convex polygon
    pub fn normals(&self) -> &[Vec2d] {
        &self.normals
    }
}

impl HasAabb for Convex {
    fn aabb(&self, transform: &Transform) -> Aabb {
        let transformed: Vec<Vec2d> = self.vertices()
                                          .into_iter()
                                          .map(|v| v.transform(transform))
                                          .collect();
        Aabb::new(&transformed).unwrap()
    }
}

impl CollidesWith<Convex> for Convex {
    fn collides_with(&self, other: &Convex, this_t: &Transform, other_t: &Transform) -> bool {
        let (_, sep_a) = find_max_separation(self, other, this_t, other_t);
        if sep_a > util::TOLERANCE {
            return false;
        }
        let (_, sep_b) = find_max_separation(other, self, this_t, other_t);
        if sep_b > util::TOLERANCE {
            return false;
        }
        true
    }
}

/// Calulcates and returns the maximum separation value on a separating axis
/// for the two Convex polygons and returns the index of the edge normal representing
/// the separating axis and the value of the separation using the GJK algorithm.
/// Algorithm sourced from Dirk Gregorius GDC talk on gamedev physics: http://gdcvault.com/play/1017646/Physics-for-Game-Programmers-The
fn find_max_separation(a: &Convex, b: &Convex, at: &Transform, bt: &Transform) -> (usize, f64) {
    let va = a.vertices();
    let na = a.normals();
    let vb = b.vertices();

    let mut best_i = 0;
    let mut max_sep = f64::MIN;

    for i in 0..va.len() {
        let vertex_a = va[i].transform(at);
        let normal = na[i].rotate(at.rotation()); // don't need full transform because normal is unit vector
        let neg_normal = -normal;

        // find the support point on b
        // by projecting the vertices in b against
        // the reversed edge normal for the edge
        let mut best_proj = f64::MIN;
        let mut support = vb[0]; // we don't transform because we don't care about the init value
        for j in 0..vb.len() {
            let vertex_b = vb[j].transform(bt);
            let proj = neg_normal * vertex_b; // scalar projection via: https://en.wikipedia.org/wiki/Scalar_projection
            if proj > best_proj {
                best_proj = proj;
                support = vertex_b;
            }
        }

        // calculate distance of support to edge,
        // save maximum distance. If the polygons intersect,
        // this will be the least negative separation
        let sep = normal * (support - vertex_a); // distance of point to edge via: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#A_vector_projection_proof
        if sep > max_sep {
            best_i = i;
            max_sep = sep;
        }
    }

    (best_i, max_sep)
}

// The type of angle three consecutive
// vertices form in 2d space
#[derive(Debug, Eq, PartialEq)]
enum VertexAngle {
    Collinear,
    Clockwise,
    CounterClockwise,
}

fn graham_scan(vertices: &[Vec2d]) -> Result<Vec<Vec2d>, ()> {
    let n = vertices.len();
    if n < 3 {
        return Err(());
    }

    let mut clone = vertices.to_vec();

    // find bottom-most vertex and swap
    // with first point
    let i = lowest_y_index(&clone);
    clone.swap(0, i);

    // sort vertices by polar coordinates with
    // respect to first point
    let sentinel = clone[0];
    clone.sort_by(|p1, p2| {
        let vertex_angle = vertex_angle(sentinel, *p1, *p2);
        match vertex_angle {
            VertexAngle::Collinear => {
                let ds1 = dist_sq(sentinel, *p1);
                let ds2 = dist_sq(sentinel, *p2);
                if ds2 >= ds1 {
                    return Ordering::Less;
                }
                return Ordering::Greater;
            }
            VertexAngle::Clockwise => Ordering::Greater,
            VertexAngle::CounterClockwise => Ordering::Less,
        }
    });

    // If two or more vertices make the same angle
    // with the first point, remove all but the furthest
    // point from the first point
    let mut m: usize = 1;
    let mut i: usize = 1;
    while i < n {
        let p1 = clone[i];
        let p2 = match i + 1 == n {
            true => clone[0],
            false => clone[i + 1],
        };
        while i < n - 1 && vertex_angle(sentinel, p1, p2) == VertexAngle::Collinear {
            i += 1;
        }

        clone[m] = clone[i];
        m += 1;
        i += 1;
    }

    // push first two vertices into hull
    let mut hull = Vec::new();
    hull.push(clone[0]);
    hull.push(clone[1]);

    // process remaining vertices
    i = 2;
    while i < m {
        // keep removing top while the angle formed by
        // next-to-top, top, and clone[i] makes a non-left turn
        let mut top = hull[hull.len() - 1];
        let mut next = hull[hull.len() - 2];
        let p = clone[i];
        while hull.len() >= 3 && vertex_angle(next, top, p) != VertexAngle::CounterClockwise {
            hull.pop();
            top = hull[hull.len() - 1];
            next = hull[hull.len() - 2];
        }
        hull.push(p);
        i += 1;
    }
    return Ok(hull);
}

// Returns the type of angle three vertices form in 2d space
fn vertex_angle(p1: Vec2d, p2: Vec2d, p3: Vec2d) -> VertexAngle {
    let x = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
    if util::feq(x, 0.0) {
        return VertexAngle::Collinear;
    }
    if x < 0.0 {
        return VertexAngle::Clockwise;
    }
    return VertexAngle::CounterClockwise;
}

// Returns the square of the distance
// of two vertices
fn dist_sq(p1: Vec2d, p2: Vec2d) -> f64 {
    (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y)
}

// iterates through vertices finding the index of the point
// with the lowest y coordinate or the one with the left-most
// x coordinate in case of a tie
fn lowest_y_index(vertices: &[Vec2d]) -> usize {
    let mut i: usize = 0;
    let mut j: usize = 0;
    let mut lowest = vertices[j];
    for p in vertices {
        if p.y < lowest.y || (util::feq(p.y, lowest.y) && p.x < lowest.x) {
            lowest = *p;
            j = i;
        }
        i += 1;
    }
    return j;
}

#[cfg(test)]
mod test {
    use std::vec::Vec;
    use collision::collidable::Convex;
    use vec2d::Vec2d;

    #[test]
    fn test_convex_from_vertices() {
        // test too few vertices
        let mut v: Vec<Vec2d> = Vec::new();
        assert_eq!(Convex::new(&mut v).is_err(), true);
        v.push(Vec2d::new(0.0, 0.0));
        v.push(Vec2d::new(1.0, 1.0));
        assert_eq!(Convex::new(&mut v).is_err(), true);

        // test basic hull
        v.push(Vec2d::new(1.0, 0.0));
        let mut clone = v.clone();
        let mut r = Convex::new(&mut clone);
        let mut r_ok = r.ok().unwrap();
        {
            let r_vertices = r_ok.vertices();
            assert_eq!(3, r_vertices.len());
            if r_vertices.iter().find(|&x| *x == Vec2d::new(0.0, 0.0)).is_none() {
                assert!(false);
            }
            if r_vertices.iter().find(|&x| *x == Vec2d::new(1.0, 1.0)).is_none() {
                assert!(false);
            }
            if r_vertices.iter().find(|&x| *x == Vec2d::new(1.0, 0.0)).is_none() {
                assert!(false);
            }
        }

        // add internal point
        v.push(Vec2d::new(0.5, 0.5));
        clone = v.clone();
        r = Convex::new(&mut clone);
        r_ok = r.ok().unwrap();
        {
            let r_vertices = r_ok.vertices();
            assert_eq!(3, r_vertices.len());
            if r_vertices.iter().find(|&x| *x == Vec2d::new(0.0, 0.0)).is_none() {
                assert!(false);
            }
            if r_vertices.iter().find(|&x| *x == Vec2d::new(1.0, 1.0)).is_none() {
                assert!(false);
            }
            if r_vertices.iter().find(|&x| *x == Vec2d::new(1.0, 0.0)).is_none() {
                assert!(false);
            }
        }

        // add collinear point
        v.push(Vec2d::new(1.0, 0.5));
        clone = v.clone();
        r = Convex::new(&mut clone);
        r_ok = r.ok().unwrap();
        {
            let r_vertices = r_ok.vertices();
            assert_eq!(3, r_vertices.len());
            if r_vertices.iter().find(|&x| *x == Vec2d::new(0.0, 0.0)).is_none() {
                assert!(false);
            }
            if r_vertices.iter().find(|&x| *x == Vec2d::new(1.0, 1.0)).is_none() {
                assert!(false);
            }
            if r_vertices.iter().find(|&x| *x == Vec2d::new(1.0, 0.0)).is_none() {
                assert!(false);
            }
        }
    }
}

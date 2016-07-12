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

/// The type of angle three consecutive
/// vertices form in 2d space
#[derive(Debug, Eq, PartialEq)]
enum VertexAngle {
    Collinear,
    Clockwise,
    CounterClockwise,
}

/// Performs a graham scan of the passed in vertices, returning
/// the resulting convex hull or an error if a hull could not be created
/// taken from: Sedgewick & Wayne, Algorithms, 4th edition, https://github.com/kevin-wayne/algs4/blob/master/src/main/java/edu/princeton/cs/algs4/GrahamScan.java
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

    let mut hull = Vec::new();
    hull.push(sentinel);

    // find first vertex not equal to sentinel
    let first = clone.iter().position(|&x| x != sentinel);
    if first.is_none() {
        // all vertices are the same point, error
        return Err(());
    }

    // find first vertex not collinear with sentinel and first
    let second = clone.iter().position(|&x| {
        vertex_angle(sentinel, clone[first.unwrap()], x) != VertexAngle::Collinear
    });
    if second.is_none() {
        // all vertices collinear with first two vertices, error
        return Err(());
    }
    hull.push(clone[second.unwrap() - 1]);

    // Graham Scan
    // assertion: At this point, we have at the minimum 3 vertices
    // necessary to create a convex hull
    for i in second.unwrap()..n {
        let mut top = hull.pop().unwrap();
        while vertex_angle(hull[hull.len() - 1], top, clone[i]) != VertexAngle::CounterClockwise {
            top = hull.pop().unwrap();
        }
        hull.push(top);
        hull.push(clone[i]);
    }
    Ok(hull)
}

/// Returns the type of angle three vertices form in 2d space
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

/// Returns the square of the distance
/// of two vertices
fn dist_sq(p1: Vec2d, p2: Vec2d) -> f64 {
    (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y)
}

/// iterates through vertices finding the index of the point
/// with the lowest y coordinate or the one with the left-most
/// x coordinate in case of a tie
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
    use collision::shapes::Convex;
    use common::Vec2d;

    #[test]
    fn test_convex_from_vertices() {
        // test too few vertices
        let mut v: Vec<Vec2d> = Vec::new();
        assert!(Convex::new(&mut v).is_err());
        v.push(Vec2d::new(0.0, 0.0));
        v.push(Vec2d::new(1.0, 1.0));
        assert!(Convex::new(&mut v).is_err());

        // test line
        v.push(Vec2d::new(2.0, 2.0));
        v.push(Vec2d::new(3.0, 3.0));
        assert!(Convex::new(&mut v).is_err());
        v.pop();
        v.pop();

        // test basic hull
        v.push(Vec2d::new(1.0, 0.0));
        let mut r = Convex::new(&mut v);
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

        // test add point at cw angle to last point
        v.push(Vec2d::new(2.0, 1.0));
        r = Convex::new(&mut v);
        r_ok = r.ok().unwrap();
        {
            let r_vertices = r_ok.vertices();
            assert_eq!(4, r_vertices.len());
            if r_vertices.iter().find(|&x| *x == Vec2d::new(0.0, 0.0)).is_none() {
                assert!(false);
            }
            if r_vertices.iter().find(|&x| *x == Vec2d::new(1.0, 1.0)).is_none() {
                assert!(false);
            }
            if r_vertices.iter().find(|&x| *x == Vec2d::new(1.0, 0.0)).is_none() {
                assert!(false);
            }
            if r_vertices.iter().find(|&x| *x == Vec2d::new(2.0, 1.0)).is_none() {
                assert!(false);
            }
        }
        v.pop();

        // add internal point
        v.push(Vec2d::new(0.5, 0.5));
        r = Convex::new(&mut v);
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

        // add collinear points
        v.push(Vec2d::new(1.0, 0.5));
        v.push(Vec2d::new(1.0, 0.25));
        r = Convex::new(&mut v);
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

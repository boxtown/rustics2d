use std::cmp::Ordering;
use std::result::Result;
use std::vec::Vec;
use collision::aabb::Aabb;
use vec2d::Vec2d;
use util;

/// Convex represents a convex polygon.
/// It contains the necessary information to be used
/// within collision detection algorithms
pub struct Convex {
    points: Vec<Vec2d>,
    aabb: Aabb,
}

impl Convex {
    /// Creates a convex polygon from the given points or returns
    /// an error if a convex polygon could not be created from the points.
    /// Only the points on the minimal convex hull will be saved within
    /// the returned polygon
    pub fn new(points: &[Vec2d]) -> Result<Convex, ()> {
        let mut clone = points.to_vec();
        match graham_scan(&mut clone) {
            Ok(hull) => {
                Ok(Convex {
                    points: hull,
                    aabb: Aabb::new(points).unwrap(),
                })
            }
            Err(_) => Err(()),
        }
    }

    /// Returns a reference to the slice of points
    /// making up the convex hull of this polygon
    pub fn points(&self) -> &[Vec2d] {
        &(*self.points)
    }
}

// The type of angle three consecutive
// points form in 2d space
#[derive(Debug, Eq, PartialEq)]
enum PointAngle {
    Collinear,
    Clockwise,
    CounterClockwise,
}

fn graham_scan(points: &mut [Vec2d]) -> Result<Vec<Vec2d>, ()> {
    let n = points.len();
    if n < 3 {
        return Err(());
    }

    // find bottom-most point and swap
    // with first point
    let i = lowest_y_index(points);
    points.swap(0, i);

    // sort points by polar coordinates with
    // respect to first point
    let sentinel = points[0];
    points.sort_by(|p1, p2| {
        let point_angle = point_angle(sentinel, *p1, *p2);
        match point_angle {
            PointAngle::Collinear => {
                let ds1 = dist_sq(sentinel, *p1);
                let ds2 = dist_sq(sentinel, *p2);
                if ds2 >= ds1 {
                    return Ordering::Less;
                }
                return Ordering::Greater;
            }
            PointAngle::Clockwise => Ordering::Greater,
            PointAngle::CounterClockwise => Ordering::Less,
        }
    });

    // If two or more points make the same angle
    // with the first point, remove all but the furthest
    // point from the first point
    let mut m: usize = 1;
    let mut i: usize = 1;
    while i < n {
        let p1 = points[i];
        let p2 = match i + 1 == n {
            true => points[0],
            false => points[i + 1],
        };
        while i < n - 1 && point_angle(sentinel, p1, p2) == PointAngle::Collinear {
            i += 1;
        }

        points[m] = points[i];
        m += 1;
        i += 1;
    }

    // push first two points into hull
    let mut hull = Vec::new();
    hull.push(points[0]);
    hull.push(points[1]);

    // process remaining points
    i = 2;
    while i < m {
        // keep removing top while the angle formed by
        // next-to-top, top, and points[i] makes a non-left turn
        let mut top = hull[hull.len() - 1];
        let mut next = hull[hull.len() - 2];
        let p = points[i];
        while hull.len() >= 3 && point_angle(next, top, p) != PointAngle::CounterClockwise {
            hull.pop();
            top = hull[hull.len() - 1];
            next = hull[hull.len() - 2];
        }
        hull.push(p);
        i += 1;
    }
    return Ok(hull);
}

// Returns the type of angle three points form in 2d space
fn point_angle(p1: Vec2d, p2: Vec2d, p3: Vec2d) -> PointAngle {
    let x = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
    if util::feq(x, 0f64) {
        return PointAngle::Collinear;
    }
    if x < 0f64 {
        return PointAngle::Clockwise;
    }
    return PointAngle::CounterClockwise;
}

// Returns the square of the distance
// of two points
fn dist_sq(p1: Vec2d, p2: Vec2d) -> f64 {
    (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y)
}

// iterates through points finding the index of the point
// with the lowest y coordinate or the one with the left-most
// x coordinate in case of a tie
fn lowest_y_index(points: &[Vec2d]) -> usize {
    let mut i: usize = 0;
    let mut j: usize = 0;
    let mut lowest = points[j];
    for p in points {
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
    fn test_convex_from_points() {
        // test too few points
        let mut v: Vec<Vec2d> = Vec::new();
        assert_eq!(Convex::new(&mut v).is_err(), true);
        v.push(Vec2d { x: 0.0, y: 0.0 });
        v.push(Vec2d { x: 1.0, y: 1.0 });
        assert_eq!(Convex::new(&mut v).is_err(), true);

        // test basic hull
        v.push(Vec2d { x: 1.0, y: 0.0 });
        let mut clone = v.clone();
        let mut r = Convex::new(&mut clone);
        let mut r_ok = r.ok().unwrap();
        {
            let r_points = r_ok.points();
            assert_eq!(3, r_points.len());
            if r_points.iter().find(|&x| *x == Vec2d { x: 0.0, y: 0.0 }).is_none() {
                assert!(false);
            }
            if r_points.iter().find(|&x| *x == Vec2d { x: 1.0, y: 1.0 }).is_none() {
                assert!(false);
            }
            if r_points.iter().find(|&x| *x == Vec2d { x: 1.0, y: 0.0 }).is_none() {
                assert!(false);
            }
        }

        // add internal point
        v.push(Vec2d { x: 0.5, y: 0.5 });
        clone = v.clone();
        r = Convex::new(&mut clone);
        r_ok = r.ok().unwrap();
        {
            let r_points = r_ok.points();
            assert_eq!(3, r_points.len());
            if r_points.iter().find(|&x| *x == Vec2d { x: 0.0, y: 0.0 }).is_none() {
                assert!(false);
            }
            if r_points.iter().find(|&x| *x == Vec2d { x: 1.0, y: 1.0 }).is_none() {
                assert!(false);
            }
            if r_points.iter().find(|&x| *x == Vec2d { x: 1.0, y: 0.0 }).is_none() {
                assert!(false);
            }
        }

        // add collinear point
        v.push(Vec2d { x: 1.0, y: 0.5 });
        clone = v.clone();
        r = Convex::new(&mut clone);
        r_ok = r.ok().unwrap();
        {
            let r_points = r_ok.points();
            assert_eq!(3, r_points.len());
            if r_points.iter().find(|&x| *x == Vec2d { x: 0.0, y: 0.0 }).is_none() {
                assert!(false);
            }
            if r_points.iter().find(|&x| *x == Vec2d { x: 1.0, y: 1.0 }).is_none() {
                assert!(false);
            }
            if r_points.iter().find(|&x| *x == Vec2d { x: 1.0, y: 0.0 }).is_none() {
                assert!(false);
            }
        }
    }
}

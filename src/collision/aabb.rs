use std::f64;
use std::result::Result;
use collision::Intersect;
use vec2d::Vec2d;

/// Aabb contains the information for an axis aligned bounding box. 
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    min: Vec2d,
    max: Vec2d,
}

impl Aabb {
    /// Returns a new instance of an Aabb formed from the passed in points
    /// or an error if an Aabb is unabled to be formed from the passed in points
    /// (namely when the number of points is less than 3)
    pub fn new(points: &[Vec2d]) -> Result<Aabb, ()> {
        if points.len() < 3 {
            return Err(());
        }

        let (xmin, xmax, ymin, ymax) = bounds_info(points);
        Ok(Aabb {
            min: Vec2d::new(xmin, ymin),
            max: Vec2d::new(xmax, ymax),
        })
    }

    /// Translates the center of the Aabb by the passed in Vec2d and updates
    /// the bounds of the bounding box
    pub fn translate(&mut self, movement: Vec2d) {
        self.min += movement;
        self.max += movement;
    }

    /// Completely reconstructs the bounds of the bounding box from the passed in
    /// Vector of Vec2ds
    pub fn reconstruct(&mut self, points: &[Vec2d]) -> Result<(), ()> {
        if points.len() < 3 {
            return Err(());
        }

        let (xmin, xmax, ymin, ymax) = bounds_info(points);
        self.min = Vec2d::new(xmin, ymin);
        self.max = Vec2d::new(xmax, ymax);
        Ok(())
    }
}

impl Intersect<Aabb> for Aabb {
    fn intersect(&self, rhs: &Self) -> bool {
        let d1 = self.min - rhs.max;
        let d2 = rhs.min - self.max;

        if d1.x > 0.0 || d1.y > 0.0 {
            return false;
        }
        if d2.x > 0.0 || d2.y > 0.0 {
            return false;
        }
        true
    }
}

// Returns bounding box information used for the creation of Aabbs from
// the passed in vector of Vec2ds
fn bounds_info(points: &[Vec2d]) -> (f64, f64, f64, f64) {
    let mut xmin = f64::MAX;
    let mut xmax = f64::MIN;
    let mut ymin = f64::MAX;
    let mut ymax = f64::MIN;

    for point in points {
        if point.x < xmin {
            xmin = point.x;
        }
        if point.x > xmax {
            xmax = point.x;
        }
        if point.y < ymin {
            ymin = point.y;
        }
        if point.y > ymax {
            ymax = point.y;
        }
    }

    (xmin, xmax, ymin, ymax)
}

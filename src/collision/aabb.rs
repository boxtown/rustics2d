use std::cmp::{Eq, PartialEq};
use std::f64;
use std::result::Result;
use vec2d::Vec2d;
use util;

/// Intersect is trait that should be implemented by objects
/// that must be able to determine whether or not they are intersecting
/// another object of the same type
pub trait Intersect {
    fn intersect(&self, rhs: &Self) -> bool;
}

/// Project2d is a trait that should be implemented by box shaped objects
/// that must be able to project themselves onto the axes
/// of a 2d plane
pub trait Project2d {
    fn projections2d(&self) -> ProjectedBox2d;
}

/// Projection is a 2d vector representing the projection of a vector
/// onto an axis on the 2d plane. The actual endpoints are encoded as
/// integers so that comparisons can be done using the CPU rather than
/// the more expensive FPU. 
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Projection {
    start: i64,
    end: i64,
}

impl Projection {
    /// Returns a new Projection with the given start and end endpoints.
    /// The endpoints will be encoded as integers but can be retrieved
    /// as floating points using dec_start and dec_end.
    pub fn new(start: f64, end: f64) -> Projection {
        Projection {
            start: util::encode_f64(start),
            end: util::encode_f64(end),
        }
    }

    /// Returns the start endpoint of the Projection decoded
    /// to an f64
    pub fn dec_start(&self) -> f64 {
        util::decode_f64(self.start)
    }

    /// Returns the end endpoint of the Projection decoded
    /// to an f64
    pub fn dec_end(&self) -> f64 {
        util::decode_f64(self.start)
    }

    /// Returns the start endpoint of the Projection encoded
    /// as an integer
    pub fn enc_start(&self) -> i64 {
        self.start
    }

    /// Returns the end endpoint of the Projection encoded
    /// as an integer
    pub fn enc_end(&self) -> i64 {
        self.end
    }
}

impl Intersect for Projection {
    fn intersect(&self, rhs: &Self) -> bool {
        (self.start >= rhs.start && self.start <= rhs.end) ||
        (rhs.start >= self.start && rhs.start <= self.end)
    }
}

/// ProjectedBox2d is an object representing the projection
/// of a box onto the axes of a 2d plane
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProjectedBox2d {
    pub x: Projection,
    pub y: Projection,
}

impl Intersect for ProjectedBox2d {
    fn intersect(&self, rhs: &Self) -> bool {
        self.x.intersect(&rhs.x) && self.y.intersect(&rhs.y)
    }
}

/// Aabb contains the information for an axis aligned bounding box. 
#[derive(Debug, Clone, Copy)]
pub struct Aabb {
    center: Vec2d,
    half_l: f64,
    half_w: f64,
    projected: ProjectedBox2d,
}

impl Aabb {
    /// Returns a new instance of an Aabb formed from the passed in points
    /// or an error if an Aabb is unabled to be formed from the passed in points
    /// (namely when the number of points is less than 3)
    pub fn new(points: &[Vec2d]) -> Result<Aabb, ()> {
        if points.len() < 3 {
            return Err(());
        }

        let (xmin, xmax, half_x, ymin, ymax, half_y) = bounds_info(points);
        Ok(Aabb {
            center: Vec2d {
                x: half_x,
                y: half_y,
            },
            half_l: half_x - xmin,
            half_w: half_y - ymin,
            projected: ProjectedBox2d {
                x: Projection::new(xmin, xmax),
                y: Projection::new(ymin, ymax),
            },
        })
    }

    /// Translates the center of the Aabb by the passed in Vec2d and updates
    /// the bounds of the bounding box
    pub fn translate(&mut self, movement: Vec2d) {
        self.center = self.center + movement;
        self.projected = ProjectedBox2d {
            x: Projection::new(self.center.x - self.half_w, self.center.x + self.half_w),
            y: Projection::new(self.center.y - self.half_l, self.center.y + self.half_l),
        };
    }

    /// Completely reconstructs the bounds of the bounding box from the passed in
    /// Vector of Vec2ds
    pub fn reconstruct(&mut self, points: &[Vec2d]) -> Result<(), ()> {
        if points.len() < 3 {
            return Err(());
        }

        let (xmin, xmax, half_x, ymin, ymax, half_y) = bounds_info(points);
        self.center = Vec2d {
            x: half_x,
            y: half_y,
        };
        self.half_l = half_x - xmin;
        self.half_w = half_y - ymin;
        self.projected = ProjectedBox2d {
            x: Projection::new(xmin, xmax),
            y: Projection::new(ymin, ymax),
        };
        Ok(())
    }
}

impl Intersect for Aabb {
    fn intersect(&self, rhs: &Self) -> bool {
        self.projected.intersect(&rhs.projected)
    }
}

impl Project2d for Aabb {
    fn projections2d(&self) -> ProjectedBox2d {
        self.projected
    }
}

// Custom PartialEq implementation that treats bounding boxes
// with NAN values as always unequal
impl PartialEq for Aabb {
    fn eq(&self, rhs: &Aabb) -> bool {
        if self.half_w == f64::NAN || self.half_l == f64::NAN {
            return false;
        }
        if rhs.half_w == f64::NAN || self.half_l == f64::NAN {
            return false;
        }
        self.center == rhs.center && self.half_l == rhs.half_l &&
        self.half_w == rhs.half_w && self.projected == rhs.projected
    }
}

// We implement Eq for Aabb because we specifically handle the case of NAN
// inside our custom PartialEq implementation
impl Eq for Aabb {}

// Returns bounding box information used for the creation of Aabbs from
// the passed in vector of Vec2ds
fn bounds_info(points: &[Vec2d]) -> (f64, f64, f64, f64, f64, f64) {
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

    let half_x = (xmax - xmin) / 2f64;
    let half_y = (ymax - ymin) / 2f64;
    (xmin, xmax, half_x, ymin, ymax, half_y)
}

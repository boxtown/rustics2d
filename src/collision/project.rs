use collision::Intersect;
use util;

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

impl Intersect<Projection> for Projection {
    fn intersect(&self, rhs: &Projection) -> bool {
        (self.start >= rhs.enc_start() && self.start <= rhs.enc_end()) ||
        (rhs.enc_start() >= self.start && rhs.enc_start() <= self.end)
    }
}

/// ProjectedBox2d is an object representing the projection
/// of a box onto the axes of a 2d plane
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProjectedBox2d {
    pub x: Projection,
    pub y: Projection,
}

impl Intersect<ProjectedBox2d> for ProjectedBox2d {
    fn intersect(&self, rhs: &Self) -> bool {
        self.x.intersect(&rhs.x) && self.y.intersect(&rhs.y)
    }
}
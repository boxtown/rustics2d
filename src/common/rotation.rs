/// Represents a 2d rotation in radians
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rotation {
    sin: f64,
    cos: f64,
}

impl Rotation {
    /// Create a new rotation with the angle
    /// in radians
    pub fn new(angle: f64) -> Rotation {
        Rotation {
            sin: angle.sin(),
            cos: angle.cos(),
        }
    }

    /// Creates a new identity rotation
    pub fn identity() -> Rotation {
        Rotation {
            sin: 0.0,
            cos: 1.0,
        }
    }

    /// Returns the angle of rotation in radians
    pub fn angle(&self) -> f64 {
        self.sin.atan2(self.cos)
    }

    /// Returns the sine of the angle of rotation
    pub fn sin(&self) -> f64 {
        self.sin
    }

    /// Returns the cosine of the angle of rotation
    pub fn cos(&self) -> f64 {
        self.cos
    }

    /// Updates the rotation with the new angle
    /// in radians
    pub fn update(&mut self, angle: f64) {
        self.sin = angle.sin();
        self.cos = angle.cos();
    }
}
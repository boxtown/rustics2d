use std::ops;

/// Represents a 2d rotation about the z-axis in radians
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

    /// Transposes the rotation as if it were a rotation
    /// matrix and returns the result.
    ///
    /// # Remarks
    ///
    /// Let `a` be the angle of rotation in radians for this
    /// Rotation. 
    ///
    /// Let `c = cos(a)` and `s = sin(a)`.
    ///
    /// This function theoretically performs:
    ///
    /// ```rust,ignore
    /// [c -s] -> [c  s]
    /// [s  c]    [-s c]
    /// ```
    /// 
    /// and logically is just a negation of `sin(a)`
    pub fn transpose(&self) -> Rotation {
        Rotation {
            sin: -self.sin,
            cos: self.cos,
        }
    }

    /// Updates the rotation with the new angle
    /// in radians
    pub fn update(&mut self, angle: f64) {
        self.sin = angle.sin();
        self.cos = angle.cos();
    }
}

impl ops::Mul for Rotation {
    type Output = Rotation;

    /// Performs a matrix multiplication
    /// of the rotation matrices represented
    /// by this Rotation and `rhs` and returns
    /// the resulting matrix as a Rotation
    ///
    /// # Remarks
    ///
    /// Let `this.a` be the angle of rotation for
    /// this rotation in radians and `rhs.a` the same
    /// for `rhs`. Let `new.a` be the new angle of rotation
    ///
    /// Let `c1 = cos(this.a), s1 = sin(this.a)`
    /// and `c2 = cos(rhs.a), s2 = sin(rhs.a)`.
    ///
    /// This function effectively performs:
    ///
    /// ```rust,ignore
    /// [c1 -s1] * [c2 -s2] -> [c1*c2-s1*s2 -c1*s2-s1*c2]
    /// [s1  c1]   [s2  c2]    [s1*c2+c1*s2 -s1*s2+c1*c2]
    /// ```
    /// where `cos(new.a) = c1*c2-s1*s2` 
    /// and `sin(new.a) = s1*c2+c1*s2`
    fn mul(self, rhs: Rotation) -> Rotation {
        Rotation {
            sin: self.sin * rhs.cos + self.cos * rhs.sin,
            cos: self.cos * rhs.cos - self.sin * rhs.sin,
        }
    }
}
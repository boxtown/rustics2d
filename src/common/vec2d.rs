use std::ops;
use common::Transform;

/// Vec2d represents a two dimensional vector
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec2d {
    pub x: f64,
    pub y: f64,
}

impl Vec2d {
    /// Creates a new `Vec2d` with the specified `x` and `y` values
    pub fn new(x: f64, y: f64) -> Vec2d {
        Vec2d { x: x, y: y }
    }

    /// Creates a new zero `Vec2d` vector
    pub fn zero() -> Vec2d {
        Vec2d { x: 0.0, y: 0.0 }
    }

    /// Returns the dot product of this `Vec2d` with another `Vec2d`
    pub fn dot(self, rhs: Vec2d) -> f64 {
        self.x * rhs.x + self.y * rhs.y
    }

    /// Returns the length of this `Vec2d`
    pub fn len(&self) -> f64 {
        (self.x * self.x + self.y + self.y).sqrt()
    }

    /// Returns the length squared of this `Vec2d`. Useful
    /// for avoiding expensive sqrt calculations
    pub fn len_sq(&self) -> f64 {
        self.x * self.x + self.y + self.y
    }

    /// normalize this vector (e.g. for Vector `v`, `v.x /= |v|`, `v.y /= |v|`)
    pub fn normalize(&self) -> Vec2d {
        let inv_len = 1.0 / self.len();
        Vec2d::new(self.x * inv_len, self.y * inv_len)
    }

    /// apply a `Transform` to a `Vec2d` and return the result
    pub fn apply(&self, transform: &Transform) -> Vec2d {
        let rotation = transform.rotation();
        let position = transform.position();

        // inlining matrix mult to avoid allocation of new vecs
        Vec2d::new((rotation.cos() * self.x - rotation.sin() * self.y) + position.x,
                   (rotation.sin() * self.x + rotation.cos() * self.y) + position.y)
    }
}

impl ops::Add for Vec2d {
    type Output = Vec2d;

    fn add(self, rhs: Vec2d) -> Vec2d {
        Vec2d {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl ops::AddAssign for Vec2d {
    fn add_assign(&mut self, rhs: Vec2d) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl ops::Sub for Vec2d {
    type Output = Vec2d;

    fn sub(self, rhs: Vec2d) -> Vec2d {
        self + -rhs
    }
}

impl ops::SubAssign for Vec2d {
    fn sub_assign(&mut self, rhs: Vec2d) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl ops::Mul for Vec2d {
    type Output = f64;

    fn mul(self, rhs: Vec2d) -> f64 {
        self.dot(rhs)
    }
}

impl ops::Mul<f64> for Vec2d {
    type Output = Vec2d;

    fn mul(self, rhs: f64) -> Vec2d {
        Vec2d {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl ops::MulAssign<f64> for Vec2d {
    fn mul_assign(&mut self, rhs: f64) {
        self.x *= rhs;
        self.y *= rhs;
    }
}

impl ops::Div<f64> for Vec2d {
    type Output = Vec2d;

    fn div(self, rhs: f64) -> Vec2d {
        Vec2d {
            x: self.x / rhs,
            y: self.y / rhs,
        }
    }
}

impl ops::DivAssign<f64> for Vec2d {
    fn div_assign(&mut self, rhs: f64) {
        self.x /= rhs;
        self.y /= rhs;
    }
}

impl ops::Neg for Vec2d {
    type Output = Vec2d;

    fn neg(self) -> Vec2d {
        Vec2d {
            x: -self.x,
            y: -self.y,
        }
    }
}

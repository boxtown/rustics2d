use std::cmp::PartialEq;
use std::ops;
use util;

/// Vec2d represents a two dimensional vector
#[derive(Debug, Clone, Copy)]
pub struct Vec2d {
	pub x: f64,
	pub y: f64
}

impl Vec2d {
	/// Returns the dot product of this Vec2d with another Vec2d
	pub fn dot(self, rhs: Vec2d) -> f64 {
		self.x * rhs.x + self.y * rhs.y
	}
}

impl ops::Add for Vec2d {
	type Output = Vec2d;

	fn add(self, rhs: Vec2d) -> Vec2d {
		Vec2d{
			x: self.x + rhs.x,
			y: self.y + rhs.y,
		}
	}
}

impl ops::Neg for Vec2d {
	type Output = Vec2d;

	fn neg(self) -> Vec2d {
		Vec2d{
			x: -self.x,
			y: -self.y,
		}
	}
}

impl PartialEq for Vec2d {
	fn eq(&self, other: &Self) -> bool {
		util::feq(self.x, other.x) && util::feq(self.y, other.y)
	}

	fn ne(&self, other: &Self) -> bool {
		!(self == other)
	}
}
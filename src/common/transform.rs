use common::{Rotation, Vec2d};

/// Represents a transform in 2d space. (e.g. a translate and a rotate)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform {
    position: Vec2d,
    rotation: Rotation,
}

impl Transform {
    /// Creates a new transform with a copy of the given position vector
    /// and rotation
    pub fn new(position: Vec2d, rotation: Rotation) -> Transform {
        Transform {
            position: position,
            rotation: rotation,
        }
    }

    /// Creates a new identity transform with the zero vector as the position
    /// and the identity rotation
    pub fn identity() -> Transform {
        Transform {
            position: Vec2d::zero(),
            rotation: Rotation::identity(),
        }
    }

    /// Returns the current position of the transform
    pub fn position(&self) -> &Vec2d {
        &self.position
    }

    /// Returns the current rotation of the transform
    pub fn rotation(&self) -> &Rotation {
        &self.rotation
    }

    /// Updates the transform with a new position and rotation
    pub fn update(&mut self, position: Vec2d, rotation: Rotation) {
        self.position = position;
        self.rotation = rotation;
    }

    /// Translate the transform by adding `translation` to the current
    /// position vector
    pub fn translate(&mut self, translation: Vec2d) {
        self.position += translation
    }

    /// Rotate the transform by adding `angle` radians to the current
    /// angle of rotation
    pub fn rotate(&mut self, angle: f64) {
        let cur_angle = self.rotation.angle();
        self.rotation.update(cur_angle + angle);
    }
}
use collision::Aabb;
use common::Transform;

/// Trait representing collidable objects
pub trait Collidable {
    /// Computes and returns an Axis-aligned Bounding Box
    /// for the collidable given a transform
    fn aabb(&self, transform: &Transform) -> Aabb;
}
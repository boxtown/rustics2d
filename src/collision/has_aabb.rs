use collision::Aabb;
use common::Transform;

/// Trait representing objects with a axis-aligned bounding box
pub trait HasAabb {
    /// Computes and returns an Axis-aligned Bounding Box
    /// for given a transform
    fn aabb(&self, transform: &Transform) -> Aabb;
}
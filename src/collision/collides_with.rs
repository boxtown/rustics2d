use collision::HasAabb;
use common::Transform;

/// Trait represents objects that may collide
/// with other objects
pub trait CollidesWith<T> : HasAabb {
    /// Returns if this object given transform `this_t` collides with `other`
    /// given transform `other_t`
    fn collides_with(&self, other: &T, this_t: &Transform, other_t: &Transform) -> bool;
}
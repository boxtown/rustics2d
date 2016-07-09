use collision::Aabb;

/// Trait representing collidable objects
pub trait Collidable {
    /// Computes and returns an Axis-aligned Bounding Box
    /// for the collidable
    fn aabb(&self) -> &Aabb;
}
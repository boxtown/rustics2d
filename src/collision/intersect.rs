/// Intersect is trait that should be implemented by objects
/// that must be able to determine whether or not they are intersecting
/// another object
pub trait Intersect<T> {
    fn intersect(&self, rhs: &T) -> bool;
}
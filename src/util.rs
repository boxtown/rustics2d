pub const TOLERANCE: f64 = 1e-10;

/// Returns true if the floats are equal or within
/// 1e-10 of each other
pub fn feq(f1: f64, f2: f64) -> bool {
    (f1 - f2).abs() < TOLERANCE
}

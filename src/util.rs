use std::i64;

const THRESHOLD: f64 = 1e-10;

/// Returns true if the floats are equal or within
/// 1e-10 of each other
pub fn feq(f1: f64, f2: f64) -> bool {
	let diff = (f1 - f2).abs();
	diff >= 0f64 && diff < THRESHOLD
}

/// Encodes an f64 as an i64 such that there is a bijective
/// mapping between the two values and comparisons work
/// as they would between the original float values
pub fn encode_f64(input: f64) -> i64 {
	let mask: i64 = -(input as i64 >> 31) | i64::MIN;
	(input as i64) ^ mask
}

/// Decodes an i64 encoded using encode_f64 into an f64
pub fn decode_f64(input: i64) -> f64 {
	let mask: i64 = ((input >> 31) - 1) | i64::MIN;
	(input ^ mask) as f64
}
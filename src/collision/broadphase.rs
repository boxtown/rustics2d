use std::clone::Clone;
use std::cmp::{Ordering, Eq, PartialEq};
use std::collections::HashSet;
use std::f64;
use std::hash::Hash;
use std::option::Option;
use std::vec::Vec;
use collision::aabb;
use util;

/// Container is a trait that represents the qualities
/// necessary for objects to have in order for them
/// to be processed by broadphase collision algorithms
pub trait Container : 
	aabb::Project2d + 
	aabb::Intersect + 
	Clone + 
	Eq + 
	Hash { }

/// IdPair is a pair of references to Containers. IdPair
/// is used by broadphase collision algorithms to represent
/// possibly colliding containers
#[derive(Debug, Eq, Hash)]
pub struct IdPair<'a, T>(&'a T, &'a T) where T : 'a + Container;

impl<'a, T> PartialEq for IdPair<'a, T> where T : 'a + Container {
	fn eq(&self, rhs: &IdPair<'a, T>) -> bool {
		if self.0 == rhs.0 && self.1 == rhs.1 {
			return true;
		}
		if self.1 == rhs.0 && self.0 == rhs.1 {
			return true;
		}
		false
	}
}

/// SweepAndPrune is a struct that holds information necessary
/// to perform sweep and prune broadphase operations and return
/// possibly colliding pairs of Containers.
#[derive(Debug)]
pub struct SweepAndPrune<'a, T> where T : 'a + Container {
	xs 			: Vec<SweepPoint<'a, T>>,
	ys 			: Vec<SweepPoint<'a, T>>,
	pub pairs	: HashSet<IdPair<'a, T>>
}

/// Returns a new empty instance of a SweepAndPrune struct
impl<'a, T> SweepAndPrune<'a, T> where T : Container {
	pub fn new() -> SweepAndPrune<'a, T> {
		SweepAndPrune{
			xs 			: vec![SweepPoint::min_sentinel(), SweepPoint::max_sentinel()],
			ys 			: vec![SweepPoint::min_sentinel(), SweepPoint::max_sentinel()],
			pairs		: HashSet::new(),
		}
	}

	/// Performs a batch insert of the passed in projecters
	/// into the SweepAndPrune struct. This function updates the possible
	/// colliding pairs given the new projecters
	pub fn batch_insert(&mut self, projecters: &[&'a T]) {
		if projecters.len() == 0 {
			return;
		}

		// Get vector of projections
		let projections: Vec<aabb::ProjectedBox2d> = projecters.iter()
			.map(|p| p.projections2d())
			.collect();

		// Create two sorted vectors of sweep points, one for projection
		// x-values and one for projections y-values
		let n = projections.len();
		let mut i = 0;
		let mut j = i;
		let mut xs: Vec<SweepPoint<'a, T>> = Vec::with_capacity(2 * n);
		let mut ys: Vec<SweepPoint<'a, T>> = Vec::with_capacity(2 * n);
		while i < n {
			let p = &projections[i];
			xs[j] 	= SweepPoint::<'a, T>{
				projecter 	: Some(projecters[i]), 
				is_start	: true,
				val 		: p.x.enc_start(),
			};
			xs[j+1] = SweepPoint::<'a, T>{
				projecter 	: Some(projecters[i]), 
				is_start	: false, 
				val 		: p.x.enc_end(),
			};
			ys[j]	= SweepPoint::<'a, T>{
				projecter 	: Some(projecters[i]), 
				is_start 	: true, 
				val 		: p.y.enc_start(),
			};
			ys[j+1] = SweepPoint::<'a, T>{
				projecter 	: Some(projecters[i]), 
				is_start 	: false, 
				val 		: p.y.enc_end(),
			};
			j += 2; 
			i += 1;
		}
		xs.sort_by(SweepPoint::compare);
		ys.sort_by(SweepPoint::compare);

		// Batch insert xs. Starting at self.xs.len() - 1 in order
		// to be sentinel aware. 
		i = self.xs.len() - 1;
		for x in xs {
			self.xs.insert(i, x.clone());

			i = i - 1;
			while self.xs[i].val > x.val {
				self.xs[i + 1] = self.xs[i].clone();
				i = i - 1;
			}
			self.xs[i + 1] = x;
		}

		// Batch insert ys. Starting at self.ys.len() - 1 in order 
		// to be sentinel aware.
		i = self.ys.len() - 1;
		for y in ys {
			self.ys.insert(i, y.clone());
			
			i = i - 1;
			while self.ys[i].val > y.val {
				let swap = self.ys[i].clone();
				let bb1 = y.projecter.unwrap();
				let bb2 = swap.projecter.unwrap();

				if y.is_start && !swap.is_start {
					if bb1.intersect(&bb2) {
						self.pairs.insert(IdPair::<'a, T>(bb1, bb2));
					}
				}
				if !y.is_start && swap.is_start {
					self.pairs.remove(&IdPair::<'a, T>(bb1, bb2));
				}

				self.ys[j + 1] = swap;
				i = i - 1;
			}
			self.ys[i + 1] = y;
		}
	}
}

// SweepPoint is a struct containing information necessary
// to perform the sweep and prune broadphase collision algorithm.
// Endpoint values are encoded as integers so that comparisons are made
// using the CPU rather than the FPU. The actual endpoint value may
// be retrieved by the decoded function
#[derive(Debug, Clone)]
struct SweepPoint<'a, T> where T : 'a + Container {
	projecter	: Option<&'a T>,
	is_start	: bool,
	val 		: i64,
}

impl<'a, T> SweepPoint<'a, T> where T : 'a + Container {
	// Returns a SweepPoint that represents the minimum sentinel value
	// for SweepPoints
	fn min_sentinel() -> SweepPoint<'a, T> {
		SweepPoint{
			projecter	: None,
			is_start	: false,
			val 		: util::encode_f64(f64::MIN),
		}
	}

	// Returns a SweepPoint that represents the maximum sentinel value 
	// for SweepPoints
	fn max_sentinel() -> SweepPoint<'a, T> {
		SweepPoint{
			projecter 	: None,
			is_start	: false,
			val 		: util::encode_f64(f64::MAX),
		}
	}

	// Returns a comparison ordering between two SweepPoints. Used by SweepAndPrune
	// to sort SweepPoints
	fn compare(a: &SweepPoint<'a, T>, b: &SweepPoint<'a, T>) -> Ordering {
		a.val.cmp(&b.val)
	}

	// Returns the decoded value of the SweepPoint.
	fn decoded(&self) -> f64 {
		util::decode_f64(self.val)
	}
}

#[cfg(test)]
mod test {
	use collision::aabb::Aabb;
	use collision::broadphase::SweepAndPrune;

	#[test]
	fn test_sap_batch_insert() {
		let mut sap: SweepAndPrune<Aabb> = SweepAndPrune::new();
	}
}
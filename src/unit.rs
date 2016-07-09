pub type M = f64;
pub type S = f64;
pub type Kg = f64;


pub trait Unit {}

impl Unit for Meter {}
impl Unit for Second {}
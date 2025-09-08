pub mod vector;
pub mod quaternion;
pub mod frame;

pub use vector::Vec3;
pub use quaternion::Quat;
pub use frame::Frame;

pub const GRAVITY: f64 = 9.81;
pub const AIR_DENSITY_SEA_LEVEL: f64 = 1.225;
pub const GAS_CONSTANT_AIR: f64 = 287.05;
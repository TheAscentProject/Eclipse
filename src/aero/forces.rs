use crate::math::Vec3;

#[derive(Debug, Clone)]
pub struct AeroForces {
    pub lift: Vec3,
    pub drag: Vec3,
    pub moment: Vec3,
}

impl AeroForces {
    pub fn zero() -> Self {
        Self {
            lift: Vec3::zero(),
            drag: Vec3::zero(),
            moment: Vec3::zero(),
        }
    }

    pub fn total_force(&self) -> Vec3 {
        self.lift + self.drag
    }

    pub fn add(&mut self, other: &AeroForces) {
        self.lift = self.lift + other.lift;
        self.drag = self.drag + other.drag;
        self.moment = self.moment + other.moment;
    }
}

pub fn compute_dynamic_pressure(density: f64, velocity: f64) -> f64 {
    0.5 * density * velocity * velocity
}
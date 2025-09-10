use crate::math::Vec3;
use crate::aero::{AeroForces, AirData, compute_dynamic_pressure};
use crate::physics::frames::FrameTransforms;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Wing {
    pub span: f64,
    pub chord: f64,
    pub area: f64,
    pub aspect_ratio: f64,
    pub segments: Vec<WingSegment>,
    pub flap_deflection: f64,
    pub blown_coefficient: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WingSegment {
    pub span_position: f64,
    pub chord: f64,
    pub twist: f64,
    pub airfoil: Airfoil,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Airfoil {
    pub cl0: f64,
    pub cl_alpha: f64,
    pub cl_max: f64,
    pub cd0: f64,
    pub cd_alpha2: f64,
    pub cm0: f64,
}

impl Wing {
    pub fn new(span: f64, chord: f64) -> Self {
        let area = span * chord;
        let aspect_ratio = span * span / area;
        
        let airfoil = Airfoil {
            cl0: 0.0,
            cl_alpha: 2.0 * std::f64::consts::PI,
            cl_max: 1.5,
            cd0: 0.01,
            cd_alpha2: 0.01,
            cm0: -0.025,
        };

        let segments = vec![
            WingSegment {
                span_position: 0.0,
                chord,
                twist: 0.0,
                airfoil: airfoil.clone(),
            },
            WingSegment {
                span_position: span,
                chord,
                twist: 0.0,
                airfoil,
            },
        ];

        Self {
            span,
            chord,
            area,
            aspect_ratio,
            segments,
            flap_deflection: 0.0,
            blown_coefficient: 0.0,
        }
    }

    pub fn compute_forces(
        &self,
        velocity: &Vec3,
        alpha: f64,
        density: f64,
        prop_wash_velocity: Option<f64>,
    ) -> AeroForces {
        let v_mag = velocity.magnitude();
        if v_mag < 1e-6 {
            return AeroForces::zero();
        }

        let effective_velocity = if let Some(wash) = prop_wash_velocity {
            ((v_mag * v_mag + wash * wash).sqrt()).max(v_mag)
        } else {
            v_mag
        };

        let q = compute_dynamic_pressure(density, effective_velocity);
        
        let mut cl = self.compute_cl(alpha);
        let cd = self.compute_cd(alpha, cl);
        
        if self.flap_deflection.abs() > 0.01 {
            cl += self.flap_deflection.to_radians() * 0.9;
        }
        
        if self.blown_coefficient > 0.0 && prop_wash_velocity.is_some() {
            let velocity_ratio = effective_velocity / v_mag;
            cl *= 1.0 + self.blown_coefficient * (velocity_ratio - 1.0);
        }
        
        cl = cl.min(self.get_cl_max());
        
        let lift_mag = q * self.area * cl;
        let drag_mag = q * self.area * cd;
        
        let lift_dir = Vec3::new(0.0, 0.0, -1.0);
        let drag_dir = velocity.normalize() * -1.0;
        
        AeroForces {
            lift: lift_dir * lift_mag,
            drag: drag_dir * drag_mag,
            moment: Vec3::new(0.0, q * self.area * self.chord * self.segments[0].airfoil.cm0, 0.0),
        }
    }

    fn compute_cl(&self, alpha: f64) -> f64 {
        let airfoil = &self.segments[0].airfoil;
        let cl_infinite = airfoil.cl0 + airfoil.cl_alpha * alpha;
        
        let e = 0.9;
        let cl_3d = cl_infinite * self.aspect_ratio / (self.aspect_ratio + 2.0 / e);
        
        self.apply_stall_model(cl_3d, alpha, airfoil.cl_max)
    }
    
    fn apply_stall_model(&self, cl_linear: f64, alpha: f64, cl_max: f64) -> f64 {
        let alpha_stall = cl_max / (2.0 * std::f64::consts::PI);
        let transition_width = 0.1;
        
        if alpha.abs() < alpha_stall - transition_width {
            cl_linear
        } else if alpha.abs() > alpha_stall + transition_width {
            cl_max * alpha.signum()
        } else {
            let blend = 0.5 * (1.0 + ((alpha.abs() - alpha_stall) / transition_width * std::f64::consts::PI).sin());
            cl_linear * (1.0 - blend) + cl_max * alpha.signum() * blend
        }
    }

    fn compute_cd(&self, alpha: f64, cl: f64) -> f64 {
        let airfoil = &self.segments[0].airfoil;
        let cd_profile = airfoil.cd0 + airfoil.cd_alpha2 * alpha * alpha;
        
        let e = 0.9;
        let cd_induced = cl * cl / (std::f64::consts::PI * e * self.aspect_ratio);
        
        cd_profile + cd_induced
    }

    fn get_cl_max(&self) -> f64 {
        let base_cl_max = self.segments[0].airfoil.cl_max;
        if self.flap_deflection > 0.0 {
            base_cl_max + self.flap_deflection.to_radians() * 1.5
        } else {
            base_cl_max
        }
    }
}
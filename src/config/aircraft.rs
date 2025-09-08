use crate::aero::{Wing, Propeller};
use crate::physics::RigidBody;
use crate::math::Vec3;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AircraftConfig {
    pub name: String,
    pub mass: f64,
    pub inertia: [[f64; 3]; 3],
    pub wing: Wing,
    pub vtol_props: Vec<PropellerMount>,
    pub cruise_props: Vec<PropellerMount>,
    pub cg_position: Vec3,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PropellerMount {
    pub propeller: Propeller,
    pub position: Vec3,
    pub direction: Vec3,
    pub cant_angle: f64,
    pub tilt_range: Option<(f64, f64)>,
}

impl AircraftConfig {
    pub fn tiltrotor_quad() -> Self {
        let wing = Wing::new(4.0, 0.5);
        
        let prop = Propeller::new(0.6, 4, 0.3);
        
        let vtol_props = vec![
            PropellerMount {
                propeller: prop.clone(),
                position: Vec3::new(1.5, -2.0, 0.0),
                direction: Vec3::new(0.0, 0.0, -1.0),
                cant_angle: 0.0,
                tilt_range: Some((-90.0, 0.0)),
            },
            PropellerMount {
                propeller: prop.clone(),
                position: Vec3::new(-1.5, -2.0, 0.0),
                direction: Vec3::new(0.0, 0.0, -1.0),
                cant_angle: 0.0,
                tilt_range: Some((-90.0, 0.0)),
            },
            PropellerMount {
                propeller: prop.clone(),
                position: Vec3::new(1.5, 2.0, 0.0),
                direction: Vec3::new(0.0, 0.0, -1.0),
                cant_angle: 0.0,
                tilt_range: Some((-90.0, 0.0)),
            },
            PropellerMount {
                propeller: prop.clone(),
                position: Vec3::new(-1.5, 2.0, 0.0),
                direction: Vec3::new(0.0, 0.0, -1.0),
                cant_angle: 0.0,
                tilt_range: Some((-90.0, 0.0)),
            },
        ];

        Self {
            name: "Tiltrotor Quad".to_string(),
            mass: 100.0,
            inertia: [
                [20.0, 0.0, 0.0],
                [0.0, 15.0, 0.0],
                [0.0, 0.0, 35.0],
            ],
            wing,
            vtol_props,
            cruise_props: vec![],
            cg_position: Vec3::zero(),
        }
    }

    pub fn lift_plus_cruise() -> Self {
        let mut wing = Wing::new(4.5, 0.6);
        wing.blown_coefficient = 0.8;
        
        let vtol_prop = Propeller::new(0.4, 4, 0.25);
        let cruise_prop = Propeller::new(0.8, 2, 0.4);

        let vtol_props = vec![
            PropellerMount {
                propeller: vtol_prop.clone(),
                position: Vec3::new(0.5, -1.0, 0.0),
                direction: Vec3::new(0.0, 0.0, -1.0),
                cant_angle: 0.0,
                tilt_range: None,
            },
            PropellerMount {
                propeller: vtol_prop.clone(),
                position: Vec3::new(-0.5, -1.0, 0.0),
                direction: Vec3::new(0.0, 0.0, -1.0),
                cant_angle: 0.0,
                tilt_range: None,
            },
            PropellerMount {
                propeller: vtol_prop.clone(),
                position: Vec3::new(0.5, 1.0, 0.0),
                direction: Vec3::new(0.0, 0.0, -1.0),
                cant_angle: 0.0,
                tilt_range: None,
            },
            PropellerMount {
                propeller: vtol_prop.clone(),
                position: Vec3::new(-0.5, 1.0, 0.0),
                direction: Vec3::new(0.0, 0.0, -1.0),
                cant_angle: 0.0,
                tilt_range: None,
            },
        ];

        let cruise_props = vec![
            PropellerMount {
                propeller: cruise_prop,
                position: Vec3::new(0.0, -2.5, 0.0),
                direction: Vec3::new(0.0, 1.0, 0.0),
                cant_angle: 0.0,
                tilt_range: None,
            },
        ];

        Self {
            name: "Lift Plus Cruise".to_string(),
            mass: 120.0,
            inertia: [
                [25.0, 0.0, 0.0],
                [0.0, 18.0, 0.0],
                [0.0, 0.0, 40.0],
            ],
            wing,
            vtol_props,
            cruise_props,
            cg_position: Vec3::zero(),
        }
    }

    pub fn to_rigid_body(&self) -> RigidBody {
        let inertia_matrix = nalgebra::Matrix3::new(
            self.inertia[0][0], self.inertia[0][1], self.inertia[0][2],
            self.inertia[1][0], self.inertia[1][1], self.inertia[1][2],
            self.inertia[2][0], self.inertia[2][1], self.inertia[2][2],
        );
        
        let mut body = RigidBody::new(self.mass, inertia_matrix);
        body.cg_offset = self.cg_position;
        body
    }
}
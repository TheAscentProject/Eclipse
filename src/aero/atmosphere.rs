use crate::math::AIR_DENSITY_SEA_LEVEL;

pub struct Atmosphere {
    pub density: f64,
}

impl Atmosphere {
    pub fn at_altitude(altitude: f64) -> Self {
        let pressure_ratio = (1.0 - altitude / 44330.0).powf(5.255);
        let density = AIR_DENSITY_SEA_LEVEL * pressure_ratio.powf(0.8);

        Self {
            density,
        }
    }

    pub fn sea_level() -> Self {
        Self::at_altitude(0.0)
    }

}
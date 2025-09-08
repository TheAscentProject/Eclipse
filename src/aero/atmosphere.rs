use crate::math::AIR_DENSITY_SEA_LEVEL;

pub struct Atmosphere {
    pub altitude: f64,
    pub temperature: f64,
    pub pressure: f64,
    pub density: f64,
}

impl Atmosphere {
    pub fn at_altitude(altitude: f64) -> Self {
        let temperature = 288.15 - 0.0065 * altitude;
        let pressure_ratio = (1.0 - altitude / 44330.0).powf(5.255);
        let pressure = 101325.0 * pressure_ratio;
        let density = AIR_DENSITY_SEA_LEVEL * pressure_ratio.powf(0.8);

        Self {
            altitude,
            temperature,
            pressure,
            density,
        }
    }

    pub fn sea_level() -> Self {
        Self::at_altitude(0.0)
    }

    pub fn speed_of_sound(&self) -> f64 {
        (1.4 * 287.05 * self.temperature).sqrt()
    }
}
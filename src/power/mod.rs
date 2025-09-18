use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BatteryConfig {
    pub capacity_mah: f64,
    pub cells: u32,
    pub parallel_strings: u32,
    pub c_rating: f64,
    pub internal_resistance_mohm_per_cell: f64,
    pub cutoff_per_cell_v: f64,
    pub nominal_cell_v: f64,
    pub full_cell_v: f64,
    /// Electrical efficiency from shaft power to battery draw (ESC+motor)
    pub motor_efficiency: f64,
    /// If true, scale thrust based on available electrical power
    pub limit_thrust: bool,
}

impl Default for BatteryConfig {
    fn default() -> Self {
        Self {
            capacity_mah: 200000.0,
            cells: 120,
            parallel_strings: 10,
            c_rating: 10.0,
            internal_resistance_mohm_per_cell: 2.0,
            cutoff_per_cell_v: 3.3,
            nominal_cell_v: 3.7,
            full_cell_v: 4.2,
            motor_efficiency: 0.92,
            limit_thrust: false,
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PowerTelemetry {
    pub bus_voltage_v: f64,
    pub bus_current_a: f64,
    pub electrical_power_w: f64,
    pub mechanical_power_w: f64,
    pub energy_used_wh: f64,
    pub soc: f64,
    pub time_remaining_s: f64,
    pub depleted: bool,
    pub power_limited: bool,
}

#[derive(Debug, Clone)]
pub struct Battery {
    pub cfg: BatteryConfig,
    capacity_wh: f64,
    energy_remaining_wh: f64,
    pub telemetry: PowerTelemetry,
    v_lp: f64,
    v_under_cutoff_time: f64,
}

impl Battery {
    pub fn new(cfg: BatteryConfig) -> Self {
    // capacity_mah is per string; total Ah scales with parallel strings,
    // pack voltage scales with series cells; Wh = Ah_total * V_nominal_pack.
    let capacity_wh = (cfg.capacity_mah / 1000.0) * cfg.parallel_strings as f64 * (cfg.cells as f64 * cfg.nominal_cell_v);
        Self {
            cfg,
            capacity_wh,
            energy_remaining_wh: capacity_wh,
            telemetry: PowerTelemetry::default(),
            v_lp: 0.0,
            v_under_cutoff_time: 0.0,
        }
    }

    pub fn soc(&self) -> f64 { (self.energy_remaining_wh / self.capacity_wh).clamp(0.0, 1.0) }

    fn open_circuit_cell_voltage(&self, soc: f64) -> f64 {
        // Simple LiPo curve approximation: flat mid, steep ends
        // Piecewise linear-ish via smoothstep
        let s = soc.clamp(0.0, 1.0);
        let v_full = self.cfg.full_cell_v;
        let v_nom = self.cfg.nominal_cell_v; // ~3.7V plateau
        let v_cut = self.cfg.cutoff_per_cell_v;

        // Blend from full->nominal for top 30%
        let top = (s - 0.7).max(0.0) / 0.3;
        let top = top.min(1.0);
        let v_top = v_nom + (v_full - v_nom) * top;

        // Blend from cutoff->nominal for bottom 60%
        let bot = (s / 0.6).min(1.0);
        let v_bot = v_cut + (v_nom - v_cut) * bot;

        // Use the higher envelope to reflect OCV progression
        v_top.max(v_bot)
    }

    fn total_internal_resistance(&self) -> f64 {
        // Series cells add, parallel strings divide
        let series_r_mohm = self.cfg.internal_resistance_mohm_per_cell * self.cfg.cells as f64;
        let strings = self.cfg.parallel_strings.max(1) as f64;
        (series_r_mohm / strings) / 1000.0
    }

    pub fn cutoff_voltage(&self) -> f64 { self.cfg.cutoff_per_cell_v * self.cfg.cells as f64 }

    pub fn max_current_a(&self) -> f64 {
        let capacity_ah = (self.cfg.capacity_mah / 1000.0) * self.cfg.parallel_strings as f64;
        self.cfg.c_rating * capacity_ah
    }

    pub fn limit_electrical_power(&self, p_request_w: f64) -> (f64, f64, f64) {
        // Predict max deliverable power given sag, Imax and cutoff voltage.
    let soc = self.soc();
    let voc = self.open_circuit_cell_voltage(soc) * self.cfg.cells as f64;
        let r = self.total_internal_resistance();
        let i_max = self.max_current_a();

        if p_request_w <= 0.0 {
            return (0.0, voc, 0.0);
        }

        // Initial guess assumes no sag
        let mut v = voc;
        let mut p = p_request_w;
        for _ in 0..6 {
            let mut i = (p / v).max(0.0);
            if i > i_max { i = i_max; }
            v = (voc - i * r).max(0.0);
            if v < self.cutoff_voltage() {
                // At cutoff, recompute i and p
                v = self.cutoff_voltage();
                let i_cut = (voc - v) / r;
                let i_lim = i_cut.min(i_max).max(0.0);
                return (v * i_lim, v, i_lim);
            }
            p = (p_request_w).min(v * i);
        }

        let i = (p / v).min(i_max).max(0.0);
        let p_deliv = v * i;
        (p_deliv, v, i)
    }

    pub fn draw_electrical_power(&mut self, p_elec_w: f64, dt: f64) {
        let p = p_elec_w.max(0.0);
        let e_wh = p * dt / 3600.0;
        let used = e_wh.min(self.energy_remaining_wh);
        self.energy_remaining_wh -= used;
        let soc = self.soc();
    let voc = self.open_circuit_cell_voltage(soc) * self.cfg.cells as f64;
        let r = self.total_internal_resistance();
        let v_pre = if p <= 1e-9 { voc } else { (voc + (voc.powi(2) - 4.0 * r * p).max(0.0).sqrt()) / 2.0 };
        let i = if v_pre > 0.0 { p / v_pre } else { 0.0 };
        let v = (voc - i * r).max(0.0);

    self.telemetry.electrical_power_w = p;
        self.telemetry.mechanical_power_w = p * self.cfg.motor_efficiency;
        // Simple low-pass on loaded voltage to avoid instant depletion at spikes
        let alpha = (dt / (0.2 + dt)).clamp(0.0, 1.0);
        self.v_lp = if self.v_lp <= 0.0 { v } else { self.v_lp + alpha * (v - self.v_lp) };
        self.telemetry.bus_voltage_v = v;
        self.telemetry.bus_current_a = if v > 0.0 { p / v } else { 0.0 };
        self.telemetry.energy_used_wh = self.capacity_wh - self.energy_remaining_wh;
        self.telemetry.soc = soc;
        // Require 2.0s of sustained under-voltage to mark depleted
        if self.v_lp <= self.cutoff_voltage() - 0.1 * self.cfg.cells as f64 {
            self.v_under_cutoff_time += dt;
        } else {
            self.v_under_cutoff_time = 0.0;
        }
        self.telemetry.depleted = self.energy_remaining_wh <= 1e-9 || self.v_under_cutoff_time >= 2.0;
        self.telemetry.time_remaining_s = if p > 1e-6 { (self.energy_remaining_wh / (p / 3600.0)).max(0.0) } else { f64::INFINITY };
    }
}

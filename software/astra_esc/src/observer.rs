//! Sensorless rotor-flux observer (Ortega-style nonlinear observer) + angle PLL.
//!
//! The observer integrates the stator voltage equation to estimate rotor flux
//! in the stationary alpha/beta frame:
//!
//!   x_dot = v - R*i + (gamma/2) * (F^2 - |psi|^2) * psi
//!   psi   = x - L*i          (rotor flux)
//!
//! The nonlinear correction term pulls the estimated flux magnitude toward the
//! nominal flux linkage F, which cancels integrator drift without the phase
//! error a plain high-pass filter would introduce.
//!
//! A PLL then tracks the angle of psi, producing a smooth theta and omega
//! (electrical rad/s) estimate. The PLL phase detector is the normalized cross
//! product, i.e. sin(theta_flux - theta_est), so no atan2 is needed per cycle.

use crate::foc::{sin_cos, wrap_angle};
use libm::sqrtf;

pub struct FluxObserver {
    // Observer states (stator flux estimate, Wb)
    x1: f32,
    x2: f32,
    // Motor parameters
    r: f32,     // phase resistance (ohm)
    l: f32,     // phase inductance (H)
    flux: f32,  // rotor flux linkage (Wb)
    gamma: f32, // observer convergence gain
    // PLL
    pll_kp: f32,
    pll_ki: f32,
    /// Estimated electrical rotor angle (rad, wrapped to [-pi, pi))
    pub theta: f32,
    /// Estimated electrical angular velocity (rad/s), sign = direction
    pub omega: f32,
}

impl FluxObserver {
    pub const fn new(r: f32, l: f32, flux: f32, gamma: f32, pll_kp: f32, pll_ki: f32) -> Self {
        Self {
            x1: 0.0,
            x2: 0.0,
            r,
            l,
            flux,
            gamma,
            pll_kp,
            pll_ki,
            theta: 0.0,
            omega: 0.0,
        }
    }

    /// Reset the observer, seeding the flux estimate at a known angle
    /// (used when leaving the aligned/open-loop state so the observer doesn't
    /// start from zero).
    pub fn seed(&mut self, theta: f32, omega: f32) {
        let (sin_t, cos_t) = sin_cos(theta);
        self.x1 = self.flux * cos_t;
        self.x2 = self.flux * sin_t;
        self.theta = theta;
        self.omega = omega;
    }

    /// One observer + PLL step. `v_*` are the applied stator voltages and
    /// `i_*` the measured currents, both in the stationary frame. `dt` in s.
    pub fn update(&mut self, v_alpha: f32, v_beta: f32, i_alpha: f32, i_beta: f32, dt: f32) {
        let l_ia = self.l * i_alpha;
        let l_ib = self.l * i_beta;

        // Rotor flux from current state
        let psi_a = self.x1 - l_ia;
        let psi_b = self.x2 - l_ib;
        let mag_sq = psi_a * psi_a + psi_b * psi_b;
        let err = self.flux * self.flux - mag_sq;

        // Observer state update
        self.x1 += (v_alpha - self.r * i_alpha + 0.5 * self.gamma * err * psi_a) * dt;
        self.x2 += (v_beta - self.r * i_beta + 0.5 * self.gamma * err * psi_b) * dt;

        // Recompute rotor flux with updated states for the PLL
        let psi_a = self.x1 - l_ia;
        let psi_b = self.x2 - l_ib;
        let mag = sqrtf(psi_a * psi_a + psi_b * psi_b);
        let mag = if mag > 1e-9 { mag } else { 1e-9 };

        // PLL phase detector: sin(angle(psi) - theta_est)
        let (sin_t, cos_t) = sin_cos(self.theta);
        let phase_err = (psi_b * cos_t - psi_a * sin_t) / mag;

        self.omega += self.pll_ki * phase_err * dt;
        self.theta = wrap_angle(self.theta + (self.omega + self.pll_kp * phase_err) * dt);
    }
}

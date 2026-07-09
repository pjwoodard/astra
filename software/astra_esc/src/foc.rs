//! Core FOC math: PI controllers, Clarke/Park transforms, SVPWM.

/// PI controller with output clamping and integrator anti-windup (clamped integrator).
pub struct Pi {
    pub kp: f32,
    pub ki: f32,
    integ: f32,
    pub limit: f32, // symmetric output limit (+/-)
}

impl Pi {
    pub const fn new(kp: f32, ki: f32, limit: f32) -> Self {
        Self { kp, ki, integ: 0.0, limit }
    }

    /// Preload the integrator (e.g. for bumpless mode transitions).
    pub fn preload(&mut self, value: f32) {
        self.integ = clampf(value, -self.limit, self.limit);
    }

    pub fn reset(&mut self) {
        self.integ = 0.0;
    }

    pub fn update(&mut self, error: f32, dt: f32) -> f32 {
        self.integ = clampf(self.integ + self.ki * error * dt, -self.limit, self.limit);
        clampf(self.kp * error + self.integ, -self.limit, self.limit)
    }
}

#[inline(always)]
pub fn clampf(x: f32, lo: f32, hi: f32) -> f32 {
    if x < lo { lo } else if x > hi { hi } else { x }
}

/// Clarke transform (amplitude-invariant) using two measured phase currents.
/// ia + ib + ic = 0 is assumed.
#[inline(always)]
pub fn clarke(ia: f32, ib: f32) -> (f32, f32) {
    const ONE_OVER_SQRT3: f32 = 0.577_350_3;
    let i_alpha = ia;
    let i_beta = ONE_OVER_SQRT3 * (ia + 2.0 * ib);
    (i_alpha, i_beta)
}

/// Park transform: stationary alpha/beta -> rotating d/q.
#[inline(always)]
pub fn park(alpha: f32, beta: f32, sin_t: f32, cos_t: f32) -> (f32, f32) {
    let d = alpha * cos_t + beta * sin_t;
    let q = -alpha * sin_t + beta * cos_t;
    (d, q)
}

/// Inverse Park transform: rotating d/q -> stationary alpha/beta.
#[inline(always)]
pub fn inv_park(d: f32, q: f32, sin_t: f32, cos_t: f32) -> (f32, f32) {
    let alpha = d * cos_t - q * sin_t;
    let beta = d * sin_t + q * cos_t;
    (alpha, beta)
}

/// Space-vector PWM via min/max common-mode injection.
/// Returns per-phase duty cycles in [DUTY_MIN, DUTY_MAX].
///
/// The max duty is capped below 1.0 so the low-side FETs always conduct for a
/// short window each cycle — required for low-side shunt current sensing.
#[inline(always)]
pub fn svpwm(v_alpha: f32, v_beta: f32, vbus: f32) -> (f32, f32, f32) {
    const SQRT3_OVER_2: f32 = 0.866_025_4;
    const DUTY_MIN: f32 = 0.02;
    const DUTY_MAX: f32 = 0.95;

    let va = v_alpha;
    let vb = -0.5 * v_alpha + SQRT3_OVER_2 * v_beta;
    let vc = -0.5 * v_alpha - SQRT3_OVER_2 * v_beta;

    let vmax = if va > vb { if va > vc { va } else { vc } } else if vb > vc { vb } else { vc };
    let vmin = if va < vb { if va < vc { va } else { vc } } else if vb < vc { vb } else { vc };
    let vcm = 0.5 * (vmax + vmin);

    let inv_vbus = if vbus > 1.0 { 1.0 / vbus } else { 1.0 };
    let da = clampf(0.5 + (va - vcm) * inv_vbus, DUTY_MIN, DUTY_MAX);
    let db = clampf(0.5 + (vb - vcm) * inv_vbus, DUTY_MIN, DUTY_MAX);
    let dc = clampf(0.5 + (vc - vcm) * inv_vbus, DUTY_MIN, DUTY_MAX);
    (da, db, dc)
}

/// Limit the magnitude of a (d,q) voltage vector to `vmax`, preserving angle
/// but prioritizing the d axis (standard circle limiter).
#[inline(always)]
pub fn limit_voltage(vd: f32, vq: f32, vmax: f32) -> (f32, f32) {
    let vd_l = clampf(vd, -vmax, vmax);
    let q_head = libm::sqrtf(vmax * vmax - vd_l * vd_l);
    let vq_l = clampf(vq, -q_head, q_head);
    (vd_l, vq_l)
}

/// Wrap an angle to [-pi, pi).
///
/// Constant-time reduction — never a loop. A subtract-until-in-range loop hangs
/// the 25 kHz ISR if it is ever handed a runaway angle (a diverging observer,
/// say): past ~5e7 `x - 2*PI == x` in f32, so it spins forever. Here a bad
/// angle instead surfaces as garbage in telemetry, which is what you want
/// during bring-up. Non-finite input is clamped to 0 so it can't propagate.
#[inline(always)]
pub fn wrap_angle(theta: f32) -> f32 {
    const PI: f32 = core::f32::consts::PI;
    const TWO_PI: f32 = 2.0 * PI;
    if !theta.is_finite() {
        return 0.0;
    }
    let wrapped = theta - TWO_PI * libm::floorf((theta + PI) / TWO_PI);
    // floorf rounding at the boundary can leave it a hair out of range; one
    // branch each way (no loop) pulls it back in.
    if wrapped >= PI {
        wrapped - TWO_PI
    } else if wrapped < -PI {
        wrapped + TWO_PI
    } else {
        wrapped
    }
}

/// Single-precision sine and cosine of the same angle, computed together.
///
/// `libm::sinf`/`cosf` are musl ports that evaluate in `f64` internally; the
/// Cortex-M4 FPU is single-precision only, so each call turns into hundreds of
/// *software* double-precision ops (~2000 cycles each). Four trig calls per
/// 25 kHz period blew the ISR to ~66 us, over its 40 us budget. This is an
/// all-`f32` minimax approximation (DirectXMath coefficients, error < 2e-7 over
/// the full circle), ~40 cycles for the pair — accuracy far beyond what FOC
/// commutation needs.
#[inline(always)]
pub fn sin_cos(x: f32) -> (f32, f32) {
    const PI: f32 = core::f32::consts::PI;
    const TWO_PI: f32 = 2.0 * PI;
    const HALF_PI: f32 = PI / 2.0;
    const INV_TWO_PI: f32 = 1.0 / TWO_PI;

    // Range-reduce to [-pi, pi] with a single round-to-nearest (all f32).
    let mut a = x - TWO_PI * libm::roundf(x * INV_TWO_PI);
    // Fold [-pi, pi] into [-pi/2, pi/2]; cos flips sign in the outer quadrants.
    let mut cos_sign = 1.0f32;
    if a > HALF_PI {
        a = PI - a;
        cos_sign = -1.0;
    } else if a < -HALF_PI {
        a = -PI - a;
        cos_sign = -1.0;
    }

    let a2 = a * a;
    // sin: odd minimax, cos: even minimax (Horner form).
    let sin = (((((-2.388_985_9e-8 * a2 + 2.752_556_2e-6) * a2 - 1.984_087_4e-4) * a2
        + 8.333_331e-3) * a2 - 1.666_666_7e-1) * a2 + 1.0) * a;
    let cos = cos_sign
        * (((((-2.605_161_5e-7 * a2 + 2.476_049_5e-5) * a2 - 1.388_837_8e-3) * a2
            + 4.166_663_8e-2) * a2 - 0.5) * a2 + 1.0);
    (sin, cos)
}

#![no_std]

use libm::sqrtf;

#[derive(Copy, Clone)]
pub struct Quat {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quat {
    pub fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }

    pub fn conjugate(&self) -> Self {
        Self::new(self.w, -self.x, -self.y, -self.z)
    }

    pub fn multiply(&self, other: &Quat) -> Self {
        Self::new(
            self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
        )
    }

    pub fn vector(&self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }

    pub fn normalize(&mut self) {
        let norm = sqrtf(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z);
        if norm > 1e-6 {
            self.w /= norm;
            self.x /= norm;
            self.y /= norm;
            self.z /= norm;
        }
    }
}

pub struct PDController {
    pub kp: f32,
    pub kd: f32,
    pub max_torque: f32,
}

impl PDController {
    pub fn compute_torque(&self, q_error: &Quat, w: &[f32; 3]) -> [f32; 3] {
        let q_err_vec = q_error.vector();
        let mut torque = [0.0f32; 3];

        for i in 0..3 {
            torque[i] = -self.kp * q_err_vec[i] - self.kd * w[i];
            if torque[i] > self.max_torque {
                torque[i] = self.max_torque;
            } else if torque[i] < -self.max_torque {
                torque[i] = -self.max_torque;
            }
        }
        torque
    }
}

#[derive(Copy, Clone)]
pub struct SatState {
    pub q: Quat,
    pub w: [f32; 3],
    pub rw: [f32; 3],
}

pub struct SatParams {
    pub i_sat_diag: f32,
    pub inv_i_sat: f32,
    pub i_rw: f32,
    pub inv_i_rw: f32,
    pub max_speed_rw: f32,
}

pub struct Energies {
    pub p_body: f32,
    pub p_wheels: f32,
}

#[derive(Copy, Clone)]
pub struct Sample {
    pub t: f32,       // seconds
    pub q: [f32; 4],  // [w,x,y,z]
    pub w: [f32; 3],  // [wx,wy,wz]
    pub rw: [f32; 3], // rw speeds
    pub p_mech: f32,
    pub p_wheels: f32,
}

/// Bundle used to avoid duplicated init code everywhere.
pub fn default_problem() -> (SatState, SatParams, PDController, Quat) {
    // 45 deg around [1,1,1]/||[1,1,1]||
    let angle_deg = 45.0f32;
    let angle_rad = angle_deg * (core::f32::consts::PI / 180.0);

    let axis_norm = sqrtf(1.0 + 1.0 + 1.0);
    let axis = [1.0 / axis_norm, 1.0 / axis_norm, 1.0 / axis_norm];

    let half_angle = angle_rad * 0.5;
    let sin_half = libm::sinf(half_angle);
    let cos_half = libm::cosf(half_angle);

    let q0 = Quat::new(
        cos_half,
        axis[0] * sin_half,
        axis[1] * sin_half,
        axis[2] * sin_half,
    );

    let state0 = SatState {
        q: q0,
        w: [0.1, -0.1, 0.2],
        rw: [0.0; 3],
    };

    let q_target = Quat::new(1.0, 0.0, 0.0, 0.0);

    let i_sat_diag = 1.33 * (0.1f32 * 0.1) / 6.0;
    let params = SatParams {
        i_sat_diag,
        inv_i_sat: 1.0 / i_sat_diag,
        i_rw: 5e-5,
        inv_i_rw: 1.0 / 5e-5,
        max_speed_rw: 8000.0 * 2.0 * core::f32::consts::PI / 60.0,
    };

    let controller = PDController {
        kp: 0.01,
        kd: 0.1,
        max_torque: 0.001,
    };

    (state0, params, controller, q_target)
}

// ---------- Shared energy helpers ----------

pub fn compute_energies(state: &SatState, torque: &[f32; 3]) -> Energies {
    let [wx, wy, wz] = state.w;
    let [rw1, rw2, rw3] = state.rw;

    let p_body = torque[0] * wx + torque[1] * wy + torque[2] * wz;
    let p_wheels = (-torque[0]) * rw1 + (-torque[1]) * rw2 + (-torque[2]) * rw3;

    Energies { p_body, p_wheels }
}

// ---------- DTSS (Euler) integrator ----------

pub fn step_dtss_euler(state: &mut SatState, params: &SatParams, torque: &[f32; 3], dt: f32) {
    let [wx, wy, wz] = state.w;
    let [rw1, rw2, rw3] = state.rw;

    let h_rw = [params.i_rw * rw1, params.i_rw * rw2, params.i_rw * rw3];

    let h_total = [
        params.i_sat_diag * wx + h_rw[0],
        params.i_sat_diag * wy + h_rw[1],
        params.i_sat_diag * wz + h_rw[2],
    ];

    let w_cross_h = [
        wy * h_total[2] - wz * h_total[1],
        wz * h_total[0] - wx * h_total[2],
        wx * h_total[1] - wy * h_total[0],
    ];

    let dw_dt = [
        params.inv_i_sat * (torque[0] - w_cross_h[0]),
        params.inv_i_sat * (torque[1] - w_cross_h[1]),
        params.inv_i_sat * (torque[2] - w_cross_h[2]),
    ];

    let omega_q = Quat::new(0.0, wx, wy, wz);
    let q_dot = state.q.multiply(&omega_q);
    let dq_dt = Quat::new(0.5 * q_dot.w, 0.5 * q_dot.x, 0.5 * q_dot.y, 0.5 * q_dot.z);

    let drw_dt = [
        -params.inv_i_rw * torque[0],
        -params.inv_i_rw * torque[1],
        -params.inv_i_rw * torque[2],
    ];

    // integrate
    state.q.w += dq_dt.w * dt;
    state.q.x += dq_dt.x * dt;
    state.q.y += dq_dt.y * dt;
    state.q.z += dq_dt.z * dt;
    state.q.normalize();

    state.w[0] += dw_dt[0] * dt;
    state.w[1] += dw_dt[1] * dt;
    state.w[2] += dw_dt[2] * dt;

    state.rw[0] += drw_dt[0] * dt;
    state.rw[1] += drw_dt[1] * dt;
    state.rw[2] += drw_dt[2] * dt;

    for s in &mut state.rw {
        if *s > params.max_speed_rw {
            *s = params.max_speed_rw;
        } else if *s < -params.max_speed_rw {
            *s = -params.max_speed_rw;
        }
    }
}

// ---------- QSS1 support ----------

type StateVec = [f32; 10];

pub fn state_to_vec(state: &SatState) -> StateVec {
    [
        state.q.w,
        state.q.x,
        state.q.y,
        state.q.z,
        state.w[0],
        state.w[1],
        state.w[2],
        state.rw[0],
        state.rw[1],
        state.rw[2],
    ]
}

pub fn vec_to_state(x: &StateVec) -> SatState {
    SatState {
        q: Quat::new(x[0], x[1], x[2], x[3]),
        w: [x[4], x[5], x[6]],
        rw: [x[7], x[8], x[9]],
    }
}

/// Derivatives f(x) using quantized state, like satellite_dynamics in the doc.
pub fn satellite_dynamics_vec(
    x: &StateVec,
    params: &SatParams,
    controller: &PDController,
    q_target: &Quat,
) -> StateVec {
    // Reconstruct state and normalize quaternion (like the f64 PC version)
    let mut state = vec_to_state(x);

    let mut q_norm = state.q;
    q_norm.normalize();
    state.q = q_norm;

    let q_err = state.q.multiply(&q_target.conjugate());
    let torque_c = controller.compute_torque(&q_err, &state.w);

    let [wx, wy, wz] = state.w;
    let [rw1, rw2, rw3] = state.rw;

    let h_rw = [params.i_rw * rw1, params.i_rw * rw2, params.i_rw * rw3];

    let h_total = [
        params.i_sat_diag * wx + h_rw[0],
        params.i_sat_diag * wy + h_rw[1],
        params.i_sat_diag * wz + h_rw[2],
    ];

    let w_cross_h = [
        wy * h_total[2] - wz * h_total[1],
        wz * h_total[0] - wx * h_total[2],
        wx * h_total[1] - wy * h_total[0],
    ];

    let dw_dt = [
        params.inv_i_sat * (torque_c[0] - w_cross_h[0]),
        params.inv_i_sat * (torque_c[1] - w_cross_h[1]),
        params.inv_i_sat * (torque_c[2] - w_cross_h[2]),
    ];

    let omega_q = Quat::new(0.0, wx, wy, wz);
    let q_dot = state.q.multiply(&omega_q);
    let dq_dt = Quat::new(0.5 * q_dot.w, 0.5 * q_dot.x, 0.5 * q_dot.y, 0.5 * q_dot.z);

    let drw_dt = [
        -params.inv_i_rw * torque_c[0],
        -params.inv_i_rw * torque_c[1],
        -params.inv_i_rw * torque_c[2],
    ];

    [
        dq_dt.w, dq_dt.x, dq_dt.y, dq_dt.z, dw_dt[0], dw_dt[1], dw_dt[2], drw_dt[0], drw_dt[1],
        drw_dt[2],
    ]
}

pub fn time_to_next_event(x_i: f32, q_i: f32, xdot_i: f32, delta_q_i: f32) -> f32 {
    if xdot_i == 0.0 {
        return f32::INFINITY;
    }
    let delta_x = q_i + xdot_i.signum() * delta_q_i - x_i;
    let dt = delta_x / xdot_i;
    if dt > 0.0 {
        dt
    } else {
        f32::INFINITY
    }
}

pub fn solve_qss1_embedded<F>(
    t0: f32,
    t_final: f32,
    x0: StateVec,
    delta_q: StateVec,
    params: &SatParams,
    controller: &PDController,
    q_target: &Quat,
    mut on_sample: F,
) where
    F: FnMut(f32, &StateVec),
{
    let mut t = t0;
    let mut x = x0;
    let mut qx = x0;

    // Initial derivatives from quantized state
    let mut deriv = satellite_dynamics_vec(&qx, params, controller, q_target);

    let mut next_event_times = [0.0f32; 10];
    for i in 0..10 {
        next_event_times[i] = time_to_next_event(x[i], qx[i], deriv[i], delta_q[i]);
    }

    // Initial sample
    on_sample(t, &x);

    while t < t_final {
        // Find minimum event time
        let mut min_time_step = f32::INFINITY;
        for i in 0..10 {
            if next_event_times[i] < min_time_step {
                min_time_step = next_event_times[i];
            }
        }

        if !min_time_step.is_finite() {
            break;
        }

        // Clamp so we don't step beyond t_final
        if t + min_time_step > t_final {
            min_time_step = t_final - t;
        }

        if min_time_step <= 0.0 {
            break;
        }

        t += min_time_step;

        // Integrate continuous state
        for i in 0..10 {
            x[i] += deriv[i] * min_time_step;
            if next_event_times[i].is_finite() {
                next_event_times[i] -= min_time_step;
            }
        }

        // Clamp reaction-wheel speeds
        for idx in 7..10 {
            if x[idx] > params.max_speed_rw {
                x[idx] = params.max_speed_rw;
            } else if x[idx] < -params.max_speed_rw {
                x[idx] = -params.max_speed_rw;
            }
        }

        // Detect triggered components
        let mut triggered = [false; 10];
        for i in 0..10 {
            if next_event_times[i].abs() < 1e-9 {
                triggered[i] = true;
            }
        }

        // Update quantized state
        for i in 0..10 {
            if triggered[i] {
                qx[i] = x[i];
            }
        }

        // Recompute derivatives and next event times
        deriv = satellite_dynamics_vec(&qx, params, controller, q_target);
        for i in 0..10 {
            next_event_times[i] = time_to_next_event(x[i], qx[i], deriv[i], delta_q[i]);
        }

        // Sample at this event
        on_sample(t, &x);
    }
}

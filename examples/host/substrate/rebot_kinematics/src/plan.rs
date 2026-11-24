//! rebot_plan — pure, memory-safe SQL planning & trajectory cores.
//!
//! Ports of `plan_to_target.plan`, `plan_to_target.plan_sequence`, and
//! `planner_server.move_one` + `traj.{joint_move,joint_sequence}` backed by a
//! single parsed `ArmModel` (zero per-waypoint XML re-parsing). A refusal
//! panics -> SQL ERROR -> rollback.

use nalgebra::{DMatrix, Isometry3, Matrix3, Matrix6, Translation3, UnitQuaternion, Vector3, Vector6};
use pgrx::prelude::*;
use serde_json::{json, Value};

use crate::config::{json_finite, Config};
use crate::model::ArmModel;

type V3 = Vector3<f64>;
type M3 = Matrix3<f64>;

const MIT_POSITION_LIMIT: f64 = 4.0 * std::f64::consts::PI;
const MAX_VIAS: usize = 5;
const MAX_WAYPOINTS: usize = 100_000;
const KP_DEFAULT: &str = "50,2:150,3:150";

fn json_v3(v: &Value, ctx: &str) -> V3 {
    let a = v
        .as_array()
        .unwrap_or_else(|| panic!("{ctx}: expected [x, y, z]"));
    if a.len() != 3 {
        panic!("{ctx}: expected 3 coordinates, got {}", a.len());
    }
    V3::new(
        json_finite(&a[0], ctx),
        json_finite(&a[1], ctx),
        json_finite(&a[2], ctx),
    )
}

fn arr3(v: &V3) -> Value {
    json!([v.x, v.y, v.z])
}

fn validate_hz(hz: f64) -> f64 {
    if !hz.is_finite() || hz <= 0.0 || hz > 500.0 {
        panic!("control_hz must be finite in (0, 500], got {hz}");
    }
    hz
}

fn validate_duration(seconds: f64, ctx: &str) -> f64 {
    if !seconds.is_finite() || seconds < 0.0 || seconds > 3600.0 {
        panic!("{ctx} must be finite in [0, 3600] s, got {seconds}");
    }
    seconds
}

// ── orientation helpers (ports of plan_to_target) ────────────────────────── //

fn rotz(deg: f64) -> M3 {
    let (c, s) = (deg.to_radians().cos(), deg.to_radians().sin());
    M3::new(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0)
}

fn rotation_between(a: &V3, b: &V3) -> M3 {
    let a = a.normalize();
    let b = b.normalize();
    let cross = a.cross(&b);
    let sine = cross.norm();
    let cosine = a.dot(&b).clamp(-1.0, 1.0);
    if sine < 1e-9 {
        if cosine > 0.0 {
            return M3::identity();
        }
        let helper = if a.x.abs() < 0.9 {
            V3::new(1.0, 0.0, 0.0)
        } else {
            V3::new(0.0, 1.0, 0.0)
        };
        let axis = a.cross(&helper).normalize();
        return 2.0 * axis * axis.transpose() - M3::identity();
    }
    let axis = cross / sine;
    let skew = M3::new(
        0.0, -axis.z, axis.y, axis.z, 0.0, -axis.x, -axis.y, axis.x, 0.0,
    );
    let angle = sine.atan2(cosine);
    M3::identity() + angle.sin() * skew + (1.0 - angle.cos()) * (skew * skew)
}

fn tool_axis_vec(name: &str) -> Option<(usize, f64)> {
    match name {
        "x+" => Some((0, 1.0)),
        "x-" => Some((0, -1.0)),
        "y+" => Some((1, 1.0)),
        "y-" => Some((1, -1.0)),
        "z+" => Some((2, 1.0)),
        "z-" => Some((2, -1.0)),
        _ => None,
    }
}

fn work_orientation(r0: &M3, tool_axis: &str, yaw_deg: f64) -> M3 {
    if tool_axis == "current" {
        return *r0;
    }
    let (index, sign) =
        tool_axis_vec(tool_axis).unwrap_or_else(|| panic!("bad tool_axis {tool_axis}"));
    let mut local = V3::zeros();
    local[index] = sign;
    let approach = r0 * local;
    let aligned = rotation_between(&approach, &V3::new(0.0, 0.0, -1.0)) * r0;
    rotz(yaw_deg) * aligned
}

fn stage_in_front(point: &V3, approach: f64) -> V3 {
    let radius = (point.x * point.x + point.y * point.y).sqrt();
    if radius <= approach {
        panic!(
            "target is {radius:.3} m from the base axis, inside the {:.0} mm approach standoff: no room to stage in front of it",
            approach * 1000.0
        );
    }
    point - V3::new(point.x, point.y, 0.0) * (approach / radius)
}

fn close_angle(cfg: &Config, close_width_mm: Option<f64>, default: f64) -> f64 {
    match close_width_mm {
        None => default,
        Some(width) => {
            let max_width = cfg.gripper_max_width_mm();
            if !width.is_finite() || !(0.0..=max_width).contains(&width) {
                panic!(
                    "close_width_mm {width:.1} is outside the jaw stroke [0, {max_width:.1}] mm"
                );
            }
            cfg.motor7_closed + width * cfg.gripper_rad_per_mm
        }
    }
}

// ── timing / sag (ports of traj.ease_fraction, sweep.*) ──────────────────── //

fn ramp_ratio(duration: f64, ramp: f64) -> f64 {
    (ramp / duration.max(1e-9)).clamp(1e-9, 0.5)
}

fn ease_fraction(u: f64, duration: f64, cfg: &Config) -> f64 {
    let r = ramp_ratio(duration, cfg.ramp_s);
    let vc = 1.0 / (1.0 - r);
    let u = u.clamp(0.0, 1.0);
    if u <= r {
        let x = u / r;
        vc * r * (x.powi(3) - 0.5 * x.powi(4))
    } else if u >= 1.0 - r {
        let y = (1.0 - u) / r;
        1.0 - vc * r * (y.powi(3) - 0.5 * y.powi(4))
    } else {
        vc * (u - 0.5 * r)
    }
}

fn seconds_for_travel(travel: f64, cfg: &Config) -> f64 {
    let peak_time = travel / cfg.peak_speed + cfg.ramp_s;
    let cruise_cap = travel / cfg.min_cruise_speed + cfg.ramp_s;
    cfg.min_seconds
        .max(cfg.max_seconds.min(peak_time).min(cruise_cap))
}

fn seconds_for(a: &[f64], b: &[f64], cfg: &Config) -> f64 {
    let travel = a
        .iter()
        .zip(b.iter())
        .map(|(x, y)| (y - x).abs())
        .fold(0.0_f64, f64::max);
    seconds_for_travel(travel, cfg)
}

fn gripper_seconds(travel: f64, cfg: &Config) -> f64 {
    if !travel.is_finite() {
        return 0.0;
    }
    (std::f64::consts::PI / 2.0) * travel.abs() / cfg.gripper_peak_speed
}

// ── via pass-through chains ──────────────────────────────────────────────── //

fn chain_path(poses: &[[f64; 6]], radius: f64, cfg: &Config) -> (Vec<[f64; 6]>, Vec<f64>) {
    let line_samples = cfg.chain_line_samples;
    let fillet_samples = cfg.chain_fillet_samples;
    if poses.len() < 2 {
        panic!("a chain needs at least two poses, got {}", poses.len());
    }
    if !poses.iter().all(|p| p.iter().all(|v| v.is_finite())) {
        panic!("chain poses must be finite");
    }
    let norm = |v: &[f64; 6]| -> f64 {
        (v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3] + v[4] * v[4] + v[5] * v[5]).sqrt()
    };
    let sub = |a: &[f64; 6], b: &[f64; 6]| -> [f64; 6] {
        let mut d = [0.0; 6];
        for j in 0..6 {
            d[j] = a[j] - b[j];
        }
        d
    };
    let lengths: Vec<f64> = (0..poses.len() - 1)
        .map(|i| norm(&sub(&poses[i + 1], &poses[i])))
        .collect();
    let mut samples: Vec<[f64; 6]> = vec![poses[0]];
    let mut cursor = poses[0];
    for index in 1..poses.len() - 1 {
        let knot = poses[index];
        let span_in = lengths[index - 1];
        let span_out = lengths[index];
        let r = radius.min(span_in / 2.0).min(span_out / 2.0);
        if r > 1e-9 && span_in > 1e-12 && span_out > 1e-12 {
            let mut u_in = [0.0; 6];
            let mut u_out = [0.0; 6];
            let mut a = [0.0; 6];
            let mut b = [0.0; 6];
            for j in 0..6 {
                u_in[j] = (knot[j] - poses[index - 1][j]) / span_in;
                u_out[j] = (poses[index + 1][j] - knot[j]) / span_out;
                a[j] = knot[j] - u_in[j] * r;
                b[j] = knot[j] + u_out[j] * r;
            }
            for step in 1..=line_samples {
                let f = step as f64 / line_samples as f64;
                let mut s = [0.0; 6];
                for j in 0..6 {
                    s[j] = cursor[j] + (a[j] - cursor[j]) * f;
                }
                samples.push(s);
            }
            for step in 1..=fillet_samples {
                let t = step as f64 / fillet_samples as f64;
                let omt = 1.0 - t;
                let mut s = [0.0; 6];
                for j in 0..6 {
                    s[j] = omt * omt * a[j] + 2.0 * t * omt * knot[j] + t * t * b[j];
                }
                samples.push(s);
            }
            cursor = b;
        } else {
            for step in 1..=line_samples {
                let f = step as f64 / line_samples as f64;
                let mut s = [0.0; 6];
                for j in 0..6 {
                    s[j] = cursor[j] + (knot[j] - cursor[j]) * f;
                }
                samples.push(s);
            }
            cursor = knot;
        }
    }
    for step in 1..=line_samples {
        let f = step as f64 / line_samples as f64;
        let mut s = [0.0; 6];
        for j in 0..6 {
            s[j] = cursor[j] + (poses[poses.len() - 1][j] - cursor[j]) * f;
        }
        samples.push(s);
    }
    let mut arc: Vec<f64> = Vec::with_capacity(samples.len());
    arc.push(0.0);
    let mut acc = 0.0;
    for i in 1..samples.len() {
        acc += norm(&sub(&samples[i], &samples[i - 1]));
        arc.push(acc);
    }
    (samples, arc)
}

fn chain_travel(samples: &[[f64; 6]]) -> f64 {
    let mut sums = [0.0_f64; 6];
    for w in samples.windows(2) {
        for j in 0..6 {
            sums[j] += (w[1][j] - w[0][j]).abs();
        }
    }
    sums.iter().cloned().fold(0.0_f64, f64::max)
}

fn chain_series(
    samples: &[[f64; 6]],
    arc: &[f64],
    seconds: f64,
    hz: f64,
    cfg: &Config,
) -> Vec<[f64; 6]> {
    let total = *arc.last().unwrap();
    let count = (seconds.max(0.2) * hz).round_ties_even().max(2.0) as usize;
    if count > MAX_WAYPOINTS {
        panic!("chain produces too many waypoints ({count})");
    }
    let duration = count as f64 / hz;
    let mut out: Vec<[f64; 6]> = Vec::with_capacity(count);
    for index in 1..=count {
        let target = ease_fraction(index as f64 / count as f64, duration, cfg) * total;
        let slot = (arc.partition_point(|&a| a <= target) as isize - 1)
            .clamp(0, arc.len() as isize - 2) as usize;
        let span = arc[slot + 1] - arc[slot];
        let weight = if span <= 1e-12 {
            0.0
        } else {
            (target - arc[slot]) / span
        };
        let mut s = [0.0; 6];
        for j in 0..6 {
            s[j] = samples[slot][j] + (samples[slot + 1][j] - samples[slot][j]) * weight;
        }
        out.push(s);
    }
    let last = *samples.last().unwrap();
    *out.last_mut().unwrap() = last;
    out
}

fn kp_vector(spec: &str) -> [f64; 6] {
    let trimmed = spec.trim();
    let effective = if trimmed.is_empty() || trimmed == "default" {
        KP_DEFAULT
    } else {
        trimmed
    };
    let mut base = 50.0;
    let mut overrides = [None::<f64>; 6];
    for part in effective.split(',') {
        let part = part.trim();
        if part.is_empty() {
            continue;
        }
        if let Some((k, v)) = part.split_once(':') {
            let idx = k
                .trim()
                .parse::<usize>()
                .unwrap_or_else(|_| panic!("bad kp motor id in '{part}'"));
            let val = v
                .trim()
                .parse::<f64>()
                .unwrap_or_else(|_| panic!("bad kp value in '{part}'"));
            if !val.is_finite() || val <= 0.0 {
                panic!("kp must be finite and > 0, got {val} in '{part}'");
            }
            if (1..=6).contains(&idx) {
                overrides[idx - 1] = Some(val);
            }
        } else {
            let b = part
                .parse::<f64>()
                .unwrap_or_else(|_| panic!("bad kp spec token '{part}'"));
            if !b.is_finite() || b <= 0.0 {
                panic!("base kp must be finite and > 0, got {b}");
            }
            base = b;
        }
    }
    let mut out = [base; 6];
    for i in 0..6 {
        if let Some(v) = overrides[i] {
            out[i] = v;
        }
    }
    out
}

fn gravity_sag(tau: &[f64], kp: &[f64; 6], cfg: &Config) -> [f64; 6] {
    let mut s = [0.0; 6];
    for j in 0..6 {
        s[j] = (tau[j] / kp[j]).clamp(-cfg.sag_limit_rad, cfg.sag_limit_rad);
    }
    s
}

fn sag_correction(q: &[f64], tau: &[f64], snap: &Value, kp_key: &str, cfg: &Config) -> [f64; 6] {
    let mut extra = [0.0; 6];
    let map = match snap.get("sag_map") {
        Some(m) if m.is_object() => m,
        _ => return extra,
    };
    let fp = snap
        .get("rig_fingerprint")
        .and_then(|v| v.as_str())
        .unwrap_or("");
    let zk = snap
        .get("zeros_key")
        .and_then(|v| v.as_str())
        .unwrap_or("none");
    if map.get("kp_key").and_then(|v| v.as_str()).unwrap_or("") != kp_key {
        return extra;
    }
    if map
        .get("rig_fingerprint")
        .and_then(|v| v.as_str())
        .unwrap_or("")
        != fp
    {
        return extra;
    }
    if map
        .get("zeros_key")
        .and_then(|v| v.as_str())
        .unwrap_or("none")
        != zk
    {
        return extra;
    }
    let env = map.get("envelope").cloned().unwrap_or(json!({}));
    let q_min = env.get("q_min").and_then(|v| v.as_array());
    let q_max = env.get("q_max").and_then(|v| v.as_array());
    for j in 0..6 {
        if let Some(mn) = q_min.and_then(|a| a.get(j)).and_then(|v| v.as_f64()) {
            if q[j] < mn - 1e-9 {
                return extra;
            }
        }
        if let Some(mx) = q_max.and_then(|a| a.get(j)).and_then(|v| v.as_f64()) {
            if q[j] > mx + 1e-9 {
                return extra;
            }
        }
    }
    let per = match map.get("per_motor") {
        Some(p) => p,
        None => return extra,
    };
    for j in 0..6 {
        let m = match per.get(&(j + 1).to_string()) {
            Some(m) => m,
            None => return extra,
        };
        let a = m.get("a").and_then(|v| v.as_f64()).unwrap_or(0.0);
        let b = m.get("b").and_then(|v| v.as_f64()).unwrap_or(0.0);
        extra[j] = (a * tau[j] + b).clamp(-cfg.sag_limit_rad, cfg.sag_limit_rad);
    }
    extra
}

fn unpark_pose_impl(q: &[f64; 6], cfg: &Config) -> [f64; 6] {
    let mut out = *q;
    if out[2] >= 0.05 {
        return out;
    }
    out[2] = (out[2] + 8.0_f64.to_radians()).min(cfg.limits[2][1]);
    out
}

// ── PlanCtx + CLIK IK solver ─────────────────────────────────────────────── //

struct PlanCtx {
    arm: ArmModel,
    cfg: Config,
}

impl PlanCtx {
    fn new(urdf: &str, cfg: Config) -> Self {
        PlanCtx {
            arm: ArmModel::new(urdf),
            cfg,
        }
    }

    fn pose(&self, q6: &[f64]) -> Isometry3<f64> {
        self.arm.fk_pose(q6)
    }

    fn gravity(&self, q6: &[f64]) -> [f64; 6] {
        self.arm.gravity(q6, &self.cfg)
    }

    fn gravity_urdf(&self, q6: &[f64]) -> [f64; 6] {
        self.arm.gravity_urdf(q6, self.cfg.gravity)
    }

    fn jacobian(&self, q6: &[f64], ee: &Isometry3<f64>) -> Matrix6<f64> {
        self.arm.jacobian(q6, ee)
    }

    fn path_safe(&self, a: &[f64], b: &[f64]) -> bool {
        self.arm.path_safe(a, b, 40, &self.cfg)
    }

    fn samples_safe(&self, samples: &[[f64; 6]]) -> bool {
        self.arm.samples_safe(samples, &self.cfg)
    }
}

fn se3_error(cur: &Isometry3<f64>, target: &Isometry3<f64>) -> (f64, Vector6<f64>) {
    let rel = cur.inverse() * target;
    let omega = rel.rotation.scaled_axis();
    let t = rel.translation.vector;
    let theta = omega.norm();
    let v = if theta < 1e-12 {
        t
    } else {
        let u = omega / theta;
        let ux = M3::new(0.0, -u.z, u.y, u.z, 0.0, -u.x, -u.y, u.x, 0.0);
        let half = theta / 2.0;
        let c = 1.0 - half * (half.cos() / half.sin());
        let v_inv = M3::identity() - half * ux + c * (ux * ux);
        v_inv * t
    };
    let mut err = Vector6::zeros();
    for r in 0..3 {
        err[r] = v[r];
        err[r + 3] = omega[r];
    }
    (err.norm(), err)
}

struct IkResult {
    q: [f64; 6],
    success: bool,
}

fn solve_ik(
    ctx: &PlanCtx,
    target: &Isometry3<f64>,
    q_init: &[f64],
    max_iter: usize,
    tolerance: f64,
    damping: f64,
    step_size: f64,
) -> IkResult {
    let mut q = [0.0; 6];
    q.copy_from_slice(&q_init[..6]);
    let (mut prev_err, mut err) = se3_error(&ctx.pose(&q), target);
    if prev_err < tolerance {
        return IkResult { q, success: true };
    }
    for _ in 0..max_iter {
        let ee = ctx.pose(&q);
        let j = ctx.jacobian(&q, &ee);
        let lam = damping * (prev_err * 10.0).max(1.0);
        let dq = damped_step_active_limits(&q, &j, &err, lam, step_size, &ctx.cfg);
        if dq.norm() < 1e-12 {
            return IkResult { q, success: false };
        }
        let mut alpha = 1.0;
        let mut advanced = false;
        for _ in 0..8 {
            let mut q_new = [0.0; 6];
            for i in 0..6 {
                q_new[i] =
                    (q[i] + alpha * dq[i]).clamp(ctx.cfg.limits[i][0], ctx.cfg.limits[i][1]);
            }
            let (new_err, err_new) = se3_error(&ctx.pose(&q_new), target);
            if new_err < prev_err {
                q = q_new;
                err = err_new;
                prev_err = new_err;
                advanced = true;
                if prev_err < tolerance {
                    return IkResult { q, success: true };
                }
                break;
            }
            alpha *= 0.5;
        }
        if !advanced {
            return IkResult { q, success: false };
        }
    }
    IkResult {
        q,
        success: prev_err < tolerance,
    }
}

fn damped_step_active_limits(
    q: &[f64],
    j: &Matrix6<f64>,
    err: &Vector6<f64>,
    damping: f64,
    step_size: f64,
    cfg: &Config,
) -> Vector6<f64> {
    let mut free = [true; 6];
    let limit_eps = 1e-10;
    let mut dq = Vector6::zeros();
    for _ in 0..7 {
        let cols: Vec<usize> = (0..6).filter(|&i| free[i]).collect();
        if cols.is_empty() {
            return dq;
        }
        let mut j_free = DMatrix::<f64>::zeros(6, cols.len());
        for (c, &orig) in cols.iter().enumerate() {
            for r in 0..6 {
                j_free[(r, c)] = j[(r, orig)];
            }
        }
        let mut system = &j_free * j_free.transpose();
        for r in 0..6 {
            system[(r, r)] += damping;
        }
        let x = system
            .lu()
            .solve(err)
            .unwrap_or_else(|| panic!("IK linear solve failed"));
        let dq_free = step_size * (j_free.transpose() * x);
        dq = Vector6::zeros();
        for (c, &orig) in cols.iter().enumerate() {
            dq[orig] = dq_free[c];
        }
        let mut any = false;
        for i in 0..6 {
            let (lo, hi) = (cfg.limits[i][0], cfg.limits[i][1]);
            let at_lower = q[i] <= lo + limit_eps;
            let at_upper = q[i] >= hi - limit_eps;
            if ((at_lower && dq[i] < 0.0) || (at_upper && dq[i] > 0.0)) && free[i] {
                free[i] = false;
                any = true;
            }
        }
        if !any {
            return dq;
        }
    }
    dq
}

// ── trajectory assembly & validation ─────────────────────────────────────── //

#[derive(Clone)]
struct Wp {
    t: f64,
    q: [f64; 7],
    tau: [f64; 7],
}

fn round6(v: f64) -> f64 {
    (v * 1e6).round_ties_even() / 1e6
}

fn max_diff(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b.iter())
        .map(|(x, y)| (x - y).abs())
        .fold(0.0_f64, f64::max)
}

fn validate_payload(p: &Value, cfg: &Config) -> Value {
    if p.get("format").and_then(|v| v.as_str()) != Some("rebot-joint-trajectory") {
        panic!("trajectory format must be 'rebot-joint-trajectory'");
    }
    let hz = p
        .get("control_hz")
        .and_then(|v| v.as_f64())
        .unwrap_or_else(|| panic!("trajectory missing control_hz"));
    validate_hz(hz);
    let ids = p
        .get("ids")
        .and_then(|v| v.as_array())
        .unwrap_or_else(|| panic!("trajectory missing ids"));
    if ids.len() != 7 {
        panic!("trajectory ids must have 7 entries, got {}", ids.len());
    }
    let wps = p["waypoints"]
        .as_array()
        .unwrap_or_else(|| panic!("no waypoints"));
    if wps.len() < 2 {
        panic!("need at least two waypoints, got {}", wps.len());
    }
    let lim_pairs: Option<[[f64; 2]; 7]> = p.get("limits").and_then(|v| v.as_array()).map(|lim| {
        if lim.len() != 7 {
            panic!("trajectory limits must have 7 entries, got {}", lim.len());
        }
        let mut out = [[0.0; 2]; 7];
        for j in 0..7 {
            let pair = lim[j]
                .as_array()
                .unwrap_or_else(|| panic!("trajectory limits[{j}] must be [lo, hi]"));
            if pair.len() != 2 {
                panic!("trajectory limits[{j}] must be [lo, hi]");
            }
            let lo = json_finite(&pair[0], "limits.lo");
            let hi = json_finite(&pair[1], "limits.hi");
            if lo > hi {
                panic!("trajectory limits[{j}] has lo ({lo}) > hi ({hi})");
            }
            out[j] = [lo, hi];
        }
        out
    });

    let max_v_ceiling = cfg.max_joint_step * hz.clamp(5.0, 200.0) * 1.001 + 1e-6;
    let first_has_tau = wps[0].get("tau").and_then(|v| v.as_array()).is_some();
    let first_has_v = wps[0].get("v").and_then(|v| v.as_array()).is_some();
    let mut times: Vec<f64> = Vec::with_capacity(wps.len());
    let mut prev_q = [0.0; 7];
    let mut peak_tau = 0.0_f64;
    let mut peak_v = 0.0_f64;
    let mut max_step = 0.0_f64;

    for (i, wp) in wps.iter().enumerate() {
        let qa = wp["q"]
            .as_array()
            .unwrap_or_else(|| panic!("waypoint {i} missing q"));
        if qa.len() != 7 {
            panic!("waypoint {i} has {} values for 7 motors", qa.len());
        }
        let mut q = [0.0; 7];
        for j in 0..7 {
            q[j] = json_finite(&qa[j], "waypoint q");
        }
        let t = json_finite(&wp["t"], "waypoint t");
        times.push(t);

        let tau_opt = wp.get("tau").and_then(|v| v.as_array());
        if tau_opt.is_some() != first_has_tau {
            panic!("waypoint {i}: tau must be present on all waypoints or none");
        }
        if let Some(tau) = tau_opt {
            if tau.len() != 7 {
                panic!("waypoint {i}: tau must have 7 values, got {}", tau.len());
            }
            for v in tau {
                peak_tau = peak_tau.max(json_finite(v, "waypoint tau").abs());
            }
        }

        let v_opt = wp.get("v").and_then(|v| v.as_array());
        if v_opt.is_some() != first_has_v {
            panic!("waypoint {i}: v must be present on all waypoints or none");
        }
        if let Some(vel) = v_opt {
            if vel.len() != 7 {
                panic!("waypoint {i}: v must have 7 values, got {}", vel.len());
            }
            for (j, v) in vel.iter().enumerate() {
                let vj = json_finite(v, "waypoint v").abs();
                if vj > max_v_ceiling {
                    panic!(
                        "joint {} velocity {vj:.3} rad/s at waypoint {i} exceeds ceiling {max_v_ceiling:.2}",
                        j + 1
                    );
                }
                peak_v = peak_v.max(vj);
            }
        }

        if let Some(lim) = &lim_pairs {
            for j in 0..7 {
                let [lo, hi] = lim[j];
                if q[j] < lo - cfg.limit_tol || q[j] > hi + cfg.limit_tol {
                    panic!(
                        "joint {} leaves its limit [{lo:.3}, {hi:.3}]: {:.3}",
                        j + 1,
                        q[j]
                    );
                }
            }
        }
        for j in 0..7 {
            if q[j].abs() > MIT_POSITION_LIMIT + cfg.limit_tol {
                panic!("joint {} at waypoint {i} outside the MIT range", j + 1);
            }
        }
        if i > 0 {
            let step = max_diff(&q, &prev_q);
            if step > cfg.max_joint_step {
                panic!(
                    "waypoints {}->{i} jump {step:.3} rad, above the {:.2} sanity limit",
                    i - 1,
                    cfg.max_joint_step
                );
            }
            max_step = max_step.max(step);
            if times[i] < times[i - 1] {
                panic!("waypoint times are not monotonic");
            }
        }
        prev_q = q;
    }
    json!({
        "waypoints": wps.len(),
        "duration_s": times[times.len() - 1] - times[0],
        "peak_tau": peak_tau,
        "peak_v": peak_v,
        "max_step_rad": max_step,
    })
}

fn build_payload(
    waypoints: &[Wp],
    hz: f64,
    limits: &[[f64; 2]; 7],
    steps: &str,
    extra: Value,
    hold_start: Option<f64>,
    hold_end: Option<f64>,
    with_tau: bool,
    cfg: &Config,
) -> (Value, Value) {
    let hz = validate_hz(hz);
    let n = waypoints.len();
    let times: Vec<f64> = waypoints.iter().map(|w| round6(w.t)).collect();
    let mut entries: Vec<Value> = Vec::with_capacity(n);
    for (i, wp) in waypoints.iter().enumerate() {
        let mut e = json!({ "t": times[i], "q": wp.q.to_vec() });
        if with_tau {
            e["tau"] = json!(wp.tau.to_vec());
        }
        if n >= 2 {
            let (ai, bi) = if i == 0 {
                (1, 0)
            } else if i == n - 1 {
                (n - 1, n - 2)
            } else {
                (i + 1, i - 1)
            };
            let dt = times[ai] - times[bi];
            let at_rest = (i > 0 && max_diff(&wp.q, &waypoints[i - 1].q) <= 1e-9)
                || (i + 1 < n && max_diff(&wp.q, &waypoints[i + 1].q) <= 1e-9);
            let v: Vec<f64> = if dt <= 1e-9 || at_rest {
                vec![0.0; 7]
            } else {
                (0..7)
                    .map(|j| round6((waypoints[ai].q[j] - waypoints[bi].q[j]) / dt))
                    .collect()
            };
            e["v"] = json!(v);
        }
        entries.push(e);
    }
    let duration = if n > 0 { times[n - 1] } else { 0.0 };
    let mut payload = json!({
        "format": "rebot-joint-trajectory",
        "version": 1,
        "control_hz": hz,
        "ids": [1, 2, 3, 4, 5, 6, 7],
        "limits": limits.iter().map(|l| json!([l[0], l[1]])).collect::<Vec<_>>(),
        "steps": steps,
        "duration_s": duration,
        "waypoints": entries,
    });
    if let (Some(hs), Some(he)) = (hold_start, hold_end) {
        payload["hold_start_s"] = json!(round6(hs));
        payload["hold_end_s"] = json!(round6(he));
    }
    if let Value::Object(map) = extra {
        for (k, v) in map {
            if !v.is_null() {
                payload[k] = v;
            }
        }
    }
    let stats = validate_payload(&payload, cfg);
    (payload, stats)
}

// ── shared helpers ───────────────────────────────────────────────────────── //

fn opt_f(options: &Value, key: &str, default: f64) -> f64 {
    match options.get(key) {
        None | Some(Value::Null) => default,
        Some(v) => json_finite(v, &format!("option '{key}'")),
    }
}

fn opt_nonneg_f(obj: &Value, key: &str, fallback: f64) -> f64 {
    let val = match obj.get(key) {
        None | Some(Value::Null) => fallback,
        Some(v) => json_finite(v, key),
    };
    if val < 0.0 {
        panic!("'{key}' must be >= 0, got {val}");
    }
    val
}

fn opt_b(options: &Value, key: &str, default: bool) -> bool {
    options
        .get(key)
        .and_then(|v| v.as_bool())
        .unwrap_or(default)
}

fn opt_s<'a>(options: &'a Value, key: &str, default: &'a str) -> &'a str {
    options
        .get(key)
        .and_then(|v| v.as_str())
        .unwrap_or(default)
}

fn read_posture(dump: &Value, cfg: &Config, clamp_limits: bool) -> ([f64; 6], f64) {
    let ids = dump["ids"]
        .as_array()
        .unwrap_or_else(|| panic!("dump needs ids"));
    if ids.len() < 6 {
        panic!("dump ids must have at least 6 entries, got {}", ids.len());
    }
    let ids6: Vec<i64> = ids
        .iter()
        .take(6)
        .map(|v| v.as_i64().unwrap_or_else(|| panic!("dump ids must be integers")))
        .collect();
    if ids6 != vec![1, 2, 3, 4, 5, 6] {
        panic!("expected motor ids 1..6 first, got {ids6:?}");
    }
    let qdump = dump["q"]
        .as_array()
        .unwrap_or_else(|| panic!("dump needs q"));
    if qdump.len() < 6 {
        panic!("dump q must have at least 6 angles, got {}", qdump.len());
    }
    let mut q = [0.0; 6];
    for j in 0..6 {
        let val = json_finite(&qdump[j], "dump q");
        q[j] = if clamp_limits {
            val.clamp(cfg.limits[j][0], cfg.limits[j][1])
        } else {
            val
        };
    }
    let gripper = match qdump.get(6) {
        Some(v) if !v.is_null() => json_finite(v, "dump q[6]"),
        _ => cfg.motor7_closed,
    };
    (q, gripper)
}

fn limits7(cfg: &Config) -> [[f64; 2]; 7] {
    let mut l = [[0.0; 2]; 7];
    for j in 0..6 {
        l[j] = [cfg.limits[j][0], cfg.limits[j][1]];
    }
    l[6] = [-0.1, 5.0];
    l
}

// ── Planner state machine ────────────────────────────────────────────────── //

struct Planner<'a> {
    ctx: &'a PlanCtx,
    hz: f64,
    period: f64,
    t: f64,
    waypoints: Vec<Wp>,
    current_q6: [f64; 6],
    current_gripper: f64,
    worst_residual: f64,
    worst_where: String,
    ik_pos_tol: f64,
    ik_angle_tol: f64,
}

impl<'a> Planner<'a> {
    fn new(
        ctx: &'a PlanCtx,
        hz: f64,
        q_start: [f64; 6],
        gripper_start: f64,
        ik_pos_tol: f64,
        ik_angle_tol: f64,
    ) -> Self {
        let hz = validate_hz(hz);
        Planner {
            ctx,
            hz,
            period: 1.0 / hz,
            t: 0.0,
            waypoints: Vec::new(),
            current_q6: q_start,
            current_gripper: gripper_start,
            worst_residual: 0.0,
            worst_where: String::new(),
            ik_pos_tol,
            ik_angle_tol,
        }
    }

    fn append(&mut self, q6: &[f64], gripper: f64) {
        if self.waypoints.len() >= MAX_WAYPOINTS {
            panic!("trajectory exceeds {MAX_WAYPOINTS} waypoints");
        }
        let tau = self.ctx.gravity(q6);
        let mut q = [0.0; 7];
        q[..6].copy_from_slice(q6);
        q[6] = gripper;
        let mut tau7 = [0.0; 7];
        tau7[..6].copy_from_slice(&tau);
        self.current_q6.copy_from_slice(q6);
        self.current_gripper = gripper;
        self.waypoints.push(Wp {
            t: self.t,
            q,
            tau: tau7,
        });
    }

    fn last_q6(&self) -> [f64; 6] {
        self.current_q6
    }

    fn try_ik_to(
        &mut self,
        place: &V3,
        rotation: &M3,
        seed: &[f64],
        label: &str,
    ) -> Result<[f64; 6], String> {
        let target = Isometry3::from_parts(
            Translation3::from(*place),
            UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix_unchecked(
                *rotation,
            )),
        );
        let res = solve_ik(self.ctx, &target, seed, 300, 1e-6, 1e-8, 1.0);
        let reached = self.ctx.pose(&res.q);
        let position_error = (reached.translation.vector - place).norm();
        let cosine =
            ((reached.rotation.to_rotation_matrix().matrix().transpose() * rotation).trace()
                - 1.0)
                / 2.0;
        let angle_error = cosine.clamp(-1.0, 1.0).acos().to_degrees();
        let accepted =
            res.success || (position_error <= self.ik_pos_tol && angle_error <= self.ik_angle_tol);
        if !accepted {
            return Err(format!(
                "IK failed at [{:.3} {:.3} {:.3}] m ({label}): no solution; reached {position_error:.4} m / {angle_error:.2} deg off. The target is probably out of reach from this pose: try a smaller --yaw, a larger --approach, or start from a pose closer to the workspace.",
                place.x, place.y, place.z
            ));
        }
        for j in 0..6 {
            if res.q[j] < self.ctx.cfg.limits[j][0] - self.ctx.cfg.limit_tol
                || res.q[j] > self.ctx.cfg.limits[j][1] + self.ctx.cfg.limit_tol
            {
                return Err(format!(
                    "IK returned joint {} = {:.3} rad at {label}, outside its limits [{:.3}, {:.3}]",
                    j + 1,
                    res.q[j],
                    self.ctx.cfg.limits[j][0],
                    self.ctx.cfg.limits[j][1]
                ));
            }
        }
        if position_error > self.worst_residual {
            self.worst_residual = position_error;
            self.worst_where = label.to_string();
        }
        Ok(res.q)
    }

    fn ik_with_fallback(
        &mut self,
        place: &V3,
        rotation: &M3,
        seed: &[f64],
        fallback_seed: &[f64],
        label: &str,
    ) -> [f64; 6] {
        match self.try_ik_to(place, rotation, seed, label) {
            Ok(q) => q,
            Err(first_err) => {
                if max_diff(seed, fallback_seed) <= 1e-12 {
                    panic!("{first_err}");
                }
                self.try_ik_to(place, rotation, fallback_seed, label)
                    .unwrap_or_else(|e| panic!("{e}"))
            }
        }
    }

    fn joint_leg(&mut self, base: &[f64], delta: &[f64], seconds: f64, gripper: f64) -> [f64; 6] {
        let seconds = validate_duration(seconds, "leg seconds");
        let count = (seconds * self.hz).round_ties_even().max(1.0) as usize;
        for index in 1..=count {
            self.t += self.period;
            let fraction = ease_fraction(
                index as f64 / count as f64,
                count as f64 / self.hz,
                &self.ctx.cfg,
            );
            let mut q = [0.0; 6];
            for j in 0..6 {
                q[j] = base[j] + delta[j] * fraction;
            }
            self.append(&q, gripper);
        }
        self.last_q6()
    }

    fn hold(&mut self, seconds: f64) {
        let seconds = validate_duration(seconds, "hold_s");
        let count = (seconds * self.hz).round_ties_even().max(1.0) as usize;
        if let Some(last) = self.waypoints.last().cloned() {
            for _ in 0..count {
                self.t += self.period;
                self.waypoints.push(Wp {
                    t: self.t,
                    q: last.q,
                    tau: last.tau,
                });
            }
        } else {
            let q6 = self.current_q6;
            let g = self.current_gripper;
            for _ in 0..count {
                self.t += self.period;
                self.append(&q6, g);
            }
        }
    }

    fn hold_g(&mut self, seconds: f64, gripper: f64) {
        let seconds = validate_duration(seconds, "hold_s");
        let count = (seconds * self.hz).round_ties_even().max(1.0) as usize;
        let q6 = self.last_q6();
        for _ in 0..count {
            self.t += self.period;
            self.append(&q6, gripper);
        }
    }

    fn gripper_leg(&mut self, target_angle: f64, seconds: f64) {
        let seconds = validate_duration(seconds, "gripper seconds");
        if self.waypoints.is_empty() {
            let q6 = self.current_q6;
            let g = self.current_gripper;
            self.append(&q6, g);
        }
        let current = self.current_gripper;
        let seconds = seconds.max(gripper_seconds(target_angle - current, &self.ctx.cfg));
        let count = (seconds * self.hz).round_ties_even().max(1.0) as usize;
        let last = self.waypoints.last().unwrap().clone();
        for index in 1..=count {
            self.t += self.period;
            let angle = current
                + (target_angle - current)
                    * ease_fraction(
                        index as f64 / count as f64,
                        count as f64 / self.hz,
                        &self.ctx.cfg,
                    );
            let mut q = last.q;
            q[6] = angle;
            self.current_gripper = angle;
            self.waypoints.push(Wp {
                t: self.t,
                q,
                tau: last.tau,
            });
        }
    }

    fn chain_leg(&mut self, knots: &[[f64; 6]], into_marker: usize, gripper: f64) -> [f64; 6] {
        let mut unique: Vec<[f64; 6]> = vec![knots[0]];
        for knot in &knots[1..] {
            if max_diff(knot, unique.last().unwrap()) > 1e-6 {
                unique.push(*knot);
            }
        }
        if unique.len() < 2 {
            return self.last_q6();
        }
        let (samples, arc) = chain_path(&unique, self.ctx.cfg.corner_radius_rad, &self.ctx.cfg);
        if !self.ctx.samples_safe(&samples) {
            panic!(
                "the via chain into marker {into_marker} dips within a link radius of the worktop; raise the via points or move the arm clear"
            );
        }
        let seconds = seconds_for_travel(chain_travel(&samples), &self.ctx.cfg);
        for q in chain_series(&samples, &arc, seconds, self.hz, &self.ctx.cfg) {
            self.t += self.period;
            self.append(&q, gripper);
        }
        self.last_q6()
    }
}

// ── sag compensation ─────────────────────────────────────────────────────── //

fn aim_with_ctx(ctx: &PlanCtx, target: &[f64], kp: &[f64; 6], extra: &[f64; 6]) -> [f64; 6] {
    let sag_at = |q: &[f64]| -> [f64; 6] {
        let tau = ctx.gravity(q);
        let g = gravity_sag(&tau, kp, &ctx.cfg);
        let mut t = [0.0; 6];
        for j in 0..6 {
            t[j] = (g[j] + extra[j]).clamp(-ctx.cfg.sag_limit_rad, ctx.cfg.sag_limit_rad);
        }
        t
    };
    let mut command = [0.0; 6];
    let s0 = sag_at(target);
    for j in 0..6 {
        command[j] = target[j] + s0[j];
    }
    for _ in 0..2 {
        let s = sag_at(&command);
        for j in 0..6 {
            command[j] = target[j] + s[j];
        }
    }
    command
}

fn compensate(
    ctx: &PlanCtx,
    waypoints: &[Wp],
    snap: &Value,
    kp_spec: &str,
    kp: &[f64; 6],
) -> Vec<Wp> {
    let mut out = Vec::with_capacity(waypoints.len());
    for wp in waypoints {
        let q6 = &wp.q[..6];
        let tau = ctx.gravity(q6);
        let extra = sag_correction(q6, &tau, snap, kp_spec, &ctx.cfg);
        let cmd = aim_with_ctx(ctx, q6, kp, &extra);
        let mut q = [0.0; 7];
        for j in 0..6 {
            q[j] = cmd[j].clamp(ctx.cfg.limits[j][0], ctx.cfg.limits[j][1]);
        }
        q[6] = wp.q[6];
        let g = ctx.gravity_urdf(&q[..6]);
        let mut tau7 = [0.0; 7];
        tau7[..6].copy_from_slice(&g);
        out.push(Wp {
            t: wp.t,
            q,
            tau: tau7,
        });
    }
    out
}

// ── single-target hover plan (rebot_plan) ────────────────────────────────── //

#[pg_extern(immutable, parallel_safe)]
fn rebot_plan(
    urdf: &str,
    dump: pgrx::JsonB,
    target: pgrx::JsonB,
    options: pgrx::JsonB,
    snap: pgrx::JsonB,
    config: pgrx::JsonB,
) -> pgrx::JsonB {
    let cfg = Config::from_json(&config.0);
    let dump = dump.0;
    let target = target.0;
    let options = options.0;
    let snap = snap.0;
    let ctx = PlanCtx::new(urdf, cfg.clone());

    let (q_start, gripper_start) = read_posture(&dump, &cfg, true);
    let ee_start = ctx.pose(&q_start);
    let p_start = ee_start.translation.vector;
    let mut p0 = p_start;
    let mut r0 = *ee_start.rotation.to_rotation_matrix().matrix();

    if target.get("format").and_then(|v| v.as_str()) != Some("rebot-target") {
        panic!("not a rebot-target file");
    }
    let point = json_v3(&target["point"], "target.point");
    let hover_m = opt_nonneg_f(&target, "hover_m", opt_nonneg_f(&options, "hover_m", cfg.default_min_z));
    let tip_offset_m = opt_nonneg_f(&target, "tip_offset_m", opt_nonneg_f(&options, "tip_m", 0.0));
    let is_tip = target.get("point_frame").and_then(|v| v.as_str()) == Some("tip");
    let hover = opt_b(&options, "hover", true);
    let lift_m = if is_tip || !hover { 0.0 } else { hover_m };
    let tip_point = point + V3::new(0.0, 0.0, lift_m);
    let flange_point = tip_point + V3::new(0.0, 0.0, tip_offset_m);
    let radius = (point.x * point.x + point.y * point.y).sqrt();
    let min_z = opt_f(&options, "min_z", cfg.default_min_z);
    let max_z = opt_f(&options, "max_z", 0.95);
    let max_reach = opt_f(&options, "max_reach", 0.75);
    let judged = if hover { tip_point } else { point };
    let floor = if hover { min_z } else { min_z.min(0.002) };
    if judged.z < floor - 1e-9 {
        panic!(
            "target z {:.4} is below the minimum {floor:.4}: refusing to plan into the table",
            judged.z
        );
    }
    if judged.z > max_z {
        panic!("target z {:.3} is above --max-z {max_z:.3}", judged.z);
    }
    if radius > max_reach {
        panic!(
            "target is {radius:.3} m from the base axis, above --max-reach {max_reach:.2}"
        );
    }

    let tool_axis = opt_s(&options, "tool_axis", "current").to_string();
    let yaw = opt_f(&options, "yaw", 0.0);
    let hz = validate_hz(opt_f(&options, "hz", cfg.default_hz));
    let hold_s = validate_duration(opt_f(&options, "hold_s", 0.6), "hold_s");
    let no_return = opt_b(&options, "no_return", false);
    let unpark_j2 = opt_f(&options, "unpark_j2", 0.0);
    let unpark_j3 = opt_f(&options, "unpark_j3", 8.0);
    let unpark_s = validate_duration(opt_f(&options, "unpark_s", 2.0), "unpark_s");
    let ik_pos_tol = opt_f(&options, "ik_pos_tol", 0.002).max(0.010);
    let ik_angle_tol = opt_f(&options, "ik_angle_tol", 0.5);
    let kp_spec = opt_s(&options, "kp_spec", KP_DEFAULT).to_string();
    let kp = kp_vector(&kp_spec);

    let mut unpark = [0.0; 6];
    unpark[1] = unpark_j2.to_radians();
    unpark[2] = unpark_j3.to_radians();
    let unparked = unpark.iter().any(|v| v.abs() > 1e-9);

    let mut pl = Planner::new(&ctx, hz, q_start, gripper_start, ik_pos_tol, ik_angle_tol);

    let mut seed = q_start;
    if unparked {
        seed = pl.joint_leg(&q_start, &unpark, unpark_s, gripper_start);
        let ee = ctx.pose(&seed);
        p0 = ee.translation.vector;
        r0 = *ee.rotation.to_rotation_matrix().matrix();
    }

    let (report_axis, report_yaw, hold_start, hold_end) = if hover {
        let hover_axis = if tool_axis == "current" {
            "z-".to_string()
        } else {
            tool_axis.clone()
        };
        let hover_yaw = if tool_axis == "current" {
            point.y.atan2(point.x).to_degrees()
        } else {
            yaw
        };
        let r_hover = work_orientation(&r0, &hover_axis, hover_yaw);
        let target_q = pl.ik_with_fallback(&flange_point, &r_hover, &seed, &q_start, "hover");
        let current_q = pl.last_q6();
        if !ctx.path_safe(&current_q, &target_q) {
            panic!(
                "the leg to the tap dips within a link radius of the worktop; tap a point further out or move the arm clear"
            );
        }
        let mut delta = [0.0; 6];
        for j in 0..6 {
            delta[j] = target_q[j] - current_q[j];
        }
        pl.joint_leg(
            &current_q,
            &delta,
            seconds_for(&current_q, &target_q, &cfg),
            gripper_start,
        );
        let hs = pl.t;
        pl.hold(hold_s);
        let he = pl.t;
        if !no_return {
            let q_last = pl.last_q6();
            if !ctx.path_safe(&q_last, &seed) {
                panic!(
                    "the return to the start pose dips within a link radius of the worktop; move the arm clear and retry"
                );
            }
            let mut d = [0.0; 6];
            for j in 0..6 {
                d[j] = seed[j] - q_last[j];
            }
            pl.joint_leg(
                &q_last,
                &d,
                seconds_for(&q_last, &seed, &cfg),
                gripper_start,
            );
        }
        (hover_axis, hover_yaw, hs, he)
    } else {
        panic!("rebot_plan currently implements the hover branch (the gateway tap path)");
    };

    let compensated = compensate(&ctx, &pl.waypoints, &snap, &kp_spec, &kp);
    let steps = if is_tip {
        format!(
            "tap target: move tip to [{:.3} {:.3} {:.3}]{}",
            point.x,
            point.y,
            point.z,
            if no_return { "" } else { ", then return" }
        )
    } else {
        format!(
            "tap target: hover {:.0} mm above [{:.3} {:.3} {:.3}]{}",
            hover_m * 1000.0,
            point.x,
            point.y,
            point.z,
            if no_return { "" } else { ", then return" }
        )
    };

    let mut provenance = json!({
        "capture": target.get("capture").cloned().unwrap_or(Value::Null),
        "frame": target.get("frame").cloned().unwrap_or(Value::Null),
        "target_source": target.get("source").cloned().unwrap_or(Value::Null),
        "target_point": arr3(&point),
        "point_frame": target.get("point_frame").cloned().unwrap_or(Value::Null),
        "tip_point": arr3(&tip_point),
        "flange_point": arr3(&flange_point),
        "surface_point": target.get("surface_point").cloned().unwrap_or(Value::Null),
        "tap_height_m": target.get("tap_height_m").cloned().unwrap_or(Value::Null),
        "tip_offset_m": tip_offset_m,
        "hover_m": if is_tip { 0.0 } else { hover_m },
        "tip_m": tip_offset_m,
    });
    if let Value::Object(m) = &mut provenance {
        m.retain(|_, v| !v.is_null());
    }

    let extra = json!({
        "capture": provenance,
        "tool_axis": report_axis,
        "yaw_deg": report_yaw,
        "dynamics": {
            "source": "rebot_plan (Rust CLIK + gravity port of reBotArm_control_py)",
            "tau": "gravity torque of the planned pose, per waypoint",
        },
    });
    let (payload, stats) = build_payload(
        &compensated,
        hz,
        &limits7(&cfg),
        &steps,
        extra,
        if hold_end > hold_start {
            Some(hold_start)
        } else {
            None
        },
        if hold_end > hold_start {
            Some(hold_end)
        } else {
            None
        },
        true,
        &cfg,
    );

    let summary = json!({
        "start_tcp": arr3(&p_start),
        "unpark_tcp": if unparked { arr3(&p0) } else { Value::Null },
        "target": arr3(&tip_point),
        "flange_point": arr3(&flange_point),
        "radius": radius,
        "hover_m": if is_tip { Value::Null } else { json!(hover_m) },
        "tip_m": tip_offset_m,
        "tap_height_m": target.get("tap_height_m").cloned().unwrap_or(Value::Null),
        "point_frame": target.get("point_frame").cloned().unwrap_or(Value::Null),
        "tool_axis": report_axis,
        "yaw_deg": report_yaw,
        "waypoints": stats["waypoints"],
        "duration_s": stats["duration_s"],
        "worst_ik_m": pl.worst_residual,
        "worst_ik_at": if pl.worst_where.is_empty() { Value::Null } else { json!(pl.worst_where) },
        "peak_tau": stats["peak_tau"],
        "max_step_rad": stats["max_step_rad"],
        "unparked": unparked,
    });
    pgrx::JsonB(json!({ "payload": payload, "summary": summary }))
}

// ── multi-marker sequence plan (rebot_plan_sequence) ─────────────────────── //

#[pg_extern(immutable, parallel_safe)]
fn rebot_plan_sequence(
    urdf: &str,
    dump: pgrx::JsonB,
    targets: pgrx::JsonB,
    options: pgrx::JsonB,
    snap: pgrx::JsonB,
    config: pgrx::JsonB,
) -> pgrx::JsonB {
    let cfg = Config::from_json(&config.0);
    let dump = dump.0;
    let targets = targets.0;
    let options = options.0;
    let snap = snap.0;
    let tarr = targets
        .as_array()
        .unwrap_or_else(|| panic!("targets must be an array"));
    if tarr.is_empty() {
        panic!("a marker sequence needs at least one target");
    }
    if tarr.len() > 20 {
        panic!("a marker sequence is limited to 20 targets");
    }
    let ctx = PlanCtx::new(urdf, cfg.clone());

    let (q_start, gripper_start) = read_posture(&dump, &cfg, true);
    let ee_start = ctx.pose(&q_start);
    let p_start = ee_start.translation.vector;
    let mut r0 = *ee_start.rotation.to_rotation_matrix().matrix();

    let tool_axis = opt_s(&options, "tool_axis", "current").to_string();
    let yaw = opt_f(&options, "yaw", 0.0);
    let approach = opt_nonneg_f(&options, "approach", 0.06);
    let hz = validate_hz(opt_f(&options, "hz", cfg.default_hz));
    let hold_s = validate_duration(opt_f(&options, "hold_s", 0.6), "hold_s");
    let grasp_q = opt_f(&options, "grasp_q", 2.0);
    let close_width_mm = options.get("close_width_mm").and_then(|v| v.as_f64());
    let unpark_j2 = opt_f(&options, "unpark_j2", 0.0);
    let unpark_j3 = opt_f(&options, "unpark_j3", 8.0);
    let unpark_s = validate_duration(opt_f(&options, "unpark_s", 2.0), "unpark_s");
    let ik_pos_tol = opt_f(&options, "ik_pos_tol", 0.002).max(0.010);
    let ik_angle_tol = opt_f(&options, "ik_angle_tol", 0.5);
    let min_z = opt_f(&options, "min_z", cfg.default_min_z);
    let max_z = opt_f(&options, "max_z", 0.95);
    let max_reach = opt_f(&options, "max_reach", 0.75);
    let kp_spec = opt_s(&options, "kp_spec", KP_DEFAULT).to_string();
    let kp = kp_vector(&kp_spec);
    let sequence_axis = if tool_axis == "current" {
        "z-".to_string()
    } else {
        tool_axis.clone()
    };

    let mut points: Vec<V3> = Vec::with_capacity(tarr.len());
    let mut hovers: Vec<f64> = Vec::with_capacity(tarr.len());
    let mut tip_offsets: Vec<f64> = Vec::with_capacity(tarr.len());
    let mut flange_points: Vec<V3> = Vec::with_capacity(tarr.len());
    let mut vias: Vec<Vec<V3>> = Vec::with_capacity(tarr.len());
    let mut via_flange_points: Vec<Vec<V3>> = Vec::with_capacity(tarr.len());
    let opt_hover = opt_nonneg_f(&options, "hover_m", cfg.default_min_z);
    let opt_tip = opt_nonneg_f(&options, "tip_m", 0.0);

    for (i, target) in tarr.iter().enumerate() {
        if target.get("format").and_then(|v| v.as_str()) != Some("rebot-target") {
            panic!("marker {}: not a rebot-target file", i + 1);
        }
        let point = json_v3(&target["point"], &format!("marker {} point", i + 1));
        let hover_m = opt_nonneg_f(target, "hover_m", opt_hover);
        let tip_offset_m = opt_nonneg_f(target, "tip_offset_m", opt_tip);
        let is_tip = target.get("point_frame").and_then(|v| v.as_str()) == Some("tip");
        let tip_point = if is_tip {
            point
        } else {
            point + V3::new(0.0, 0.0, hover_m)
        };
        let flange_point = tip_point + V3::new(0.0, 0.0, tip_offset_m);
        let radius = (point.x * point.x + point.y * point.y).sqrt();
        if tip_point.z < min_z - 1e-9 {
            panic!(
                "marker {}: tip z {:.4} is below the minimum {min_z:.4}",
                i + 1,
                tip_point.z
            );
        }
        if tip_point.z > max_z {
            panic!(
                "marker {}: tip z {:.3} is above --max-z {max_z:.3}",
                i + 1,
                tip_point.z
            );
        }
        if radius > max_reach {
            panic!(
                "marker {}: target is {radius:.3} m from the base axis, above --max-reach {max_reach:.2}",
                i + 1
            );
        }
        let via_raw = match target.get("via") {
            None | Some(Value::Null) => Vec::new(),
            Some(v) => v
                .as_array()
                .unwrap_or_else(|| {
                    panic!(
                        "marker {}: 'via' must be a list of at most {MAX_VIAS} [x, y, z] points",
                        i + 1
                    )
                })
                .clone(),
        };
        if via_raw.len() > MAX_VIAS {
            panic!(
                "marker {}: 'via' must be a list of at most {MAX_VIAS} [x, y, z] points",
                i + 1
            );
        }
        let mut marker_vias: Vec<V3> = Vec::with_capacity(via_raw.len());
        let mut marker_via_flanges: Vec<V3> = Vec::with_capacity(via_raw.len());
        for (vi, via) in via_raw.iter().enumerate() {
            let via_index = vi + 1;
            let mut vp = json_v3(via, &format!("marker {} via {via_index}", i + 1));
            if !is_tip {
                vp += V3::new(0.0, 0.0, hover_m);
            }
            if vp.z < min_z - 1e-9 {
                panic!(
                    "marker {} via {via_index}: tip z {:.4} is below the minimum {min_z:.4}",
                    i + 1,
                    vp.z
                );
            }
            if vp.z > max_z {
                panic!(
                    "marker {} via {via_index}: tip z {:.3} is above --max-z {max_z:.3}",
                    i + 1,
                    vp.z
                );
            }
            let vr = (vp.x * vp.x + vp.y * vp.y).sqrt();
            if vr > max_reach {
                panic!(
                    "marker {} via {via_index}: point is {vr:.3} m from the base axis, above --max-reach {max_reach:.2}",
                    i + 1
                );
            }
            marker_vias.push(vp);
            marker_via_flanges.push(vp + V3::new(0.0, 0.0, tip_offset_m));
        }
        points.push(tip_point);
        hovers.push(if is_tip { 0.0 } else { hover_m });
        tip_offsets.push(tip_offset_m);
        flange_points.push(flange_point);
        vias.push(marker_vias);
        via_flange_points.push(marker_via_flanges);
    }

    let mut pl = Planner::new(&ctx, hz, q_start, gripper_start, ik_pos_tol, ik_angle_tol);

    let mut gripper = cfg.motor7_open;
    if (gripper_start - cfg.motor7_open).abs() > 1e-6 {
        pl.t += pl.period;
        pl.append(&q_start, gripper_start);
        pl.gripper_leg(cfg.motor7_open, 0.8_f64.max(hold_s));
    }

    let mut unpark = [0.0; 6];
    unpark[1] = unpark_j2.to_radians();
    unpark[2] = unpark_j3.to_radians();
    let unparked = unpark.iter().any(|v| v.abs() > 1e-9);
    let mut seed = q_start;
    if unparked {
        seed = pl.joint_leg(&q_start, &unpark, unpark_s, gripper);
        let ee = ctx.pose(&seed);
        r0 = *ee.rotation.to_rotation_matrix().matrix();
    }
    let start_seed = seed;
    let odd_close = close_angle(&cfg, close_width_mm, grasp_q);

    let mut marker_q_ik: Vec<[f64; 6]> = Vec::with_capacity(points.len());
    let mut via_q_ik: Vec<Vec<[f64; 6]>> = vec![Vec::new(); points.len()];
    let mut marker_holds: Vec<(f64, f64)> = Vec::with_capacity(points.len());
    let mut yaw_degrees: Vec<f64> = Vec::with_capacity(points.len());

    for (i, point) in points.iter().enumerate() {
        let index = i + 1;
        let place = flange_points[i];
        let yaw_deg = point.y.atan2(point.x).to_degrees();
        yaw_degrees.push((yaw_deg * 1000.0).round_ties_even() / 1000.0);
        let r_marker = work_orientation(&r0, &sequence_axis, yaw_deg);
        let label = format!("marker {index}");
        let target_q = pl.ik_with_fallback(&place, &r_marker, &seed, &start_seed, &label);
        marker_q_ik.push(target_q);
        let current_q = pl.last_q6();

        let arrival_q = if index % 2 == 1 {
            let radius = (point.x * point.x + point.y * point.y).sqrt();
            if radius <= approach {
                panic!(
                    "marker {index}: target is {radius:.3} m from the base axis, inside the {:.0} mm approach standoff: no room to stage in front of it",
                    approach * 1000.0
                );
            }
            let stage_point = stage_in_front(point, approach);
            let stage_place = stage_point + V3::new(0.0, 0.0, tip_offsets[i]);
            let slabel = format!("stage {index}");
            pl.ik_with_fallback(&stage_place, &r_marker, &seed, &start_seed, &slabel)
        } else {
            target_q
        };

        let mut via_qs: Vec<[f64; 6]> = Vec::with_capacity(via_flange_points[i].len());
        for (vi, via_place) in via_flange_points[i].iter().enumerate() {
            let vlabel = format!("via {index}.{}", vi + 1);
            let via_q = pl.ik_with_fallback(via_place, &r_marker, &seed, &start_seed, &vlabel);
            via_qs.push(via_q);
            via_q_ik[i].push(via_q);
        }

        if !via_qs.is_empty() {
            let mut knots: Vec<[f64; 6]> = vec![current_q];
            knots.extend(via_qs.iter().copied());
            knots.push(arrival_q);
            seed = pl.chain_leg(&knots, index, gripper);
        } else if index % 2 == 1 {
            if !ctx.path_safe(&current_q, &arrival_q) {
                panic!(
                    "the leg to the staging point for marker {index} dips within a link radius of the worktop; space the markers further apart or move the arm clear"
                );
            }
            let mut d = [0.0; 6];
            for j in 0..6 {
                d[j] = arrival_q[j] - current_q[j];
            }
            seed = pl.joint_leg(
                &current_q,
                &d,
                seconds_for(&current_q, &arrival_q, &cfg),
                gripper,
            );
        } else {
            if !ctx.path_safe(&current_q, &target_q) {
                panic!(
                    "the leg to marker {index} dips within a link radius of the worktop; space the markers further apart or move the arm clear"
                );
            }
            let mut d = [0.0; 6];
            for j in 0..6 {
                d[j] = target_q[j] - current_q[j];
            }
            seed = pl.joint_leg(
                &current_q,
                &d,
                seconds_for(&current_q, &target_q, &cfg),
                gripper,
            );
        }
        if index % 2 == 1 {
            let current_q = pl.last_q6();
            if !ctx.path_safe(&current_q, &target_q) {
                panic!(
                    "the slide-in to marker {index} dips within a link radius of the worktop; raise the marker or move the arm clear"
                );
            }
            let mut d = [0.0; 6];
            for j in 0..6 {
                d[j] = target_q[j] - current_q[j];
            }
            seed = pl.joint_leg(
                &current_q,
                &d,
                seconds_for(&current_q, &target_q, &cfg),
                gripper,
            );
        }
        let hold_t0 = pl.t;
        pl.hold_g(0.25, gripper);
        let next_gripper = if index % 2 == 1 {
            odd_close
        } else {
            cfg.motor7_open
        };
        if (next_gripper - gripper).abs() > 1e-6 {
            pl.gripper_leg(next_gripper, 0.8_f64.max(hold_s));
        }
        gripper = next_gripper;
        pl.hold_g(0.25, gripper);
        marker_holds.push((hold_t0, pl.t));
    }

    let q_last = pl.last_q6();
    if !ctx.path_safe(&q_last, &q_start) {
        panic!(
            "the return from marker {} to the start pose dips within a link radius of the worktop; move the arm clear and retry",
            points.len()
        );
    }
    let mut d = [0.0; 6];
    for j in 0..6 {
        d[j] = q_start[j] - q_last[j];
    }
    pl.joint_leg(&q_last, &d, seconds_for(&q_last, &q_start, &cfg), gripper);

    let compensated = compensate(&ctx, &pl.waypoints, &snap, &kp_spec, &kp);
    let close_text = match close_width_mm {
        None => format!("close to {odd_close:.2} rad"),
        Some(w) => format!("close to {w:.1} mm"),
    };
    let total_vias: usize = vias.iter().map(|v| v.len()).sum();
    let mut steps = format!(
        "{} markers: stage {:.0} mm in front and slide in on odd, direct on even; gripper open, {close_text} on odd, open on even; return",
        points.len(),
        approach * 1000.0
    );
    if total_vias > 0 {
        steps += &format!("; {total_vias} via points passed through without stopping");
    }

    let targets_prov: Vec<Value> = points
        .iter()
        .enumerate()
        .map(|(i, point)| {
            json!({
                "point": arr3(point),
                "flange_point": arr3(&flange_points[i]),
                "hover_m": hovers[i],
                "tip_offset_m": tip_offsets[i],
                "surface_point": tarr[i].get("surface_point").cloned().unwrap_or(Value::Null),
                "tap_height_m": tarr[i].get("tap_height_m").cloned().unwrap_or(Value::Null),
                "capture": tarr[i].get("capture").cloned().unwrap_or(Value::Null),
                "source": tarr[i].get("source").cloned().unwrap_or(Value::Null),
                "tap": tarr[i].get("tap").cloned().unwrap_or(Value::Null),
                "via": vias[i].iter().map(arr3).collect::<Vec<_>>(),
                "via_flange_points": via_flange_points[i].iter().map(arr3).collect::<Vec<_>>(),
                "via_q_ik": via_q_ik[i].iter().map(|q| q.to_vec()).collect::<Vec<_>>(),
                "q_ik": marker_q_ik[i].to_vec(),
                "hold_t_start": marker_holds[i].0,
                "hold_t_end": marker_holds[i].1,
            })
        })
        .collect();
    let mut provenance = json!({
        "sequence": true,
        "hover_m": opt_hover,
        "tip_m": opt_tip,
        "targets": targets_prov,
        "tool_axis": sequence_axis,
        "yaw_deg": yaw,
        "marker_yaw_deg": yaw_degrees,
        "gripper": {
            "open": cfg.motor7_open,
            "half": cfg.motor7_half(),
            "closed": cfg.motor7_closed,
            "odd_close": odd_close
        },
    });
    if total_vias > 0 {
        provenance["via_corner_radius_rad"] = json!(cfg.corner_radius_rad);
    }
    let extra = json!({
        "capture": provenance,
        "tool_axis": sequence_axis,
        "yaw_deg": yaw,
        "marker_yaw_deg": yaw_degrees,
        "dynamics": {
            "source": "rebot_plan_sequence (Rust CLIK + gravity port of reBotArm_control_py)",
            "tau": "gravity torque of the planned pose, per waypoint",
        },
    });
    let (payload, stats) = build_payload(
        &compensated,
        hz,
        &limits7(&cfg),
        &steps,
        extra,
        None,
        None,
        true,
        &cfg,
    );

    let tip_m_summary = if !tip_offsets.is_empty()
        && tip_offsets
            .iter()
            .all(|v| (v - tip_offsets[0]).abs() < 1e-9)
    {
        json!(tip_offsets[0])
    } else {
        Value::Null
    };
    let summary = json!({
        "markers": points.len(),
        "targets": points.iter().map(arr3).collect::<Vec<_>>(),
        "flange_points": flange_points.iter().map(arr3).collect::<Vec<_>>(),
        "q_ik": marker_q_ik.iter().map(|q| q.to_vec()).collect::<Vec<_>>(),
        "via_counts": vias.iter().map(|v| v.len()).collect::<Vec<_>>(),
        "via_points": total_vias,
        "via_q_ik": via_q_ik.iter().map(|qs| qs.iter().map(|q| q.to_vec()).collect::<Vec<_>>()).collect::<Vec<_>>(),
        "radii": points.iter().map(|p| (p.x * p.x + p.y * p.y).sqrt()).collect::<Vec<_>>(),
        "hover_m": opt_hover,
        "hovers_m": hovers,
        "tap_heights_m": tarr.iter().map(|t| t.get("tap_height_m").cloned().unwrap_or(Value::Null)).collect::<Vec<_>>(),
        "tip_offsets_m": tip_offsets,
        "tip_m": tip_m_summary,
        "start_tcp": arr3(&p_start),
        "waypoints": stats["waypoints"],
        "duration_s": stats["duration_s"],
        "worst_ik_m": pl.worst_residual,
        "worst_ik_at": if pl.worst_where.is_empty() { Value::Null } else { json!(pl.worst_where) },
        "peak_tau": stats["peak_tau"],
        "max_step_rad": stats["max_step_rad"],
        "unparked": unparked,
    });
    pgrx::JsonB(json!({ "payload": payload, "summary": summary }))
}

// ── /move trajectory builders ────────────────────────────────────────────── //

fn parse_limits(v: &Value, cfg: &Config) -> [[f64; 2]; 7] {
    let mut l = limits7(cfg);
    if let Some(arr) = v.as_array() {
        if arr.len() != 7 {
            panic!("limits must have 7 entries, got {}", arr.len());
        }
        for j in 0..7 {
            let pair = arr[j]
                .as_array()
                .unwrap_or_else(|| panic!("limits[{j}] must be [lo, hi]"));
            if pair.len() != 2 {
                panic!("limits[{j}] must be [lo, hi]");
            }
            l[j] = [
                json_finite(&pair[0], "limits.lo"),
                json_finite(&pair[1], "limits.hi"),
            ];
        }
    }
    l
}

fn q7(v: &Value) -> [f64; 7] {
    let a = v
        .as_array()
        .unwrap_or_else(|| panic!("expected 7 joint angles"));
    if a.len() != 7 {
        panic!("expected 7 joint angles, got {}", a.len());
    }
    let mut q = [0.0; 7];
    for j in 0..7 {
        q[j] = json_finite(&a[j], "q7");
    }
    q
}

fn wp7(t: f64, q: [f64; 7]) -> Wp {
    Wp {
        t,
        q,
        tau: [0.0; 7],
    }
}

fn joint_move_impl(
    qf: [f64; 7],
    qt: [f64; 7],
    seconds: f64,
    cfg: &Config,
    hz: f64,
    hold_s: f64,
    limits: &[[f64; 2]; 7],
    label: &str,
) -> (Value, Value) {
    let hz = validate_hz(hz);
    let seconds = validate_duration(seconds, "seconds");
    let hold_s = validate_duration(hold_s, "hold_s");
    if !qf.iter().chain(qt.iter()).all(|v| v.is_finite()) {
        panic!("joint angles must be finite");
    }
    let travel = max_diff(&qf, &qt);
    let seconds = if travel < 1e-6 {
        seconds.max(0.2)
    } else {
        seconds
    };
    let period = 1.0 / hz;
    let count = (seconds.max(0.2) * hz).round_ties_even().max(2.0) as usize;
    let hold = (hold_s * hz).round_ties_even().max(1.0) as usize;
    if 1 + count + hold > MAX_WAYPOINTS {
        panic!("joint move exceeds {MAX_WAYPOINTS} waypoints");
    }
    let mut wps = Vec::with_capacity(1 + count + hold);
    wps.push(wp7(0.0, qf));
    let mut t = 0.0;
    for index in 1..=count {
        t += period;
        let frac = ease_fraction(index as f64 / count as f64, count as f64 / hz, cfg);
        let mut q = [0.0; 7];
        for j in 0..7 {
            q[j] = qf[j] + (qt[j] - qf[j]) * frac;
        }
        wps.push(wp7(t, q));
    }
    for _ in 0..hold {
        t += period;
        wps.push(wp7(t, qt));
    }
    let steps = if label.is_empty() {
        format!("joint move {:.2} rad over {:.1} s", travel, seconds)
    } else {
        label.to_string()
    };
    build_payload(&wps, hz, limits, &steps, json!({}), None, None, false, cfg)
}

fn joint_sequence_impl(
    points: &[[f64; 7]],
    durations: &[f64],
    cfg: &Config,
    hz: f64,
    hold_s: f64,
    limits: &[[f64; 2]; 7],
    label: &str,
) -> (Value, Value) {
    let hz = validate_hz(hz);
    let hold_s = validate_duration(hold_s, "hold_s");
    if points.len() < 2 {
        panic!("a sequence needs at least two poses");
    }
    if durations.len() != points.len() - 1 {
        panic!(
            "{} durations for {} legs",
            durations.len(),
            points.len() - 1
        );
    }
    let period = 1.0 / hz;
    let hold = (hold_s.max(0.0) * hz).round_ties_even().max(1.0) as usize;
    let mut wps = vec![wp7(0.0, points[0])];
    let mut t = 0.0;
    for index in 1..points.len() {
        let (start, end) = (points[index - 1], points[index]);
        let dur = validate_duration(durations[index - 1], "leg duration");
        let count = (dur.max(0.2) * hz).round_ties_even().max(2.0) as usize;
        if wps.len() + count + hold > MAX_WAYPOINTS {
            panic!("joint sequence exceeds {MAX_WAYPOINTS} waypoints");
        }
        for step in 1..=count {
            t += period;
            let frac = ease_fraction(step as f64 / count as f64, count as f64 / hz, cfg);
            let mut q = [0.0; 7];
            for j in 0..7 {
                q[j] = start[j] + (end[j] - start[j]) * frac;
            }
            wps.push(wp7(t, q));
        }
        for _ in 0..hold {
            t += period;
            wps.push(wp7(t, end));
        }
    }
    let steps = if label.is_empty() {
        format!("{} poses, one pass", points.len())
    } else {
        label.to_string()
    };
    build_payload(&wps, hz, limits, &steps, json!({}), None, None, false, cfg)
}

/// Gravity sag per joint (`sweep.gravity_sag`): `clip(tau_g / kp, ±sag_limit_rad)`.
#[pg_extern(immutable, parallel_safe)]
fn rebot_sag_rad(urdf: &str, q6: Vec<f64>, kp_spec: &str, config: pgrx::JsonB) -> Vec<f64> {
    let cfg = Config::from_json(&config.0);
    let ctx = PlanCtx::new(urdf, cfg.clone());
    let kp = kp_vector(kp_spec);
    let tau = ctx.gravity(&q6);
    gravity_sag(&tau, &kp, &cfg).to_vec()
}

/// A joint-space move with a hold (`traj.joint_move`).
#[pg_extern(immutable, parallel_safe)]
fn rebot_joint_move(
    q_from: Vec<f64>,
    q_to: Vec<f64>,
    seconds: f64,
    config: pgrx::JsonB,
    hz: default!(Option<f64>, "NULL"),
    hold_s: default!(f64, 0.4),
    limits: default!(pgrx::JsonB, "null"),
    label: default!(&str, "''"),
) -> pgrx::JsonB {
    let cfg = Config::from_json(&config.0);
    if q_from.len() != 7 || q_to.len() != 7 {
        panic!("a joint move needs 7 angles at both ends");
    }
    let mut qf = [0.0; 7];
    let mut qt = [0.0; 7];
    qf.copy_from_slice(&q_from);
    qt.copy_from_slice(&q_to);
    let lim = parse_limits(&limits.0, &cfg);
    let (payload, _) = joint_move_impl(
        qf,
        qt,
        seconds,
        &cfg,
        hz.unwrap_or(cfg.default_hz),
        hold_s,
        &lim,
        label,
    );
    pgrx::JsonB(payload)
}

/// One trajectory walking through poses in order, holding at each (`traj.joint_sequence`).
#[pg_extern(immutable, parallel_safe)]
fn rebot_joint_sequence(
    poses: pgrx::JsonB,
    seconds: pgrx::JsonB,
    config: pgrx::JsonB,
    hz: default!(Option<f64>, "NULL"),
    hold_s: default!(f64, 0.4),
    limits: default!(pgrx::JsonB, "null"),
    label: default!(&str, "''"),
) -> pgrx::JsonB {
    let cfg = Config::from_json(&config.0);
    let parr = poses
        .0
        .as_array()
        .unwrap_or_else(|| panic!("poses must be an array"));
    let points: Vec<[f64; 7]> = parr.iter().map(q7).collect();
    if points.len() < 2 {
        panic!("a sequence needs at least two poses");
    }
    let durations: Vec<f64> = match seconds.0.as_array() {
        Some(a) => a.iter().map(|v| json_finite(v, "seconds")).collect(),
        None => {
            let s = seconds.0.as_f64().unwrap_or(0.2);
            vec![s; points.len() - 1]
        }
    };
    let lim = parse_limits(&limits.0, &cfg);
    let (payload, _) = joint_sequence_impl(
        &points,
        &durations,
        &cfg,
        hz.unwrap_or(cfg.default_hz),
        hold_s,
        &lim,
        label,
    );
    pgrx::JsonB(payload)
}

/// The `/move` compute core (`planner_server.move_one` trajectory build).
#[pg_extern(immutable, parallel_safe)]
fn rebot_move_build(
    urdf: &str,
    pose: pgrx::JsonB,
    targets: pgrx::JsonB,
    options: pgrx::JsonB,
    snap: pgrx::JsonB,
    config: pgrx::JsonB,
) -> pgrx::JsonB {
    let cfg = Config::from_json(&config.0);
    let pose = pose.0;
    let options = options.0;
    let snap = snap.0;
    let ctx = PlanCtx::new(urdf, cfg.clone());
    // Do NOT pre-clamp: check encoder excursion against the 20 mrad sag gate first!
    let (mut q_from, gripper_default) = read_posture(&pose, &cfg, false);
    let tarr = targets
        .0
        .as_array()
        .unwrap_or_else(|| panic!("targets must be an array of [6]"));
    if tarr.is_empty() {
        panic!("'targets' must be a non-empty list");
    }
    let mut targets6: Vec<[f64; 6]> = Vec::with_capacity(tarr.len());
    for item in tarr {
        let a = item
            .as_array()
            .unwrap_or_else(|| panic!("each target must be six joint angles"));
        if a.len() != 6 {
            panic!("every target must be six joint angles");
        }
        let mut q = [0.0; 6];
        for j in 0..6 {
            q[j] = json_finite(&a[j], "target");
            if q[j] < cfg.limits[j][0] || q[j] > cfg.limits[j][1] {
                panic!("a target is outside the joint limits: [{:.3}]", q[j]);
            }
        }
        targets6.push(q);
    }
    let gripper = match options.get("gripper") {
        Some(v) if !v.is_null() => json_finite(v, "option 'gripper'"),
        _ => gripper_default,
    };
    let kp_spec = opt_s(&options, "kp_spec", KP_DEFAULT).to_string();
    let kp = kp_vector(&kp_spec);
    let hold = opt_f(&options, "hold", 0.0).clamp(0.0, 30.0);

    // Clamp only a small start-pose sag (<= 20 mrad); a larger excursion is a fault.
    let excursion = (0..6)
        .map(|j| (cfg.limits[j][0] - q_from[j]).max(q_from[j] - cfg.limits[j][1]))
        .fold(0.0_f64, f64::max);
    if excursion > 0.02 {
        panic!(
            "start pose is {:.1} mrad outside joint limits (above 20 mrad sag clamp): inspect encoders before moving",
            1000.0 * excursion
        );
    }
    if excursion > 0.0 {
        for j in 0..6 {
            q_from[j] = q_from[j].clamp(cfg.limits[j][0], cfg.limits[j][1]);
        }
    }

    let lift6 = unpark_pose_impl(&q_from, &cfg);
    let unparked = (0..6).any(|j| (lift6[j] - q_from[j]).abs() > 1e-9);
    if unparked {
        targets6.insert(0, lift6);
    }

    let mut chain: Vec<[f64; 6]> = vec![q_from];
    chain.extend(targets6.iter().copied());
    for index in 0..chain.len() - 1 {
        if !ctx.path_safe(&chain[index], &chain[index + 1]) {
            panic!(
                "leg {} of the move dips within a link radius of the worktop; move the arm clear first",
                index + 1
            );
        }
    }

    let mut commands: Vec<[f64; 6]> = Vec::with_capacity(targets6.len());
    let mut applied: Vec<f64> = Vec::with_capacity(targets6.len());
    for target in &targets6 {
        let tau = ctx.gravity(target);
        let extra = sag_correction(target, &tau, &snap, &kp_spec, &cfg);
        let cmd = aim_with_ctx(&ctx, target, &kp, &extra);
        let mut clipped = [0.0; 6];
        for j in 0..6 {
            clipped[j] = cmd[j].clamp(cfg.limits[j][0], cfg.limits[j][1]);
        }
        let max_off = (0..6)
            .map(|j| (clipped[j] - target[j]).abs())
            .fold(0.0_f64, f64::max);
        applied.push((1000.0 * max_off * 10.0).round_ties_even() / 10.0);
        commands.push(clipped);
    }

    let mut cmd_chain: Vec<[f64; 6]> = vec![q_from];
    cmd_chain.extend(commands.iter().copied());
    let wanted: Vec<f64> = (0..cmd_chain.len() - 1)
        .map(|i| seconds_for(&cmd_chain[i], &cmd_chain[i + 1], &cfg))
        .collect();
    let given = options.get("seconds");
    let seconds: Vec<f64> = match given.and_then(|v| v.as_array()) {
        Some(arr) => {
            let mut g: Vec<f64> = arr.iter().map(|v| json_finite(v, "seconds")).collect();
            if unparked {
                g.insert(0, wanted[0]);
            }
            if g.len() != wanted.len() {
                panic!(
                    "'seconds' must be one per leg ({} got {})",
                    wanted.len(),
                    g.len()
                );
            }
            (0..wanted.len()).map(|i| wanted[i].max(g[i])).collect()
        }
        None => match given.and_then(|v| v.as_f64()) {
            Some(s) => {
                if !s.is_finite() {
                    panic!("seconds must be finite");
                }
                wanted.iter().map(|w| w.max(s)).collect()
            }
            None => wanted.clone(),
        },
    };
    let seconds: Vec<f64> = seconds
        .iter()
        .map(|s| s.clamp(cfg.min_seconds, cfg.max_seconds))
        .collect();

    let mut points: Vec<[f64; 7]> = Vec::with_capacity(cmd_chain.len());
    for pose6 in &cmd_chain {
        let mut p = [0.0; 7];
        p[..6].copy_from_slice(pose6);
        p[6] = gripper;
        points.push(p);
    }
    let mut q_from7 = [0.0; 7];
    q_from7[..6].copy_from_slice(&q_from);
    // Start from the arm's actual current gripper angle so motor 7 ramps smoothly!
    q_from7[6] = gripper_default;

    let lim = limits7(&cfg);
    let (payload, stats) = if targets6.len() == 1 {
        let label = format!(
            "calibration move to [{}]",
            targets6[0]
                .iter()
                .map(|v| format!("{v:+.2}"))
                .collect::<Vec<_>>()
                .join(", ")
        );
        joint_move_impl(
            q_from7,
            points[points.len() - 1],
            seconds[0],
            &cfg,
            cfg.default_hz,
            hold,
            &lim,
            &label,
        )
    } else {
        let label = format!("{} poses, one pass", targets6.len());
        joint_sequence_impl(&points, &seconds, &cfg, cfg.default_hz, hold, &lim, &label)
    };
    let total_s: f64 = seconds.iter().sum();
    let summary = json!({
        "poses": targets6.len(),
        "seconds": (total_s * 100.0).round_ties_even() / 100.0,
        "leg_seconds": seconds,
        "hold": hold,
        "sag_mrad": applied,
        "unparked": unparked,
        "waypoints": stats["waypoints"],
        "duration_s": stats["duration_s"],
    });
    pgrx::JsonB(json!({ "payload": payload, "summary": summary }))
}

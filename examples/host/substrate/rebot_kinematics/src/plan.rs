//! rebot_plan — the plan logic as a pure, memory-safe SQL function core.
//!
//! Port of plan_to_target.plan() (the hover branch the live gateway uses) and
//! its helpers, composing the validated kinematics/gravity/collision primitives.
//! The IK is a faithful port of reBotArm_control_py.solve_ik (damped-LS CLIK:
//! LOCAL-frame Jacobian, log6 error, adaptive damping, active joint-limit
//! constraints, backtracking line search) — NOT openrr/k's different solver, so
//! solutions match the Pinocchio reference. Pure jsonb -> jsonb: no database
//! access, testable outside Postgres. A refusal panics -> SQL ERROR -> rollback.
//!
//! Scope: single-target hover plan (the gateway's tap path). The CLI-only
//! Cartesian pick branch and plan_sequence are follow-ups.

use pgrx::prelude::*;
use serde_json::{json, Value};
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3, Matrix3, Vector6, Matrix6, DMatrix};
use crate::{load_chain_arm, joint_axis, gravity_torques_impl, path_safe_impl, ARM_JOINTS, Config};

type V3 = Vector3<f64>;
type M3 = Matrix3<f64>;

const MIT_POSITION_LIMIT: f64 = 4.0 * std::f64::consts::PI;

fn json_v3(v: &Value) -> V3 {
    let a = v.as_array().unwrap_or_else(|| panic!("expected [x, y, z]"));
    V3::new(a[0].as_f64().unwrap(), a[1].as_f64().unwrap(), a[2].as_f64().unwrap())
}
fn arr3(v: &V3) -> Value { json!([v.x, v.y, v.z]) }

// ── orientation helpers (ports of plan_to_target) ────────────────────────── //

fn rotz(deg: f64) -> M3 {
    let (c, s) = (deg.to_radians().cos(), deg.to_radians().sin());
    M3::new(c, -s, 0.0,  s, c, 0.0,  0.0, 0.0, 1.0)
}

fn rotation_between(a: &V3, b: &V3) -> M3 {
    let a = a.normalize();
    let b = b.normalize();
    let cross = a.cross(&b);
    let sine = cross.norm();
    let cosine = a.dot(&b).clamp(-1.0, 1.0);
    if sine < 1e-9 {
        if cosine > 0.0 { return M3::identity(); }
        let helper = if a.x.abs() < 0.9 { V3::new(1.0, 0.0, 0.0) } else { V3::new(0.0, 1.0, 0.0) };
        let axis = a.cross(&helper).normalize();
        return 2.0 * axis * axis.transpose() - M3::identity();
    }
    let axis = cross / sine;
    let skew = M3::new(0.0, -axis.z, axis.y,  axis.z, 0.0, -axis.x,  -axis.y, axis.x, 0.0);
    let angle = sine.atan2(cosine);
    M3::identity() + angle.sin() * skew + (1.0 - angle.cos()) * (skew * skew)
}

fn tool_axis_vec(name: &str) -> Option<(usize, f64)> {
    match name {
        "x+" => Some((0, 1.0)), "x-" => Some((0, -1.0)),
        "y+" => Some((1, 1.0)), "y-" => Some((1, -1.0)),
        "z+" => Some((2, 1.0)), "z-" => Some((2, -1.0)),
        _ => None,
    }
}

fn work_orientation(r0: &M3, tool_axis: &str, yaw_deg: f64) -> M3 {
    if tool_axis == "current" { return *r0; }
    let (index, sign) = tool_axis_vec(tool_axis).unwrap_or_else(|| panic!("bad tool_axis {tool_axis}"));
    let mut local = V3::zeros();
    local[index] = sign;
    let approach = r0 * local;
    let aligned = rotation_between(&approach, &V3::new(0.0, 0.0, -1.0)) * r0;
    rotz(yaw_deg) * aligned
}

#[allow(dead_code)] // used by the Cartesian pick branch (follow-up)
fn blend_rotation(a: &M3, b: &M3, fraction: f64) -> M3 {
    let m = a.transpose() * b;
    let angle = ((m.trace() - 1.0) / 2.0).clamp(-1.0, 1.0).acos();
    if angle < 1e-9 { return *a; }
    let axis = V3::new(m[(2,1)] - m[(1,2)], m[(0,2)] - m[(2,0)], m[(1,0)] - m[(0,1)])
        / (2.0 * angle.sin());
    let skew = M3::new(0.0, -axis.z, axis.y,  axis.z, 0.0, -axis.x,  -axis.y, axis.x, 0.0);
    let step = angle * fraction;
    a * (M3::identity() + step.sin() * skew + (1.0 - step.cos()) * (skew * skew))
}

fn stage_in_front(point: &V3, approach: f64) -> V3 {
    let radius = (point.x * point.x + point.y * point.y).sqrt();
    if radius <= approach {
        panic!("target is {radius:.3} m from the base axis, inside the {:.0} mm approach standoff: \
                no room to stage in front of it", approach * 1000.0);
    }
    point - V3::new(point.x, point.y, 0.0) * (approach / radius)
}

fn close_angle(cfg: &Config, close_width_mm: Option<f64>, default: f64) -> f64 {
    match close_width_mm {
        None => default,
        Some(width) => {
            let max_width = cfg.gripper_max_width_mm();
            if !(0.0..=max_width).contains(&width) {
                panic!("close_width_mm {width:.1} is outside the jaw stroke [0, {max_width:.1}] mm");
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
    if u <= r { vc * u * u / (2.0 * r) }
    else if u >= 1.0 - r { 1.0 - vc * (1.0 - u).powi(2) / (2.0 * r) }
    else { vc * (u - 0.5 * r) }
}

fn seconds_for(a: &[f64], b: &[f64], cfg: &Config) -> f64 {
    let travel = a.iter().zip(b.iter()).map(|(x, y)| (y - x).abs()).fold(0.0_f64, f64::max);
    let peak_time = travel / cfg.peak_speed + cfg.ramp_s;
    let cruise_cap = travel / cfg.min_cruise_speed + cfg.ramp_s;
    cfg.min_seconds.max(cfg.max_seconds.min(peak_time).min(cruise_cap))
}

fn gripper_seconds(travel: f64, cfg: &Config) -> f64 {
    if !travel.is_finite() { return 0.0; }
    (std::f64::consts::PI / 2.0) * travel.abs() / cfg.gripper_peak_speed
}

fn kp_vector(spec: &str) -> [f64; 6] {
    let mut base = 50.0;
    let mut overrides = [None::<f64>; 6];
    for part in spec.split(',') {
        let part = part.trim();
        if part.is_empty() { continue; }
        if let Some((k, v)) = part.split_once(':') {
            if let Ok(idx) = k.trim().parse::<usize>() {
                if (1..=6).contains(&idx) { overrides[idx - 1] = v.trim().parse().ok(); }
            }
        } else if let Ok(b) = part.parse::<f64>() { base = b; }
    }
    let mut out = [base; 6];
    for i in 0..6 { if let Some(v) = overrides[i] { out[i] = v; } }
    out
}

fn gravity_sag(tau: &[f64], kp: &[f64; 6], cfg: &Config) -> [f64; 6] {
    let mut s = [0.0; 6];
    for j in 0..6 { s[j] = (tau[j] / kp[j]).clamp(-cfg.sag_limit_rad, cfg.sag_limit_rad); }
    s
}

fn sag_correction(q: &[f64], tau: &[f64], snap: &Value, kp_key: &str, cfg: &Config) -> [f64; 6] {
    let mut extra = [0.0; 6];
    let map = match snap.get("sag_map") { Some(m) if m.is_object() => m, _ => return extra };
    let fp = snap.get("rig_fingerprint").and_then(|v| v.as_str()).unwrap_or("");
    let zk = snap.get("zeros_key").and_then(|v| v.as_str()).unwrap_or("none");
    if map.get("kp_key").and_then(|v| v.as_str()).unwrap_or("") != kp_key { return extra; }
    if map.get("rig_fingerprint").and_then(|v| v.as_str()).unwrap_or("") != fp { return extra; }
    if map.get("zeros_key").and_then(|v| v.as_str()).unwrap_or("none") != zk { return extra; }
    let env = map.get("envelope").cloned().unwrap_or(json!({}));
    let q_min = env.get("q_min").and_then(|v| v.as_array());
    let q_max = env.get("q_max").and_then(|v| v.as_array());
    for j in 0..6 {
        if let Some(mn) = q_min.and_then(|a| a.get(j)).and_then(|v| v.as_f64()) {
            if q[j] < mn - 1e-9 { return extra; }
        }
        if let Some(mx) = q_max.and_then(|a| a.get(j)).and_then(|v| v.as_f64()) {
            if q[j] > mx + 1e-9 { return extra; }
        }
    }
    let per = match map.get("per_motor") { Some(p) => p, None => return extra };
    for j in 0..6 {
        let m = match per.get(&(j + 1).to_string()) { Some(m) => m, None => return extra };
        let a = m.get("a").and_then(|v| v.as_f64()).unwrap_or(0.0);
        let b = m.get("b").and_then(|v| v.as_f64()).unwrap_or(0.0);
        extra[j] = (a * tau[j] + b).clamp(-cfg.sag_limit_rad, cfg.sag_limit_rad);
    }
    extra
}

// ── the CLIK IK solver (faithful port of reBotArm_control_py.solve_ik) ───── //

struct PlanCtx {
    robot: urdf_rs::Robot,
    chain: k::Chain<f64>,
    urdf: String,
    cfg: Config,
}

impl PlanCtx {
    fn new(urdf: &str, cfg: Config) -> Self {
        let robot = urdf_rs::read_from_string(urdf).unwrap_or_else(|e| panic!("URDF parse failed: {e}"));
        let chain = k::Chain::<f64>::from(&robot);
        PlanCtx { robot, chain, urdf: urdf.to_string(), cfg }
    }

    fn pose(&self, q6: &[f64]) -> Isometry3<f64> {
        let mut q = q6.to_vec();
        while q.len() < self.chain.dof() { q.push(0.0); }
        self.chain.set_joint_positions(&q).unwrap_or_else(|e| panic!("set_joint_positions: {e}"));
        self.chain.update_transforms();
        let node = self.chain.find_link("gripper_end").unwrap_or_else(|| panic!("gripper_end not found"));
        let t = node.world_transform().unwrap();
        Isometry3::from_parts(t.translation, t.rotation)
    }

    fn gravity(&self, q6: &[f64]) -> [f64; 6] {
        let (_, ch) = load_chain_arm(&self.urdf, q6);
        let tau = gravity_torques_impl(&self.robot, &ch, &self.cfg);
        let mut out = [0.0; 6];
        out.copy_from_slice(&tau);
        out
    }

    // Generalized gravity from the FULL URDF inertials (matches Pinocchio RNEA,
    // which the reference plan emits as tau). The numpy LINK_INERTIALS table
    // (gravity()) is the approximation the reference uses for the aim command;
    // this one is the true model for the emitted feedforward torque.
    fn gravity_urdf(&self, q6: &[f64]) -> [f64; 6] {
        let mut q = q6.to_vec();
        while q.len() < self.chain.dof() { q.push(0.0); }
        self.chain.set_joint_positions(&q).unwrap();
        self.chain.update_transforms();
        // child_link -> parent_link map from the URDF joints.
        let mut parent: std::collections::HashMap<String, String> = std::collections::HashMap::new();
        let mut joint_child: std::collections::HashMap<String, String> = std::collections::HashMap::new();
        for j in &self.robot.joints {
            parent.insert(j.child.link.clone(), j.parent.link.clone());
            joint_child.insert(j.name.clone(), j.child.link.clone());
        }
        // ancestors(link): the set of links from its parent up to the root.
        let ancestors = |link: &str| -> std::collections::HashSet<String> {
            let mut set = std::collections::HashSet::new();
            let mut cur = link.to_string();
            while let Some(p) = parent.get(&cur) {
                set.insert(p.clone());
                cur = p.clone();
            }
            set
        };
        // links with mass, world COM.
        let mut bodies: Vec<(String, f64, V3)> = Vec::new();
        for l in &self.robot.links {
            let inertial = &l.inertial;
            let mass = inertial.mass.value;
            if mass <= 0.0 { continue; }
            let node = match self.chain.find_link(&l.name) { Some(n) => n, None => continue };
            let t = node.world_transform().unwrap();
            let com_local = V3::new(inertial.origin.xyz[0], inertial.origin.xyz[1], inertial.origin.xyz[2]);
            let com = t.rotation * com_local + t.translation.vector;
            bodies.push((l.name.clone(), mass, com));
        }
        let mut tau = [0.0; 6];
        for j in 0..6 {
            let node = self.chain.find(ARM_JOINTS[j]).unwrap();
            let t = node.world_transform().unwrap();
            let p_j = t.translation.vector;
            let z = t.rotation * joint_axis(&self.robot, ARM_JOINTS[j]);
            let child = joint_child.get(ARM_JOINTS[j]).cloned().unwrap_or_default();
            let mut total = 0.0;
            for (lname, mass, com) in &bodies {
                // downstream of joint j iff its child link is an ancestor of the
                // body (or the body is the child link itself).
                if lname != &child && !ancestors(lname).contains(&child) { continue; }
                let force = V3::new(0.0, 0.0, -mass * self.cfg.gravity);
                total += z.dot(&(com - p_j).cross(&force));
            }
            tau[j] = -total;
        }
        tau
    }

    // Body (LOCAL-frame) Jacobian of gripper_end, 6x6, columns [linear; angular].
    fn jacobian(&self, q6: &[f64], ee: &Isometry3<f64>) -> Matrix6<f64> {
        let mut q = q6.to_vec();
        while q.len() < self.chain.dof() { q.push(0.0); }
        self.chain.set_joint_positions(&q).unwrap();
        self.chain.update_transforms();
        let r_t = ee.rotation.to_rotation_matrix().matrix().transpose();
        let p_ee = ee.translation.vector;
        let mut j = Matrix6::zeros();
        for col in 0..6 {
            let node = self.chain.find(ARM_JOINTS[col]).unwrap();
            let t = node.world_transform().unwrap();
            let z = t.rotation * joint_axis(&self.robot, ARM_JOINTS[col]);
            let p_j = t.translation.vector;
            let lin_w = z.cross(&(p_ee - p_j));
            let lin = r_t * lin_w;
            let ang = r_t * z;
            for r in 0..3 { j[(r, col)] = lin[r]; }
            for r in 0..3 { j[(r + 3, col)] = ang[r]; }
        }
        j
    }

    fn path_safe(&self, a: &[f64], b: &[f64]) -> bool {
        let (_, ch) = load_chain_arm(&self.urdf, a);
        path_safe_impl(&ch, a, b, 40, &self.cfg)
    }
}

// log6 error: pin.log6(T_cur^-1 * T_target).vector, [linear; angular].
fn se3_error(cur: &Isometry3<f64>, target: &Isometry3<f64>) -> (f64, Vector6<f64>) {
    let rel = cur.inverse() * target;
    let omega = rel.rotation.scaled_axis();
    let t = rel.translation.vector;
    let theta = omega.norm();
    let v = if theta < 1e-12 {
        t
    } else {
        let u = omega / theta;
        let ux = M3::new(0.0, -u.z, u.y,  u.z, 0.0, -u.x,  -u.y, u.x, 0.0);
        // V_inv = I - (theta/2) ux + (1 - (theta/2) cot(theta/2)) ux^2
        let half = theta / 2.0;
        let c = 1.0 - half * (half.cos() / half.sin());
        let v_inv = M3::identity() - half * ux + c * (ux * ux);
        v_inv * t
    };
    let mut err = Vector6::zeros();
    for r in 0..3 { err[r] = v[r]; }
    for r in 0..3 { err[r + 3] = omega[r]; }
    (err.norm(), err)
}

struct IkResult { q: [f64; 6], success: bool }

fn solve_ik(ctx: &PlanCtx, target: &Isometry3<f64>, q_init: &[f64],
            max_iter: usize, tolerance: f64, damping: f64, step_size: f64) -> IkResult {
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
                q_new[i] = (q[i] + alpha * dq[i]).clamp(ctx.cfg.limits[i][0], ctx.cfg.limits[i][1]);
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
    IkResult { q, success: prev_err < tolerance }
}

fn damped_step_active_limits(q: &[f64], j: &Matrix6<f64>, err: &Vector6<f64>,
                             damping: f64, step_size: f64, cfg: &Config) -> Vector6<f64> {
    let mut free = [true; 6];
    let limit_eps = 1e-10;
    let mut dq = Vector6::zeros();
    for _ in 0..7 {
        let cols: Vec<usize> = (0..6).filter(|&i| free[i]).collect();
        if cols.is_empty() { return dq; }
        let mut j_free = DMatrix::<f64>::zeros(6, cols.len());
        for (c, &orig) in cols.iter().enumerate() {
            for r in 0..6 { j_free[(r, c)] = j[(r, orig)]; }
        }
        let mut system = &j_free * j_free.transpose();
        for r in 0..6 { system[(r, r)] += damping; }
        let x = system.lu().solve(err).unwrap_or_else(|| panic!("IK linear solve failed"));
        let dq_free = step_size * (j_free.transpose() * x);
        dq = Vector6::zeros();
        for (c, &orig) in cols.iter().enumerate() { dq[orig] = dq_free[c]; }
        let mut newly_blocked = [false; 6];
        for i in 0..6 {
            let (lo, hi) = (cfg.limits[i][0], cfg.limits[i][1]);
            let at_lower = q[i] <= lo + limit_eps;
            let at_upper = q[i] >= hi - limit_eps;
            if (at_lower && dq[i] < 0.0) || (at_upper && dq[i] > 0.0) {
                newly_blocked[i] = true;
            }
        }
        let mut any = false;
        for i in 0..6 {
            if newly_blocked[i] && free[i] { free[i] = false; any = true; }
        }
        if !any { return dq; }
    }
    dq
}

// ── trajectory assembly (port of traj.build / validate) ──────────────────── //

#[derive(Clone)]
struct Wp { t: f64, q: [f64; 7], tau: [f64; 7] }

fn round6(v: f64) -> f64 { (v * 1e6).round_ties_even() / 1e6 }

fn build_payload(waypoints: &[Wp], hz: f64, limits: &[[f64; 2]; 7], steps: &str,
                 extra: Value, hold_start: Option<f64>, hold_end: Option<f64>,
                 with_tau: bool, cfg: &Config) -> Value {
    let n = waypoints.len();
    // traj.waypoint rounds t to 6 decimals at creation; build derives v from the
    // rounded series. Match exactly so parity is bit-level on time/velocity.
    let times: Vec<f64> = waypoints.iter().map(|w| round6(w.t)).collect();
    let mut entries: Vec<Value> = Vec::with_capacity(n);
    for (i, wp) in waypoints.iter().enumerate() {
        let mut e = json!({ "t": times[i], "q": wp.q.to_vec() });
        if with_tau { e["tau"] = json!(wp.tau.to_vec()); }
        if n >= 2 {
            let (ai, bi) = if i == 0 { (1, 0) }
                else if i == n - 1 { (n - 1, n - 2) }
                else { (i + 1, i - 1) };
            let dt = times[ai] - times[bi];
            let at_rest = (i > 0 && max_diff(&wp.q, &waypoints[i - 1].q) <= 1e-9)
                || (i + 1 < n && max_diff(&wp.q, &waypoints[i + 1].q) <= 1e-9);
            let v: Vec<f64> = if dt <= 1e-9 || at_rest {
                vec![0.0; 7]
            } else {
                (0..7).map(|j| round6((waypoints[ai].q[j] - waypoints[bi].q[j]) / dt)).collect()
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
        for (k, v) in map { if !v.is_null() { payload[k] = v; } }
    }
    validate_payload(&payload, cfg);
    payload
}

fn max_diff(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b.iter()).map(|(x, y)| (x - y).abs()).fold(0.0_f64, f64::max)
}

fn validate_payload(p: &Value, cfg: &Config) -> Value {
    let wps = p["waypoints"].as_array().unwrap_or_else(|| panic!("no waypoints"));
    if wps.len() < 2 { panic!("need at least two waypoints, got {}", wps.len()); }
    let limits = p["limits"].as_array();
    let mut times: Vec<f64> = Vec::new();
    let mut peak_tau = 0.0_f64;
    let mut max_step = 0.0_f64;
    for (i, wp) in wps.iter().enumerate() {
        let q = wp["q"].as_array().unwrap();
        if q.len() != 7 { panic!("waypoint {i} has {} values for 7 motors", q.len()); }
        for v in q { if !v.as_f64().unwrap().is_finite() { panic!("waypoint {i} non-finite q"); } }
        let t = wp["t"].as_f64().unwrap();
        if !t.is_finite() { panic!("waypoint {i} non-finite time"); }
        times.push(t);
        if let Some(tau) = wp["tau"].as_array() {
            for v in tau { peak_tau = peak_tau.max(v.as_f64().unwrap().abs()); }
        }
        if let Some(lim) = limits {
            for j in 0..7 {
                let lo = lim[j][0].as_f64().unwrap();
                let hi = lim[j][1].as_f64().unwrap();
                let qj = q[j].as_f64().unwrap();
                if qj < lo - cfg.limit_tol || qj > hi + cfg.limit_tol {
                    panic!("joint {} leaves its limit [{lo:.3}, {hi:.3}]: {qj:.3}", j + 1);
                }
            }
        }
        for j in 0..7 {
            if q[j].as_f64().unwrap().abs() > MIT_POSITION_LIMIT + cfg.limit_tol {
                panic!("joint {} at waypoint {i} outside the MIT range", j + 1);
            }
        }
        if i > 0 {
            let prev = wps[i - 1]["q"].as_array().unwrap();
            let step = (0..7).map(|j| (q[j].as_f64().unwrap() - prev[j].as_f64().unwrap()).abs())
                .fold(0.0_f64, f64::max);
            if step > cfg.max_joint_step {
                panic!("waypoints {}->{i} jump {step:.3} rad, above the {:.2} sanity limit", i - 1, cfg.max_joint_step);
            }
            max_step = max_step.max(step);
            if times[i] < times[i - 1] { panic!("waypoint times are not monotonic"); }
        }
    }
    json!({
        "waypoints": wps.len(),
        "duration_s": times[times.len() - 1] - times[0],
        "peak_tau": peak_tau,
        "max_step_rad": max_step,
    })
}

// ── the hover plan (port of plan_to_target.plan, options.hover = True) ───── //

fn opt_f(options: &Value, key: &str, default: f64) -> f64 {
    options.get(key).and_then(|v| v.as_f64()).unwrap_or(default)
}
fn opt_b(options: &Value, key: &str, default: bool) -> bool {
    options.get(key).and_then(|v| v.as_bool()).unwrap_or(default)
}
fn opt_s<'a>(options: &'a Value, key: &str, default: &'a str) -> &'a str {
    options.get(key).and_then(|v| v.as_str()).unwrap_or(default)
}

struct Planner<'a> {
    ctx: &'a PlanCtx,
    hz: f64,
    period: f64,
    t: f64,
    waypoints: Vec<Wp>,
    worst_residual: f64,
    worst_where: String,
    ik_pos_tol: f64,
    ik_angle_tol: f64,
}

impl<'a> Planner<'a> {
    fn append(&mut self, q6: &[f64], gripper: f64) {
        let tau = self.ctx.gravity(q6);
        let mut q = [0.0; 7];
        q[..6].copy_from_slice(q6);
        q[6] = gripper;
        let mut tau7 = [0.0; 7];
        tau7[..6].copy_from_slice(&tau);
        self.waypoints.push(Wp { t: self.t, q, tau: tau7 });
    }

    fn last_q6(&self) -> [f64; 6] {
        let mut q = [0.0; 6];
        q.copy_from_slice(&self.waypoints.last().unwrap().q[..6]);
        q
    }

    fn ik_to(&mut self, place: &V3, rotation: &M3, seed: &[f64], label: &str) -> [f64; 6] {
        let target = Isometry3::from_parts(Translation3::from(*place),
            UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix_unchecked(*rotation)));
        let res = solve_ik(self.ctx, &target, seed, 300, 1e-6, 1e-8, 1.0);
        let reached = self.ctx.pose(&res.q);
        let position_error = (reached.translation.vector - place).norm();
        let cosine = ((reached.rotation.to_rotation_matrix().matrix().transpose() * rotation).trace() - 1.0) / 2.0;
        let angle_error = cosine.clamp(-1.0, 1.0).acos().to_degrees();
        let accepted = res.success || (position_error <= self.ik_pos_tol && angle_error <= self.ik_angle_tol);
        if !accepted {
            panic!("IK failed at [{:.3} {:.3} {:.3}] m ({label}): no solution; reached {position_error:.4} m / \
                    {angle_error:.2} deg off. The target is probably out of reach from this pose: try a smaller \
                    --yaw, a larger --approach, or start from a pose closer to the workspace.",
                place.x, place.y, place.z);
        }
        if position_error > self.worst_residual {
            self.worst_residual = position_error;
            self.worst_where = label.to_string();
        }
        for j in 0..6 {
            if res.q[j] < self.ctx.cfg.limits[j][0] - self.ctx.cfg.limit_tol
                || res.q[j] > self.ctx.cfg.limits[j][1] + self.ctx.cfg.limit_tol {
                panic!("IK returned joint {} = {:.3} rad at {label}, outside its limits [{:.3}, {:.3}]",
                    j + 1, res.q[j], self.ctx.cfg.limits[j][0], self.ctx.cfg.limits[j][1]);
            }
        }
        res.q
    }

    fn joint_leg(&mut self, base: &[f64], delta: &[f64], seconds: f64, gripper: f64) -> [f64; 6] {
        let count = (seconds * self.hz).round_ties_even().max(1.0) as usize;
        for index in 1..=count {
            self.t += self.period;
            let fraction = ease_fraction(index as f64 / count as f64, count as f64 / self.hz, &self.ctx.cfg);
            let mut q = [0.0; 6];
            for j in 0..6 { q[j] = base[j] + delta[j] * fraction; }
            self.append(&q, gripper);
        }
        self.last_q6()
    }

    fn hold(&mut self, seconds: f64) {
        let count = (seconds * self.hz).round_ties_even().max(1.0) as usize;
        let last = self.waypoints.last().unwrap().clone();
        for _ in 0..count {
            self.t += self.period;
            self.waypoints.push(Wp { t: self.t, q: last.q, tau: last.tau });
        }
    }

    // Sequence hold: re-append at the current pose with the given gripper angle
    // (plan_sequence.hold recomputes gravity via append; q[:6] is constant so the
    // torque matches the carried value, but the gripper angle is explicit).
    fn hold_g(&mut self, seconds: f64, gripper: f64) {
        let count = (seconds * self.hz).round_ties_even().max(1.0) as usize;
        let q6 = self.last_q6();
        for _ in 0..count {
            self.t += self.period;
            self.append(&q6, gripper);
        }
    }

    fn gripper_leg(&mut self, target_angle: f64, seconds: f64) {
        let current = self.waypoints.last().unwrap().q[6];
        let seconds = seconds.max(gripper_seconds(target_angle - current, &self.ctx.cfg));
        let count = (seconds * self.hz).round_ties_even().max(1.0) as usize;
        let last = self.waypoints.last().unwrap().clone();
        for index in 1..=count {
            self.t += self.period;
            let angle = current + (target_angle - current)
                * ease_fraction(index as f64 / count as f64, count as f64 / self.hz, &self.ctx.cfg);
            let mut q = last.q;
            q[6] = angle;
            self.waypoints.push(Wp { t: self.t, q, tau: last.tau });
        }
    }
}

/// The plan core. dump = {ids, q}; target = rebot-target {point, point_frame,
/// tip_offset_m, hover_m, ...}; options = the planner Options subset; snap =
/// {rig_fingerprint, zeros_key, sag_map?} for sag compensation (null map = model only).
/// Returns {payload, summary}. Pure: no database access.
#[pg_extern]
fn rebot_plan(urdf: &str, dump: pgrx::JsonB, target: pgrx::JsonB,
              options: pgrx::JsonB, snap: pgrx::JsonB, config: pgrx::JsonB) -> pgrx::JsonB {
    let cfg = Config::from_json(&config.0);
    let dump = dump.0;
    let target = target.0;
    let options = options.0;
    let snap = snap.0;
    let ctx = PlanCtx::new(urdf, cfg.clone());

    // posture(dump)
    let ids = dump["ids"].as_array().unwrap_or_else(|| panic!("dump needs ids"));
    let ids6: Vec<i64> = ids.iter().take(6).map(|v| v.as_i64().unwrap()).collect();
    if ids6 != vec![1, 2, 3, 4, 5, 6] { panic!("expected motor ids 1..6 first, got {ids6:?}"); }
    let qdump = dump["q"].as_array().unwrap_or_else(|| panic!("dump needs q"));
    let mut q_start = [0.0; 6];
    for j in 0..6 { q_start[j] = qdump[j].as_f64().unwrap(); }
    let gripper_start = qdump.get(6).and_then(|v| v.as_f64()).unwrap_or(cfg.motor7_closed);
    for j in 0..6 { q_start[j] = q_start[j].clamp(cfg.limits[j][0], cfg.limits[j][1]); }

    let ee_start = ctx.pose(&q_start);
    let p0 = ee_start.translation.vector;
    let r0 = ee_start.rotation.to_rotation_matrix().matrix().clone();
    let p_start = p0;

    // target geometry
    if target.get("format").and_then(|v| v.as_str()) != Some("rebot-target") {
        panic!("not a rebot-target file");
    }
    let point = json_v3(&target["point"]);
    let hover_m = target.get("hover_m").and_then(|v| v.as_f64()).unwrap_or(opt_f(&options, "hover_m", cfg.default_min_z));
    let tip_offset_m = target.get("tip_offset_m").and_then(|v| v.as_f64()).unwrap_or(opt_f(&options, "tip_m", 0.0));
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
        panic!("target z {:.4} is below the minimum {floor:.4}: refusing to plan into the table", judged.z);
    }
    if judged.z > max_z { panic!("target z {:.3} is above --max-z {max_z:.3}", judged.z); }
    if radius > max_reach { panic!("target is {radius:.3} m from the base axis, above --max-reach {max_reach:.2}"); }

    let tool_axis = opt_s(&options, "tool_axis", "current").to_string();
    let yaw = opt_f(&options, "yaw", 0.0);
    let approach = opt_f(&options, "approach", 0.06);
    let hz = opt_f(&options, "hz", 30.0);
    let hold_s = opt_f(&options, "hold_s", 0.6);
    let no_return = opt_b(&options, "no_return", false);
    let unpark_j2 = opt_f(&options, "unpark_j2", 0.0);
    let unpark_j3 = opt_f(&options, "unpark_j3", 8.0);
    let unpark_s = opt_f(&options, "unpark_s", 2.0);
    let ik_pos_tol = opt_f(&options, "ik_pos_tol", 0.002).max(0.010);
    let ik_angle_tol = opt_f(&options, "ik_angle_tol", 0.5);
    let kp_spec = opt_s(&options, "kp_spec", "50,2:150,3:150").to_string();
    let kp = kp_vector(&kp_spec);

    let mut p0 = p0;
    let mut r0 = r0;
    let mut r_work = work_orientation(&r0, &tool_axis, yaw);
    let pre = stage_in_front(&point, approach);
    let mut unpark = [0.0; 6];
    unpark[1] = unpark_j2.to_radians();
    unpark[2] = unpark_j3.to_radians();
    let unparked = unpark.iter().any(|v| v.abs() > 1e-9);

    let mut pl = Planner {
        ctx: &ctx, hz, period: 1.0 / hz, t: 0.0, waypoints: Vec::new(),
        worst_residual: 0.0, worst_where: String::new(), ik_pos_tol, ik_angle_tol,
    };

    let mut seed = q_start;
    if unparked {
        seed = pl.joint_leg(&q_start, &unpark, unpark_s, gripper_start);
        let ee = ctx.pose(&seed);
        p0 = ee.translation.vector;
        r0 = ee.rotation.to_rotation_matrix().matrix().clone();
        r_work = work_orientation(&r0, &tool_axis, yaw);
    }

    let (report_axis, report_yaw, hold_start, hold_end) = if hover {
        let hover_point = flange_point;
        let hover_axis = if tool_axis == "current" { "z-".to_string() } else { tool_axis.clone() };
        let hover_yaw = if tool_axis == "current" { point.y.atan2(point.x).to_degrees() } else { yaw };
        let r_hover = work_orientation(&r0, &hover_axis, hover_yaw);
        let start_seed = seed;
        let target_q = match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| pl.ik_to(&hover_point, &r_hover, &seed, "hover"))) {
            Ok(q) => q,
            Err(_) => pl.ik_to(&hover_point, &r_hover, &start_seed, "hover"),
        };
        let current_q = pl.last_q6();
        if !ctx.path_safe(&current_q, &target_q) {
            panic!("the leg to the tap dips within a link radius of the worktop; tap a point further out or move the arm clear");
        }
        let mut delta = [0.0; 6];
        for j in 0..6 { delta[j] = target_q[j] - current_q[j]; }
        pl.joint_leg(&current_q, &delta, seconds_for(&current_q, &target_q, &cfg), gripper_start);
        let hs = pl.t;
        pl.hold(hold_s);
        let he = pl.t;
        if !no_return {
            let q_last = pl.last_q6();
            if !ctx.path_safe(&q_last, &start_seed) {
                panic!("the return to the start pose dips within a link radius of the worktop; move the arm clear and retry");
            }
            let mut d = [0.0; 6];
            for j in 0..6 { d[j] = start_seed[j] - q_last[j]; }
            pl.joint_leg(&q_last, &d, seconds_for(&q_last, &start_seed, &cfg), gripper_start);
        }
        (hover_axis, hover_yaw, hs, he)
    } else {
        panic!("rebot_plan currently implements the hover branch (the gateway tap path); \
                the Cartesian pick branch is a follow-up");
    };
    let _ = r_work;
    let _ = pre;

    // compensate_waypoints: command the sag offset at emission (§7.3b).
    let compensated = compensate(&ctx, &pl.waypoints, &snap, &kp_spec, &kp);

    let steps = if is_tip {
        format!("tap target: move tip to [{:.3} {:.3} {:.3}]{}", point.x, point.y, point.z,
            if no_return { "" } else { ", then return" })
    } else {
        format!("tap target: hover {:.0} mm above [{:.3} {:.3} {:.3}]{}", hover_m * 1000.0,
            point.x, point.y, point.z, if no_return { "" } else { ", then return" })
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

    let limits7 = limits7(&cfg);

    let extra = json!({
        "capture": provenance,
        "tool_axis": report_axis,
        "yaw_deg": report_yaw,
        "dynamics": {
            "source": "rebot_plan (Rust CLIK + gravity port of reBotArm_control_py)",
            "tau": "gravity torque of the planned pose, per waypoint",
        },
    });
    let payload = build_payload(&compensated, hz, &limits7, &steps, extra,
        if hold_end > hold_start { Some(hold_start) } else { None },
        if hold_end > hold_start { Some(hold_end) } else { None }, true, &cfg);
    let stats = validate_payload(&payload, &cfg);

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

// aim() using the context's gravity (avoids the unreachable stub).
fn aim_with_ctx(ctx: &PlanCtx, target: &[f64], kp: &[f64; 6], extra: &[f64; 6]) -> [f64; 6] {
    let sag_at = |q: &[f64]| -> [f64; 6] {
        let tau = ctx.gravity(q);
        let g = gravity_sag(&tau, kp, &ctx.cfg);
        let mut t = [0.0; 6];
        for j in 0..6 { t[j] = (g[j] + extra[j]).clamp(-ctx.cfg.sag_limit_rad, ctx.cfg.sag_limit_rad); }
        t
    };
    let mut command = [0.0; 6];
    let s0 = sag_at(target);
    for j in 0..6 { command[j] = target[j] + s0[j]; }
    for _ in 0..2 {
        let s = sag_at(&command);
        for j in 0..6 { command[j] = target[j] + s[j]; }
    }
    command
}

// compensate_waypoints: command the sag offset at emission (§7.3b). The command
// uses the numpy gravity model (as the reference does); the emitted tau uses the
// full URDF RNEA gravity. Shared by the hover and sequence branches.
fn compensate(ctx: &PlanCtx, waypoints: &[Wp], snap: &Value, kp_spec: &str, kp: &[f64; 6]) -> Vec<Wp> {
    let mut out = Vec::with_capacity(waypoints.len());
    for wp in waypoints {
        let q6 = &wp.q[..6];
        let tau = ctx.gravity(q6);
        let extra = sag_correction(q6, &tau, snap, kp_spec, &ctx.cfg);
        let cmd = aim_with_ctx(ctx, q6, kp, &extra);
        let mut q = [0.0; 7];
        for j in 0..6 { q[j] = cmd[j].clamp(ctx.cfg.limits[j][0], ctx.cfg.limits[j][1]); }
        q[6] = wp.q[6];
        let g = ctx.gravity_urdf(&q[..6]);
        let mut tau7 = [0.0; 7];
        tau7[..6].copy_from_slice(&g);
        out.push(Wp { t: wp.t, q, tau: tau7 });
    }
    out
}

fn limits7(cfg: &Config) -> [[f64; 2]; 7] {
    let mut l = [[0.0; 2]; 7];
    for j in 0..6 { l[j] = [cfg.limits[j][0], cfg.limits[j][1]]; }
    l[6] = [-0.1, 5.0];
    l
}

fn read_posture(dump: &Value, cfg: &Config) -> ([f64; 6], f64) {
    let ids = dump["ids"].as_array().unwrap_or_else(|| panic!("dump needs ids"));
    let ids6: Vec<i64> = ids.iter().take(6).map(|v| v.as_i64().unwrap()).collect();
    if ids6 != vec![1, 2, 3, 4, 5, 6] { panic!("expected motor ids 1..6 first, got {ids6:?}"); }
    let qdump = dump["q"].as_array().unwrap_or_else(|| panic!("dump needs q"));
    let mut q = [0.0; 6];
    for j in 0..6 { q[j] = qdump[j].as_f64().unwrap().clamp(cfg.limits[j][0], cfg.limits[j][1]); }
    let gripper = qdump.get(6).and_then(|v| v.as_f64()).unwrap_or(cfg.motor7_closed);
    (q, gripper)
}

/// The multi-marker sequence core (port of plan_to_target.plan_sequence) — the
/// gateway's gripper="alternate" path. Endpoint-first per marker (one IK solve,
/// then joint-space interpolation), gripper open/close on odd/even markers,
/// stage-and-slide on odd markers. Pure jsonb -> jsonb. `targets` is an array of
/// rebot-target objects. Returns {payload, summary}.
#[pg_extern]
fn rebot_plan_sequence(urdf: &str, dump: pgrx::JsonB, targets: pgrx::JsonB,
                       options: pgrx::JsonB, snap: pgrx::JsonB, config: pgrx::JsonB) -> pgrx::JsonB {
    let cfg = Config::from_json(&config.0);
    let dump = dump.0;
    let targets = targets.0;
    let options = options.0;
    let snap = snap.0;
    let tarr = targets.as_array().unwrap_or_else(|| panic!("targets must be an array"));
    if tarr.is_empty() { panic!("a marker sequence needs at least one target"); }
    if tarr.len() > 20 { panic!("a marker sequence is limited to 20 targets"); }
    let ctx = PlanCtx::new(urdf, cfg.clone());

    let (q_start, gripper_start) = read_posture(&dump, &cfg);
    let ee_start = ctx.pose(&q_start);
    let p_start = ee_start.translation.vector;
    let mut r0 = ee_start.rotation.to_rotation_matrix().matrix().clone();

    let tool_axis = opt_s(&options, "tool_axis", "current").to_string();
    let yaw = opt_f(&options, "yaw", 0.0);
    let approach = opt_f(&options, "approach", 0.06);
    let hz = opt_f(&options, "hz", 30.0);
    let hold_s = opt_f(&options, "hold_s", 0.6);
    let grasp_q = opt_f(&options, "grasp_q", 2.0);
    let close_width_mm = options.get("close_width_mm").and_then(|v| v.as_f64());
    let unpark_j2 = opt_f(&options, "unpark_j2", 0.0);
    let unpark_j3 = opt_f(&options, "unpark_j3", 8.0);
    let unpark_s = opt_f(&options, "unpark_s", 2.0);
    let ik_pos_tol = opt_f(&options, "ik_pos_tol", 0.002).max(0.010);
    let ik_angle_tol = opt_f(&options, "ik_angle_tol", 0.5);
    let min_z = opt_f(&options, "min_z", cfg.default_min_z);
    let max_z = opt_f(&options, "max_z", 0.95);
    let max_reach = opt_f(&options, "max_reach", 0.75);
    let kp_spec = opt_s(&options, "kp_spec", "50,2:150,3:150").to_string();
    let kp = kp_vector(&kp_spec);
    let sequence_axis = if tool_axis == "current" { "z-".to_string() } else { tool_axis.clone() };

    // Validate every marker before the first move; collect tip/flange points.
    let mut points: Vec<V3> = Vec::new();
    let mut hovers: Vec<f64> = Vec::new();
    let mut tip_offsets: Vec<f64> = Vec::new();
    let mut flange_points: Vec<V3> = Vec::new();
    for (i, target) in tarr.iter().enumerate() {
        if target.get("format").and_then(|v| v.as_str()) != Some("rebot-target") {
            panic!("marker {}: not a rebot-target file", i + 1);
        }
        let point = json_v3(&target["point"]);
        let hover_m = target.get("hover_m").and_then(|v| v.as_f64())
            .unwrap_or(opt_f(&options, "hover_m", cfg.default_min_z));
        let tip_offset_m = target.get("tip_offset_m").and_then(|v| v.as_f64())
            .unwrap_or(opt_f(&options, "tip_m", 0.0));
        let is_tip = target.get("point_frame").and_then(|v| v.as_str()) == Some("tip");
        let tip_point = if is_tip { point } else { point + V3::new(0.0, 0.0, hover_m) };
        let flange_point = tip_point + V3::new(0.0, 0.0, tip_offset_m);
        let radius = (point.x * point.x + point.y * point.y).sqrt();
        if tip_point.z < min_z - 1e-9 {
            panic!("marker {}: tip z {:.4} is below the minimum {min_z:.4}", i + 1, tip_point.z);
        }
        if tip_point.z > max_z { panic!("marker {}: tip z {:.3} is above --max-z {max_z:.3}", i + 1, tip_point.z); }
        if radius > max_reach {
            panic!("marker {}: target is {radius:.3} m from the base axis, above --max-reach {max_reach:.2}", i + 1);
        }
        points.push(tip_point);
        hovers.push(if is_tip { 0.0 } else { hover_m });
        tip_offsets.push(tip_offset_m);
        flange_points.push(flange_point);
    }

    let mut pl = Planner {
        ctx: &ctx, hz, period: 1.0 / hz, t: 0.0, waypoints: Vec::new(),
        worst_residual: 0.0, worst_where: String::new(), ik_pos_tol, ik_angle_tol,
    };

    // The edge starts from the current gripper angle; ramp it open first.
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
        r0 = ee.rotation.to_rotation_matrix().matrix().clone();
    }
    let start_seed = seed;
    let odd_close = close_angle(&cfg, close_width_mm, grasp_q);

    let mut marker_q_ik: Vec<[f64; 6]> = Vec::new();
    let mut marker_holds: Vec<(f64, f64)> = Vec::new();
    let mut yaw_degrees: Vec<f64> = Vec::new();
    for (i, point) in points.iter().enumerate() {
        let index = i + 1;
        let place = flange_points[i];
        let yaw_deg = point.y.atan2(point.x).to_degrees();
        yaw_degrees.push((yaw_deg * 1000.0).round_ties_even() / 1000.0);
        let r_marker = work_orientation(&r0, &sequence_axis, yaw_deg);
        let label = format!("marker {index}");
        let target_q = match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| pl.ik_to(&place, &r_marker, &seed, &label))) {
            Ok(q) => q,
            Err(_) => pl.ik_to(&place, &r_marker, &start_seed, &label),
        };
        marker_q_ik.push(target_q);
        let mut current_q = pl.last_q6();
        if index % 2 == 1 {
            let stage_point = stage_in_front(point, approach);
            let stage_place = stage_point + V3::new(0.0, 0.0, tip_offsets[i]);
            let slabel = format!("stage {index}");
            let stage_q = match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| pl.ik_to(&stage_place, &r_marker, &seed, &slabel))) {
                Ok(q) => q,
                Err(_) => pl.ik_to(&stage_place, &r_marker, &start_seed, &slabel),
            };
            if !ctx.path_safe(&current_q, &stage_q) {
                panic!("the leg to the staging point for marker {index} dips within a link radius of the worktop; space the markers further apart or move the arm clear");
            }
            let mut d = [0.0; 6];
            for j in 0..6 { d[j] = stage_q[j] - current_q[j]; }
            pl.joint_leg(&current_q, &d, seconds_for(&current_q, &stage_q, &cfg), gripper);
            current_q = pl.last_q6();
            if !ctx.path_safe(&current_q, &target_q) {
                panic!("the slide-in to marker {index} dips within a link radius of the worktop; raise the marker or move the arm clear");
            }
            let mut d = [0.0; 6];
            for j in 0..6 { d[j] = target_q[j] - current_q[j]; }
            pl.joint_leg(&current_q, &d, seconds_for(&current_q, &target_q, &cfg), gripper);
        } else {
            if !ctx.path_safe(&current_q, &target_q) {
                panic!("the leg to marker {index} dips within a link radius of the worktop; space the markers further apart or move the arm clear");
            }
            let mut d = [0.0; 6];
            for j in 0..6 { d[j] = target_q[j] - current_q[j]; }
            pl.joint_leg(&current_q, &d, seconds_for(&current_q, &target_q, &cfg), gripper);
        }
        let hold_t0 = pl.t;
        pl.hold_g(0.25, gripper);
        let next_gripper = if index % 2 == 1 { odd_close } else { cfg.motor7_open };
        if (next_gripper - gripper).abs() > 1e-6 {
            pl.gripper_leg(next_gripper, 0.8_f64.max(hold_s));
        }
        gripper = next_gripper;
        pl.hold_g(0.25, gripper);
        marker_holds.push((hold_t0, pl.t));
        seed = target_q;
    }

    // Return in joint space to the start pose.
    let q_last = pl.last_q6();
    if !ctx.path_safe(&q_last, &q_start) {
        panic!("the return from marker {} to the start pose dips within a link radius of the worktop; move the arm clear and retry", points.len());
    }
    let mut d = [0.0; 6];
    for j in 0..6 { d[j] = q_start[j] - q_last[j]; }
    pl.joint_leg(&q_last, &d, seconds_for(&q_last, &q_start, &cfg), gripper);

    let compensated = compensate(&ctx, &pl.waypoints, &snap, &kp_spec, &kp);

    let close_text = match close_width_mm {
        None => format!("close to {odd_close:.2} rad"),
        Some(w) => format!("close to {w:.1} mm"),
    };
    let steps = format!("{} markers: stage {:.0} mm in front and slide in on odd, direct on even; \
                         gripper open, {close_text} on odd, open on even; return",
                        points.len(), approach * 1000.0);

    let targets_prov: Vec<Value> = points.iter().enumerate().map(|(i, point)| {
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
            "q_ik": marker_q_ik[i].to_vec(),
            "hold_t_start": marker_holds[i].0,
            "hold_t_end": marker_holds[i].1,
        })
    }).collect();
    let provenance = json!({
        "sequence": true,
        "hover_m": opt_f(&options, "hover_m", cfg.default_min_z),
        "tip_m": opt_f(&options, "tip_m", 0.0),
        "targets": targets_prov,
        "tool_axis": sequence_axis,
        "yaw_deg": yaw,
        "marker_yaw_deg": yaw_degrees,
        "gripper": {"open": cfg.motor7_open, "half": cfg.motor7_half(),
                    "closed": cfg.motor7_closed, "odd_close": odd_close},
    });
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
    let payload = build_payload(&compensated, hz, &limits7(&cfg), &steps, extra, None, None, true, &cfg);
    let stats = validate_payload(&payload, &cfg);

    let tip_m_summary = if !tip_offsets.is_empty()
        && tip_offsets.iter().all(|v| (v - tip_offsets[0]).abs() < 1e-9) {
        json!(tip_offsets[0])
    } else { Value::Null };
    let summary = json!({
        "markers": points.len(),
        "targets": points.iter().map(arr3).collect::<Vec<_>>(),
        "flange_points": flange_points.iter().map(arr3).collect::<Vec<_>>(),
        "q_ik": marker_q_ik.iter().map(|q| q.to_vec()).collect::<Vec<_>>(),
        "radii": points.iter().map(|p| (p.x * p.x + p.y * p.y).sqrt()).collect::<Vec<_>>(),
        "hover_m": opt_f(&options, "hover_m", cfg.default_min_z),
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

// ── /move trajectory builders (ports of traj.joint_move / joint_sequence) ── //

fn parse_limits(v: &Value, cfg: &Config) -> [[f64; 2]; 7] {
    let mut l = limits7(cfg);
    if let Some(arr) = v.as_array() {
        if arr.len() == 7 {
            for j in 0..7 {
                l[j] = [arr[j][0].as_f64().unwrap(), arr[j][1].as_f64().unwrap()];
            }
        }
    }
    l
}

fn q7(v: &Value) -> [f64; 7] {
    let a = v.as_array().unwrap_or_else(|| panic!("expected 7 joint angles"));
    if a.len() != 7 { panic!("expected 7 joint angles, got {}", a.len()); }
    let mut q = [0.0; 7];
    for j in 0..7 { q[j] = a[j].as_f64().unwrap(); }
    q
}

fn wp7(t: f64, q: [f64; 7]) -> Wp { Wp { t, q, tau: [0.0; 7] } }

/// Gravity sag per joint (sweep.gravity_sag): clip(tau_g/kp, ±SAG_LIMIT_RAD).
#[pg_extern]
fn rebot_sag_rad(urdf: &str, q6: Vec<f64>, kp_spec: &str, config: pgrx::JsonB) -> Vec<f64> {
    let cfg = Config::from_json(&config.0);
    let ctx = PlanCtx::new(urdf, cfg.clone());
    let kp = kp_vector(kp_spec);
    let tau = ctx.gravity(&q6);
    gravity_sag(&tau, &kp, &cfg).to_vec()
}

/// The startup elbow peel (sweep.unpark_pose): lift j3 off the j2 casing stop.
#[pg_extern]
fn rebot_unpark_pose(q: Vec<f64>, config: pgrx::JsonB) -> Vec<f64> {
    let cfg = Config::from_json(&config.0);
    if q.len() != 6 { panic!("expected 6 joint angles"); }
    let mut q = q;
    if q[2] >= 0.05 { return q; }
    q[2] = (q[2] + 8.0_f64.to_radians()).min(cfg.limits[2][1]);
    q
}

/// Validate a trajectory payload (traj.validate); returns the stats or raises.
#[pg_extern]
fn rebot_traj_validate(payload: pgrx::JsonB, config: pgrx::JsonB) -> pgrx::JsonB {
    let cfg = Config::from_json(&config.0);
    pgrx::JsonB(validate_payload(&payload.0, &cfg))
}

/// Assemble a trajectory payload from waypoints (traj.build). waypoints is an
/// array of {t, q[7], tau?[7]}; with_tau controls whether tau is emitted.
#[pg_extern]
fn rebot_traj_build(waypoints: pgrx::JsonB, hz: f64, limits: pgrx::JsonB,
                    steps: &str, extra: pgrx::JsonB, config: pgrx::JsonB,
                    with_tau: default!(bool, true)) -> pgrx::JsonB {
    let cfg = Config::from_json(&config.0);
    let arr = waypoints.0.as_array().unwrap_or_else(|| panic!("waypoints must be an array"));
    let wps: Vec<Wp> = arr.iter().map(|w| {
        let q = q7(&w["q"]);
        let mut tau = [0.0; 7];
        if let Some(t) = w["tau"].as_array() {
            for j in 0..7 { tau[j] = t[j].as_f64().unwrap(); }
        }
        Wp { t: w["t"].as_f64().unwrap(), q, tau }
    }).collect();
    let payload = build_payload(&wps, hz, &parse_limits(&limits.0, &cfg), steps, extra.0, None, None, with_tau, &cfg);
    pgrx::JsonB(payload)
}

/// A joint-space move with a hold (traj.joint_move). q_from/q_to are 7-vectors.
#[pg_extern]
fn rebot_joint_move(q_from: Vec<f64>, q_to: Vec<f64>, seconds: f64,
                    config: pgrx::JsonB,
                    hz: default!(f64, 30.0), hold_s: default!(f64, 0.4),
                    limits: default!(pgrx::JsonB, "null"), label: default!(&str, "''")) -> pgrx::JsonB {
    let cfg = Config::from_json(&config.0);
    if q_from.len() != 7 || q_to.len() != 7 { panic!("a joint move needs 7 angles at both ends"); }
    if !q_from.iter().chain(q_to.iter()).all(|v| v.is_finite()) { panic!("joint angles must be finite"); }
    let travel = q_from.iter().zip(q_to.iter()).map(|(a, b)| (b - a).abs()).fold(0.0_f64, f64::max);
    let seconds = if travel < 1e-6 { seconds.max(0.2) } else { seconds };
    let period = 1.0 / hz;
    let count = (seconds.max(0.2) * hz).round_ties_even().max(2.0) as usize;
    let mut wps = vec![wp7(0.0, q_from_arr(&q_from))];
    let mut t = 0.0;
    let qf = q_from_arr(&q_from);
    let qt = q_from_arr(&q_to);
    for index in 1..=count {
        t += period;
        let frac = ease_fraction(index as f64 / count as f64, count as f64 / hz, &cfg);
        let mut q = [0.0; 7];
        for j in 0..7 { q[j] = qf[j] + (qt[j] - qf[j]) * frac; }
        wps.push(wp7(t, q));
    }
    let hold = (hold_s * hz).round_ties_even().max(1.0) as usize;
    for _ in 0..hold { t += period; wps.push(wp7(t, qt)); }
    let steps = if label.is_empty() {
        format!("joint move {:.2} rad over {:.1} s", travel, seconds)
    } else { label.to_string() };
    let payload = build_payload(&wps, hz, &parse_limits(&limits.0, &cfg), &steps, json!({}), None, None, false, &cfg);
    pgrx::JsonB(payload)
}

fn q_from_arr(v: &[f64]) -> [f64; 7] {
    let mut q = [0.0; 7];
    q.copy_from_slice(v);
    q
}

/// One file walking through poses in order, holding at each (traj.joint_sequence).
/// poses is N x 7; seconds is N-1 durations (or a scalar applied to every leg).
#[pg_extern]
fn rebot_joint_sequence(poses: pgrx::JsonB, seconds: pgrx::JsonB,
                        config: pgrx::JsonB,
                        hz: default!(f64, 30.0), hold_s: default!(f64, 0.4),
                        limits: default!(pgrx::JsonB, "null"), label: default!(&str, "''")) -> pgrx::JsonB {
    let cfg = Config::from_json(&config.0);
    let parr = poses.0.as_array().unwrap_or_else(|| panic!("poses must be an array"));
    let points: Vec<[f64; 7]> = parr.iter().map(q7).collect();
    if points.len() < 2 { panic!("a sequence needs at least two poses"); }
    let durations: Vec<f64> = match seconds.0.as_array() {
        Some(a) => a.iter().map(|v| v.as_f64().unwrap()).collect(),
        None => {
            let s = seconds.0.as_f64().unwrap_or(0.2);
            vec![s; points.len() - 1]
        }
    };
    if durations.len() != points.len() - 1 {
        panic!("{} durations for {} legs", durations.len(), points.len() - 1);
    }
    let period = 1.0 / hz;
    let hold = (hold_s.max(0.0) * hz).round_ties_even().max(1.0) as usize;
    let mut wps = vec![wp7(0.0, points[0])];
    let mut t = 0.0;
    for index in 1..points.len() {
        let (start, end) = (points[index - 1], points[index]);
        let count = (durations[index - 1].max(0.2) * hz).round_ties_even().max(2.0) as usize;
        for step in 1..=count {
            t += period;
            let frac = ease_fraction(step as f64 / count as f64, count as f64 / hz, &cfg);
            let mut q = [0.0; 7];
            for j in 0..7 { q[j] = start[j] + (end[j] - start[j]) * frac; }
            wps.push(wp7(t, q));
        }
        for _ in 0..hold { t += period; wps.push(wp7(t, end)); }
    }
    let steps = if label.is_empty() { format!("{} poses, one pass", points.len()) } else { label.to_string() };
    let payload = build_payload(&wps, hz, &parse_limits(&limits.0, &cfg), &steps, json!({}), None, None, false, &cfg);
    pgrx::JsonB(payload)
}

/// The /move compute core (port of planner_server.move_one's trajectory build):
/// peel from rest, worktop gates, sag-compensated commands, policy-paced legs,
/// joint_move/joint_sequence assembly. The gateway keeps the edge I/O (pose read,
/// /play, telemetry); this returns the trajectory + summary. Pure jsonb -> jsonb.
#[pg_extern]
fn rebot_move_build(urdf: &str, pose: pgrx::JsonB, targets: pgrx::JsonB,
                    options: pgrx::JsonB, snap: pgrx::JsonB, config: pgrx::JsonB) -> pgrx::JsonB {
    let cfg = Config::from_json(&config.0);
    let pose = pose.0;
    let options = options.0;
    let snap = snap.0;
    let ctx = PlanCtx::new(urdf, cfg.clone());
    let (q_from6, gripper_default) = read_posture(&pose, &cfg);
    let tarr = targets.0.as_array().unwrap_or_else(|| panic!("targets must be an array of [6]"));
    if tarr.is_empty() { panic!("'targets' must be a non-empty list"); }
    let mut targets6: Vec<[f64; 6]> = Vec::new();
    for item in tarr {
        let a = item.as_array().unwrap_or_else(|| panic!("each target must be six joint angles"));
        if a.len() != 6 { panic!("every target must be six joint angles"); }
        let mut q = [0.0; 6];
        for j in 0..6 { q[j] = a[j].as_f64().unwrap(); }
        if !q.iter().all(|v| v.is_finite()) { panic!("every target must be finite"); }
        for j in 0..6 {
            if q[j] < cfg.limits[j][0] || q[j] > cfg.limits[j][1] {
                panic!("a target is outside the joint limits: [{:.3}]", q[j]);
            }
        }
        targets6.push(q);
    }
    let gripper = options.get("gripper").and_then(|v| v.as_f64()).unwrap_or(gripper_default);
    let kp_spec = opt_s(&options, "kp_spec", "50,2:150,3:150").to_string();
    let kp = kp_vector(&kp_spec);
    let hold = opt_f(&options, "hold", 0.0).clamp(0.0, 30.0);

    // Clamp a small start-pose sag past a limit (a large excursion is a fault).
    let mut q_from = q_from6;
    let excursion = (0..6).map(|j| (cfg.limits[j][0] - q_from[j]).max(q_from[j] - cfg.limits[j][1]))
        .fold(0.0_f64, f64::max);
    if excursion > 0.0 && excursion <= 0.02 {
        for j in 0..6 { q_from[j] = q_from[j].clamp(cfg.limits[j][0], cfg.limits[j][1]); }
    }

    // Startup peel: prepend the elbow lift when starting from rest.
    let lift = rebot_unpark_pose(q_from.to_vec(), pgrx::JsonB(config.0.clone()));
    let mut lift6 = [0.0; 6];
    lift6.copy_from_slice(&lift);
    let unparked = (0..6).any(|j| (lift6[j] - q_from[j]).abs() > 1e-9);
    if unparked { targets6.insert(0, lift6); }

    // Worktop gate on every leg (on the targets, where the arm will be).
    let mut chain: Vec<[f64; 6]> = vec![q_from];
    chain.extend(targets6.iter().copied());
    for index in 0..chain.len() - 1 {
        if !ctx.path_safe(&chain[index], &chain[index + 1]) {
            panic!("leg {} of the move dips within a link radius of the worktop; move the arm clear first", index + 1);
        }
    }

    // Sag-compensated commands (the learned residual where a keyed map exists).
    let mut commands: Vec<[f64; 6]> = Vec::new();
    let mut applied: Vec<f64> = Vec::new();
    for target in &targets6 {
        let tau = ctx.gravity(target);
        let extra = sag_correction(target, &tau, &snap, &kp_spec, &cfg);
        let cmd = aim_with_ctx(&ctx, target, &kp, &extra);
        let mut clipped = [0.0; 6];
        for j in 0..6 { clipped[j] = cmd[j].clamp(cfg.limits[j][0], cfg.limits[j][1]); }
        let max_off = (0..6).map(|j| (clipped[j] - target[j]).abs()).fold(0.0_f64, f64::max);
        applied.push((1000.0 * max_off * 10.0).round_ties_even() / 10.0);
        commands.push(clipped);
    }

    // Policy-paced leg durations (caller may ask slower, never quicker).
    let mut cmd_chain: Vec<[f64; 6]> = vec![q_from];
    cmd_chain.extend(commands.iter().copied());
    let wanted: Vec<f64> = (0..cmd_chain.len() - 1)
        .map(|i| seconds_for(&cmd_chain[i], &cmd_chain[i + 1], &cfg)).collect();
    let given = options.get("seconds");
    let seconds: Vec<f64> = match given.and_then(|v| v.as_array()) {
        Some(arr) => {
            let mut g: Vec<f64> = arr.iter().map(|v| v.as_f64().unwrap()).collect();
            if unparked { g.insert(0, wanted[0]); }
            if g.len() != wanted.len() {
                panic!("'seconds' must be one per leg ({} got {})", wanted.len(), g.len());
            }
            (0..wanted.len()).map(|i| wanted[i].max(g[i])).collect()
        }
        None => match given.and_then(|v| v.as_f64()) {
            Some(s) => wanted.iter().map(|w| w.max(s)).collect(),
            None => wanted.clone(),
        }
    };
    let seconds: Vec<f64> = seconds.iter().map(|s| s.clamp(cfg.min_seconds, cfg.max_seconds)).collect();

    // Assemble: single target -> joint_move; several -> joint_sequence.
    let mut points: Vec<[f64; 7]> = Vec::new();
    for pose6 in &cmd_chain {
        let mut p = [0.0; 7];
        p[..6].copy_from_slice(pose6);
        p[6] = gripper;
        points.push(p);
    }
    let mut q_from7 = [0.0; 7];
    q_from7[..6].copy_from_slice(&q_from);
    q_from7[6] = gripper;
    let limits = json!(limits7(&cfg).iter().map(|l| json!([l[0], l[1]])).collect::<Vec<_>>());
    let payload = if targets6.len() == 1 {
        let label = format!("calibration move to [{}]",
            targets6[0].iter().map(|v| format!("{v:+.2}")).collect::<Vec<_>>().join(", "));
        rebot_joint_move(q_from7.to_vec(), points[points.len() - 1].to_vec(),
                         seconds[0], pgrx::JsonB(config.0.clone()), 30.0, hold,
                         pgrx::JsonB(limits), &label).0
    } else {
        let poses = json!(points.iter().map(|p| p.to_vec()).collect::<Vec<_>>());
        let secs = json!(seconds);
        let label = format!("{} poses, one pass", targets6.len());
        rebot_joint_sequence(pgrx::JsonB(poses), pgrx::JsonB(secs),
                             pgrx::JsonB(config.0.clone()), 30.0, hold,
                             pgrx::JsonB(limits), &label).0
    };
    let stats = validate_payload(&payload, &cfg);
    let summary = json!({
        "seconds": (seconds.iter().sum::<f64>() * 100.0).round_ties_even() / 100.0,
        "leg_seconds": seconds,
        "hold": hold,
        "poses": targets6.len(),
        "sag_mrad": applied,
        "waypoints": stats["waypoints"],
        "duration_s": stats["duration_s"],
        "unparked": unparked,
    });
    pgrx::JsonB(json!({ "payload": payload, "summary": summary }))
}




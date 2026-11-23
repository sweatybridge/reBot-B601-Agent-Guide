//! reBot kinematics as a Postgres extension, backed by openrr/k (pure Rust,
//! nalgebra). Memory-safe FK/IK callable as SQL steps inside pg_durable workflows.
//!
//! No C++ FFI: a bad input raises a SQL ERROR (pgrx catches the panic at the FFI
//! boundary), never a backend crash — this is the in-worker half of the spec's
//! crash-isolation invariant. Collision checking and gravity dynamics are NOT in
//! `k`; they remain separate functions (spec §3.2 gaps).
//!
//! URDF is parsed per call for correctness-first simplicity; caching the loaded
//! chain per session is a later optimization for hot IK loops.

use pgrx::prelude::*;
use k::prelude::*;
use nalgebra::{Isometry3, Translation3, UnitQuaternion};

pg_module_magic!();

mod plan;
mod calib;
mod config;

pub(crate) use config::Config;

fn load_chain(urdf: &str) -> k::Chain<f64> {
    let robot = urdf_rs::read_from_string(urdf)
        .unwrap_or_else(|e| panic!("URDF parse failed: {e}"));
    k::Chain::<f64>::from(&robot)
}

/// Forward kinematics: world pose of `end_joint` at joint positions `q` (rad).
/// Returns [x, y, z, qw, qx, qy, qz] — metres + unit quaternion.
#[pg_extern]
fn rebot_fk(urdf: &str, q: Vec<f64>, end_joint: &str) -> Vec<f64> {
    let chain = load_chain(urdf);
    chain
        .set_joint_positions(&q)
        .unwrap_or_else(|e| panic!("set_joint_positions: {e}"));
    chain.update_transforms();
    let node = chain
        .find(end_joint)
        .unwrap_or_else(|| panic!("joint not found: {end_joint}"));
    let t = node
        .world_transform()
        .unwrap_or_else(|| panic!("no transform for {end_joint}"));
    let tr = t.translation.vector;
    let r = t.rotation;
    vec![tr.x, tr.y, tr.z, r.w, r.i, r.j, r.k]
}

/// Inverse kinematics: joint positions reaching `target` = [x, y, z, qw, qx, qy,
/// qz] (metres + unit quaternion, exactly the pose `rebot_fk` returns) from seed
/// `q0`. `end_joint` defines a serial sub-chain; `q0` must match its DOF (e.g. 6
/// for joint1..joint6). Full-pose (6-DOF) constraints. Returns the solved joints.
/// Raises a SQL ERROR if the solver does not converge (refused, not guessed).
#[pg_extern]
fn rebot_ik(urdf: &str, end_joint: &str, target: Vec<f64>, q0: Vec<f64>) -> Vec<f64> {
    if target.len() != 7 {
        panic!("target must be [x, y, z, qw, qx, qy, qz] (as returned by rebot_fk)");
    }
    let chain = load_chain(urdf);
    let node = chain
        .find(end_joint)
        .unwrap_or_else(|| panic!("joint not found: {end_joint}"));
    let arm = k::SerialChain::from_end(node);
    arm.set_joint_positions(&q0)
        .unwrap_or_else(|e| panic!("seed q0: {e}"));
    let pose = Isometry3::from_parts(
        Translation3::new(target[0], target[1], target[2]),
        UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            target[3], target[4], target[5], target[6],
        )),
    );
    let solver = k::JacobianIkSolver::<f64>::default();
    solver
        .solve(&arm, &pose)
        .unwrap_or_else(|e| panic!("IK did not converge: {e}"));
    arm.joint_positions()
}

/// Degrees of freedom of the URDF chain (sanity check / metadata).
#[pg_extern]
fn rebot_dof(urdf: &str) -> i32 {
    load_chain(urdf).dof() as i32
}

/// gripper_end position in the base frame at joint positions q6 (6 arm joints).
#[pg_extern]
fn rebot_fk_tip(urdf: &str, q6: Vec<f64>) -> Vec<f64> {
    let (_robot, chain) = load_chain_arm(urdf, &q6);
    let node = chain.find_link("gripper_end").unwrap_or_else(|| panic!("gripper_end not found"));
    let p = node.world_transform().unwrap().translation.vector;
    vec![p.x, p.y, p.z]
}

/// The tool tip point in the base frame: gripper_end position + R(gripper_end) @
/// tip_offset. The oracle predicts the tip pixel from this (locate_target.project).
#[pg_extern]
fn rebot_tip_point(urdf: &str, q6: Vec<f64>, tip_offset: Vec<f64>) -> Vec<f64> {
    if tip_offset.len() != 3 { panic!("tip_offset must be [x, y, z]"); }
    let (_robot, chain) = load_chain_arm(urdf, &q6);
    let node = chain.find_link("gripper_end").unwrap_or_else(|| panic!("gripper_end not found"));
    let t = node.world_transform().unwrap();
    let off = nalgebra::Vector3::new(tip_offset[0], tip_offset[1], tip_offset[2]);
    let p = t.translation.vector + t.rotation * off;
    vec![p.x, p.y, p.z]
}

// --------------------------------------------------------------------------- //
// Gravity dynamics + collision checking.
// Ports of rebot_ik.gravity_torques and sweep.path_safe (pure numpy reference),
// using openrr/k for the joint/link world frames. Memory-safe: bad input raises a
// SQL ERROR, never a backend crash.
// --------------------------------------------------------------------------- //

pub(crate) const ARM_JOINTS: [&str; 6] = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"];

/// Load the chain and set the 6 arm joints (gripper joints padded to 0).
pub(crate) fn load_chain_arm(urdf: &str, q6: &[f64]) -> (urdf_rs::Robot, k::Chain<f64>) {
    if q6.len() != 6 {
        panic!("expected 6 arm joint angles, got {}", q6.len());
    }
    let robot = urdf_rs::read_from_string(urdf).unwrap_or_else(|e| panic!("URDF parse failed: {e}"));
    let chain = k::Chain::<f64>::from(&robot);
    let mut q = q6.to_vec();
    while q.len() < chain.dof() {
        q.push(0.0);
    }
    chain.set_joint_positions(&q).unwrap_or_else(|e| panic!("set_joint_positions: {e}"));
    chain.update_transforms();
    (robot, chain)
}

pub(crate) fn joint_axis(robot: &urdf_rs::Robot, name: &str) -> nalgebra::Vector3<f64> {
    let j = robot.joints.iter().find(|j| j.name == name)
        .unwrap_or_else(|| panic!("joint not in URDF: {name}"));
    let a = *j.axis.xyz;
    nalgebra::Vector3::new(a[0], a[1], a[2])
}

/// Joint origins (joint1..6) plus the gripper_end tool point, base frame (7x3).
pub(crate) fn joint_points(chain: &k::Chain<f64>) -> Vec<[f64; 3]> {
    let mut pts = Vec::with_capacity(7);
    for name in ARM_JOINTS {
        let node = chain.find(name).unwrap_or_else(|| panic!("joint not found: {name}"));
        let p = node.world_transform().unwrap().translation.vector;
        pts.push([p.x, p.y, p.z]);
    }
    let tool = chain.find_link("gripper_end").unwrap_or_else(|| panic!("gripper_end link not found"));
    let p = tool.world_transform().unwrap().translation.vector;
    pts.push([p.x, p.y, p.z]);
    pts
}

/// tau_g = dU/dq: motor torque to hold the pose against gravity (6 arm joints).
/// tau[j] = -sum_{links>=j} z_j . cross(p_com - p_j, [0,0,-m*g]).
pub(crate) fn gravity_torques_impl(robot: &urdf_rs::Robot, chain: &k::Chain<f64>, cfg: &Config) -> Vec<f64> {
    let mut coms: Vec<(f64, nalgebra::Vector3<f64>)> = Vec::with_capacity(7);
    for (lname, mass, com) in &cfg.link_inertials {
        let node = chain.find_link(lname).unwrap_or_else(|| panic!("link not found: {lname}"));
        let t = node.world_transform().unwrap();
        let p_com = t.rotation * nalgebra::Vector3::new(com[0], com[1], com[2]) + t.translation.vector;
        coms.push((*mass, p_com));
    }
    let mut tau = vec![0.0; 6];
    for j in 0..6 {
        let node = chain.find(ARM_JOINTS[j]).unwrap();
        let t = node.world_transform().unwrap();
        let p_j = t.translation.vector;
        // Rotation about the joint axis leaves the axis invariant, so the
        // post-rotation frame gives the same world axis as the origin frame.
        let z_j = t.rotation * joint_axis(robot, ARM_JOINTS[j]);
        let mut total = 0.0;
        for (mass, p_com) in coms.iter().skip(j) {
            let force = nalgebra::Vector3::new(0.0, 0.0, -mass * cfg.gravity);
            total += z_j.dot(&(p_com - p_j).cross(&force));
        }
        tau[j] = -total;
    }
    tau
}

/// sweep.path_safe: is the straight joint-space line a->b clear of the worktop?
pub(crate) fn path_safe_impl(chain: &k::Chain<f64>, a: &[f64], b: &[f64], samples: usize, cfg: &Config) -> bool {
    if samples < 2 {
        panic!("samples must be >= 2");
    }
    for s in 0..samples {
        let t = s as f64 / (samples as f64 - 1.0);
        let mut q: Vec<f64> = a.iter().zip(b.iter()).map(|(x, y)| x + (y - x) * t).collect();
        while q.len() < chain.dof() {
            q.push(0.0);
        }
        chain.set_joint_positions(&q).unwrap_or_else(|e| panic!("set_joint_positions: {e}"));
        chain.update_transforms();
        let points = joint_points(chain);
        for index in 1..points.len() {
            let start = points[index - 1];
            let end = points[index];
            let radius = cfg.link_radii[index.min(cfg.link_radii.len() - 1)];
            for fraction in [0.0, 0.5] {
                let cz = start[2] + (end[2] - start[2]) * fraction;
                if cz - radius < cfg.clearance {
                    return false;
                }
            }
            if (end[0] * end[0] + end[1] * end[1]).sqrt() > cfg.link_radius_max {
                return false;
            }
        }
    }
    true
}

/// Joint torques (Nm, 6 arm joints) to hold pose `q` against gravity.
/// `config` is the REQUIRED world.arm_config doc (no hardcoded fallback).
#[pg_extern]
fn rebot_gravity_torques(urdf: &str, q: Vec<f64>, config: pgrx::JsonB) -> Vec<f64> {
    let cfg = Config::from_json(&config.0);
    let (robot, chain) = load_chain_arm(urdf, &q);
    gravity_torques_impl(&robot, &chain, &cfg)
}

/// Joint origins + tool point (7 points, flattened to 21 floats) at pose `q`.
#[pg_extern]
fn rebot_joint_points(urdf: &str, q: Vec<f64>) -> Vec<f64> {
    let (_robot, chain) = load_chain_arm(urdf, &q);
    joint_points(&chain).iter().flat_map(|p| p.iter().copied()).collect()
}

/// Is the straight joint-space interpolation a->b clear of the worktop (sweep.path_safe)?
#[pg_extern]
fn rebot_path_safe(urdf: &str, a: Vec<f64>, b: Vec<f64>, config: pgrx::JsonB,
                   samples: default!(i32, 40)) -> bool {
    let cfg = Config::from_json(&config.0);
    let (_robot, chain) = load_chain_arm(urdf, &a);
    path_safe_impl(&chain, &a, &b, samples as usize, &cfg)
}

// --------------------------------------------------------------------------- //
// Sag / gain residual map fitting — port of identify.fit_sag_map (§7.3b).
// Huber IRLS affine per joint, hold-out validation, FK sensitivity for validated_mm.
// --------------------------------------------------------------------------- //

struct SagHold { q: Vec<f64>, tau: Vec<f64>, r: Vec<f64>, tip: Vec<f64> }

fn json_f64_arr(v: &serde_json::Value) -> Vec<f64> {
    v.as_array().map(|a| a.iter().filter_map(|x| x.as_f64()).collect()).unwrap_or_default()
}

fn parse_holds(v: &serde_json::Value) -> Vec<SagHold> {
    v.as_array().map(|arr| arr.iter().map(|h| SagHold {
        q: json_f64_arr(&h["q"]), tau: json_f64_arr(&h["tau"]),
        r: json_f64_arr(&h["r"]), tip: json_f64_arr(&h["tip"]),
    }).collect()).unwrap_or_default()
}

/// Weighted least-squares affine y ~ a*x + b via 2x2 normal equations.
fn wls_affine(x: &[f64], y: &[f64], w: &[f64]) -> (f64, f64) {
    let (mut sxx, mut sx, mut sw, mut sxy, mut sy) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for i in 0..x.len() {
        let wi = w[i];
        sxx += wi * x[i] * x[i]; sx += wi * x[i]; sw += wi;
        sxy += wi * x[i] * y[i]; sy += wi * y[i];
    }
    let det = sxx * sw - sx * sx;
    if det.abs() < 1e-12 { (0.0, sy / sw.max(1e-12)) }
    else { ((sxy * sw - sy * sx) / det, (sxx * sy - sx * sxy) / det) }
}

/// Huber IRLS affine fit (identify._huber_affine): OLS then 20 reweight iterations.
/// `huber_rad` is the learning config's sag_huber_rad (required, no fallback).
fn huber_affine(x: &[f64], y: &[f64], huber_rad: f64) -> (f64, f64) {
    let n = x.len();
    let ones = vec![1.0; n];
    let mut coef = wls_affine(x, y, &ones);
    for _ in 0..20 {
        let w: Vec<f64> = (0..n).map(|i| {
            let r = (y[i] - (coef.0 * x[i] + coef.1)).abs();
            if r > huber_rad { huber_rad / r.max(1e-12) } else { 1.0 }
        }).collect();
        coef = wls_affine(x, y, &w);
    }
    coef
}

fn per_joint_rms(holds: &[SagHold], pm: &[(f64, f64); 6]) -> f64 {
    if holds.is_empty() { return 0.0; }
    let mut total = 0.0;
    for h in holds {
        for j in 0..6 {
            let pred = pm[j].0 * h.tau[j] + pm[j].1;
            total += (h.r[j] - pred).powi(2);
        }
    }
    1000.0 * (total / (holds.len() * 6) as f64).sqrt()
}

/// Tip sensitivity per joint (m/rad) by finite difference of the FK (identify._tool_mm_per_rad).
fn tool_mm_per_rad(urdf: &str, q6: &[f64]) -> [f64; 6] {
    let fk = |q: &[f64]| -> nalgebra::Vector3<f64> {
        let (_r, chain) = load_chain_arm(urdf, q);
        let t = chain.find_link("gripper_end").unwrap().world_transform().unwrap();
        t.translation.vector
    };
    let base = fk(q6);
    let mut out = [0.0; 6];
    for j in 0..6 {
        let mut moved = q6.to_vec();
        moved[j] += 1e-4;
        out[j] = (fk(&moved) - base).norm() / 1e-4;
    }
    out
}

/// Fit the per-joint sag residual map from hold samples (identify.fit_sag_map).
/// `holds` is a jsonb array of {q[7], tau[6], r[6], tip[3]}. Returns the fit doc.
/// `config` is the REQUIRED world.arm_config "learning" group (no hardcoded
/// fallback); `min_holds` defaults to the config's sag_min_holds when NULL.
#[pg_extern]
fn rebot_fit_sag_map(
    urdf: &str, holds: pgrx::JsonB, kp_key: &str, rig_fingerprint: &str, zeros_key: &str,
    config: pgrx::JsonB, min_holds: default!(Option<i32>, "NULL"),
) -> pgrx::JsonB {
    use serde_json::json;
    let cfg = config.0.as_object()
        .unwrap_or_else(|| panic!("learning config must be a jsonb object"));
    let gate = |key: &str| -> f64 {
        cfg.get(key).unwrap_or_else(|| panic!("learning config missing required key '{key}'"))
            .as_f64().unwrap_or_else(|| panic!("learning config {key} must be a number"))
    };
    let sag_huber_rad = gate("sag_huber_rad");
    let sag_validate_max_mrad = gate("sag_validate_max_mrad");
    let sag_validate_ratio = gate("sag_validate_ratio");
    let sag_span_radius_m = gate("sag_span_radius_m");
    let sag_span_height_m = gate("sag_span_height_m");
    let hs = parse_holds(&holds.0);
    let min_holds = match min_holds {
        Some(v) => v as usize,
        None => gate("sag_min_holds") as usize,
    };
    if hs.len() < min_holds {
        return pgrx::JsonB(json!({"ok": false, "reasons": [
            format!("only {} holds (need {}): let more executions accumulate", hs.len(), min_holds)]}));
    }
    let radius: Vec<f64> = hs.iter().map(|h| (h.tip[0].powi(2) + h.tip[1].powi(2)).sqrt()).collect();
    let height: Vec<f64> = hs.iter().map(|h| h.tip[2]).collect();
    let rmin = radius.iter().cloned().fold(f64::INFINITY, f64::min);
    let rmax = radius.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let hmin = height.iter().cloned().fold(f64::INFINITY, f64::min);
    let hmax = height.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    if rmax - rmin < sag_span_radius_m || hmax - hmin < sag_span_height_m {
        return pgrx::JsonB(json!({"ok": false, "reasons": [
            format!("the holds do not span the workspace (radius span {:.2} m, height span {:.2} m)", rmax - rmin, hmax - hmin)]}));
    }
    let train: Vec<&SagHold> = hs.iter().enumerate().filter(|(i, _)| i % 5 != 4).map(|(_, h)| h).collect();
    let test: Vec<&SagHold> = hs.iter().enumerate().filter(|(i, _)| i % 5 == 4).map(|(_, h)| h).collect();
    let mut pm = [(0.0_f64, 0.0_f64); 6];
    for j in 0..6 {
        let x: Vec<f64> = train.iter().map(|h| h.tau[j]).collect();
        let y: Vec<f64> = train.iter().map(|h| h.r[j]).collect();
        pm[j] = huber_affine(&x, &y, sag_huber_rad);
    }
    let to_owned = |v: &[&SagHold]| -> Vec<SagHold> {
        v.iter().map(|h| SagHold { q: h.q.clone(), tau: h.tau.clone(), r: h.r.clone(), tip: h.tip.clone() }).collect()
    };
    let train_rms = per_joint_rms(&to_owned(&train), &pm);
    let test_rms = per_joint_rms(&to_owned(&test), &pm);
    if test_rms > sag_validate_max_mrad || test_rms > (sag_validate_ratio * train_rms).max(1.0) {
        return pgrx::JsonB(json!({"ok": false, "rms_mrad": (test_rms * 1000.0).round() / 1000.0,
            "reasons": [format!("hold-out validation failed: {:.1} mrad held out vs {:.1} mrad train (an overfit map is not applied)", test_rms, train_rms)]}));
    }
    // envelope
    let dof = hs[0].q.len();
    let mut q_min = vec![f64::INFINITY; dof];
    let mut q_max = vec![f64::NEG_INFINITY; dof];
    for h in &hs { for j in 0..dof { q_min[j] = q_min[j].min(h.q[j]); q_max[j] = q_max[j].max(h.q[j]); } }
    // validated_mm via FK sensitivity at the mean pose
    let mean_q6: Vec<f64> = (0..6).map(|j| hs.iter().map(|h| h.q[j]).sum::<f64>() / hs.len() as f64).collect();
    let sens = tool_mm_per_rad(urdf, &mean_q6);
    let mut per_joint = [0.0_f64; 6];
    for j in 0..6 {
        let mut acc = 0.0;
        for h in &test { let e = h.r[j] - (pm[j].0 * h.tau[j] + pm[j].1); acc += e * e; }
        per_joint[j] = if test.is_empty() { 0.0 } else { (acc / test.len() as f64).sqrt() };
    }
    let validated_mm = 1000.0 * (0..6).map(|j| (per_joint[j] * sens[j]).powi(2)).sum::<f64>().sqrt();
    let per_motor: serde_json::Map<String, serde_json::Value> = (0..6)
        .map(|j| (format!("{}", j + 1), json!({"a": pm[j].0, "b": pm[j].1}))).collect();
    pgrx::JsonB(json!({
        "ok": true,
        "rms_mrad": (test_rms * 1000.0).round() / 1000.0,
        "train_rms_mrad": (train_rms * 1000.0).round() / 1000.0,
        "map": {
            "version": 1, "kp_key": kp_key, "rig_fingerprint": rig_fingerprint, "zeros_key": zeros_key,
            "per_motor": per_motor, "samples": hs.len(),
            "rms_mrad": (test_rms * 1000.0).round() / 1000.0,
            "validated_mm": (validated_mm * 100.0).round() / 100.0,
            "envelope": {"q_min": q_min, "q_max": q_max, "radius_m": [rmin, rmax], "height_m": [hmin, hmax]}
        }
    }))
}

// --------------------------------------------------------------------------- //
// Capsule-capsule self-collision (two-arm cooperation prerequisite).
// Links are capsules between consecutive joint points with the sweep RADII;
// non-adjacent pairs are checked (adjacent links share a joint by construction).
// Deliberate contacts (e.g. the parked j4 hinge on the j2 casing) must be excepted
// by the caller — this reports geometric intersection, not intent.
// --------------------------------------------------------------------------- //

type V3 = nalgebra::Vector3<f64>;

/// Closest distance between segments p1-q1 and p2-q2 (Ericson, Real-Time Collision
/// Detection). Used for capsule-capsule clearance.
fn seg_seg_distance(p1: V3, q1: V3, p2: V3, q2: V3) -> f64 {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.dot(&d1);
    let e = d2.dot(&d2);
    let f = d2.dot(&r);
    let eps = 1e-12;
    let (mut s, mut t);
    if a <= eps && e <= eps {
        return r.norm();
    }
    if a <= eps {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = d1.dot(&r);
        if e <= eps {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            let b = d1.dot(&d2);
            let denom = a * e - b * b;
            s = if denom.abs() > eps { ((b * f - c * e) / denom).clamp(0.0, 1.0) } else { 0.0 };
            t = (b * s + f) / e;
            if t < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if t > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            }
        }
    }
    let c1 = p1 + d1 * s;
    let c2 = p2 + d2 * t;
    (c1 - c2).norm()
}

fn v3(p: [f64; 3]) -> V3 { V3::new(p[0], p[1], p[2]) }

/// True if no non-adjacent link capsules intersect at this pose.
fn self_collision_free(chain: &k::Chain<f64>, cfg: &Config) -> bool {
    let pts = joint_points(chain);
    for i in 1..pts.len() {
        for j in (i + 2)..pts.len() {
            let dist = seg_seg_distance(v3(pts[i - 1]), v3(pts[i]), v3(pts[j - 1]), v3(pts[j]));
            if dist < cfg.link_radii[i] + cfg.link_radii[j] {
                return false;
            }
        }
    }
    true
}

/// Minimum clearance (m) between any non-adjacent link capsules (surface gap).
fn min_self_clearance(chain: &k::Chain<f64>, cfg: &Config) -> f64 {
    let pts = joint_points(chain);
    let mut best = f64::INFINITY;
    for i in 1..pts.len() {
        for j in (i + 2)..pts.len() {
            let dist = seg_seg_distance(v3(pts[i - 1]), v3(pts[i]), v3(pts[j - 1]), v3(pts[j]));
            best = best.min(dist - cfg.link_radii[i] - cfg.link_radii[j]);
        }
    }
    best
}

/// Is the pose free of (non-adjacent) link self-collision? Deliberate contacts must
/// be excepted by the caller.
#[pg_extern]
fn rebot_self_collision_free(urdf: &str, q: Vec<f64>, config: pgrx::JsonB) -> bool {
    let cfg = Config::from_json(&config.0);
    let (_robot, chain) = load_chain_arm(urdf, &q);
    self_collision_free(&chain, &cfg)
}

/// Closest distance between two segments (each given as [x,y,z] endpoints). The
/// primitive for inter-arm capsule clearance: clearance = dist - radius_a - radius_b.
#[pg_extern]
fn rebot_seg_seg_distance(p1: Vec<f64>, q1: Vec<f64>, p2: Vec<f64>, q2: Vec<f64>) -> f64 {
    let v = |a: Vec<f64>| {
        if a.len() != 3 { panic!("segment endpoint must be [x,y,z]"); }
        V3::new(a[0], a[1], a[2])
    };
    seg_seg_distance(v(p1), v(q1), v(p2), v(q2))
}

/// Minimum surface gap (m) between non-adjacent link capsules; negative = intersecting.
#[pg_extern]
fn rebot_min_self_clearance(urdf: &str, q: Vec<f64>, config: pgrx::JsonB) -> f64 {
    let cfg = Config::from_json(&config.0);
    let (_robot, chain) = load_chain_arm(urdf, &q);
    min_self_clearance(&chain, &cfg)
}

/// Is the straight joint-space line a->b free of self-collision at every sample?
#[pg_extern]
fn rebot_path_collision_free(urdf: &str, a: Vec<f64>, b: Vec<f64>, config: pgrx::JsonB,
    samples: default!(i32, 40)) -> bool {
    let cfg = Config::from_json(&config.0);
    let (_robot, chain) = load_chain_arm(urdf, &a);
    let samples = samples as usize;
    if samples < 2 { panic!("samples must be >= 2"); }
    for s in 0..samples {
        let t = s as f64 / (samples as f64 - 1.0);
        let mut q: Vec<f64> = a.iter().zip(b.iter()).map(|(x, y)| x + (y - x) * t).collect();
        while q.len() < chain.dof() { q.push(0.0); }
        chain.set_joint_positions(&q).unwrap_or_else(|e| panic!("set_joint_positions: {e}"));
        chain.update_transforms();
        if !self_collision_free(&chain, &cfg) { return false; }
    }
    true
}

// --------------------------------------------------------------------------- //
// Trajectory time law — port of traj.ease_fraction / ease_velocity (S-ramp
// trapezoid: smoothstep ramp into constant-velocity cruise, zero accel at ends).
// --------------------------------------------------------------------------- //

fn ramp_ratio(duration: f64, ramp: f64) -> f64 {
    (ramp / duration.max(1e-9)).max(1e-9).min(0.5)
}

/// Fraction of a leg's travel at normalized time u (traj.ease_fraction).
#[pg_extern]
fn rebot_ease_fraction(u: f64, duration: f64, ramp: default!(f64, 0.15)) -> f64 {
    let r = ramp_ratio(duration, ramp);
    let vc = 1.0 / (1.0 - r);
    let u = u.max(0.0).min(1.0);
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

/// d(ease_fraction)/dt in 1/s (traj.ease_velocity); multiply by travel for rad/s.
#[pg_extern]
fn rebot_ease_velocity(u: f64, duration: f64, ramp: default!(f64, 0.15)) -> f64 {
    let r = ramp_ratio(duration, ramp);
    let vc = 1.0 / (1.0 - r);
    let u = u.max(0.0).min(1.0);
    let duration = duration.max(1e-9);
    if u <= r {
        let x = u / r;
        vc * (3.0 * x * x - 2.0 * x.powi(3)) / duration
    } else if u >= 1.0 - r {
        let y = (1.0 - u) / r;
        vc * (3.0 * y * y - 2.0 * y.powi(3)) / duration
    } else {
        vc / duration
    }
}

// --------------------------------------------------------------------------- //
// Sweep timing / limits + projection (ports of sweep.seconds_for / within_limits
// and locate_target.project's no-distortion path).
// --------------------------------------------------------------------------- //

/// Leg duration from the furthest-travelling joint (sweep.seconds_for).
/// peak_speed/minimum/maximum default to the config's motion values when NULL.
#[pg_extern]
fn rebot_seconds_for(a: Vec<f64>, b: Vec<f64>,
                     config: pgrx::JsonB,
                     peak_speed: default!(Option<f64>, "NULL"),
                     minimum: default!(Option<f64>, "NULL"),
                     maximum: default!(Option<f64>, "NULL")) -> f64 {
    let cfg = Config::from_json(&config.0);
    let peak_speed = peak_speed.unwrap_or(cfg.peak_speed);
    let minimum = minimum.unwrap_or(cfg.min_seconds);
    let maximum = maximum.unwrap_or(cfg.max_seconds);
    let travel = a.iter().zip(b.iter()).map(|(x, y)| (y - x).abs()).fold(0.0_f64, f64::max);
    let peak_time = travel / peak_speed + cfg.ramp_s;
    let cruise_cap = travel / cfg.min_cruise_speed + cfg.ramp_s;
    minimum.max(maximum.min(peak_time).min(cruise_cap))
}

/// Joint vector within limits (sweep.within_limits).
#[pg_extern]
fn rebot_within_limits(q: Vec<f64>, config: pgrx::JsonB, margin: default!(f64, 0.0)) -> bool {
    let cfg = Config::from_json(&config.0);
    if q.len() != 6 { panic!("expected 6 joint angles"); }
    q.iter().zip(cfg.limits.iter()).all(|(v, l)| *v >= l[0] + margin && *v <= l[1] - margin)
}

/// Project a base-frame point to pixels (locate_target.project, no distortion).
/// K is row-major 3x3 (9), T_base_cam row-major 4x4 (16). NULL if behind the camera.
#[pg_extern]
fn rebot_project(k: Vec<f64>, t_base_cam: Vec<f64>, point: Vec<f64>) -> Option<Vec<f64>> {
    if k.len() != 9 || t_base_cam.len() != 16 || point.len() != 3 {
        panic!("K must be 9, T_base_cam 16, point 3");
    }
    // p_cam = R^T (point - t)
    let dx = point[0] - t_base_cam[3];
    let dy = point[1] - t_base_cam[7];
    let dz = point[2] - t_base_cam[11];
    let px = t_base_cam[0] * dx + t_base_cam[4] * dy + t_base_cam[8] * dz;
    let py = t_base_cam[1] * dx + t_base_cam[5] * dy + t_base_cam[9] * dz;
    let pz = t_base_cam[2] * dx + t_base_cam[6] * dy + t_base_cam[10] * dz;
    if pz <= 1e-9 { return None; }
    let qx = k[0] * px + k[1] * py + k[2] * pz;
    let qy = k[3] * px + k[4] * py + k[5] * pz;
    let qz = k[6] * px + k[7] * py + k[8] * pz;
    Some(vec![qx / qz, qy / qz])
}

/// Project with lens distortion (locate_target.project's cv2.projectPoints path).
/// dist = [k1,k2,p1,p2,k3] (radial/tangential). NULL if behind the camera.
#[pg_extern]
fn rebot_project_dist(k: Vec<f64>, t_base_cam: Vec<f64>, point: Vec<f64>, dist: Vec<f64>) -> Option<Vec<f64>> {
    if k.len() != 9 || t_base_cam.len() != 16 || point.len() != 3 {
        panic!("K must be 9, T_base_cam 16, point 3");
    }
    let dx = point[0] - t_base_cam[3];
    let dy = point[1] - t_base_cam[7];
    let dz = point[2] - t_base_cam[11];
    let px = t_base_cam[0] * dx + t_base_cam[4] * dy + t_base_cam[8] * dz;
    let py = t_base_cam[1] * dx + t_base_cam[5] * dy + t_base_cam[9] * dz;
    let pz = t_base_cam[2] * dx + t_base_cam[6] * dy + t_base_cam[10] * dz;
    if pz <= 1e-9 { return None; }
    let x = px / pz;
    let y = py / pz;
    let (k1, k2, p1, p2, k3) = (
        *dist.get(0).unwrap_or(&0.0), *dist.get(1).unwrap_or(&0.0),
        *dist.get(2).unwrap_or(&0.0), *dist.get(3).unwrap_or(&0.0),
        *dist.get(4).unwrap_or(&0.0));
    let r2 = x * x + y * y;
    let radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    let xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    let yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
    Some(vec![k[0] * xd + k[2], k[4] * yd + k[5]])
}

// --------------------------------------------------------------------------- //
// Ray/plane intersection with iterative undistortion (port of
// extrinsics.ray_plane + extrinsics.undistort). Pure math, memory-safe.
// --------------------------------------------------------------------------- //

/// Fixed-point inversion of the radial/tangential distortion (extrinsics.undistort,
/// 10 iterations; the phone's distortion is mild enough for this to converge).
fn undistort(xd: f64, yd: f64, dist: &[f64]) -> (f64, f64) {
    let k1 = *dist.first().unwrap_or(&0.0);
    let k2 = *dist.get(1).unwrap_or(&0.0);
    let p1 = *dist.get(2).unwrap_or(&0.0);
    let p2 = *dist.get(3).unwrap_or(&0.0);
    let k3 = *dist.get(4).unwrap_or(&0.0);
    if k1 == 0.0 && k2 == 0.0 && p1 == 0.0 && p2 == 0.0 && k3 == 0.0 {
        return (xd, yd);
    }
    let (mut x, mut y) = (xd, yd);
    for _ in 0..10 {
        let r2 = x * x + y * y;
        let radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
        let dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        let dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        x = (xd - dx) / radial;
        y = (yd - dy) / radial;
    }
    (x, y)
}

/// Where the pixel's ray meets the horizontal plane (extrinsics.ray_plane).
/// K row-major 3x3 (9), dist [k1,k2,p1,p2,k3] (padded with 0), T_base_cam
/// row-major 4x4 (16). Returns {point:[x,y,z], distance, grazing_deg}.
/// Degenerate/parallel/away rays raise a SQL ERROR (Python: ExtrinsicsError).
#[pg_extern]
pub(crate) fn rebot_ray_plane(k: Vec<f64>, dist: Vec<f64>, t_base_cam: Vec<f64>,
                   u: f64, v: f64, plane_z: f64) -> pgrx::JsonB {
    if k.len() != 9 || t_base_cam.len() != 16 {
        panic!("K must be 9, T_base_cam 16");
    }
    let xd = (u - k[2]) / k[0];
    let yd = (v - k[5]) / k[4];
    let (x, y) = undistort(xd, yd, &dist);

    // direction = R @ [x, y, 1]; origin = translation column.
    let t = &t_base_cam;
    let mut d = [
        t[0] * x + t[1] * y + t[2],
        t[4] * x + t[5] * y + t[6],
        t[8] * x + t[9] * y + t[10],
    ];
    let norm = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    if norm < 1e-12 {
        panic!("degenerate ray for pixel ({u}, {v})");
    }
    for c in d.iter_mut() { *c /= norm; }
    if d[2].abs() < 1e-6 {
        panic!("the ray is parallel to the table plane");
    }
    let distance = (plane_z - t[11]) / d[2];
    if distance <= 0.0 {
        panic!("the ray points away from the table plane");
    }
    let point = [t[3] + distance * d[0], t[7] + distance * d[1], t[11] + distance * d[2]];
    let grazing = d[2].abs().min(1.0).asin().to_degrees();
    pgrx::JsonB(serde_json::json!({
        "point": point, "distance": distance, "grazing_deg": grazing,
    }))
}

#[cfg(any(test, feature = "pg_test"))]
#[pg_schema]
mod tests {
    use pgrx::prelude::*;

    // Self-contained 2-DoF planar chain: base -> l1 (0.1 m) -> l2 (0.5 m).
    const SAMPLE_URDF: &str = r#"
<robot name="test">
  <link name="base"/>
  <link name="l1"/>
  <link name="l2"/>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="l1"/><child link="l2"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
</robot>
"#;

    #[pg_test]
    fn test_dof() {
        assert_eq!(crate::rebot_dof(SAMPLE_URDF), 2);
    }

    #[pg_test]
    fn test_fk_zero() {
        let pose = crate::rebot_fk(SAMPLE_URDF, vec![0.0, 0.0], "j2");
        assert_eq!(pose.len(), 7);
        assert!((pose[2] - 0.6).abs() < 1e-9, "tip z = {}", pose[2]);
    }
}


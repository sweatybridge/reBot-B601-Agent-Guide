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

// --------------------------------------------------------------------------- //
// Gravity dynamics + collision checking.
// Ports of rebot_ik.gravity_torques and sweep.path_safe (pure numpy reference),
// using openrr/k for the joint/link world frames. Memory-safe: bad input raises a
// SQL ERROR, never a backend crash.
// --------------------------------------------------------------------------- //

const GRAVITY: f64 = 9.80665;
const ARM_JOINTS: [&str; 6] = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"];

// (link, mass kg, COM in that link frame) — matches rebot_ik.LINK_INERTIALS.
// Gripper fingers omitted (<0.1 Nm); gripper_end is the lumped tool mass.
const LINK_INERTIALS: [(&str, f64, [f64; 3]); 7] = [
    ("link1", 0.4655, [0.00817652, 0.00122584, 0.03314197]),
    ("link2", 1.5520, [-0.11786481, -0.00342226, -0.02715519]),
    ("link3", 1.2520, [0.10342715, -0.06641379, -0.02710687]),
    ("link4", 0.4600, [0.04755445, -0.04411135, -0.03141119]),
    ("link5", 0.2012, [0.01144006, 0.00000479, 0.03804875]),
    ("link6", 0.1000, [0.00001802, 0.00000002, 0.00434806]),
    ("gripper_end", 0.6500, [-0.11275169, 0.00012844, -0.00000573]),
];

// sweep.py collision model: per-link capsule radii, worktop clearance, reach cap.
const RADII: [f64; 7] = [0.055, 0.055, 0.050, 0.045, 0.035, 0.030, 0.020];
const CLEARANCE: f64 = 0.012;
const LINK_RADIUS_MAX: f64 = 0.85;

/// Load the chain and set the 6 arm joints (gripper joints padded to 0).
fn load_chain_arm(urdf: &str, q6: &[f64]) -> (urdf_rs::Robot, k::Chain<f64>) {
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

fn joint_axis(robot: &urdf_rs::Robot, name: &str) -> nalgebra::Vector3<f64> {
    let j = robot.joints.iter().find(|j| j.name == name)
        .unwrap_or_else(|| panic!("joint not in URDF: {name}"));
    let a = *j.axis.xyz;
    nalgebra::Vector3::new(a[0], a[1], a[2])
}

/// Joint origins (joint1..6) plus the gripper_end tool point, base frame (7x3).
fn joint_points(chain: &k::Chain<f64>) -> Vec<[f64; 3]> {
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
fn gravity_torques_impl(robot: &urdf_rs::Robot, chain: &k::Chain<f64>) -> Vec<f64> {
    let mut coms: Vec<(f64, nalgebra::Vector3<f64>)> = Vec::with_capacity(7);
    for (lname, mass, com) in LINK_INERTIALS {
        let node = chain.find_link(lname).unwrap_or_else(|| panic!("link not found: {lname}"));
        let t = node.world_transform().unwrap();
        let p_com = t.rotation * nalgebra::Vector3::new(com[0], com[1], com[2]) + t.translation.vector;
        coms.push((mass, p_com));
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
            let force = nalgebra::Vector3::new(0.0, 0.0, -mass * GRAVITY);
            total += z_j.dot(&(p_com - p_j).cross(&force));
        }
        tau[j] = -total;
    }
    tau
}

/// sweep.path_safe: is the straight joint-space line a->b clear of the worktop?
fn path_safe_impl(chain: &k::Chain<f64>, a: &[f64], b: &[f64], samples: usize) -> bool {
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
            let radius = RADII[index.min(RADII.len() - 1)];
            for fraction in [0.0, 0.5] {
                let cz = start[2] + (end[2] - start[2]) * fraction;
                if cz - radius < CLEARANCE {
                    return false;
                }
            }
            if (end[0] * end[0] + end[1] * end[1]).sqrt() > LINK_RADIUS_MAX {
                return false;
            }
        }
    }
    true
}

/// Joint torques (Nm, 6 arm joints) to hold pose `q` against gravity.
#[pg_extern]
fn rebot_gravity_torques(urdf: &str, q: Vec<f64>) -> Vec<f64> {
    let (robot, chain) = load_chain_arm(urdf, &q);
    gravity_torques_impl(&robot, &chain)
}

/// Joint origins + tool point (7 points, flattened to 21 floats) at pose `q`.
#[pg_extern]
fn rebot_joint_points(urdf: &str, q: Vec<f64>) -> Vec<f64> {
    let (_robot, chain) = load_chain_arm(urdf, &q);
    joint_points(&chain).iter().flat_map(|p| p.iter().copied()).collect()
}

/// Is the straight joint-space interpolation a->b clear of the worktop (sweep.path_safe)?
#[pg_extern]
fn rebot_path_safe(urdf: &str, a: Vec<f64>, b: Vec<f64>, samples: default!(i32, 40)) -> bool {
    let (_robot, chain) = load_chain_arm(urdf, &a);
    path_safe_impl(&chain, &a, &b, samples as usize)
}

// --------------------------------------------------------------------------- //
// Sag / gain residual map fitting — port of identify.fit_sag_map (§7.3b).
// Huber IRLS affine per joint, hold-out validation, FK sensitivity for validated_mm.
// --------------------------------------------------------------------------- //

const SAG_HUBER_RAD: f64 = 0.010;
const SAG_VALIDATE_MAX_MRAD: f64 = 15.0;
const SAG_VALIDATE_RATIO: f64 = 2.0;
const SAG_SPAN_RADIUS_M: f64 = 0.10;
const SAG_SPAN_HEIGHT_M: f64 = 0.05;

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
fn huber_affine(x: &[f64], y: &[f64]) -> (f64, f64) {
    let n = x.len();
    let ones = vec![1.0; n];
    let mut coef = wls_affine(x, y, &ones);
    for _ in 0..20 {
        let w: Vec<f64> = (0..n).map(|i| {
            let r = (y[i] - (coef.0 * x[i] + coef.1)).abs();
            if r > SAG_HUBER_RAD { SAG_HUBER_RAD / r.max(1e-12) } else { 1.0 }
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
#[pg_extern]
fn rebot_fit_sag_map(
    urdf: &str, holds: pgrx::JsonB, kp_key: &str, rig_fingerprint: &str, zeros_key: &str,
    min_holds: default!(i32, 30),
) -> pgrx::JsonB {
    use serde_json::json;
    let hs = parse_holds(&holds.0);
    let min_holds = min_holds as usize;
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
    if rmax - rmin < SAG_SPAN_RADIUS_M || hmax - hmin < SAG_SPAN_HEIGHT_M {
        return pgrx::JsonB(json!({"ok": false, "reasons": [
            format!("the holds do not span the workspace (radius span {:.2} m, height span {:.2} m)", rmax - rmin, hmax - hmin)]}));
    }
    let train: Vec<&SagHold> = hs.iter().enumerate().filter(|(i, _)| i % 5 != 4).map(|(_, h)| h).collect();
    let test: Vec<&SagHold> = hs.iter().enumerate().filter(|(i, _)| i % 5 == 4).map(|(_, h)| h).collect();
    let mut pm = [(0.0_f64, 0.0_f64); 6];
    for j in 0..6 {
        let x: Vec<f64> = train.iter().map(|h| h.tau[j]).collect();
        let y: Vec<f64> = train.iter().map(|h| h.r[j]).collect();
        pm[j] = huber_affine(&x, &y);
    }
    let to_owned = |v: &[&SagHold]| -> Vec<SagHold> {
        v.iter().map(|h| SagHold { q: h.q.clone(), tau: h.tau.clone(), r: h.r.clone(), tip: h.tip.clone() }).collect()
    };
    let train_rms = per_joint_rms(&to_owned(&train), &pm);
    let test_rms = per_joint_rms(&to_owned(&test), &pm);
    if test_rms > SAG_VALIDATE_MAX_MRAD || test_rms > (SAG_VALIDATE_RATIO * train_rms).max(1.0) {
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
fn self_collision_free(chain: &k::Chain<f64>) -> bool {
    let pts = joint_points(chain);
    for i in 1..pts.len() {
        for j in (i + 2)..pts.len() {
            let dist = seg_seg_distance(v3(pts[i - 1]), v3(pts[i]), v3(pts[j - 1]), v3(pts[j]));
            if dist < RADII[i] + RADII[j] {
                return false;
            }
        }
    }
    true
}

/// Minimum clearance (m) between any non-adjacent link capsules (surface gap).
fn min_self_clearance(chain: &k::Chain<f64>) -> f64 {
    let pts = joint_points(chain);
    let mut best = f64::INFINITY;
    for i in 1..pts.len() {
        for j in (i + 2)..pts.len() {
            let dist = seg_seg_distance(v3(pts[i - 1]), v3(pts[i]), v3(pts[j - 1]), v3(pts[j]));
            best = best.min(dist - RADII[i] - RADII[j]);
        }
    }
    best
}

/// Is the pose free of (non-adjacent) link self-collision? Deliberate contacts must
/// be excepted by the caller.
#[pg_extern]
fn rebot_self_collision_free(urdf: &str, q: Vec<f64>) -> bool {
    let (_robot, chain) = load_chain_arm(urdf, &q);
    self_collision_free(&chain)
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
fn rebot_min_self_clearance(urdf: &str, q: Vec<f64>) -> f64 {
    let (_robot, chain) = load_chain_arm(urdf, &q);
    min_self_clearance(&chain)
}

/// Is the straight joint-space line a->b free of self-collision at every sample?
#[pg_extern]
fn rebot_path_collision_free(urdf: &str, a: Vec<f64>, b: Vec<f64>, samples: default!(i32, 40)) -> bool {
    let (_robot, chain) = load_chain_arm(urdf, &a);
    let samples = samples as usize;
    if samples < 2 { panic!("samples must be >= 2"); }
    for s in 0..samples {
        let t = s as f64 / (samples as f64 - 1.0);
        let mut q: Vec<f64> = a.iter().zip(b.iter()).map(|(x, y)| x + (y - x) * t).collect();
        while q.len() < chain.dof() { q.push(0.0); }
        chain.set_joint_positions(&q).unwrap_or_else(|e| panic!("set_joint_positions: {e}"));
        chain.update_transforms();
        if !self_collision_free(&chain) { return false; }
    }
    true
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
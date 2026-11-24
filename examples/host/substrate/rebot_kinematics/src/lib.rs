//! reBot kinematics, dynamics, collision, planning, and calibration as a
//! memory-safe pgrx Postgres extension (pure Rust: openrr/k + nalgebra).
//!
//! No C++ FFI: bad input raises a SQL ERROR (pgrx catches panics at the FFI
//! boundary), never a backend crash. All exported functions are pure
//! (`immutable, parallel_safe`).

use k::prelude::*;
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use pgrx::prelude::*;

pg_module_magic!();

mod calib;
mod camera;
mod config;
mod model;
mod plan;

pub(crate) use config::Config;
use config::{json_finite, json_pos_usize};
use model::{seg_seg_distance, ArmModel, V3};

fn load_chain(urdf: &str) -> k::Chain<f64> {
    let robot = urdf_rs::read_from_string(urdf)
        .unwrap_or_else(|e| panic!("URDF parse failed: {e}"));
    k::Chain::<f64>::from(&robot)
}

/// Forward kinematics: world pose of `end_joint` at joint positions `q` (rad).
/// Returns `[x, y, z, qw, qx, qy, qz]` — metres + unit quaternion.
#[pg_extern(immutable, parallel_safe)]
fn rebot_fk(urdf: &str, q: Vec<f64>, end_joint: &str) -> Vec<f64> {
    if !q.iter().all(|v| v.is_finite()) {
        panic!("joint positions must be finite");
    }
    let chain = load_chain(urdf);
    if q.len() != chain.dof() {
        panic!(
            "joint length mismatch: expected {}, got {}",
            chain.dof(),
            q.len()
        );
    }
    chain.set_joint_positions_unchecked(&q);
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

/// Inverse kinematics: joint positions reaching `target` = `[x, y, z, qw, qx, qy, qz]`
/// from seed `q0` along serial chain ending at `end_joint`.
#[pg_extern(immutable, parallel_safe)]
fn rebot_ik(urdf: &str, end_joint: &str, target: Vec<f64>, q0: Vec<f64>) -> Vec<f64> {
    if target.len() != 7 || !target.iter().all(|v| v.is_finite()) {
        panic!("target must be finite [x, y, z, qw, qx, qy, qz]");
    }
    if !q0.iter().all(|v| v.is_finite()) {
        panic!("seed q0 must be finite");
    }
    let q_quat = nalgebra::Quaternion::new(target[3], target[4], target[5], target[6]);
    let quat = UnitQuaternion::try_new(q_quat, 1e-12)
        .unwrap_or_else(|| panic!("target orientation quaternion must have non-zero norm"));
    let chain = load_chain(urdf);
    let node = chain
        .find(end_joint)
        .unwrap_or_else(|| panic!("joint not found: {end_joint}"));
    let arm = k::SerialChain::from_end(node);
    arm.set_joint_positions(&q0)
        .unwrap_or_else(|e| panic!("seed q0: {e}"));
    let pose = Isometry3::from_parts(Translation3::new(target[0], target[1], target[2]), quat);
    let solver = k::JacobianIkSolver::<f64>::default();
    solver
        .solve(&arm, &pose)
        .unwrap_or_else(|e| panic!("IK did not converge: {e}"));
    arm.joint_positions()
}

/// Degrees of freedom of the URDF chain.
#[pg_extern(immutable, parallel_safe)]
fn rebot_dof(urdf: &str) -> i32 {
    load_chain(urdf).dof() as i32
}

/// `gripper_end` position in the base frame at 6 arm joint angles `q6`.
#[pg_extern(immutable, parallel_safe)]
fn rebot_fk_tip(urdf: &str, q6: Vec<f64>) -> Vec<f64> {
    let arm = ArmModel::new(urdf);
    let p = arm.fk_pose(&q6).translation.vector;
    vec![p.x, p.y, p.z]
}

/// Tool tip point in the base frame: `gripper_end + R(gripper_end) * tip_offset`.
#[pg_extern(immutable, parallel_safe)]
fn rebot_tip_point(urdf: &str, q6: Vec<f64>, tip_offset: Vec<f64>) -> Vec<f64> {
    let arm = ArmModel::new(urdf);
    let p = arm.tip_point(&q6, &tip_offset);
    vec![p.x, p.y, p.z]
}

/// Joint torques (Nm, 6 arm joints) to hold pose `q` against gravity.
#[pg_extern(immutable, parallel_safe)]
fn rebot_gravity_torques(urdf: &str, q: Vec<f64>, config: pgrx::JsonB) -> Vec<f64> {
    let cfg = Config::from_json(&config.0);
    let arm = ArmModel::new(urdf);
    arm.gravity(&q, &cfg).to_vec()
}

/// Joint origins + `gripper_end` (7 points, flattened to 21 floats) at pose `q`.
#[pg_extern(immutable, parallel_safe)]
fn rebot_joint_points(urdf: &str, q: Vec<f64>) -> Vec<f64> {
    let arm = ArmModel::new(urdf);
    arm.joint_points(&q)
        .iter()
        .flat_map(|p| p.iter().copied())
        .collect()
}

/// Is the straight joint-space interpolation `a -> b` clear of the worktop (`sweep.path_safe`)?
#[pg_extern(immutable, parallel_safe)]
fn rebot_path_safe(
    urdf: &str,
    a: Vec<f64>,
    b: Vec<f64>,
    config: pgrx::JsonB,
    samples: default!(i32, 40),
) -> bool {
    if !(2..=10_000).contains(&samples) {
        panic!("samples must be in 2..=10000, got {samples}");
    }
    let cfg = Config::from_json(&config.0);
    let arm = ArmModel::new(urdf);
    arm.path_safe(&a, &b, samples as usize, &cfg)
}

// --------------------------------------------------------------------------- //
// Sag / gain residual map fitting — port of identify.fit_sag_map (§7.3b).
// --------------------------------------------------------------------------- //

struct SagHold {
    q: Vec<f64>,
    tau: [f64; 6],
    r: [f64; 6],
    tip: [f64; 3],
}

fn parse_fixed_arr<const N: usize>(v: &serde_json::Value, ctx: &str) -> [f64; N] {
    let a = v
        .as_array()
        .unwrap_or_else(|| panic!("{ctx} must be an array of {N} numbers"));
    if a.len() < N {
        panic!("{ctx} must have at least {N} numbers, got {}", a.len());
    }
    let mut out = [0.0; N];
    for i in 0..N {
        out[i] = json_finite(&a[i], ctx);
    }
    out
}

fn parse_holds(v: &serde_json::Value) -> Vec<SagHold> {
    let Some(arr) = v.as_array() else {
        return Vec::new();
    };
    arr.iter()
        .enumerate()
        .map(|(idx, h)| {
            let qa = h
                .get("q")
                .and_then(|x| x.as_array())
                .unwrap_or_else(|| panic!("hold {idx} missing q array"));
            if qa.len() < 6 {
                panic!("hold {idx} q must have at least 6 angles");
            }
            let q: Vec<f64> = qa.iter().map(|x| json_finite(x, "hold.q")).collect();
            SagHold {
                q,
                tau: parse_fixed_arr::<6>(&h["tau"], "hold.tau"),
                r: parse_fixed_arr::<6>(&h["r"], "hold.r"),
                tip: parse_fixed_arr::<3>(&h["tip"], "hold.tip"),
            }
        })
        .collect()
}

fn wls_affine(x: &[f64], y: &[f64], w: &[f64]) -> (f64, f64) {
    let (mut sxx, mut sx, mut sw, mut sxy, mut sy) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for i in 0..x.len() {
        let wi = w[i];
        sxx += wi * x[i] * x[i];
        sx += wi * x[i];
        sw += wi;
        sxy += wi * x[i] * y[i];
        sy += wi * y[i];
    }
    let det = sxx * sw - sx * sx;
    if det.abs() < 1e-12 {
        (0.0, sy / sw.max(1e-12))
    } else {
        ((sxy * sw - sy * sx) / det, (sxx * sy - sx * sxy) / det)
    }
}

fn huber_affine(x: &[f64], y: &[f64], huber_rad: f64) -> (f64, f64) {
    let n = x.len();
    let ones = vec![1.0; n];
    let mut coef = wls_affine(x, y, &ones);
    for _ in 0..20 {
        let w: Vec<f64> = (0..n)
            .map(|i| {
                let r = (y[i] - (coef.0 * x[i] + coef.1)).abs();
                if r > huber_rad {
                    huber_rad / r.max(1e-12)
                } else {
                    1.0
                }
            })
            .collect();
        coef = wls_affine(x, y, &w);
    }
    coef
}

fn per_joint_rms(holds: &[&SagHold], pm: &[(f64, f64); 6]) -> f64 {
    if holds.is_empty() {
        return 0.0;
    }
    let mut total = 0.0;
    for h in holds {
        for j in 0..6 {
            let pred = pm[j].0 * h.tau[j] + pm[j].1;
            total += (h.r[j] - pred).powi(2);
        }
    }
    1000.0 * (total / (holds.len() * 6) as f64).sqrt()
}

fn tool_mm_per_rad(arm: &ArmModel, q6: &[f64]) -> [f64; 6] {
    let base = arm.fk_pose(q6).translation.vector;
    let mut out = [0.0; 6];
    let mut moved = [0.0; 6];
    moved.copy_from_slice(&q6[..6]);
    for j in 0..6 {
        let orig = moved[j];
        moved[j] = orig + 1e-4;
        let shifted = arm.fk_pose(&moved).translation.vector;
        out[j] = (shifted - base).norm() / 1e-4;
        moved[j] = orig;
    }
    out
}

/// Fit the per-joint sag residual map from hold samples (`identify.fit_sag_map`).
#[pg_extern(immutable, parallel_safe)]
fn rebot_fit_sag_map(
    urdf: &str,
    holds: pgrx::JsonB,
    kp_key: &str,
    rig_fingerprint: &str,
    zeros_key: &str,
    config: pgrx::JsonB,
    min_holds: default!(Option<i32>, "NULL"),
) -> pgrx::JsonB {
    use serde_json::json;
    let cfg = config
        .0
        .as_object()
        .unwrap_or_else(|| panic!("learning config must be a jsonb object"));
    let gate = |key: &str| -> f64 {
        let v = cfg
            .get(key)
            .unwrap_or_else(|| panic!("learning config missing required key '{key}'"));
        json_finite(v, &format!("learning config {key}"))
    };
    let sag_huber_rad = gate("sag_huber_rad");
    let sag_validate_max_mrad = gate("sag_validate_max_mrad");
    let sag_validate_ratio = gate("sag_validate_ratio");
    let sag_span_radius_m = gate("sag_span_radius_m");
    let sag_span_height_m = gate("sag_span_height_m");
    let hs = parse_holds(&holds.0);
    let min_holds = match min_holds {
        Some(v) => {
            if v < 1 {
                panic!("min_holds must be >= 1, got {v}");
            }
            v as usize
        }
        None => json_pos_usize(
            cfg.get("sag_min_holds")
                .unwrap_or_else(|| panic!("learning config missing required key 'sag_min_holds'")),
            "learning config sag_min_holds",
            100_000,
        ),
    };
    if hs.len() < min_holds {
        return pgrx::JsonB(json!({
            "ok": false,
            "reasons": [format!("only {} holds (need {}): let more executions accumulate", hs.len(), min_holds)]
        }));
    }
    let radius: Vec<f64> = hs
        .iter()
        .map(|h| (h.tip[0].powi(2) + h.tip[1].powi(2)).sqrt())
        .collect();
    let height: Vec<f64> = hs.iter().map(|h| h.tip[2]).collect();
    let rmin = radius.iter().cloned().fold(f64::INFINITY, f64::min);
    let rmax = radius.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let hmin = height.iter().cloned().fold(f64::INFINITY, f64::min);
    let hmax = height.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    if rmax - rmin < sag_span_radius_m || hmax - hmin < sag_span_height_m {
        return pgrx::JsonB(json!({
            "ok": false,
            "reasons": [format!("the holds do not span the workspace (radius span {:.2} m, height span {:.2} m)", rmax - rmin, hmax - hmin)]
        }));
    }
    let train: Vec<&SagHold> = hs
        .iter()
        .enumerate()
        .filter(|(i, _)| i % 5 != 4)
        .map(|(_, h)| h)
        .collect();
    let test: Vec<&SagHold> = hs
        .iter()
        .enumerate()
        .filter(|(i, _)| i % 5 == 4)
        .map(|(_, h)| h)
        .collect();
    let mut pm = [(0.0_f64, 0.0_f64); 6];
    for j in 0..6 {
        let x: Vec<f64> = train.iter().map(|h| h.tau[j]).collect();
        let y: Vec<f64> = train.iter().map(|h| h.r[j]).collect();
        pm[j] = huber_affine(&x, &y, sag_huber_rad);
    }
    let train_rms = per_joint_rms(&train, &pm);
    let test_rms = per_joint_rms(&test, &pm);
    if test_rms > sag_validate_max_mrad || test_rms > (sag_validate_ratio * train_rms).max(1.0) {
        return pgrx::JsonB(json!({
            "ok": false,
            "rms_mrad": (test_rms * 1000.0).round_ties_even() / 1000.0,
            "reasons": [format!("hold-out validation failed: {:.1} mrad held out vs {:.1} mrad train (an overfit map is not applied)", test_rms, train_rms)]
        }));
    }
    let dof = hs.iter().map(|h| h.q.len()).min().unwrap_or(6);
    let mut q_min = vec![f64::INFINITY; dof];
    let mut q_max = vec![f64::NEG_INFINITY; dof];
    for h in &hs {
        for j in 0..dof {
            q_min[j] = q_min[j].min(h.q[j]);
            q_max[j] = q_max[j].max(h.q[j]);
        }
    }
    let mean_q6: Vec<f64> = (0..6)
        .map(|j| hs.iter().map(|h| h.q[j]).sum::<f64>() / hs.len() as f64)
        .collect();
    let arm = ArmModel::new(urdf);
    let sens = tool_mm_per_rad(&arm, &mean_q6);
    let mut per_joint = [0.0_f64; 6];
    for j in 0..6 {
        let mut acc = 0.0;
        for h in &test {
            let e = h.r[j] - (pm[j].0 * h.tau[j] + pm[j].1);
            acc += e * e;
        }
        per_joint[j] = if test.is_empty() {
            0.0
        } else {
            (acc / test.len() as f64).sqrt()
        };
    }
    let validated_mm = 1000.0
        * (0..6)
            .map(|j| (per_joint[j] * sens[j]).powi(2))
            .sum::<f64>()
            .sqrt();
    let per_motor: serde_json::Map<String, serde_json::Value> = (0..6)
        .map(|j| (format!("{}", j + 1), json!({"a": pm[j].0, "b": pm[j].1})))
        .collect();
    pgrx::JsonB(json!({
        "ok": true,
        "rms_mrad": (test_rms * 1000.0).round_ties_even() / 1000.0,
        "train_rms_mrad": (train_rms * 1000.0).round_ties_even() / 1000.0,
        "map": {
            "version": 1,
            "kp_key": kp_key,
            "rig_fingerprint": rig_fingerprint,
            "zeros_key": zeros_key,
            "per_motor": per_motor,
            "samples": hs.len(),
            "rms_mrad": (test_rms * 1000.0).round_ties_even() / 1000.0,
            "validated_mm": (validated_mm * 100.0).round_ties_even() / 100.0,
            "envelope": {"q_min": q_min, "q_max": q_max, "radius_m": [rmin, rmax], "height_m": [hmin, hmax]}
        }
    }))
}

// --------------------------------------------------------------------------- //
// Capsule-capsule self-collision & inter-arm segment distance.
// --------------------------------------------------------------------------- //

#[pg_extern(immutable, parallel_safe)]
fn rebot_self_collision_free(urdf: &str, q: Vec<f64>, config: pgrx::JsonB) -> bool {
    let cfg = Config::from_json(&config.0);
    let arm = ArmModel::new(urdf);
    arm.self_collision_free(&q, &cfg)
}

#[pg_extern(immutable, parallel_safe)]
fn rebot_seg_seg_distance(p1: Vec<f64>, q1: Vec<f64>, p2: Vec<f64>, q2: Vec<f64>) -> f64 {
    let v = |a: Vec<f64>| {
        if a.len() != 3 || !a.iter().all(|x| x.is_finite()) {
            panic!("segment endpoint must be finite [x,y,z]");
        }
        V3::new(a[0], a[1], a[2])
    };
    seg_seg_distance(v(p1), v(q1), v(p2), v(q2))
}

#[pg_extern(immutable, parallel_safe)]
fn rebot_min_self_clearance(urdf: &str, q: Vec<f64>, config: pgrx::JsonB) -> f64 {
    let cfg = Config::from_json(&config.0);
    let arm = ArmModel::new(urdf);
    arm.min_self_clearance(&q, &cfg)
}

// --------------------------------------------------------------------------- //
// Joint limits + distorted projection + ray-plane intersection.
// --------------------------------------------------------------------------- //

#[pg_extern(immutable, parallel_safe)]
fn rebot_within_limits(
    q: Vec<f64>,
    config: pgrx::JsonB,
    margin: default!(f64, 0.0),
) -> bool {
    let cfg = Config::from_json(&config.0);
    if q.len() != 6 || !q.iter().all(|v| v.is_finite()) || !margin.is_finite() {
        panic!("expected 6 finite joint angles and finite margin");
    }
    q.iter()
        .zip(cfg.limits.iter())
        .all(|(v, l)| *v >= l[0] + margin && *v <= l[1] - margin)
}

/// Project a base-frame point with optional Brown-Conrady distortion (`dist` may be empty).
#[pg_extern(immutable, parallel_safe)]
fn rebot_project_dist(
    k: Vec<f64>,
    t_base_cam: Vec<f64>,
    point: Vec<f64>,
    dist: Vec<f64>,
) -> Option<Vec<f64>> {
    camera::project_dist(&k, &t_base_cam, &point, &dist)
        .unwrap_or_else(|e| panic!("{e}"))
        .map(|uv| uv.to_vec())
}

/// Where the pixel's ray meets the horizontal plane (`extrinsics.ray_plane`).
#[pg_extern(immutable, parallel_safe)]
fn rebot_ray_plane(
    k: Vec<f64>,
    dist: Vec<f64>,
    t_base_cam: Vec<f64>,
    u: f64,
    v: f64,
    plane_z: f64,
) -> pgrx::JsonB {
    let (point, distance, grazing) = camera::ray_plane_impl(&k, &dist, &t_base_cam, u, v, plane_z)
        .unwrap_or_else(|e| panic!("{e}"));
    pgrx::JsonB(serde_json::json!({
        "point": point,
        "distance": distance,
        "grazing_deg": grazing,
    }))
}

#[cfg(any(test, feature = "pg_test"))]
#[pg_schema]
mod tests {
    use pgrx::prelude::*;

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

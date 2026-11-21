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
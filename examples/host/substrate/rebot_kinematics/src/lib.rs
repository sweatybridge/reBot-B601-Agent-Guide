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
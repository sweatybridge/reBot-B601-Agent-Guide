//! Shared URDF kinematic & dynamic model (`ArmModel`).
//!
//! Parsed ONCE per SQL call and reused across FK, Jacobian, gravity (both the
//! simplified `cfg.link_inertials` table and full URDF RNEA), worktop capsule
//! safety, and self-collision checks — eliminating thousands of redundant XML
//! parses and HashSet allocations inside planning and calibration loops.
//!
//! Joint-position updates validate finiteness and length, then call
//! `set_joint_positions_unchecked` so FK/gravity/Jacobian evaluations never
//! panic on real torque-off encoder sag (j2 < 0), near-stop sag compensation,
//! or finite-difference zero perturbations. Limit enforcement lives explicitly
//! in `within_limits`, IK clamping, and trajectory validation against
//! `world.arm_config.limits`.

use std::collections::{HashMap, HashSet};

use nalgebra::{Isometry3, Matrix6, Vector3};

use crate::config::Config;

pub(crate) const ARM_JOINTS: [&str; 6] = [
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
];

pub(crate) type V3 = Vector3<f64>;

/// Precomputed inertial body from the full URDF for RNEA gravity torques.
/// `downstream_mask` has bit `j` set iff the body is downstream of `ARM_JOINTS[j]`.
struct UrdfBody {
    link_name: String,
    mass: f64,
    com_local: V3,
    downstream_mask: u8,
}

pub(crate) struct ArmModel {
    chain: k::Chain<f64>,
    axes: [V3; 6],
    urdf_bodies: Vec<UrdfBody>,
}

impl ArmModel {
    pub fn new(urdf: &str) -> Self {
        let robot = urdf_rs::read_from_string(urdf)
            .unwrap_or_else(|e| panic!("URDF parse failed: {e}"));
        let chain = k::Chain::<f64>::from(&robot);

        let mut axes = [V3::zeros(); 6];
        for (idx, name) in ARM_JOINTS.iter().enumerate() {
            let j = robot
                .joints
                .iter()
                .find(|j| j.name == *name)
                .unwrap_or_else(|| panic!("joint not in URDF: {name}"));
            let a = *j.axis.xyz;
            axes[idx] = V3::new(a[0], a[1], a[2]);
        }

        // Precompute parent-link graph and per-link downstream bitmask for the 6 arm joints.
        let mut parent: HashMap<&str, &str> = HashMap::new();
        let mut joint_child: HashMap<&str, &str> = HashMap::new();
        for j in &robot.joints {
            parent.insert(j.child.link.as_str(), j.parent.link.as_str());
            joint_child.insert(j.name.as_str(), j.child.link.as_str());
        }
        let child_links: [&str; 6] =
            std::array::from_fn(|j| joint_child.get(ARM_JOINTS[j]).copied().unwrap_or(""));

        let mut urdf_bodies = Vec::new();
        for l in &robot.links {
            let mass = l.inertial.mass.value;
            if mass <= 0.0 || chain.find_link(&l.name).is_none() {
                continue;
            }
            let mut ancs: HashSet<&str> = HashSet::new();
            let mut cur = l.name.as_str();
            ancs.insert(cur);
            while let Some(&p) = parent.get(cur) {
                if !ancs.insert(p) {
                    break;
                }
                cur = p;
            }
            let mut mask = 0_u8;
            for (j, child) in child_links.iter().enumerate() {
                if !child.is_empty() && ancs.contains(child) {
                    mask |= 1 << j;
                }
            }
            if mask != 0 {
                let xyz = l.inertial.origin.xyz;
                urdf_bodies.push(UrdfBody {
                    link_name: l.name.clone(),
                    mass,
                    com_local: V3::new(xyz[0], xyz[1], xyz[2]),
                    downstream_mask: mask,
                });
            }
        }

        ArmModel {
            chain,
            axes,
            urdf_bodies,
        }
    }

    /// Validate finiteness and set the 6 arm joints (padding any extra gripper
    /// DOF with 0.0) without URDF limit-tag panics, then update transforms.
    pub fn set_arm_joints(&self, q6: &[f64]) {
        if q6.len() != 6 {
            panic!("expected 6 arm joint angles, got {}", q6.len());
        }
        if !q6.iter().all(|v| v.is_finite()) {
            panic!("joint angles must be finite");
        }
        let dof = self.chain.dof();
        let mut q = Vec::with_capacity(dof.max(6));
        q.extend_from_slice(q6);
        while q.len() < dof {
            q.push(0.0);
        }
        q.truncate(dof);
        self.chain.set_joint_positions_unchecked(&q);
        self.chain.update_transforms();
    }

    /// World pose of `gripper_end` at `q6`.
    pub fn fk_pose(&self, q6: &[f64]) -> Isometry3<f64> {
        self.set_arm_joints(q6);
        let node = self
            .chain
            .find_link("gripper_end")
            .unwrap_or_else(|| panic!("gripper_end not found"));
        let t = node
            .world_transform()
            .unwrap_or_else(|| panic!("no transform for gripper_end"));
        Isometry3::from_parts(t.translation, t.rotation)
    }

    /// Tool tip point in the base frame: `gripper_end + R(gripper_end) * tip_offset`.
    pub fn tip_point(&self, q6: &[f64], tip_offset: &[f64]) -> V3 {
        if tip_offset.len() != 3 || !tip_offset.iter().all(|v| v.is_finite()) {
            panic!("tip_offset must be finite [x, y, z]");
        }
        let t = self.fk_pose(q6);
        let off = V3::new(tip_offset[0], tip_offset[1], tip_offset[2]);
        t.translation.vector + t.rotation * off
    }

    /// Joint origins (joint1..6) plus `gripper_end` in the base frame (7 points).
    pub fn joint_points_current(&self) -> [[f64; 3]; 7] {
        let mut pts = [[0.0; 3]; 7];
        for (i, name) in ARM_JOINTS.iter().enumerate() {
            let node = self
                .chain
                .find(name)
                .unwrap_or_else(|| panic!("joint not found: {name}"));
            let p = node.world_transform().unwrap().translation.vector;
            pts[i] = [p.x, p.y, p.z];
        }
        let tool = self
            .chain
            .find_link("gripper_end")
            .unwrap_or_else(|| panic!("gripper_end link not found"));
        let p = tool.world_transform().unwrap().translation.vector;
        pts[6] = [p.x, p.y, p.z];
        pts
    }

    pub fn joint_points(&self, q6: &[f64]) -> [[f64; 3]; 7] {
        self.set_arm_joints(q6);
        self.joint_points_current()
    }

    /// Simplified gravity torques using `cfg.link_inertials` (matches `rebot_ik.gravity_torques`).
    pub fn gravity(&self, q6: &[f64], cfg: &Config) -> [f64; 6] {
        self.set_arm_joints(q6);
        let mut coms: Vec<(f64, V3)> = Vec::with_capacity(cfg.link_inertials.len());
        for (lname, mass, com) in &cfg.link_inertials {
            let node = self
                .chain
                .find_link(lname)
                .unwrap_or_else(|| panic!("link not found: {lname}"));
            let t = node.world_transform().unwrap();
            let p_com = t.rotation * V3::new(com[0], com[1], com[2]) + t.translation.vector;
            coms.push((*mass, p_com));
        }
        let mut tau = [0.0; 6];
        for j in 0..6 {
            let node = self.chain.find(ARM_JOINTS[j]).unwrap();
            let t = node.world_transform().unwrap();
            let p_j = t.translation.vector;
            let z_j = t.rotation * self.axes[j];
            let mut total = 0.0;
            for (mass, p_com) in coms.iter().skip(j) {
                let force = V3::new(0.0, 0.0, -mass * cfg.gravity);
                total += z_j.dot(&(p_com - p_j).cross(&force));
            }
            tau[j] = -total;
        }
        tau
    }

    /// Generalized gravity torques from ALL URDF link inertials (matches Pinocchio RNEA).
    pub fn gravity_urdf(&self, q6: &[f64], gravity_g: f64) -> [f64; 6] {
        self.set_arm_joints(q6);
        let mut world_bodies: Vec<(f64, V3, u8)> = Vec::with_capacity(self.urdf_bodies.len());
        for b in &self.urdf_bodies {
            let node = self.chain.find_link(&b.link_name).unwrap();
            let t = node.world_transform().unwrap();
            let com = t.rotation * b.com_local + t.translation.vector;
            world_bodies.push((b.mass, com, b.downstream_mask));
        }
        let mut tau = [0.0; 6];
        for j in 0..6 {
            let bit = 1_u8 << j;
            let node = self.chain.find(ARM_JOINTS[j]).unwrap();
            let t = node.world_transform().unwrap();
            let p_j = t.translation.vector;
            let z_j = t.rotation * self.axes[j];
            let mut total = 0.0;
            for &(mass, com, mask) in &world_bodies {
                if mask & bit == 0 {
                    continue;
                }
                let force = V3::new(0.0, 0.0, -mass * gravity_g);
                total += z_j.dot(&(com - p_j).cross(&force));
            }
            tau[j] = -total;
        }
        tau
    }

    /// Body (LOCAL-frame) 6x6 Jacobian of `gripper_end`, columns `[linear; angular]`.
    pub fn jacobian(&self, q6: &[f64], ee: &Isometry3<f64>) -> Matrix6<f64> {
        self.set_arm_joints(q6);
        let r_t = ee.rotation.to_rotation_matrix().matrix().transpose();
        let p_ee = ee.translation.vector;
        let mut jac = Matrix6::zeros();
        for col in 0..6 {
            let node = self.chain.find(ARM_JOINTS[col]).unwrap();
            let t = node.world_transform().unwrap();
            let z = t.rotation * self.axes[col];
            let p_j = t.translation.vector;
            let lin = r_t * z.cross(&(p_ee - p_j));
            let ang = r_t * z;
            for r in 0..3 {
                jac[(r, col)] = lin[r];
                jac[(r + 3, col)] = ang[r];
            }
        }
        jac
    }

    /// Worktop capsule clearance + reach cylinder gate for a single pose `q6`.
    pub fn pose_safe(&self, q6: &[f64], cfg: &Config) -> bool {
        let points = self.joint_points(q6);
        for index in 1..points.len() {
            let start = points[index - 1];
            let end = points[index];
            if !start.iter().chain(end.iter()).all(|v| v.is_finite()) {
                return false;
            }
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
        true
    }

    /// Straight joint-space interpolation `a -> b` worktop gate (`sweep.path_safe`).
    pub fn path_safe(&self, a: &[f64], b: &[f64], samples: usize, cfg: &Config) -> bool {
        if a.len() != 6 || b.len() != 6 {
            panic!("path_safe expects 6-joint poses, got {} and {}", a.len(), b.len());
        }
        if !a.iter().chain(b.iter()).all(|v| v.is_finite()) {
            panic!("path_safe poses must be finite");
        }
        if !(2..=10_000).contains(&samples) {
            panic!("samples must be in 2..=10000, got {samples}");
        }
        let mut q = [0.0; 6];
        for s in 0..samples {
            let t = s as f64 / (samples as f64 - 1.0);
            for j in 0..6 {
                q[j] = a[j] + (b[j] - a[j]) * t;
            }
            if !self.pose_safe(&q, cfg) {
                return false;
            }
        }
        true
    }

    /// Capsule worktop gate on a dense sample series (`sweep.samples_safe`).
    pub fn samples_safe(&self, samples: &[[f64; 6]], cfg: &Config) -> bool {
        for q6 in samples {
            if !self.pose_safe(q6, cfg) {
                return false;
            }
        }
        true
    }

    /// Minimum surface gap (m) between non-adjacent link capsules; negative = intersecting.
    pub fn min_self_clearance(&self, q6: &[f64], cfg: &Config) -> f64 {
        let pts = self.joint_points(q6);
        let mut best = f64::INFINITY;
        for i in 1..pts.len() {
            for j in (i + 2)..pts.len() {
                let dist = seg_seg_distance(
                    v3(pts[i - 1]),
                    v3(pts[i]),
                    v3(pts[j - 1]),
                    v3(pts[j]),
                );
                if !dist.is_finite() {
                    return f64::NEG_INFINITY;
                }
                best = best.min(dist - cfg.link_radii[i] - cfg.link_radii[j]);
            }
        }
        best
    }

    /// True if no non-adjacent link capsules intersect at `q6`.
    pub fn self_collision_free(&self, q6: &[f64], cfg: &Config) -> bool {
        let gap = self.min_self_clearance(q6, cfg);
        gap.is_finite() && gap >= 0.0
    }
}

#[inline]
fn v3(p: [f64; 3]) -> V3 {
    V3::new(p[0], p[1], p[2])
}

/// Closest distance between segments p1-q1 and p2-q2 (Ericson, Real-Time Collision
/// Detection). Used for intra-arm and inter-arm capsule clearance.
pub(crate) fn seg_seg_distance(p1: V3, q1: V3, p2: V3, q2: V3) -> f64 {
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
            s = if denom.abs() > eps {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
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

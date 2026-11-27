//! Exact convex-hull mesh self-collision and table surface clearance via `parry3d-f64`.
//!
//! Uses the precomputed per-link CAD convex hulls in `crate::meshes::B601_LINK_HULLS`
//! (extracted from `examples/host/app/cad/meshes/*.STL` with <0.5 mm support error).
//! Polyhedra are built ONCE per Postgres worker via `OnceLock` so per-pose checks
//! take microseconds with zero runtime filesystem I/O.

use std::sync::OnceLock;

use nalgebra::{Isometry3, Point3};
use parry3d_f64::query;
use parry3d_f64::shape::ConvexPolyhedron;

use crate::meshes::B601_LINK_HULLS;
use crate::model::ArmModel;

/// Non-adjacent link index pairs into `B601_LINK_HULLS` checked for self-collision.
///
/// Link indices:
///   0: base_link, 1: link1, 2: link2, 3: link3, 4: link4,
///   5: link5, 6: link6, 7: gripper_end, 8: gripper_left, 9: gripper_right.
///
/// Excluded:
/// - Adjacent joint-connected pairs (0-1, 1-2, 2-3, 3-4, 4-5, 5-6, 6-7, 7-8, 7-9)
/// - Opposing gripper fingers (8-9), which close to contact
/// - Rigidly coupled flange/jaw housings (5-7, 6-8, 6-9) whose convex hulls graze
///   at normal wrist rotations
const CHECKED_PAIRS: &[(usize, usize)] = &[
    // base_link against forearm, wrist, and gripper
    (0, 2),
    (0, 3),
    (0, 4),
    (0, 5),
    (0, 6),
    (0, 7),
    (0, 8),
    (0, 9),
    // link1 against forearm, wrist, and gripper
    (1, 3),
    (1, 4),
    (1, 5),
    (1, 6),
    (1, 7),
    (1, 8),
    (1, 9),
    // link2 (upper arm) against wrist and gripper — catches the b602e4 j4-into-j2 bind
    (2, 4),
    (2, 5),
    (2, 6),
    (2, 7),
    (2, 8),
    (2, 9),
    // link3 (forearm) against distal wrist and gripper
    (3, 5),
    (3, 6),
    (3, 7),
    (3, 8),
    (3, 9),
    // link4 against flange and gripper
    (4, 6),
    (4, 7),
    (4, 8),
    (4, 9),
    // link5 against gripper fingers
    (5, 8),
    (5, 9),
];

/// Moving links checked against the worktop plane (excludes static `base_link`
/// resting at z=0 and `link1` rotating on top of `base_link`).
const MOVING_LINK_INDICES: &[usize] = &[2, 3, 4, 5, 6, 7, 8, 9];

/// Maximum convex-hull penetration (m) tolerated for modeled self-collision
/// (accounts for convex-hull wrapping of concave motor/bracket cutouts and
/// minor resting contact when parked on the j2/j3 stops).
pub(crate) const MESH_SELF_PENETRATION_TOL_M: f64 = 0.008;

/// Noise floor (m) for deciding whether a path step is descending further below
/// the table surface (mirrors RUA's `surface_guard::DESCENT_NOISE_METERS`).
pub(crate) const DESCENT_NOISE_M: f64 = 1e-6;

struct LinkHull {
    name: &'static str,
    poly: ConvexPolyhedron,
    points: Vec<Point3<f64>>,
}

struct B601Hulls {
    links: Vec<LinkHull>,
}

static HULLS: OnceLock<B601Hulls> = OnceLock::new();

fn b601_hulls() -> &'static B601Hulls {
    HULLS.get_or_init(|| {
        let mut links = Vec::with_capacity(B601_LINK_HULLS.len());
        for def in B601_LINK_HULLS {
            let points: Vec<Point3<f64>> = def
                .vertices
                .iter()
                .map(|v| Point3::new(v[0], v[1], v[2]))
                .collect();
            let poly = ConvexPolyhedron::from_convex_hull(&points)
                .unwrap_or_else(|| panic!("convex hull failed for link {}", def.name));
            links.push(LinkHull {
                name: def.name,
                poly,
                points,
            });
        }
        B601Hulls { links }
    })
}

/// Collect world poses of all 10 B601 links from an already-updated `ArmModel`.
/// Returns `None` if the URDF does not contain the standard B601 links (e.g. a
/// synthetic test URDF).
fn link_world_poses(arm: &ArmModel) -> Option<[Isometry3<f64>; 10]> {
    let hulls = b601_hulls();
    let mut poses = [Isometry3::identity(); 10];
    for (idx, link) in hulls.links.iter().enumerate() {
        poses[idx] = arm.link_world_transform(link.name)?;
    }
    Some(poses)
}

/// Exact signed surface clearance (m) between all checked non-adjacent B601
/// link convex hulls at `(q6, jaw_slide_m)`.
///
/// Positive = separated by that distance; negative = penetration depth.
/// Returns `None` if the URDF is not a B601 link tree.
pub(crate) fn mesh_self_clearance(arm: &ArmModel, q6: &[f64], jaw_slide_m: f64) -> Option<f64> {
    arm.set_arm_and_jaw(q6, jaw_slide_m);
    mesh_self_clearance_current(arm)
}

pub(crate) fn mesh_self_clearance_current(arm: &ArmModel) -> Option<f64> {
    let hulls = b601_hulls();
    let poses = link_world_poses(arm)?;
    let mut min_signed = f64::INFINITY;
    for &(i, j) in CHECKED_PAIRS {
        let poly_a = &hulls.links[i].poly;
        let poly_b = &hulls.links[j].poly;
        // query::contact with a generous prediction returns exact signed
        // distance (negative when penetrating, positive when separated).
        let signed = match query::contact(&poses[i], poly_a, &poses[j], poly_b, 0.25) {
            Ok(Some(contact)) => contact.dist,
            Ok(None) => query::distance(&poses[i], poly_a, &poses[j], poly_b)
                .unwrap_or(f64::INFINITY),
            Err(_) => f64::NEG_INFINITY,
        };
        if !signed.is_finite() {
            return Some(f64::NEG_INFINITY);
        }
        if signed < min_signed {
            min_signed = signed;
        }
    }
    Some(min_signed)
}

/// Exact lowest world-Z coordinate (m) of any vertex on moving B601 link hulls
/// (`link2..gripper_right`) at `(q6, jaw_slide_m)`.
///
/// Returns `None` if the URDF is not a B601 link tree.
pub(crate) fn mesh_lowest_z(arm: &ArmModel, q6: &[f64], jaw_slide_m: f64) -> Option<f64> {
    arm.set_arm_and_jaw(q6, jaw_slide_m);
    mesh_lowest_z_current(arm)
}

pub(crate) fn mesh_lowest_z_current(arm: &ArmModel) -> Option<f64> {
    let hulls = b601_hulls();
    let poses = link_world_poses(arm)?;
    let mut lowest = f64::INFINITY;
    for &idx in MOVING_LINK_INDICES {
        let iso = &poses[idx];
        for pt in &hulls.links[idx].points {
            let z = (iso * pt).z;
            if z < lowest {
                lowest = z;
            }
        }
    }
    Some(lowest)
}

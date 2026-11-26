//! Required arm configuration for the compute cores (option B: the SQL wrapper
//! reads `world.arm_config` and passes the doc in; the cores never read the table
//! themselves and have NO hardcoded fallback constants). A missing or invalid
//! field is a hard refusal (panic -> SQL ERROR -> rollback), so the live compute
//! path can never silently run on a baked-in or out-of-domain value.

use serde_json::Value;

#[derive(Clone)]
pub(crate) struct Config {
    pub gravity: f64,
    pub limits: [[f64; 2]; 6],
    pub link_inertials: Vec<(String, f64, [f64; 3])>,
    pub link_radii: [f64; 7],
    pub clearance: f64,
    pub link_radius_max: f64,
    pub motor7_open: f64,
    pub motor7_closed: f64,
    pub gripper_rad_per_mm: f64,
    pub default_min_z: f64,
    pub sag_limit_rad: f64,
    pub tau_sag_scale: f64,
    pub limit_tol: f64,
    pub max_joint_step: f64,
    pub ramp_s: f64,
    pub min_cruise_speed: f64,
    pub min_seconds: f64,
    pub max_seconds: f64,
    pub peak_speed: f64,
    pub gripper_peak_speed: f64,
    pub default_hz: f64,
    pub corner_radius_rad: f64,
    pub chain_line_samples: usize,
    pub chain_fillet_samples: usize,
}

/// Parse a required positive integer from JSON (accepts integer-valued floats
/// such as `6.0` as well as JSON integers; rejects non-finite, negative, zero,
/// fractional, or huge values).
pub(crate) fn json_pos_usize(v: &Value, ctx: &str, max_val: usize) -> usize {
    let n = v
        .as_f64()
        .unwrap_or_else(|| panic!("{ctx} must be a positive integer"));
    if !n.is_finite() || n.fract() != 0.0 || n < 1.0 || n > max_val as f64 {
        panic!("{ctx} must be an integer in 1..={max_val}, got {n}");
    }
    n as usize
}

/// Parse a required finite number from JSON.
pub(crate) fn json_finite(v: &Value, ctx: &str) -> f64 {
    let n = v
        .as_f64()
        .unwrap_or_else(|| panic!("{ctx} must be a finite number"));
    if !n.is_finite() {
        panic!("{ctx} must be finite");
    }
    n
}

impl Config {
    /// Halfway between open and closed (sequence provenance reference angle).
    pub fn motor7_half(&self) -> f64 {
        (self.motor7_open + self.motor7_closed) / 2.0
    }

    /// Full jaw stroke in mm (the close_width_mm gate).
    pub fn gripper_max_width_mm(&self) -> f64 {
        (self.motor7_open - self.motor7_closed) / self.gripper_rad_per_mm
    }

    /// Parse and domain-validate a REQUIRED config doc. Every field must be
    /// present, finite, and within valid physical/mathematical bounds.
    pub fn from_json(doc: &Value) -> Config {
        let obj = doc
            .as_object()
            .unwrap_or_else(|| panic!("arm config must be a jsonb object"));
        let req = |key: &str| -> &Value {
            obj.get(key)
                .unwrap_or_else(|| panic!("arm config missing required key '{key}'"))
        };
        let f = |v: &Value, ctx: &str| -> f64 { json_finite(v, &format!("arm config {ctx}")) };
        let pos = |v: f64, ctx: &str| -> f64 {
            if v <= 0.0 {
                panic!("arm config {ctx} must be > 0, got {v}");
            }
            v
        };
        let nonneg = |v: f64, ctx: &str| -> f64 {
            if v < 0.0 {
                panic!("arm config {ctx} must be >= 0, got {v}");
            }
            v
        };

        let gravity = f(req("gravity"), "gravity");

        let limits_v = req("limits")
            .as_array()
            .unwrap_or_else(|| panic!("arm config limits must be an array"));
        if limits_v.len() != 6 {
            panic!("arm config limits must have 6 joints");
        }
        let mut limits = [[0.0_f64; 2]; 6];
        for (j, pair) in limits_v.iter().enumerate() {
            let a = pair
                .as_array()
                .unwrap_or_else(|| panic!("arm config limits[{j}] must be [lo,hi]"));
            if a.len() != 2 {
                panic!("arm config limits[{j}] must be [lo,hi]");
            }
            let lo = f(&a[0], "limits.lo");
            let hi = f(&a[1], "limits.hi");
            if lo > hi {
                panic!("arm config limits[{j}] has lo ({lo}) > hi ({hi})");
            }
            limits[j] = [lo, hi];
        }

        let inert_v = req("link_inertials")
            .as_array()
            .unwrap_or_else(|| panic!("arm config link_inertials must be an array"));
        if inert_v.len() < 6 {
            panic!(
                "arm config link_inertials must have at least 6 entries, got {}",
                inert_v.len()
            );
        }
        let mut link_inertials = Vec::with_capacity(inert_v.len());
        for row in inert_v {
            let a = row
                .as_array()
                .unwrap_or_else(|| panic!("arm config link_inertials row must be [name,mass,[com]]"));
            if a.len() != 3 {
                panic!("arm config link_inertials row must be [name,mass,[com]]");
            }
            let name = a[0]
                .as_str()
                .unwrap_or_else(|| panic!("arm config link_inertials name must be a string"))
                .to_string();
            let mass = nonneg(f(&a[1], "link_inertials.mass"), "link_inertials.mass");
            let cv = a[2]
                .as_array()
                .unwrap_or_else(|| panic!("arm config link_inertials com must be [x,y,z]"));
            if cv.len() != 3 {
                panic!("arm config link_inertials com must be [x,y,z]");
            }
            let com = [f(&cv[0], "com.x"), f(&cv[1], "com.y"), f(&cv[2], "com.z")];
            link_inertials.push((name, mass, com));
        }

        let radii_v = req("link_radii")
            .as_array()
            .unwrap_or_else(|| panic!("arm config link_radii must be an array"));
        if radii_v.len() != 7 {
            panic!("arm config link_radii must have 7 entries");
        }
        let mut link_radii = [0.0_f64; 7];
        for (i, r) in radii_v.iter().enumerate() {
            link_radii[i] = nonneg(f(r, "link_radii"), "link_radii");
        }

        let collision = req("collision");
        let clearance = f(
            collision
                .get("clearance")
                .unwrap_or_else(|| panic!("arm config collision.clearance required")),
            "collision.clearance",
        );
        let link_radius_max = pos(
            f(
                collision
                    .get("link_radius_max")
                    .unwrap_or_else(|| panic!("arm config collision.link_radius_max required")),
                "collision.link_radius_max",
            ),
            "collision.link_radius_max",
        );

        let gripper = req("gripper");
        let motor7_open = f(
            gripper
                .get("motor7_open")
                .unwrap_or_else(|| panic!("arm config gripper.motor7_open required")),
            "gripper.motor7_open",
        );
        let motor7_closed = f(
            gripper
                .get("motor7_closed")
                .unwrap_or_else(|| panic!("arm config gripper.motor7_closed required")),
            "gripper.motor7_closed",
        );
        let gripper_rad_per_mm = pos(
            f(
                gripper
                    .get("rad_per_mm")
                    .unwrap_or_else(|| panic!("arm config gripper.rad_per_mm required")),
                "gripper.rad_per_mm",
            ),
            "gripper.rad_per_mm",
        );

        let motion = req("motion");
        let mf = |key: &str| -> f64 {
            f(
                motion
                    .get(key)
                    .unwrap_or_else(|| panic!("arm config motion.{key} required")),
                &format!("motion.{key}"),
            )
        };
        let chain_n = |key: &str| -> usize {
            let v = motion
                .get(key)
                .unwrap_or_else(|| panic!("arm config motion.{key} required"));
            json_pos_usize(v, &format!("arm config motion.{key}"), 1000)
        };

        let min_seconds = nonneg(mf("min_seconds"), "motion.min_seconds");
        let max_seconds = pos(mf("max_seconds"), "motion.max_seconds");
        if min_seconds > max_seconds {
            panic!(
                "arm config motion.min_seconds ({min_seconds}) > max_seconds ({max_seconds})"
            );
        }

        Config {
            gravity,
            limits,
            link_inertials,
            link_radii,
            clearance,
            link_radius_max,
            motor7_open,
            motor7_closed,
            gripper_rad_per_mm,
            default_min_z: mf("default_min_z"),
            sag_limit_rad: nonneg(mf("sag_limit_rad"), "motion.sag_limit_rad"),
            tau_sag_scale: nonneg(mf("tau_sag_scale"), "motion.tau_sag_scale"),
            limit_tol: nonneg(mf("limit_tol"), "motion.limit_tol"),
            max_joint_step: pos(mf("max_joint_step"), "motion.max_joint_step"),
            ramp_s: nonneg(mf("ramp_s"), "motion.ramp_s"),
            min_cruise_speed: pos(mf("min_cruise_speed"), "motion.min_cruise_speed"),
            min_seconds,
            max_seconds,
            peak_speed: pos(mf("peak_speed"), "motion.peak_speed"),
            gripper_peak_speed: pos(mf("gripper_peak_speed"), "motion.gripper_peak_speed"),
            default_hz: pos(mf("default_hz"), "motion.default_hz"),
            corner_radius_rad: nonneg(mf("corner_radius_rad"), "motion.corner_radius_rad"),
            chain_line_samples: chain_n("chain_line_samples"),
            chain_fillet_samples: chain_n("chain_fillet_samples"),
        }
    }
}

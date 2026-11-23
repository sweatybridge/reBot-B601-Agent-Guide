//! Required arm configuration for the compute cores (option B: the SQL wrapper
//! reads `world.arm_config` and passes the doc in; the cores never read the table
//! themselves and have NO hardcoded fallback constants). A missing doc or field is
//! a hard refusal (panic -> SQL ERROR -> rollback), so the live compute path can
//! never silently run on a baked-in value that is hard to debug.
//!
//! The doc shape matches migration `0014` / `rebot_ik.default_config()`. `tool` is
//! URDF-derived and not used by the Rust cores, so it is not parsed here. The
//! structural joint names (`ARM_JOINTS`) and the mathematical MIT range (4*pi) are
//! not tuning config and stay as constants in their modules.

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
    pub limit_tol: f64,
    pub max_joint_step: f64,
    pub ramp_s: f64,
    pub min_cruise_speed: f64,
    pub min_seconds: f64,
    pub max_seconds: f64,
    pub peak_speed: f64,
    pub gripper_peak_speed: f64,
}

impl Config {
    /// Halfway between open and closed (sequence provenance reference angle).
    pub fn motor7_half(&self) -> f64 { (self.motor7_open + self.motor7_closed) / 2.0 }
    /// Full jaw stroke in mm (the close_width_mm gate).
    pub fn gripper_max_width_mm(&self) -> f64 {
        (self.motor7_open - self.motor7_closed) / self.gripper_rad_per_mm
    }

    /// Parse a REQUIRED config doc. Every field must be present and well-typed;
    /// anything missing panics (the wrapper must supply a complete world.arm_config).
    pub fn from_json(doc: &Value) -> Config {
        let obj = doc.as_object().unwrap_or_else(|| panic!("arm config must be a jsonb object"));
        let req = |key: &str| -> &Value {
            obj.get(key).unwrap_or_else(|| panic!("arm config missing required key '{key}'"))
        };
        let f = |v: &Value, ctx: &str| -> f64 {
            v.as_f64().unwrap_or_else(|| panic!("arm config {ctx} must be a number"))
        };
        let gravity = f(req("gravity"), "gravity");

        let limits_v = req("limits").as_array()
            .unwrap_or_else(|| panic!("arm config limits must be an array"));
        if limits_v.len() != 6 { panic!("arm config limits must have 6 joints"); }
        let mut limits = [[0.0_f64; 2]; 6];
        for (j, pair) in limits_v.iter().enumerate() {
            let a = pair.as_array().unwrap_or_else(|| panic!("arm config limits[{j}] must be [lo,hi]"));
            if a.len() != 2 { panic!("arm config limits[{j}] must be [lo,hi]"); }
            limits[j] = [f(&a[0], "limits.lo"), f(&a[1], "limits.hi")];
        }

        let inert_v = req("link_inertials").as_array()
            .unwrap_or_else(|| panic!("arm config link_inertials must be an array"));
        let mut link_inertials = Vec::with_capacity(inert_v.len());
        for row in inert_v {
            let a = row.as_array().unwrap_or_else(|| panic!("arm config link_inertials row must be [name,mass,[com]]"));
            if a.len() != 3 { panic!("arm config link_inertials row must be [name,mass,[com]]"); }
            let name = a[0].as_str().unwrap_or_else(|| panic!("arm config link_inertials name must be a string")).to_string();
            let mass = f(&a[1], "link_inertials.mass");
            let cv = a[2].as_array().unwrap_or_else(|| panic!("arm config link_inertials com must be [x,y,z]"));
            if cv.len() != 3 { panic!("arm config link_inertials com must be [x,y,z]"); }
            let com = [f(&cv[0], "com.x"), f(&cv[1], "com.y"), f(&cv[2], "com.z")];
            link_inertials.push((name, mass, com));
        }

        let radii_v = req("link_radii").as_array()
            .unwrap_or_else(|| panic!("arm config link_radii must be an array"));
        if radii_v.len() != 7 { panic!("arm config link_radii must have 7 entries"); }
        let mut link_radii = [0.0_f64; 7];
        for (i, r) in radii_v.iter().enumerate() { link_radii[i] = f(r, "link_radii"); }

        let collision = req("collision");
        let clearance = f(collision.get("clearance").unwrap_or_else(|| panic!("arm config collision.clearance required")), "clearance");
        let link_radius_max = f(collision.get("link_radius_max").unwrap_or_else(|| panic!("arm config collision.link_radius_max required")), "link_radius_max");

        let gripper = req("gripper");
        let motor7_open = f(gripper.get("motor7_open").unwrap_or_else(|| panic!("arm config gripper.motor7_open required")), "motor7_open");
        let motor7_closed = f(gripper.get("motor7_closed").unwrap_or_else(|| panic!("arm config gripper.motor7_closed required")), "motor7_closed");
        let gripper_rad_per_mm = f(gripper.get("rad_per_mm").unwrap_or_else(|| panic!("arm config gripper.rad_per_mm required")), "rad_per_mm");

        let motion = req("motion");
        let mf = |key: &str| -> f64 {
            f(motion.get(key).unwrap_or_else(|| panic!("arm config motion.{key} required")), key)
        };

        Config {
            gravity, limits, link_inertials, link_radii, clearance, link_radius_max,
            motor7_open, motor7_closed, gripper_rad_per_mm,
            default_min_z: mf("default_min_z"), sag_limit_rad: mf("sag_limit_rad"),
            limit_tol: mf("limit_tol"), max_joint_step: mf("max_joint_step"),
            ramp_s: mf("ramp_s"), min_cruise_speed: mf("min_cruise_speed"),
            min_seconds: mf("min_seconds"), max_seconds: mf("max_seconds"),
            peak_speed: mf("peak_speed"), gripper_peak_speed: mf("gripper_peak_speed"),
        }
    }
}
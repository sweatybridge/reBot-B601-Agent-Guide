//! rebot-edge - play a planned joint-space trajectory on a B601-RS over SocketCAN.
//!
//! A single-file, dependency-light player. It reads the trajectory JSON
//! produced by `host/app/plan_with_pinocchio.py`, re-checks it against the
//! file's own limits, and streams MIT frames at the file's control rate.
//! Planning stays on the workstation; the board only sends joint angles, the
//! optional gravity feedforward, and the optional velocity feedforward (the
//! MIT frame's kd term damps toward the commanded velocity; with v=0 it brakes
//! the motion itself).
//!
//! Safety model:
//!
//! * dry run is the default; nothing moves without `--execute`
//! * the file's joint limits and the per-waypoint step are checked again here
//! * the arm eases onto the planned start pose instead of jumping to it
//! * a fault frame or a follow error aborts, and the arm is ramped home before
//!   torque is dropped (unless `--stay-enabled`)
//! * Ctrl-C / SIGTERM ramp home before disabling
//!
//! `--check` validates a file and prints the plan without touching CAN, which is
//! useful on the workstation and in CI.
//!
//! Protocol reference: Seeed's `robstride_dynamics` (python/robstride_dynamics),
//! mirrored here byte for byte: extended CAN id = (type << 24) | (extra << 8) |
//! device id, host id 0xFF, big-endian MIT payload, little-endian parameter I/O.

use std::collections::{BTreeMap, BTreeSet};
use std::ffi::CString;
use std::fs;
use std::io::{self, BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::os::unix::io::RawFd;
use std::path::{Path, PathBuf};
use std::process::{Command as Process, Stdio};
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serde::{Deserialize, Serialize};

// --------------------------------------------------------------------------- //
// Constants
// --------------------------------------------------------------------------- //

const TRAJECTORY_FORMAT: &str = "rebot-joint-trajectory";
const MAX_JOINT_STEP: f64 = 0.5; // rad between consecutive waypoints, sanity only
const LIMIT_TOL: f64 = 0.002; // encoders read a few mrad past a stop at rest
const MAX_KP: f64 = 150.0; // Nm/rad, vendor j2/j3 value
const MAX_KD: f64 = 10.0; // Nm/(rad/s), vendor j2/j3 value
const MIN_HZ: f64 = 5.0;
const MAX_HZ: f64 = 200.0;
const MIT_POSITION_LIMIT: f64 = 4.0 * std::f64::consts::PI;
const DEFAULT_MODEL_SPEC: &str = "1:rs-06,2:rs-06,3:rs-06";
const HOST_ID: u8 = 0xFF;
// Motor 7 is the gripper on the reference arm. `--gripper-stall` watches it
// during playback: a blocked jaw can sit at a position error indefinitely, with
// the position loop pressing, while every other joint tracks normally.
const GRIPPER_ID: u8 = 7;
const GRIPPER_STALL_ERROR: f64 = 0.15; // rad of commanded-vs-measured gap
const GRIPPER_STALL_VELOCITY: f64 = 0.08; // rad/s below which the jaw is "not moving"
const GRIPPER_STALL_MIN_S: f64 = 0.10;
const GRIPPER_STALL_MAX_S: f64 = 5.0;
// The return ramp. The duration follows the same rule as the way out -- the
// cosine-calibrated peak-speed cap (peak = pi/2 x average) -- so a long way home
// is not rushed, and a short nudge still takes long enough to have a beginning
// and an end. The trapezoid profile's cruise stays under that cap for any ramp
// longer than ~0.4 s, so the formula remains a conservative floor.
const RETURN_PEAK_SPEED: f64 = 0.35; // rad/s, base speed for the return cap
const RETURN_MIN_S: f64 = 3.0;       // s, floor for any return ramp
// Every exit (normal end, fault, Ctrl-C/SIGTERM) ramps home at three times the
// base speed, so a fully open jaw closes in ~7.3 s (half-closed in ~3.7 s).
// The arm joints use the same duration.
const SHUTDOWN_PEAK_SPEED: f64 = 3.0 * RETURN_PEAK_SPEED;
// Velocity ramp at each end of an internally generated leg, seconds. Mirrors
// traj.py's RAMP_S: trapezoid velocity (short ramps, constant cruise) so slow
// legs do not linger below the drive's stiction breakaway speed (~0.2 rad/s).
const MOVE_RAMP_S: f64 = 0.15;

// RobStride communication types (see robstride_dynamics/protocol.py).
const COMM_OPERATION_CONTROL: u32 = 1;
const COMM_OPERATION_STATUS: u32 = 2;
const COMM_ENABLE: u32 = 3;
const COMM_DISABLE: u32 = 4;
const COMM_READ_PARAMETER: u32 = 17;
const COMM_WRITE_PARAMETER: u32 = 18;
const COMM_FAULT_REPORT: u32 = 21;
const COMM_ACTIVE_REPORT: u32 = 24;

const PARAM_MODE: u16 = 0x7005; // run_mode, int8: 0 = MIT
const PARAM_MECHANICAL_POSITION: u16 = 0x7019; // float32, rad

// Linux SocketCAN (avoid depending on libc definitions that musl may lack).
const AF_CAN: libc::c_int = 29;
const CAN_RAW: libc::c_int = 1;
// `struct can_frame.can_id` needs this flag for a 29-bit id; without it the
// kernel transmits an 11-bit standard frame and the motors silently ignore it.
const CAN_EFF_FLAG: u32 = 0x8000_0000;

static INTERRUPTED: AtomicBool = AtomicBool::new(false);

// --------------------------------------------------------------------------- //
// Errors
// --------------------------------------------------------------------------- //

#[derive(Debug)]
struct Error(String);

impl Error {
    fn new(message: impl Into<String>) -> Self {
        Self(message.into())
    }
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for Error {}

impl From<io::Error> for Error {
    fn from(error: io::Error) -> Self {
        Self(error.to_string())
    }
}

impl From<serde_json::Error> for Error {
    fn from(error: serde_json::Error) -> Self {
        Self(error.to_string())
    }
}

type Result<T> = std::result::Result<T, Error>;

macro_rules! bail {
    ($($arg:tt)*) => {
        return Err(Error::new(format!($($arg)*)))
    };
}

// --------------------------------------------------------------------------- //
// Trajectory file
// --------------------------------------------------------------------------- //

#[derive(Debug, Deserialize)]
#[allow(dead_code)] // `version` and `steps` are parsed for completeness, not printed
struct TrajectoryFile {
    #[serde(default)]
    format: Option<String>,
    #[serde(default)]
    version: Option<u32>,
    #[serde(default)]
    created: Option<String>,
    #[serde(default)]
    control_hz: Option<f64>,
    #[serde(default)]
    ids: Vec<u8>,
    #[serde(default)]
    limits: Option<Vec<Vec<f64>>>,
    #[serde(default)]
    steps: Option<String>,
    #[serde(default)]
    duration_s: Option<f64>,
    #[serde(default)]
    hold_start_s: Option<f64>,
    #[serde(default)]
    hold_end_s: Option<f64>,
    #[serde(default)]
    start_tcp: Option<Vec<f64>>,
    #[serde(default)]
    end_tcp: Option<Vec<f64>>,
    #[serde(default)]
    delta_xyz: Option<Vec<f64>>,
    #[serde(default)]
    peak_joint_speed: Option<f64>,
    #[serde(default)]
    worst_ik_residual_m: Option<f64>,
    #[serde(default)]
    sag_compensation: Option<Correction>,
    #[serde(default)]
    trim: Option<Correction>,
    #[serde(default)]
    waypoints: Vec<Waypoint>,
}

#[derive(Debug, Deserialize)]
struct Waypoint {
    t: f64,
    q: Vec<f64>,
    #[serde(default)]
    tau: Option<Vec<f64>>,
    /// Velocity feedforward, rad/s. The MIT frame's kd term damps toward this
    /// velocity; without it (v=0) kd brakes the motion itself and the arm
    /// pulses at the waypoint rate. Optional: older files play with v=0.
    #[serde(default)]
    v: Option<Vec<f64>>,
}

#[derive(Debug, Deserialize)]
struct Correction {
    #[serde(default)]
    kp: BTreeMap<String, f64>,
}

#[derive(Debug)]
struct Trajectory {
    path: PathBuf,
    raw: TrajectoryFile,
    ids: Vec<u8>,
    hz: u32,
    has_tau: bool,
    peak_tau: f64,
    has_v: bool,
    peak_v: f64,
    start: Vec<f64>,
    last_t: f64,
    duration_s: f64,
}

fn load_trajectory(path: &Path) -> Result<Trajectory> {
    let text = fs::read_to_string(path)
        .map_err(|error| Error::new(format!("cannot read {}: {error}", path.display())))?;
    parse_trajectory(path, &text)
}

fn parse_trajectory(path: &Path, text: &str) -> Result<Trajectory> {
    let raw: TrajectoryFile = serde_json::from_str(text).map_err(|error| {
        Error::new(format!(
            "{}: cannot parse trajectory JSON: {error}",
            path.display()
        ))
    })?;
    let ids = raw.ids.clone();

    match raw.format.as_deref() {
        Some(TRAJECTORY_FORMAT) => {}
        other => bail!(
            "{}: not a {TRAJECTORY_FORMAT} file (found {})",
            path.display(),
            other
                .map(|value| format!("{value:?}"))
                .unwrap_or_else(|| "no 'format'".into())
        ),
    }

    if raw.waypoints.len() < 2 {
        bail!("{}: needs at least two waypoints", path.display());
    }
    if ids.is_empty() || ids.iter().any(|id| !(1..=7).contains(id)) {
        bail!("{}: 'ids' must list motor IDs 1..7", path.display());
    }
    for (index, id) in ids.iter().enumerate() {
        if ids[..index].contains(id) {
            bail!("{}: motor ID {id} is listed twice", path.display());
        }
    }

    for (index, waypoint) in raw.waypoints.iter().enumerate() {
        if waypoint.q.len() != ids.len() {
            bail!(
                "{}: waypoint {} has {} values for {} motors",
                path.display(),
                index,
                waypoint.q.len(),
                ids.len()
            );
        }
    }

    for (index, waypoint) in raw.waypoints.iter().enumerate() {
        if !waypoint.t.is_finite() {
            bail!("{}: waypoint {index} has a non-finite time", path.display());
        }
        if waypoint.q.iter().any(|q| !q.is_finite()) {
            bail!(
                "{}: waypoint {index} has a non-finite joint angle",
                path.display()
            );
        }
        if let Some(tau) = &waypoint.tau {
            if tau.len() != ids.len() {
                bail!(
                    "{}: waypoint {} has {} tau values for {} motors",
                    path.display(),
                    index,
                    tau.len(),
                    ids.len()
                );
            }
            if tau.iter().any(|value| !value.is_finite()) {
                bail!(
                    "{}: waypoint {index} has a non-finite torque",
                    path.display()
                );
            }
        }
        if let Some(v) = &waypoint.v {
            if v.len() != ids.len() {
                bail!(
                    "{}: waypoint {} has {} v values for {} motors",
                    path.display(),
                    index,
                    v.len(),
                    ids.len()
                );
            }
            if v.iter().any(|value| !value.is_finite()) {
                bail!(
                    "{}: waypoint {index} has a non-finite velocity",
                    path.display()
                );
            }
        }
    }

    let times: Vec<f64> = raw.waypoints.iter().map(|waypoint| waypoint.t).collect();
    if times.windows(2).any(|pair| pair[1] < pair[0]) {
        bail!("{}: waypoint times are not monotonic", path.display());
    }

    // The planner's own limits, re-applied here: a hand-edited or truncated file
    // must not be able to drive the arm somewhere the planner would have refused.
    check_against_limits(&raw, &ids)?;

    // Refuse anything that the MIT frame would silently clip: on a corrupt file
    // that means commanding +-4*pi instead of the angle that was asked for.
    for (index, waypoint) in raw.waypoints.iter().enumerate() {
        for (joint, q) in waypoint.q.iter().enumerate() {
            if q.abs() > MIT_POSITION_LIMIT + LIMIT_TOL {
                bail!(
                    "{}: joint {} at waypoint {} is {:+.3} rad, outside the MIT frame's +-{:.3} rad range",
                    path.display(),
                    ids[joint],
                    index,
                    q,
                    MIT_POSITION_LIMIT
                );
            }
        }
    }

    let has_tau = raw.waypoints.iter().any(|waypoint| waypoint.tau.is_some());
    if has_tau && raw.waypoints.iter().any(|waypoint| waypoint.tau.is_none()) {
        bail!(
            "{}: 'tau' is present on some waypoints but not all; add it everywhere or nowhere",
            path.display()
        );
    }
    let peak_tau = raw
        .waypoints
        .iter()
        .filter_map(|waypoint| waypoint.tau.as_ref())
        .flat_map(|tau| tau.iter())
        .fold(0.0_f64, |peak, value| peak.max(value.abs()));

    let has_v = raw.waypoints.iter().any(|waypoint| waypoint.v.is_some());
    if has_v && raw.waypoints.iter().any(|waypoint| waypoint.v.is_none()) {
        bail!(
            "{}: 'v' is present on some waypoints but not all; add it everywhere or nowhere",
            path.display()
        );
    }
    let peak_v = raw
        .waypoints
        .iter()
        .filter_map(|waypoint| waypoint.v.as_ref())
        .flat_map(|v| v.iter())
        .fold(0.0_f64, |peak, value| peak.max(value.abs()));

    let hz = raw.control_hz.unwrap_or(20.0);
    if !hz.is_finite() {
        bail!("{}: 'control_hz' is not a finite number", path.display());
    }
    let hz = hz.clamp(MIN_HZ, MAX_HZ) as u32;

    // The velocity must agree with the step gate: the fastest motion 0.5 rad
    // per waypoint permits at this rate is MAX_JOINT_STEP * hz. A v that
    // outruns the q series is corrupt, and the MIT frame would silently clip
    // it. The 0.1% slack absorbs the planner's six-decimal time rounding.
    if has_v {
        let ceiling = MAX_JOINT_STEP * f64::from(hz) * 1.001 + 1e-6;
        for (index, waypoint) in raw.waypoints.iter().enumerate() {
            if let Some(v) = &waypoint.v {
                for (joint, value) in v.iter().enumerate() {
                    if value.abs() > ceiling {
                        bail!(
                            "{}: joint {} at waypoint {} commands {:+.3} rad/s, above the \
                             {ceiling:.3} rad/s the step gate permits at {hz} Hz",
                            path.display(),
                            ids[joint],
                            index,
                            value
                        );
                    }
                }
            }
        }
    }

    let last_t = raw
        .waypoints
        .last()
        .map(|waypoint| waypoint.t)
        .unwrap_or(0.0);
    let duration_s = raw.duration_s.unwrap_or(last_t);
    if !duration_s.is_finite() {
        bail!("{}: 'duration_s' is not a finite number", path.display());
    }

    Ok(Trajectory {
        path: path.to_path_buf(),
        ids,
        hz,
        has_tau,
        peak_tau,
        has_v,
        peak_v,
        start: raw.waypoints[0].q.clone(),
        last_t,
        duration_s,
        raw,
    })
}

fn check_against_limits(raw: &TrajectoryFile, ids: &[u8]) -> Result<()> {
    // A file without limits is accepted (the planner normally writes them); a
    // file whose limit block does not match the motor list is ignored, because
    // there is nothing to index it by.
    if let Some(limits) = &raw.limits {
        if limits.len() == ids.len() {
            for (joint, limit) in limits.iter().enumerate() {
                if limit.len() != 2 {
                    bail!(
                        "limits[{}] must be [lower, upper], found {} values",
                        joint,
                        limit.len()
                    );
                }
                let (lower, upper) = (limit[0], limit[1]);
                if !lower.is_finite() || !upper.is_finite() || lower > upper {
                    bail!("limits[{joint}] is not a valid [lower, upper] pair");
                }
                let column: Vec<f64> = raw
                    .waypoints
                    .iter()
                    .map(|waypoint| waypoint.q[joint])
                    .collect();
                let low = column.iter().copied().fold(f64::INFINITY, f64::min);
                let high = column.iter().copied().fold(f64::NEG_INFINITY, f64::max);
                if low < lower - LIMIT_TOL || high > upper + LIMIT_TOL {
                    bail!(
                        "joint {} leaves its limit [{lower:.3}, {upper:.3}]: {low:.3}..{high:.3}",
                        ids[joint],
                    );
                }
            }
        }
    }

    for pair in raw.waypoints.windows(2) {
        let step = pair[0]
            .q
            .iter()
            .zip(&pair[1].q)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0_f64, f64::max);
        if step > MAX_JOINT_STEP {
            bail!("waypoints jump {step:.3} rad, above the {MAX_JOINT_STEP:.2} sanity limit");
        }
    }

    Ok(())
}

// --------------------------------------------------------------------------- //
// Motor models and MIT frames
// --------------------------------------------------------------------------- //

#[derive(Clone, Copy, Debug)]
struct Model {
    name: &'static str,
    position: f64,
    velocity: f64,
    kp: f64,
    kd: f64,
    torque: f64,
}

fn model_by_name(name: &str) -> Option<Model> {
    let (name, velocity, torque, kp, kd): (&'static str, f64, f64, f64, f64) = match name {
        "rs-00" => ("rs-00", 50.0, 17.0, 500.0, 5.0),
        "rs-01" => ("rs-01", 44.0, 17.0, 500.0, 5.0),
        "rs-02" => ("rs-02", 44.0, 17.0, 500.0, 5.0),
        "rs-03" => ("rs-03", 50.0, 60.0, 5000.0, 100.0),
        "rs-04" => ("rs-04", 15.0, 120.0, 5000.0, 100.0),
        "rs-05" => ("rs-05", 33.0, 17.0, 500.0, 5.0),
        "rs-06" => ("rs-06", 20.0, 60.0, 5000.0, 100.0),
        _ => return None,
    };
    Some(Model {
        name,
        position: MIT_POSITION_LIMIT,
        velocity,
        kp,
        kd,
        torque,
    })
}

fn parse_model_spec(spec: &str, default: &str, ids: &[u8]) -> Result<Vec<Model>> {
    let fallback =
        model_by_name(default).ok_or_else(|| Error::new(format!("unknown --model '{default}'")))?;
    let mut models = vec![fallback; ids.len()];

    if spec.trim().is_empty() {
        return Ok(models);
    }

    for part in spec.split(',') {
        let part = part.trim();
        if part.is_empty() {
            continue;
        }
        let (key, value) = part.split_once(':').ok_or_else(|| {
            Error::new(format!(
                "--model-spec: cannot parse '{part}' (expected ID:MODEL)"
            ))
        })?;
        let motor_id: u8 = key
            .trim()
            .parse()
            .map_err(|_| Error::new(format!("--model-spec: cannot parse ID in '{part}'")))?;
        let Some(index) = ids.iter().position(|id| *id == motor_id) else {
            bail!(
                "--model-spec: motor IDs are {} (got {motor_id})",
                ids.iter()
                    .map(|id| id.to_string())
                    .collect::<Vec<_>>()
                    .join(", ")
            );
        };
        let model = model_by_name(value.trim()).ok_or_else(|| {
            Error::new(format!(
                "--model-spec: unknown model '{}' (known: rs-00 .. rs-06)",
                value.trim()
            ))
        })?;
        models[index] = model;
    }
    Ok(models)
}

fn clamp_u16(value: f64) -> u16 {
    if !value.is_finite() {
        // NaN must not decode to the extreme: u16 0 is -max (−4π position,
        // −Tmax torque). Mid-scale is the neutral zero for the symmetric
        // fields, the safest direction for a defensive clamp to fail in.
        0x7FFF
    } else if value <= 0.0 {
        0
    } else if value >= 65535.0 {
        0xFFFF
    } else {
        value as u16
    }
}

/// Build the MIT operation frame (RobStride MIT-mode layout).
///
/// The model matters: rs-06 shares the 4*pi position range with rs-00 but not
/// the gain ranges (5000 vs 500 Nm/rad) or the torque range (60 vs 17 Nm).
fn mit_frame(
    model: Model,
    motor_id: u8,
    position: f64,
    kp: f64,
    kd: f64,
    velocity: f64,
    torque: f64,
) -> (u32, [u8; 8]) {
    let position = position.clamp(-model.position, model.position);
    let velocity = velocity.clamp(-model.velocity, model.velocity);
    let kp = kp.clamp(0.0, model.kp);
    let kd = kd.clamp(0.0, model.kd);
    let torque = torque.clamp(-model.torque, model.torque);

    let position_u16 = clamp_u16(((position / model.position) + 1.0) * 0x7FFF as f64);
    let velocity_u16 = clamp_u16(((velocity / model.velocity) + 1.0) * 0x7FFF as f64);
    let kp_u16 = clamp_u16((kp / model.kp) * 0xFFFF as f64);
    let kd_u16 = clamp_u16((kd / model.kd) * 0xFFFF as f64);
    let torque_u16 = clamp_u16(((torque / model.torque) + 1.0) * 0x7FFF as f64);

    let mut data = [0u8; 8];
    data[0..2].copy_from_slice(&position_u16.to_be_bytes());
    data[2..4].copy_from_slice(&velocity_u16.to_be_bytes());
    data[4..6].copy_from_slice(&kp_u16.to_be_bytes());
    data[6..8].copy_from_slice(&kd_u16.to_be_bytes());

    let id = (COMM_OPERATION_CONTROL << 24) | ((torque_u16 as u32) << 8) | (motor_id as u32);
    (id, data)
}

#[derive(Clone, Copy, Debug)]
struct State {
    position: f64,
    velocity: f64,
    torque: f64,
    temperature: f64,
}

fn decode_status(model: Model, data: &[u8]) -> Result<State> {
    if data.len() < 8 {
        bail!("short status payload ({} bytes)", data.len());
    }
    let position_u16 = u16::from_be_bytes([data[0], data[1]]);
    let velocity_u16 = u16::from_be_bytes([data[2], data[3]]);
    let torque_u16 = u16::from_be_bytes([data[4], data[5]]);
    let temperature_i16 = i16::from_be_bytes([data[6], data[7]]);

    Ok(State {
        position: (f64::from(position_u16) / 0x7FFF as f64 - 1.0) * model.position,
        velocity: (f64::from(velocity_u16) / 0x7FFF as f64 - 1.0) * model.velocity,
        torque: (f64::from(torque_u16) / 0x7FFF as f64 - 1.0) * model.torque,
        temperature: f64::from(temperature_i16) * 0.1,
    })
}

fn ext_id(communication_type: u32, extra_data: u16, device_id: u8) -> u32 {
    (communication_type << 24) | ((extra_data as u32) << 8) | u32::from(device_id)
}

/// Type-24 request that switches active reporting on/off. The 01..06 prefix is
/// the magic sequence from the manual; byte 6 is the report switch.
fn active_report_frame(id: u8, enabled: bool) -> (u32, [u8; 8]) {
    let mut data = [0u8; 8];
    data[0..6].copy_from_slice(&[1, 2, 3, 4, 5, 6]);
    data[6] = u8::from(enabled);
    (ext_id(COMM_ACTIVE_REPORT, HOST_ID as u16, id), data)
}

// --------------------------------------------------------------------------- //
// SocketCAN
// --------------------------------------------------------------------------- //

#[repr(C)]
struct CanFrameRaw {
    can_id: u32,
    can_dlc: u8,
    __pad: u8,
    __res0: u8,
    __len8_dlc: u8,
    data: [u8; 8],
}

#[repr(C)]
struct SockAddrCan {
    can_family: libc::sa_family_t,
    can_ifindex: libc::c_int,
    can_addr: [u8; 8],
}

#[derive(Clone, Copy, Debug)]
struct Frame {
    id: u32,
    data: [u8; 8],
    len: usize,
}

impl Frame {
    fn comm(&self) -> u32 {
        (self.id >> 24) & 0x1F
    }

    fn status_device(&self) -> u8 {
        ((self.id >> 8) & 0xFF) as u8
    }

    fn target(&self) -> u8 {
        (self.id & 0xFF) as u8
    }

    /// Responses put the host in the low byte and the motor in the next byte
    /// (see the RX layout in Seeed's `TWAI_CAN_MI_Motor.h`), while an echoing
    /// firmware may simply return the request id. Only one parameter read is
    /// outstanding at a time, so accepting either layout is unambiguous and
    /// keeps both kinds of firmware working.
    fn is_from(&self, id: u8) -> bool {
        self.target() == id || self.status_device() == id
    }
}

struct CanBus {
    fd: RawFd,
    channel: String,
    reader: Option<Reader>,
    /// Motors with Type-24 active reporting enabled, so `Drop` can switch them
    /// back off even on an error path.
    active_reports: Vec<u8>,
}

impl CanBus {
    fn open(channel: &str) -> Result<Self> {
        let name = CString::new(channel)
            .map_err(|_| Error::new(format!("channel name contains a NUL: {channel:?}")))?;
        let ifindex = unsafe { libc::if_nametoindex(name.as_ptr()) };
        if ifindex == 0 {
            bail!("CAN channel '{channel}' does not exist");
        }

        let fd = unsafe { libc::socket(AF_CAN, libc::SOCK_RAW | libc::SOCK_CLOEXEC, CAN_RAW) };
        if fd < 0 {
            let error = io::Error::last_os_error();
            bail!(
                "cannot open a CAN_RAW socket ({error}).\n\
                 This needs root or CAP_NET_RAW:\n  \
                 sudo ./rebot-edge ..."
            );
        }

        let address = SockAddrCan {
            can_family: AF_CAN as libc::sa_family_t,
            can_ifindex: ifindex as libc::c_int,
            can_addr: [0u8; 8],
        };
        let rc = unsafe {
            libc::bind(
                fd,
                &address as *const SockAddrCan as *const libc::sockaddr,
                std::mem::size_of::<SockAddrCan>() as libc::socklen_t,
            )
        };
        if rc != 0 {
            let error = io::Error::last_os_error();
            unsafe { libc::close(fd) };
            bail!("cannot bind CAN socket to '{channel}': {error}");
        }

        let flags = unsafe { libc::fcntl(fd, libc::F_GETFL) };
        if flags < 0 || unsafe { libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) } < 0 {
            let error = io::Error::last_os_error();
            unsafe { libc::close(fd) };
            bail!("cannot make '{channel}' non-blocking: {error}");
        }

        Ok(Self {
            fd,
            channel: channel.to_string(),
            reader: None,
            active_reports: Vec::new(),
        })
    }

    fn send(&self, id: u32, data: &[u8]) -> Result<()> {
        if data.len() > 8 {
            bail!("CAN payload is {} bytes (max 8)", data.len());
        }
        let mut frame = CanFrameRaw {
            can_id: CAN_EFF_FLAG | id,
            can_dlc: data.len() as u8,
            __pad: 0,
            __res0: 0,
            __len8_dlc: 0,
            data: [0u8; 8],
        };
        frame.data[..data.len()].copy_from_slice(data);

        let mut attempts = 0;
        loop {
            let written = unsafe {
                libc::write(
                    self.fd,
                    &frame as *const CanFrameRaw as *const libc::c_void,
                    std::mem::size_of::<CanFrameRaw>(),
                )
            };
            if written == std::mem::size_of::<CanFrameRaw>() as isize {
                return Ok(());
            }
            if written < 0 {
                let error = io::Error::last_os_error();
                match error.raw_os_error() {
                    Some(code) if code == libc::EINTR => continue,
                    Some(code)
                        if (code == libc::EAGAIN || code == libc::ENOBUFS) && attempts < 20 =>
                    {
                        attempts += 1;
                        self.wait_writable(Duration::from_millis(5));
                        continue;
                    }
                    _ => {
                        bail!("write to {} failed: {error}", self.channel);
                    }
                }
            }
            bail!("short write to {} ({written} bytes)", self.channel);
        }
    }

    fn wait_writable(&self, timeout: Duration) {
        let mut pollfd = libc::pollfd {
            fd: self.fd,
            events: libc::POLLOUT,
            revents: 0,
        };
        let millis = timeout.as_millis().min(i32::MAX as u128) as i32;
        unsafe { libc::poll(&mut pollfd, 1, millis) };
    }

    fn recv(&self, timeout: Duration) -> Result<Option<Frame>> {
        let deadline = Instant::now() + timeout;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            let millis = remaining.as_millis().min(i32::MAX as u128) as i32;
            let mut pollfd = libc::pollfd {
                fd: self.fd,
                events: libc::POLLIN,
                revents: 0,
            };
            let ready = unsafe { libc::poll(&mut pollfd, 1, millis) };
            if ready < 0 {
                let error = io::Error::last_os_error();
                if error.raw_os_error() == Some(libc::EINTR) {
                    continue;
                }
                bail!("poll on {} failed: {error}", self.channel);
            }
            if ready == 0 {
                return Ok(None);
            }
            // POLLERR/POLLHUP/POLLNVAL make poll return immediately with no
            // POLLIN; continuing would busy-spin to the deadline. The bus is
            // gone (interface down / fd invalid) -- report it, do not spin.
            if pollfd.revents & (libc::POLLERR | libc::POLLHUP | libc::POLLNVAL) != 0 {
                bail!("{}: poll error (revents {:#x})", self.channel, pollfd.revents);
            }
            if pollfd.revents & libc::POLLIN == 0 {
                continue;
            }

            let mut raw: CanFrameRaw = unsafe { std::mem::zeroed() };
            let read = unsafe {
                libc::read(
                    self.fd,
                    &mut raw as *mut CanFrameRaw as *mut libc::c_void,
                    std::mem::size_of::<CanFrameRaw>(),
                )
            };
            if read < 0 {
                let error = io::Error::last_os_error();
                match error.raw_os_error() {
                    Some(code) if code == libc::EINTR || code == libc::EAGAIN => continue,
                    _ => bail!("read from {} failed: {error}", self.channel),
                }
            }
            if read as usize != std::mem::size_of::<CanFrameRaw>() {
                continue;
            }
            return Ok(Some(Frame {
                id: raw.can_id,
                data: raw.data,
                len: (raw.can_dlc as usize).min(8),
            }));
        }
    }

    fn recv_nowait(&self) -> Result<Option<Frame>> {
        self.recv(Duration::ZERO)
    }

    /// Discard queued status frames so the socket buffer cannot back up, and
    /// refuse to keep commanding a motor that has already reported a fault.
    fn drain(&self, budget: usize) -> Result<()> {
        for _ in 0..budget {
            let Some(frame) = self.recv_nowait()? else {
                break;
            };
            if frame.comm() == COMM_FAULT_REPORT {
                bail!(
                    "motor id {} reported a fault frame: {}",
                    frame.status_device(),
                    hex(&frame.data)
                );
            }
        }
        Ok(())
    }

    fn read_position(&self, id: u8, timeout: Duration) -> Result<f64> {
        let mut payload = [0u8; 8];
        payload[0..2].copy_from_slice(&PARAM_MECHANICAL_POSITION.to_le_bytes());
        self.send(ext_id(COMM_READ_PARAMETER, HOST_ID as u16, id), &payload)?;

        let deadline = Instant::now() + timeout;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                bail!("no reply to the 0x7019 read from motor {id}");
            }
            let Some(frame) = self.recv(remaining.min(Duration::from_millis(10)))? else {
                continue;
            };
            if frame.comm() == COMM_FAULT_REPORT {
                bail!(
                    "motor id {} reported a fault frame: {}",
                    frame.status_device(),
                    hex(&frame.data)
                );
            }
            if frame.comm() != COMM_READ_PARAMETER {
                continue;
            }
            // The response may echo the request (motor id in the low byte) or use
            // the receive layout (host in the low byte, motor in the next one).
            if !frame.is_from(id) {
                continue;
            }
            let bytes = [frame.data[4], frame.data[5], frame.data[6], frame.data[7]];
            let value = f32::from_le_bytes(bytes) as f64;
            if !value.is_finite() {
                bail!("motor {id} returned a non-finite position");
            }
            return Ok(value);
        }
    }

    /// Latest status frame for one motor, matched by device ID. No request is
    /// sent: this is the reply to the operation frame that was just written.
    fn read_status(&self, model: Model, id: u8, timeout: Duration) -> Result<Option<State>> {
        let deadline = Instant::now() + timeout;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Ok(None);
            }
            let Some(frame) = self.recv(remaining.min(Duration::from_millis(10)))? else {
                continue;
            };
            if frame.comm() == COMM_FAULT_REPORT {
                bail!(
                    "motor id {} reported a fault frame: {}",
                    frame.status_device(),
                    hex(&frame.data)
                );
            }
            if frame.comm() != COMM_OPERATION_STATUS || frame.status_device() != id {
                continue;
            }
            return decode_status(model, &frame.data[..frame.len]).map(Some);
        }
    }

    /// One pass over the socket, filling every joint asked for. Blocking reads
    /// per joint are wasteful at 30 Hz; misses are simply absent.
    fn poll_states(
        &self,
        ids: &[u8],
        models: &[Model],
        timeout: Duration,
    ) -> Result<Vec<Option<State>>> {
        let mut found: Vec<Option<State>> = vec![None; ids.len()];
        let mut remaining = ids.len();
        let deadline = Instant::now() + timeout;

        while remaining > 0 {
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                break;
            }
            let Some(frame) = self.recv(left.min(Duration::from_millis(5)))? else {
                continue;
            };
            if frame.comm() == COMM_FAULT_REPORT {
                bail!(
                    "motor id {} reported a fault frame: {}",
                    frame.status_device(),
                    hex(&frame.data)
                );
            }
            if frame.comm() != COMM_OPERATION_STATUS {
                continue;
            }
            let device = frame.status_device();
            let Some(index) = ids.iter().position(|id| *id == device) else {
                continue;
            };
            if found[index].is_none() {
                found[index] = Some(decode_status(models[index], &frame.data[..frame.len])?);
                remaining -= 1;
            }
        }
        Ok(found)
    }

    /// Drain frames that are already queued, without ever waiting. This is the
    /// no-reader-thread path: the status replies to the previous command have
    /// normally arrived by the next tick, and a missing joint keeps its cached
    /// value. No `poll(2)` wait is involved.
    fn poll_available(&self, ids: &[u8], models: &[Model]) -> Result<Vec<Option<State>>> {
        self.poll_available_inner(ids, models, false)
    }

    /// Same drain, but a fault frame is reported and skipped instead of
    /// aborting. Only the shutdown ramp uses this: a faulted motor may keep
    /// emitting fault frames, and bailing on each one would strand the arm
    /// wherever the playback stopped (§ module doc: every exit ramps home).
    fn poll_available_tolerant(&self, ids: &[u8], models: &[Model]) -> Result<Vec<Option<State>>> {
        self.poll_available_inner(ids, models, true)
    }

    fn poll_available_inner(
        &self,
        ids: &[u8],
        models: &[Model],
        tolerate_faults: bool,
    ) -> Result<Vec<Option<State>>> {
        let mut found: Vec<Option<State>> = vec![None; ids.len()];
        for _ in 0..128 {
            let Some(frame) = self.recv_nowait()? else {
                break;
            };
            if frame.comm() == COMM_FAULT_REPORT {
                if !tolerate_faults {
                    bail!(
                        "motor id {} reported a fault frame: {}",
                        frame.status_device(),
                        hex(&frame.data)
                    );
                }
                println!(
                    "  fault frame during the return ramp: motor id {} {} (skipped)",
                    frame.status_device(),
                    hex(&frame.data)
                );
                continue;
            }
            if frame.comm() != COMM_OPERATION_STATUS && frame.comm() != COMM_ACTIVE_REPORT {
                continue;
            }
            let device = frame.status_device();
            let Some(index) = ids.iter().position(|id| *id == device) else {
                continue;
            };
            found[index] = Some(decode_status(models[index], &frame.data[..frame.len])?);
        }
        Ok(found)
    }
}

impl Drop for CanBus {
    fn drop(&mut self) {
        // Leave the motors quiet: active reporting survives until power-off and
        // would keep filling the bus after this process exits.
        let active = self.active_reports.clone();
        for id in active {
            let _ = self.set_active_report(id, false);
        }
        self.stop_reader();
        unsafe { libc::close(self.fd) };
    }
}

impl CanBus {
    /// A second fd on the same socket for the receive thread. `dup` shares the
    /// socket but not the descriptor, so each side closes only its own.
    fn try_clone(&self) -> Result<Self> {
        let fd = unsafe { libc::dup(self.fd) };
        if fd < 0 {
            bail!(
                "cannot duplicate the CAN socket: {}",
                io::Error::last_os_error()
            );
        }
        Ok(Self {
            fd,
            channel: self.channel.clone(),
            reader: None,
            active_reports: Vec::new(),
        })
    }

    /// Start the background receiver. From this point on the main thread must
    /// not call `recv()`/`read_status()`/`drain()` on this socket; it only
    /// sends, and reads the published snapshot.
    fn start_reader(&mut self, ids: &[u8], models: &[Model]) -> Result<()> {
        if self.reader.is_some() {
            return Ok(());
        }
        let reader_bus = self.try_clone()?;
        let telemetry = Arc::new(Telemetry::new(ids.to_vec(), models.to_vec()));
        let stop = Arc::new(AtomicBool::new(false));
        let thread_telemetry = Arc::clone(&telemetry);
        let thread_stop = Arc::clone(&stop);
        let handle = thread::Builder::new()
            .name("can-rx".into())
            .spawn(move || reader_loop(reader_bus, thread_telemetry, thread_stop))
            .map_err(|error| Error::new(format!("cannot start the CAN reader thread: {error}")))?;
        self.reader = Some(Reader {
            stop,
            handle: Some(handle),
            telemetry,
        });
        Ok(())
    }

    fn stop_reader(&mut self) {
        if let Some(mut reader) = self.reader.take() {
            reader.stop.store(true, Ordering::SeqCst);
            if let Some(handle) = reader.handle.take() {
                let _ = handle.join();
            }
        }
    }

    fn latest_states(&self) -> Option<Vec<Option<State>>> {
        self.reader
            .as_ref()
            .and_then(|reader| reader.telemetry.latest())
    }

    fn reader_fault(&self) -> Option<String> {
        self.reader
            .as_ref()
            .and_then(|reader| reader.telemetry.fault())
    }

    fn clear_reader_fault(&self) {
        if let Some(reader) = &self.reader {
            reader.telemetry.clear_fault();
        }
    }

    /// Enable/disable Type-24 active reporting. Reports arrive as Type-24 frames
    /// at the motor's default interval (10 ms); EPScan_time can change it, but
    /// the firmware's floor is around 10-20 ms.
    fn set_active_report(&mut self, id: u8, enabled: bool) -> Result<()> {
        let (frame_id, data) = active_report_frame(id, enabled);
        self.send(frame_id, &data)?;
        if enabled {
            if !self.active_reports.contains(&id) {
                self.active_reports.push(id);
            }
        } else {
            self.active_reports.retain(|active| *active != id);
        }
        Ok(())
    }
}

struct Reader {
    stop: Arc<AtomicBool>,
    handle: Option<thread::JoinHandle<()>>,
    telemetry: Arc<Telemetry>,
}

/// Latest status frames, published by the receive thread. Locking is only ever
/// held for an assignment or a clone, so the control tick never waits on the
/// bus; a busy cache simply falls back to the previous sample.
struct Telemetry {
    ids: Vec<u8>,
    models: Vec<Model>,
    states: Mutex<Vec<Option<State>>>,
    fault_flag: AtomicBool,
    fault: Mutex<Option<String>>,
}

impl Telemetry {
    fn new(ids: Vec<u8>, models: Vec<Model>) -> Self {
        let states = vec![None; ids.len()];
        Self {
            ids,
            models,
            states: Mutex::new(states),
            fault_flag: AtomicBool::new(false),
            fault: Mutex::new(None),
        }
    }

    fn handle(&self, frame: Frame) {
        if frame.comm() == COMM_FAULT_REPORT {
            self.set_fault(format!(
                "motor id {} reported a fault frame: {}",
                frame.status_device(),
                hex(&frame.data)
            ));
            return;
        }
        if frame.comm() != COMM_OPERATION_STATUS && frame.comm() != COMM_ACTIVE_REPORT {
            return;
        }

        let device = frame.status_device();
        let Some(index) = self.ids.iter().position(|id| *id == device) else {
            return;
        };
        match decode_status(self.models[index], &frame.data[..frame.len]) {
            Ok(state) => {
                if let Ok(mut states) = self.states.lock() {
                    states[index] = Some(state);
                }
            }
            Err(error) => self.set_fault(error.to_string()),
        }
    }

    fn latest(&self) -> Option<Vec<Option<State>>> {
        self.states.try_lock().ok().map(|states| states.clone())
    }

    fn set_fault(&self, message: String) {
        // Store the message before publishing the flag, so a reader that sees
        // the flag can always recover the reason.
        if let Ok(mut fault) = self.fault.lock() {
            if fault.is_none() {
                *fault = Some(message);
            }
        }
        self.fault_flag.store(true, Ordering::SeqCst);
    }

    fn fault(&self) -> Option<String> {
        if !self.fault_flag.load(Ordering::SeqCst) {
            return None;
        }
        self.fault.lock().ok().and_then(|fault| fault.clone())
    }

    /// Drop the latched fault. Only the shutdown ramp may call this: the
    /// interlock exists to stop trajectory playback, not to strand the arm
    /// mid-workspace. A fresh fault frame re-latches immediately.
    fn clear_fault(&self) {
        if let Ok(mut fault) = self.fault.lock() {
            *fault = None;
        }
        self.fault_flag.store(false, Ordering::SeqCst);
    }
}

fn reader_loop(bus: CanBus, telemetry: Arc<Telemetry>, stop: Arc<AtomicBool>) {
    while !stop.load(Ordering::Relaxed) {
        match bus.recv(Duration::from_millis(20)) {
            Ok(Some(frame)) => telemetry.handle(frame),
            Ok(None) => {}
            Err(error) => {
                telemetry.set_fault(error.to_string());
                break;
            }
        }
    }
}

fn hex(data: &[u8]) -> String {
    let mut out = String::with_capacity(data.len() * 2);
    for byte in data {
        out.push_str(&format!("{byte:02x}"));
    }
    out
}

/// The part of the CAN layer the control loop needs. `CanBus` is the real
/// implementation; tests use an in-memory motor simulator so bring-up, the
/// playback loop and shutdown can be exercised without hardware or `vcan`.
trait Transport {
    fn send(&self, id: u32, data: &[u8]) -> Result<()>;
    fn drain(&self, budget: usize) -> Result<()>;
    fn read_position(&self, id: u8, timeout: Duration) -> Result<f64>;
    fn read_status(&self, model: Model, id: u8, timeout: Duration) -> Result<Option<State>>;
    fn poll_states(
        &self,
        ids: &[u8],
        models: &[Model],
        timeout: Duration,
    ) -> Result<Vec<Option<State>>>;

    /// Non-blocking snapshot of already-queued status frames.
    fn poll_available(&self, ids: &[u8], models: &[Model]) -> Result<Vec<Option<State>>> {
        self.poll_states(ids, models, Duration::ZERO)
    }

    /// Shutdown-ramp variant: fault frames are skipped, not fatal. Test doubles
    /// without fault frames can keep the strict path.
    fn poll_available_tolerant(&self, ids: &[u8], models: &[Model]) -> Result<Vec<Option<State>>> {
        self.poll_available(ids, models)
    }

    /// Start a background receiver. The real bus returns true; test doubles
    /// keep the synchronous path and return false.
    fn start_reader(&mut self, _ids: &[u8], _models: &[Model]) -> Result<bool> {
        Ok(false)
    }

    fn stop_reader(&mut self) {}

    fn latest_states(&self) -> Option<Vec<Option<State>>> {
        None
    }

    fn reader_fault(&self) -> Option<String> {
        None
    }

    fn clear_reader_fault(&self) {}

    fn set_active_report(&mut self, _id: u8, _enabled: bool) -> Result<()> {
        Ok(())
    }
}

impl Transport for CanBus {
    fn send(&self, id: u32, data: &[u8]) -> Result<()> {
        CanBus::send(self, id, data)
    }

    fn drain(&self, budget: usize) -> Result<()> {
        CanBus::drain(self, budget)
    }

    fn read_position(&self, id: u8, timeout: Duration) -> Result<f64> {
        CanBus::read_position(self, id, timeout)
    }

    fn read_status(&self, model: Model, id: u8, timeout: Duration) -> Result<Option<State>> {
        CanBus::read_status(self, model, id, timeout)
    }

    fn poll_states(
        &self,
        ids: &[u8],
        models: &[Model],
        timeout: Duration,
    ) -> Result<Vec<Option<State>>> {
        CanBus::poll_states(self, ids, models, timeout)
    }

    fn poll_available(&self, ids: &[u8], models: &[Model]) -> Result<Vec<Option<State>>> {
        CanBus::poll_available(self, ids, models)
    }

    fn poll_available_tolerant(&self, ids: &[u8], models: &[Model]) -> Result<Vec<Option<State>>> {
        CanBus::poll_available_tolerant(self, ids, models)
    }

    fn start_reader(&mut self, ids: &[u8], models: &[Model]) -> Result<bool> {
        CanBus::start_reader(self, ids, models)?;
        Ok(true)
    }

    fn stop_reader(&mut self) {
        CanBus::stop_reader(self)
    }

    fn latest_states(&self) -> Option<Vec<Option<State>>> {
        CanBus::latest_states(self)
    }

    fn reader_fault(&self) -> Option<String> {
        CanBus::reader_fault(self)
    }

    fn clear_reader_fault(&self) {
        CanBus::clear_reader_fault(self)
    }

    fn set_active_report(&mut self, id: u8, enabled: bool) -> Result<()> {
        CanBus::set_active_report(self, id, enabled)
    }
}

// --------------------------------------------------------------------------- //
// Arm / control loop
// --------------------------------------------------------------------------- //

struct Arm<T: Transport> {
    bus: T,
    ids: Vec<u8>,
    models: Vec<Model>,
    kp: Vec<f64>,
    kd: Vec<f64>,
    period: Duration,
    start: Vec<f64>,
    last: Vec<f64>,
    enabled: bool,
    telemetry_active: bool,
}

impl<T: Transport> Arm<T> {
    fn new(bus: T, ids: Vec<u8>, models: Vec<Model>, kp: Vec<f64>, kd: Vec<f64>, hz: u32) -> Self {
        let count = ids.len();
        Self {
            bus,
            ids,
            models,
            kp,
            kd,
            period: Duration::from_secs_f64(1.0 / f64::from(hz)),
            start: vec![f64::NAN; count],
            last: vec![f64::NAN; count],
            enabled: false,
            telemetry_active: false,
        }
    }

    /// Start the background receiver. From here on `command()` only sends,
    /// and `latest_states()`/`reader_fault()` are the only ways to read the bus.
    fn start_telemetry(&mut self) -> Result<()> {
        self.telemetry_active = self.bus.start_reader(&self.ids, &self.models)?;
        Ok(())
    }

    fn stop_telemetry(&mut self) {
        if self.telemetry_active {
            self.bus.stop_reader();
            self.telemetry_active = false;
        }
    }

    fn start_active_reports(&mut self) -> Result<()> {
        for joint in 0..self.ids.len() {
            if let Err(error) = self.bus.set_active_report(self.ids[joint], true) {
                self.stop_active_reports();
                return Err(error);
            }
        }
        Ok(())
    }

    fn stop_active_reports(&mut self) {
        for joint in 0..self.ids.len() {
            let _ = self.bus.set_active_report(self.ids[joint], false);
        }
    }

    fn write_mode_mit(&self, id: u8) -> Result<()> {
        let mut payload = [0u8; 6];
        payload[0..2].copy_from_slice(&PARAM_MODE.to_le_bytes());
        // run_mode = 0 (MIT), then the two padding bytes the SDK sends.
        self.bus
            .send(ext_id(COMM_WRITE_PARAMETER, HOST_ID as u16, id), &payload)
    }

    fn enable(&self, id: u8) -> Result<()> {
        self.bus
            .send(ext_id(COMM_ENABLE, HOST_ID as u16, id), &[0u8; 8])
    }

    fn disable(&self, id: u8) -> Result<()> {
        self.bus
            .send(ext_id(COMM_DISABLE, HOST_ID as u16, id), &[0u8; 8])
    }

    /// Best-effort disable of every motor, for the failure path where only some
    /// motors were enabled. Returns the first error but still tries all of them.
    fn disable_all(&self) -> Result<()> {
        let mut first_error = None;
        for &id in &self.ids {
            if let Err(error) = self.disable(id) {
                first_error.get_or_insert(error);
            }
        }
        if let Err(error) = self.bus.drain(64) {
            first_error.get_or_insert(error);
        }
        match first_error {
            Some(error) => Err(error),
            None => Ok(()),
        }
    }

    fn bring_up(&mut self) -> Result<()> {
        // Every failure path below attempts disable_all: earlier joints in the
        // loop are already enabled, and a previous process may have left torque
        // on. `enabled` is still false here, so shut_down() would no-op.
        for joint in 0..self.ids.len() {
            if let Err(error) = self.write_mode_mit(self.ids[joint]) {
                let _ = self.disable_all();
                return Err(error);
            }
        }
        thread::sleep(Duration::from_millis(100));
        if let Err(error) = self.bus.drain(64) {
            let _ = self.disable_all();
            return Err(error);
        }

        for joint in 0..self.ids.len() {
            let id = self.ids[joint];
            if let Err(error) = self.enable(id) {
                // Some earlier joint may already be enabled; never leave torque
                // on because a later frame failed.
                let _ = self.disable_all();
                return Err(error);
            }
            let state =
                match self
                    .bus
                    .read_status(self.models[joint], id, Duration::from_millis(500))
                {
                    Ok(Some(state)) => state,
                    Ok(None) => {
                        let _ = self.disable_all();
                        bail!("no status frame from motor {id} after enable");
                    }
                    Err(error) => {
                        let _ = self.disable_all();
                        return Err(error);
                    }
                };
            self.start[joint] = state.position;
            self.last[joint] = state.position;
            println!(
                "  motor {:<2} start={:+.4} rad (vel={:+.3}, tq={:+.2}, {:.1} C)",
                id, state.position, state.velocity, state.torque, state.temperature
            );
            if let Err(error) = self.bus.drain(64) {
                let _ = self.disable_all();
                return Err(error);
            }
        }
        self.enabled = true;
        Ok(())
    }

    fn command(
        &mut self,
        targets: &[f64],
        torques: Option<&[f64]>,
        velocities: Option<&[f64]>,
        drain: bool,
    ) -> Result<()> {
        debug_assert_eq!(targets.len(), self.ids.len());
        if self.telemetry_active {
            if let Some(fault) = self.bus.reader_fault() {
                bail!("{fault}");
            }
        }
        for joint in 0..self.ids.len() {
            let torque = torques.map_or(0.0, |values| values[joint]);
            let velocity = velocities.map_or(0.0, |values| values[joint]);
            let (id, data) = mit_frame(
                self.models[joint],
                self.ids[joint],
                targets[joint],
                self.kp[joint],
                self.kd[joint],
                velocity,
                torque,
            );
            self.bus.send(id, &data)?;
            self.last[joint] = targets[joint];
        }
        // The reader owns the receive queue while it is active; draining here
        // would steal its status frames.
        if drain && !self.telemetry_active {
            self.bus.drain(64)?;
        }
        Ok(())
    }

    /// Cosine-eased leg from where we last commanded to `targets`.
    ///
    /// The start point is the last commanded position, not the pose captured at
    /// bring-up: a leg that returns to that pose must ramp into it, not step.
    fn move_leg(
        &mut self,
        targets: &[f64],
        duration: f64,
        label: &str,
        track: bool,
        honor_interrupt: bool,
        tolerate_faults: bool,
    ) -> Result<()> {
        let begin: Vec<f64> = self.last[..targets.len()].to_vec();
        let t0 = Instant::now();
        // One step per tick period, at least one.
        let steps = (duration / self.period.as_secs_f64()).round().max(1.0) as u32;
        let sample_at: BTreeSet<u32> = if track {
            [0.25_f64, 0.5, 0.75, 1.0]
                .iter()
                .map(|fraction| (f64::from(steps) * fraction).round() as u32)
                .collect()
        } else {
            BTreeSet::new()
        };
        let mut latest: Vec<Option<State>> = vec![None; self.ids.len()];
        // Trapezoid velocity profile (mirrors traj.py): short ramps into a
        // constant-velocity cruise, so slow legs do not linger below the drive's
        // stiction breakaway speed. The analytic derivative feeds the MIT frame's
        // v field; with v=0 the kd term brakes the leg itself.
        let leg_seconds = f64::from(steps) * self.period.as_secs_f64();

        for step in 0..=steps {
            if honor_interrupt && interrupted() {
                break;
            }
            if !self.telemetry_active {
                let states = if tolerate_faults {
                    self.bus.poll_available_tolerant(&self.ids, &self.models)?
                } else {
                    self.bus.poll_available(&self.ids, &self.models)?
                };
                for (joint, state) in states.iter().enumerate() {
                    if let Some(state) = state {
                        latest[joint] = Some(*state);
                    }
                }
            }
            let u = f64::from(step) / f64::from(steps);
            let fraction = ease_fraction(u, leg_seconds, MOVE_RAMP_S);
            let command: Vec<f64> = begin
                .iter()
                .zip(targets)
                .map(|(from, to)| from + (to - from) * fraction)
                .collect();
            let speed = ease_velocity(u, leg_seconds, MOVE_RAMP_S);
            let velocities: Vec<f64> = begin
                .iter()
                .zip(targets)
                .map(|(from, to)| (to - from) * speed)
                .collect();
            let is_sample = sample_at.contains(&step);
            self.command(&command, None, Some(&velocities), false)?;

            if is_sample {
                let percent = (100.0 * f64::from(step) / f64::from(steps)).round() as i32;
                if self.telemetry_active {
                    if let Some(states) = self.bus.latest_states() {
                        for (joint, state) in states.iter().enumerate() {
                            if let Some(state) = state {
                                println!(
                                    "    {:<8} {:3}% commanded {:+.4} measured {:+.4}",
                                    label, percent, self.last[joint], state.position
                                );
                            }
                        }
                    }
                } else {
                    for (joint, state) in latest.iter().enumerate() {
                        if let Some(state) = state {
                            println!(
                                "    {:<8} {:3}% commanded {:+.4} measured {:+.4}",
                                label, percent, self.last[joint], state.position
                            );
                        }
                    }
                }
            }

            sleep_until(t0 + self.period.mul_f64(f64::from(step + 1)));
        }

        println!(
            "    {:<8} done in {:+.2}s",
            label,
            t0.elapsed().as_secs_f64()
        );
        Ok(())
    }

    fn hold(&mut self, targets: &[f64], seconds: f64, honor_interrupt: bool) -> Result<()> {
        let t0 = Instant::now();
        while t0.elapsed().as_secs_f64() < seconds {
            if honor_interrupt && interrupted() {
                break;
            }
            self.command(targets, None, None, true)?;
            thread::sleep(self.period);
        }
        Ok(())
    }

    fn shut_down(&mut self, stay_enabled: bool, track: bool) -> Result<()> {
        if !self.enabled {
            return Ok(());
        }
        // A fault abort leaves the reader's fault latched, and command() refuses
        // to send while it is set. The return ramp is the documented exception
        // (module doc: every exit ramps home), so drop the latch for the ramp
        // only. A fresh fault frame re-latches and stops the ramp; the disable
        // below still runs either way.
        if let Some(fault) = self.bus.reader_fault() {
            println!("  fault latched during playback: {fault}");
            println!("  clearing the latch for the return ramp");
            self.bus.clear_reader_fault();
        }
        println!("  returning to the start pose...");
        // All joints, including the gripper, return to the pose measured before
        // playback. The ramp is capped at 1.05 rad/s so an open jaw closes in
        // about 7.3 s. On a gripper stall the play loop has already rewritten
        // `start[gripper]` to the blocked opening, so the return holds that
        // instead of pressing further.
        let home = self.start.clone();

        // Ramp home first, then hold; attempt the disable even if the ramp fails,
        // so a bad status frame cannot leave torque on by accident. The ramp is
        // timed by the distance it has to cover, not by a constant. Fault frames
        // on the bus are skipped, not fatal: a faulted motor may keep emitting
        // them, and bailing on each one would strand the arm at the abort pose.
        let duration = shutdown_ramp_duration(&self.last, &home);
        let ramp = self
            .move_leg(&home, duration, "home", track, false, true)
            .and_then(|_| self.hold(&home, 0.5, false));

        if stay_enabled {
            println!("  --stay-enabled: leaving torque ON (motors keep holding).");
            return ramp;
        }

        println!("  disabling torque (gravity-loaded joints may sag)");
        let disable_result = self.disable_all();
        self.enabled = false;
        ramp?;
        disable_result
    }
}

/// Trapezoid velocity profile: fraction of travel at normalized time u, with
/// linear velocity ramps of `ramp` seconds at each end (mirrors traj.py's
/// ease_fraction). Legs shorter than 2*ramp degenerate to a triangle; both
/// shapes start and end at rest.
fn ease_fraction(u: f64, duration: f64, ramp: f64) -> f64 {
    let r = (ramp / duration.max(1e-9)).clamp(1e-9, 0.5);
    let vc = 1.0 / (1.0 - r);
    let u = u.clamp(0.0, 1.0);
    if u <= r {
        vc * u * u / (2.0 * r)
    } else if u >= 1.0 - r {
        1.0 - vc * (1.0 - u) * (1.0 - u) / (2.0 * r)
    } else {
        vc * (u - 0.5 * r)
    }
}

/// d(ease_fraction)/dt in 1/s; multiply by the joint travel for rad/s.
fn ease_velocity(u: f64, duration: f64, ramp: f64) -> f64 {
    let duration = duration.max(1e-9);
    let r = (ramp / duration).clamp(1e-9, 0.5);
    let vc = 1.0 / (1.0 - r);
    let u = u.clamp(0.0, 1.0);
    if u <= r {
        vc * u / (r * duration)
    } else if u >= 1.0 - r {
        vc * (1.0 - u) / (r * duration)
    } else {
        vc / duration
    }
}

fn ramp_duration_at(from: &[f64], to: &[f64], peak_speed: f64) -> f64 {
    let travel = from
        .iter()
        .zip(to)
        .map(|(a, b)| (a - b).abs())
        .fold(0.0_f64, f64::max);
    if !travel.is_finite() {
        return RETURN_MIN_S;
    }
    (0.5 * std::f64::consts::PI * travel / peak_speed).max(RETURN_MIN_S)
}

#[cfg(test)]
fn ramp_duration(from: &[f64], to: &[f64]) -> f64 {
    ramp_duration_at(from, to, RETURN_PEAK_SPEED)
}

fn shutdown_ramp_duration(from: &[f64], to: &[f64]) -> f64 {
    ramp_duration_at(from, to, SHUTDOWN_PEAK_SPEED)
}

fn sleep_until(deadline: Instant) {
    let now = Instant::now();
    if deadline > now {
        thread::sleep(deadline - now);
    }
}

fn interrupted() -> bool {
    INTERRUPTED.load(Ordering::SeqCst)
}

extern "C" fn on_signal(_signal: libc::c_int) {
    INTERRUPTED.store(true, Ordering::SeqCst);
}

fn install_signal_handlers() -> Result<()> {
    unsafe {
        let mut action: libc::sigaction = std::mem::zeroed();
        action.sa_sigaction = on_signal as libc::sighandler_t;
        libc::sigemptyset(&mut action.sa_mask);
        action.sa_flags = 0;
        for signal in [libc::SIGINT, libc::SIGTERM] {
            if libc::sigaction(signal, &action, std::ptr::null_mut()) != 0 {
                bail!(
                    "cannot install the signal handler: {}",
                    io::Error::last_os_error()
                );
            }
        }
    }
    Ok(())
}

// --------------------------------------------------------------------------- //
// CLI
// --------------------------------------------------------------------------- //

#[derive(Debug)]
struct Args {
    trajectory: PathBuf,
    channel: String,
    model: String,
    model_spec: String,
    kp_spec: String,
    kd_spec: String,
    ramp: f64,
    speed: f64,
    max_follow_error: f64,
    gripper_stall: f64,
    allow_kp_mismatch: bool,
    log: Option<PathBuf>,
    delay: f64,
    execute: bool,
    stay_enabled: bool,
    track: bool,
    check: bool,
    active_report: bool,
}

impl Default for Args {
    fn default() -> Self {
        Self {
            trajectory: PathBuf::new(),
            channel: "can0".into(),
            model: "rs-00".into(),
            model_spec: DEFAULT_MODEL_SPEC.into(),
            kp_spec: "50,2:150,3:150".into(),
            kd_spec: "3,2:10,3:10,4:5,5:4,6:4,7:4".into(),
            ramp: 2.0,
            speed: 1.0,
            max_follow_error: 0.0,
            gripper_stall: 0.0,
            allow_kp_mismatch: false,
            log: None,
            delay: 0.0,
            execute: false,
            stay_enabled: false,
            track: false,
            check: false,
            active_report: false,
        }
    }
}

#[derive(Debug)]
struct DumpArgs {
    channel: String,
    ids_spec: String,
}

/// `rebot-edge serve`: the same binary, reachable over HTTP instead of argv.
///
/// The server does not reimplement the player. `POST /play` writes the body to a
/// temp file and re-executes this binary with the configured flags, so a
/// trajectory over HTTP and the same file on the command line go through one
/// code path, one validation, and one signal handler. `/stop` is a SIGTERM to
/// that child, which is exactly what Ctrl-C does: ramp home, then drop torque.
#[derive(Debug, Clone)]
struct ServeArgs {
    listen: String,
    token: String,
    token_file: String,
    allow_execute: bool,
    allow_no_token: bool,
    channel: String,
    ids_spec: String,
    model: String,
    model_spec: String,
    kp_spec: String,
    kd_spec: String,
    ramp: f64,
    gripper_stall: f64,
    stay_enabled: bool,
    track: bool,
}

impl Default for ServeArgs {
    fn default() -> Self {
        Self {
            listen: "0.0.0.0:8081".into(),
            token: String::new(),
            token_file: String::new(),
            allow_execute: false,
            allow_no_token: false,
            channel: "can0".into(),
            ids_spec: "1,2,3,4,5,6,7".into(),
            model: "rs-00".into(),
            model_spec: DEFAULT_MODEL_SPEC.into(),
            kp_spec: "50,2:150,3:150".into(),
            kd_spec: "3,2:10,3:10,4:5,5:4,6:4,7:4".into(),
            ramp: 2.0,
            gripper_stall: 0.0,
            stay_enabled: false,
            track: false,
        }
    }
}

#[derive(Debug)]
enum Command {
    Help,
    Version,
    Dump(Box<DumpArgs>),
    Serve(Box<ServeArgs>),
    Run(Box<Args>),
}

const USAGE: &str = "\
rebot-edge - replay a planned joint-space trajectory on a B601-RS over SocketCAN

USAGE:
    rebot-edge [OPTIONS] <TRAJECTORY.json>
    rebot-edge dump [--channel IFACE] [--ids 1,2,...]
    rebot-edge serve [OPTIONS]

The input is the JSON written by host/app/plan_with_pinocchio.py (format
'rebot-joint-trajectory'). Dry run is the default; nothing moves without
--execute. `dump` is the read-only pose snapshot (parameter 0x7019) that the
planner takes as its `--joints` input; it enables nothing.

`serve` replaces the ssh path: it exposes GET /health, GET /pose,
POST /play?dry=1, POST /play (trajectory JSON in the body) and POST /stop over
plain HTTP. Execution stays off until the server is started with
--allow-execute, and a token is required on anything but loopback.

OPTIONS:
    --channel <IFACE>        SocketCAN interface (default: can0)
    --model <MODEL>          Fallback motor model for MIT scaling (default: rs-00)
    --model-spec <SPEC>      Per-joint models, ID:MODEL,... (default:
                             1:rs-06,2:rs-06,3:rs-06; pass '' for a uniform arm)
    --kp <SPEC>              Position gain: base and/or ID:VALUE (default: 50,2:150,3:150)
    --kd <SPEC>              Damping gain: base and/or ID:VALUE
                             (default: 3,2:10,3:10,4:5,5:4,6:4,7:4)
    --ramp <SECONDS>         Ease-on time to the planned start pose (default: 2.0)
    --speed <FACTOR>         Time scale, 0.5 = twice as slow (default: 1.0)
    --delay <SECONDS>        Hold the start pose before the motion starts
    --max-follow-error <RAD> Abort when measured position lags by more than this
                             (checked once a second of trajectory time; 0 disables)
    --gripper-stall <SECONDS> Abort if motor 7 is >0.15 rad from its command and
                             moving <0.08 rad/s for this long; 0 disables,
                             otherwise 0.1..5.0 s
                             (default: 0; the systemd unit uses 0.3)
    --log <FILE>             Write commanded and measured angles as JSON
    --allow-kp-mismatch      Play a sag-compensated file at a different kp
    --stay-enabled           Leave torque on when the script exits
    --active-report          Enable Type-24 active reporting while playing;
                             reports arrive at the motor's default 10 ms interval
    --track                  Print measured position at 25/50/75/100 %
    --execute                Actually move. Without this the player is read-only
    --check                  Validate and describe the file without touching CAN
    --fast-frames            Accepted for compatibility; Rust frames are always
                             hand-built and need no numpy
    -h, --help               Print this help
    -V, --version            Print the version

EXAMPLES:
    rebot-edge --check trajectories/seq_pino_peel.json
    rebot-edge trajectories/seq_pino_peel.json
    sudo ./rebot-edge trajectories/seq_pino_peel.json --execute --track
    ./rebot-edge dump --ids 1,2,3,4,5,6,7 > current.json
    sudo ./rebot-edge serve --listen 0.0.0.0:8081 --token secret --allow-execute

SERVE OPTIONS:
    --listen <ADDR>          Address to bind (default: 0.0.0.0:8081)
    --token <SECRET>         Shared secret for every request except GET /health
    --token-file <FILE>      Read the secret from a file instead (keeps it out of ps)
    --allow-execute          Let POST /play move the arm (default: dry runs only)
    --allow-no-token         Permit a non-loopback listen without a token
    --ids <SPEC>             Joints for GET /pose (default: 1,2,3,4,5,6,7)
    --channel/--model/--model-spec/--kp/--kd/--ramp/--gripper-stall/
    --stay-enabled/--track  Defaults for the player that /play re-executes
";

fn parse_args(argv: Vec<String>) -> Result<Command> {
    if argv.first().map(String::as_str) == Some("dump") {
        return parse_dump_args(argv.into_iter().skip(1).collect());
    }
    if argv.first().map(String::as_str) == Some("serve") {
        return parse_serve_args(argv.into_iter().skip(1).collect());
    }

    let mut args = Args::default();
    let mut trajectory: Option<PathBuf> = None;
    let mut iter = argv.into_iter().peekable();

    while let Some(raw) = iter.next() {
        let (flag, inline) = if raw.starts_with("--") {
            match raw.split_once('=') {
                Some((flag, value)) => (flag.to_string(), Some(value.to_string())),
                None => (raw.clone(), None),
            }
        } else {
            (raw.clone(), None)
        };

        match flag.as_str() {
            "-h" | "--help" => return Ok(Command::Help),
            "-V" | "--version" => return Ok(Command::Version),
            "--execute" => args.execute = true,
            "--stay-enabled" => args.stay_enabled = true,
            "--active-report" => args.active_report = true,
            "--track" => args.track = true,
            "--allow-kp-mismatch" => args.allow_kp_mismatch = true,
            "--check" => args.check = true,
            "--fast-frames" => {}
            "--sdk-path" => bail!(
                "--sdk-path is not supported; the Rust player talks to SocketCAN directly"
            ),
            "--channel" => args.channel = take_value(&mut iter, inline, "--channel")?,
            "--model" => args.model = take_value(&mut iter, inline, "--model")?,
            "--model-spec" => args.model_spec = take_value(&mut iter, inline, "--model-spec")?,
            "--kp" => args.kp_spec = take_value(&mut iter, inline, "--kp")?,
            "--kd" => args.kd_spec = take_value(&mut iter, inline, "--kd")?,
            "--ramp" => {
                args.ramp = parse_number(&take_value(&mut iter, inline, "--ramp")?, "--ramp")?
            }
            "--speed" => {
                args.speed = parse_number(&take_value(&mut iter, inline, "--speed")?, "--speed")?
            }
            "--delay" => {
                args.delay = parse_number(&take_value(&mut iter, inline, "--delay")?, "--delay")?
            }
            "--max-follow-error" => {
                args.max_follow_error = parse_number(
                    &take_value(&mut iter, inline, "--max-follow-error")?,
                    "--max-follow-error",
                )?
            }
            "--gripper-stall" => {
                args.gripper_stall = parse_number(
                    &take_value(&mut iter, inline, "--gripper-stall")?,
                    "--gripper-stall",
                )?
            }
            "--log" => {
                let value = take_value(&mut iter, inline, "--log")?;
                args.log = if value.is_empty() {
                    None
                } else {
                    Some(PathBuf::from(value))
                };
            }
            _ => {
                if flag.starts_with('-') && flag != "-" {
                    bail!("unknown option '{flag}' (try --help)");
                }
                if trajectory.is_some() {
                    bail!("unexpected argument '{raw}'");
                }
                trajectory = Some(PathBuf::from(raw));
            }
        }
    }

    let Some(trajectory) = trajectory else {
        bail!("missing trajectory JSON\n\n{USAGE}");
    };
    if args.check && args.execute {
        bail!("--check and --execute are mutually exclusive");
    }
    if !args.ramp.is_finite() || !args.speed.is_finite() || !args.delay.is_finite() {
        bail!("--ramp, --speed and --delay must be finite numbers");
    }
    check_gripper_stall(args.gripper_stall)?;
    args.trajectory = trajectory;
    Ok(Command::Run(Box::new(args)))
}

fn parse_dump_args(argv: Vec<String>) -> Result<Command> {
    let mut channel = "can0".to_string();
    let mut ids_spec = "1,2,3,4,5,6,7".to_string();
    let mut iter = argv.into_iter().peekable();

    while let Some(raw) = iter.next() {
        let (flag, inline) = if raw.starts_with("--") {
            match raw.split_once('=') {
                Some((flag, value)) => (flag.to_string(), Some(value.to_string())),
                None => (raw.clone(), None),
            }
        } else {
            (raw.clone(), None)
        };

        match flag.as_str() {
            "-h" | "--help" => return Ok(Command::Help),
            "-V" | "--version" => return Ok(Command::Version),
            "--channel" => channel = take_value(&mut iter, inline, "--channel")?,
            "--ids" => ids_spec = take_value(&mut iter, inline, "--ids")?,
            _ => bail!("unknown dump option '{flag}' (try --help)"),
        }
    }

    Ok(Command::Dump(Box::new(DumpArgs { channel, ids_spec })))
}

fn parse_serve_args(argv: Vec<String>) -> Result<Command> {
    let mut serve = ServeArgs::default();
    let mut iter = argv.into_iter().peekable();

    while let Some(raw) = iter.next() {
        let (flag, inline) = if raw.starts_with("--") {
            match raw.split_once('=') {
                Some((flag, value)) => (flag.to_string(), Some(value.to_string())),
                None => (raw.clone(), None),
            }
        } else {
            (raw.clone(), None)
        };

        match flag.as_str() {
            "-h" | "--help" => return Ok(Command::Help),
            "-V" | "--version" => return Ok(Command::Version),
            "--listen" => serve.listen = take_value(&mut iter, inline, "--listen")?,
            "--token" => serve.token = take_value(&mut iter, inline, "--token")?,
            "--token-file" => serve.token_file = take_value(&mut iter, inline, "--token-file")?,
            "--allow-execute" => serve.allow_execute = true,
            "--allow-no-token" => serve.allow_no_token = true,
            "--channel" => serve.channel = take_value(&mut iter, inline, "--channel")?,
            "--ids" => serve.ids_spec = take_value(&mut iter, inline, "--ids")?,
            "--model" => serve.model = take_value(&mut iter, inline, "--model")?,
            "--model-spec" => serve.model_spec = take_value(&mut iter, inline, "--model-spec")?,
            "--kp" => serve.kp_spec = take_value(&mut iter, inline, "--kp")?,
            "--kd" => serve.kd_spec = take_value(&mut iter, inline, "--kd")?,
            "--ramp" => {
                serve.ramp = parse_number(&take_value(&mut iter, inline, "--ramp")?, "--ramp")?
            }
            "--gripper-stall" => {
                serve.gripper_stall = parse_number(
                    &take_value(&mut iter, inline, "--gripper-stall")?,
                    "--gripper-stall",
                )?
            }
            "--stay-enabled" => serve.stay_enabled = true,
            "--track" => serve.track = true,
            _ => bail!("unknown serve option '{flag}' (try --help)"),
        }
    }
    check_gripper_stall(serve.gripper_stall)?;

    Ok(Command::Serve(Box::new(serve)))
}

fn take_value(
    iter: &mut std::iter::Peekable<std::vec::IntoIter<String>>,
    inline: Option<String>,
    flag: &str,
) -> Result<String> {
    if let Some(value) = inline {
        return Ok(value);
    }
    iter.next()
        .ok_or_else(|| Error::new(format!("{flag} needs a value")))
}

fn parse_number(text: &str, flag: &str) -> Result<f64> {
    let value: f64 = text
        .parse()
        .map_err(|_| Error::new(format!("{flag}: cannot parse '{text}' as a number")))?;
    if !value.is_finite() {
        bail!("{flag}: '{text}' is not a finite number");
    }
    Ok(value)
}

fn check_gripper_stall(seconds: f64) -> Result<()> {
    if seconds == 0.0 {
        return Ok(());
    }
    if !(GRIPPER_STALL_MIN_S..=GRIPPER_STALL_MAX_S).contains(&seconds) {
        bail!(
            "--gripper-stall must be 0 (off) or between {GRIPPER_STALL_MIN_S:.1} and \
             {GRIPPER_STALL_MAX_S:.1} s"
        );
    }
    Ok(())
}

fn parse_gain_spec(
    spec: &str,
    default: f64,
    ceiling: f64,
    flag: &str,
) -> Result<(f64, BTreeMap<u8, f64>)> {
    let mut base = default.clamp(0.0, ceiling);
    let mut overrides: BTreeMap<u8, f64> = BTreeMap::new();

    for part in spec.split(',') {
        let part = part.trim();
        if part.is_empty() {
            continue;
        }
        if let Some((key, value)) = part.split_once(':') {
            let motor_id: u8 = key.trim().parse().map_err(|_| {
                Error::new(format!(
                    "--{flag}: cannot parse '{part}' (expected ID:VALUE)"
                ))
            })?;
            if !(1..=7).contains(&motor_id) {
                bail!("--{flag}: motor IDs are 1..7 (got {motor_id})");
            }
            let gain = parse_number(value.trim(), &format!("--{flag}"))?;
            overrides.insert(motor_id, gain.clamp(0.0, ceiling));
        } else {
            base = parse_number(part, &format!("--{flag}"))?.clamp(0.0, ceiling);
        }
    }
    Ok((base, overrides))
}

fn format_gains(ids: &[u8], gains: &[f64]) -> String {
    ids.iter()
        .zip(gains)
        .map(|(id, gain)| format!("{id}:{gain:.1}"))
        .collect::<Vec<_>>()
        .join(" ")
}

// --------------------------------------------------------------------------- //
// Reporting / logging
// --------------------------------------------------------------------------- //

fn fmt_vec(values: Option<&[f64]>) -> String {
    match values {
        Some(values) if !values.is_empty() => format!(
            "[{}]",
            values
                .iter()
                .map(|value| format!("{value:+.4}"))
                .collect::<Vec<_>>()
                .join(" ")
        ),
        _ => "?".into(),
    }
}

fn banner(
    trajectory: &Trajectory,
    args: &Args,
    models: &[Model],
    kp: &[f64],
    mismatched: &[u8],
) -> Result<()> {
    let speed = args.speed.clamp(0.1, 5.0);
    println!("Planned trajectory");
    println!("  file      : {}", trajectory.path.display());
    println!(
        "  motors    : {}",
        trajectory
            .ids
            .iter()
            .map(|id| id.to_string())
            .collect::<Vec<_>>()
            .join(", ")
    );
    println!(
        "  created   : {}",
        trajectory.raw.created.as_deref().unwrap_or("?")
    );
    println!(
        "  duration  : {:.2} s at {} Hz (x{:.2} speed -> {:.2} s)",
        trajectory.duration_s,
        trajectory.hz,
        speed,
        trajectory.duration_s / speed
    );
    println!(
        "  cartesian : start {} -> end {} (delta {} m)",
        fmt_vec(trajectory.raw.start_tcp.as_deref()),
        fmt_vec(trajectory.raw.end_tcp.as_deref()),
        fmt_vec(trajectory.raw.delta_xyz.as_deref())
    );
    println!(
        "  peak speed: {:.3} rad/s, worst IK residual {:.3} mm",
        trajectory.raw.peak_joint_speed.unwrap_or(0.0),
        1000.0 * trajectory.raw.worst_ik_residual_m.unwrap_or(0.0)
    );
    println!("  kp        : {}", format_gains(&trajectory.ids, kp));
    if trajectory.has_tau {
        println!(
            "  tau       : modelled gravity feedforward, peak {:.2} Nm",
            trajectory.peak_tau
        );
    }
    if trajectory.has_v {
        println!(
            "  velocity  : feedforward on every waypoint, peak {:.2} rad/s",
            trajectory.peak_v
        );
    }
    println!(
        "  models    : {}",
        models
            .iter()
            .zip(&trajectory.ids)
            .map(|(model, id)| format!("{id}:{}", model.name))
            .collect::<Vec<_>>()
            .join(", ")
    );

    let bound = trajectory
        .raw
        .sag_compensation
        .as_ref()
        .or(trajectory.raw.trim.as_ref());
    if let Some(bound) = bound {
        let bound_kp = bound_kp_map(bound);
        let corrected = bound_kp
            .iter()
            .map(|(id, value)| format!("{id}:{value:.1}"))
            .collect::<Vec<_>>()
            .join(", ");
        println!(
            "  corrected : {}{}",
            corrected,
            if mismatched.is_empty() {
                ""
            } else {
                "   <- MISMATCH with --kp"
            }
        );
    }

    if args.active_report {
        let timing = if args.execute {
            "enabled during playback"
        } else {
            "requires --execute"
        };
        println!("  report    : Type-24 active reporting, motor default 10 ms ({timing})");
    }

    let mode = if args.check {
        "check (offline - no CAN access)"
    } else if args.execute {
        "EXECUTE - the arm will move"
    } else {
        "dry run (read-only)"
    };
    println!("  mode      : {mode}");
    Ok(())
}

fn bound_kp_map(bound: &Correction) -> BTreeMap<u8, f64> {
    bound
        .kp
        .iter()
        .filter_map(|(key, value)| key.parse::<u8>().ok().map(|id| (id, *value)))
        .collect()
}

fn find_kp_mismatch(ids: &[u8], kp: &[f64], bound: Option<&Correction>) -> Vec<u8> {
    let Some(bound) = bound else {
        return Vec::new();
    };
    let bound_kp = bound_kp_map(bound);
    ids.iter()
        .copied()
        .enumerate()
        .filter(|(index, id)| {
            bound_kp
                .get(id)
                .is_some_and(|value| (kp[*index] - value).abs() > 0.02 * value.abs())
        })
        .map(|(_, id)| id)
        .collect()
}

#[derive(Serialize)]
struct LogSample {
    t: f64,
    cmd: Vec<f64>,
    meas: Vec<f64>,
}

#[derive(Serialize)]
struct LogFile<'a> {
    format: &'a str,
    version: u32,
    created: String,
    trajectory: String,
    ids: &'a [u8],
    kp: BTreeMap<String, f64>,
    kd: BTreeMap<String, f64>,
    control_hz: u32,
    speed: f64,
    samples: &'a [LogSample],
    // v2 additions, flattened so a v1 reader sees the same top-level shape and
    // ignores what it does not know. Each field is present only when it happened.
    #[serde(flatten)]
    notes: &'a LogNotes,
}

/// What went wrong (or how the ticks ran), collected during playback and
/// written after it -- never on the realtime loop.
#[derive(Serialize, Default)]
struct LogNotes {
    #[serde(skip_serializing_if = "Option::is_none")]
    abort: Option<LogAbort>,
    #[serde(skip_serializing_if = "Option::is_none")]
    gripper_stall: Option<LogGripperStall>,
    #[serde(skip_serializing_if = "Option::is_none")]
    tick: Option<LogTick>,
    #[serde(skip_serializing_if = "Option::is_none")]
    follow_error_max: Option<LogFollowError>,
}

#[derive(Serialize)]
struct LogAbort {
    t: f64,
    reason: String,
}

#[derive(Serialize)]
struct LogGripperStall {
    t: f64,
    commanded: f64,
    measured: f64,
}

#[derive(Serialize)]
struct LogTick {
    mean_ms: f64,
    p95_ms: f64,
    max_ms: f64,
}

#[derive(Serialize)]
struct LogFollowError {
    motor: u8,
    rad: f64,
}

/// A fault frame already queued at bring-up aborts the first poll, before any
/// sample is logged. The abort note is still the evidence the caller needs:
/// write the log whenever there is something to record, not only samples.
fn log_has_evidence(samples: &[LogSample], notes: &LogNotes) -> bool {
    !samples.is_empty() || notes.abort.is_some() || notes.gripper_stall.is_some()
}

fn write_log(
    path: &Path,
    trajectory: &Trajectory,
    args: &Args,
    kp: &BTreeMap<String, f64>,
    kd: &BTreeMap<String, f64>,
    samples: &[LogSample],
    notes: &LogNotes,
) -> Result<()> {
    let payload = LogFile {
        format: "rebot-trajectory-log",
        version: 2,
        created: utc_timestamp(),
        trajectory: trajectory.path.display().to_string(),
        ids: &trajectory.ids,
        kp: kp.clone(),
        kd: kd.clone(),
        control_hz: trajectory.hz,
        speed: args.speed.clamp(0.1, 5.0),
        samples,
        notes,
    };
    let text = serde_json::to_string(&payload)?;
    fs::write(path, text)
        .map_err(|error| Error::new(format!("cannot write {}: {error}", path.display())))?;
    println!(
        "  log       : {} ({} waypoints)",
        path.display(),
        samples.len()
    );
    Ok(())
}

/// UTC ISO-8601, no chrono dependency. An explicit `Z` is unambiguous and good
/// enough for a run log.
fn utc_timestamp() -> String {
    let seconds = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|duration| duration.as_secs() as i64)
        .unwrap_or(0);
    let days = seconds.div_euclid(86_400);
    let second_of_day = seconds.rem_euclid(86_400);
    let (year, month, day) = civil_from_days(days);
    format!(
        "{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}Z",
        second_of_day / 3600,
        (second_of_day % 3600) / 60,
        second_of_day % 60
    )
}

fn civil_from_days(days: i64) -> (i64, u32, u32) {
    // Howard Hinnant's days -> civil date algorithm.
    let z = days + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = (z - era * 146_097) as u64;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let year = yoe as i64 + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let month = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let year = if month <= 2 { year + 1 } else { year };
    (year, month, day)
}

// --------------------------------------------------------------------------- //
// Playback
// --------------------------------------------------------------------------- //

fn can_is_up(channel: &str) -> bool {
    let path = format!("/sys/class/net/{channel}/operstate");
    match fs::read_to_string(&path) {
        // CAN interfaces (and vcan) often report "unknown" when administratively
        // up because the driver has no carrier state to report; "down" is the
        // state that matters.
        Ok(state) => matches!(state.trim(), "up" | "unknown"),
        Err(_) => false,
    }
}

/// A blocked gripper shows a persistent position error while its measured
/// velocity is near zero. Normal closing lags by up to ~0.4 rad, but the jaw is
/// moving then; this predicate deliberately requires both conditions.
fn gripper_stall_condition(target: f64, measured: f64, velocity: f64) -> bool {
    (target - measured).abs() > GRIPPER_STALL_ERROR
        && velocity.abs() < GRIPPER_STALL_VELOCITY
}

fn read_drift<T: Transport>(arm: &mut Arm<T>, trajectory: &Trajectory) -> Result<f64> {
    let mut worst = 0.0_f64;
    for joint in 0..arm.ids.len() {
        let current = arm
            .bus
            .read_position(arm.ids[joint], Duration::from_millis(500))?;
        arm.start[joint] = current;
        arm.last[joint] = current;
        let drift = trajectory.start[joint] - current;
        worst = worst.max(drift.abs());
        println!(
            "  motor {:<2} now {:+.4}  plan starts {:+.4}  (drift {:+.4})",
            arm.ids[joint], current, trajectory.start[joint], drift
        );
    }
    println!("  worst drift from plan start: {worst:.4} rad");
    if worst > 20.0_f64.to_radians() {
        bail!(
            "the arm is more than 20 deg from the plan's start pose; \
             re-plan from a fresh `rebot-edge dump` instead of ramping across"
        );
    }
    Ok(worst)
}

/// The waypoint indices `--track` reports at 25/50/75/100% of the trajectory.
/// The play loop indexes `0..len-1`, so 100% must be `len-1`: using `len`
/// (the old computation) never matches and silently drops the final sample.
fn track_samples(len: usize) -> BTreeSet<usize> {
    if len == 0 {
        return BTreeSet::new();
    }
    let last = (len - 1) as f64;
    [0.25_f64, 0.5, 0.75, 1.0]
        .iter()
        .map(|fraction| (last * fraction).round() as usize)
        .collect()
}

fn play<T: Transport>(
    arm: &mut Arm<T>,
    trajectory: &Trajectory,
    args: &Args,
    logged: &mut Vec<LogSample>,
    notes: &mut LogNotes,
) -> Result<i32> {
    let speed = args.speed.clamp(0.1, 5.0);
    let sample_at: BTreeSet<usize> = if args.track {
        track_samples(trajectory.raw.waypoints.len())
    } else {
        BTreeSet::new()
    };

    arm.bring_up()?;
    // The reader thread is only needed for active reporting. Without it the
    // playback loop uses a zero-wait drain of the queued command replies.
    if args.active_report {
        arm.start_telemetry()?;
        if let Err(error) = arm.start_active_reports() {
            arm.stop_telemetry();
            return Err(error);
        }
    }

    let mut last_elapsed = 0.0_f64;
    let result = (|| -> Result<i32> {
        let drift = trajectory
            .start
            .iter()
            .zip(&arm.last)
            .map(|(planned, current)| (planned - current).abs())
            .fold(0.0_f64, f64::max);
        if drift > 1e-4 {
            println!("\nAligning to the planned start pose...");
            let targets = trajectory.start.clone();
            arm.move_leg(
                &targets,
                args.ramp.clamp(0.5, 10.0),
                "align",
                args.track,
                true,
                false,
            )?;
            if interrupted() {
                println!("Interrupted - ramping home before disabling.");
                return Ok(130);
            }
        }

        if args.delay > 0.0 {
            println!(
                "  waiting {:.0} s while holding the start pose - get your ruler ready",
                args.delay
            );
            let deadline = Instant::now() + Duration::from_secs_f64(args.delay);
            let mut announced: i64 = -1;
            while Instant::now() < deadline {
                if interrupted() {
                    println!("Interrupted - ramping home before disabling.");
                    return Ok(130);
                }
                arm.command(&trajectory.start, None, None, true)?;
                let remaining = deadline
                    .saturating_duration_since(Instant::now())
                    .as_secs_f64()
                    .ceil() as i64;
                if remaining != announced && remaining >= 0 {
                    announced = remaining;
                    println!("    starting in {remaining:2} s");
                }
                thread::sleep(arm.period);
            }
            println!("    GO");
        }

        if let (Some(start), Some(end)) = (trajectory.raw.hold_start_s, trajectory.raw.hold_end_s) {
            println!(
                "  far point reached at t=+{start:.1} s, held until t=+{end:.1} s (hold {:.1} s)",
                end - start
            );
        }

        println!("\nPlaying {} waypoints...", trajectory.raw.waypoints.len());
        let t0 = Instant::now();
        let total = trajectory.last_t / speed;
        let mut next_follow_check = 0.5_f64;
        let mut abort_reason: Option<String> = None;
        let mut stamps: Vec<Instant> = Vec::with_capacity(trajectory.raw.waypoints.len());
        let mut held = arm.start.clone();
        let mut held_velocity = vec![0.0_f64; arm.ids.len()];
        let gripper = arm.ids.iter().position(|id| *id == GRIPPER_ID);
        let mut gripper_stall_since: Option<Instant> = None;
        let logging = args.log.is_some();

        for (index, waypoint) in trajectory.raw.waypoints.iter().enumerate() {
            if interrupted() {
                break;
            }

            let targets = waypoint.q.clone();
            let elapsed = waypoint.t / speed;
            last_elapsed = elapsed;
            stamps.push(Instant::now());

            // Keep the latest measured pose without ever waiting: the reader
            // thread when active reporting is on, otherwise a zero-wait drain of
            // the status replies already queued by the previous command.
            if arm.telemetry_active {
                if let Some(states) = arm.bus.latest_states() {
                    for (joint, state) in states.iter().enumerate() {
                        if let Some(state) = state {
                            held[joint] = state.position;
                            held_velocity[joint] = state.velocity;
                        }
                    }
                }
            } else {
                let states = arm.bus.poll_available(&arm.ids, &arm.models)?;
                for (joint, state) in states.iter().enumerate() {
                    if let Some(state) = state {
                        held[joint] = state.position;
                        held_velocity[joint] = state.velocity;
                    }
                }
            }

            if let Some(gripper) = gripper {
                if args.gripper_stall > 0.0
                    && gripper_stall_condition(targets[gripper], held[gripper], held_velocity[gripper])
                {
                    let since = *gripper_stall_since.get_or_insert_with(Instant::now);
                    if since.elapsed().as_secs_f64() >= args.gripper_stall {
                        // Do not send another closing frame. Hold the measured
                        // opening for the return ramp so the jaws stop pressing;
                        // shut_down() then ramps the arm home and drops torque.
                        arm.start[gripper] = held[gripper];
                        arm.last[gripper] = held[gripper];
                        notes.gripper_stall = Some(LogGripperStall {
                            t: elapsed,
                            commanded: targets[gripper],
                            measured: held[gripper],
                        });
                        abort_reason = Some(format!(
                            "gripper motor {} stalled: commanded {:+.3}, measured {:+.3} \
                             ({:.3} rad error, {:.3} rad/s); the jaws are blocked",
                            arm.ids[gripper],
                            targets[gripper],
                            held[gripper],
                            targets[gripper] - held[gripper],
                            held_velocity[gripper]
                        ));
                        break;
                    }
                } else {
                    gripper_stall_since = None;
                }
            }

            let sample = args.track && sample_at.contains(&index);
            let check_due = args.max_follow_error > 0.0 && elapsed >= next_follow_check;
            let torques = waypoint.tau.as_deref();
            arm.command(&targets, torques, waypoint.v.as_deref(), false)?;

            if logging {
                logged.push(LogSample {
                    t: round4(elapsed),
                    cmd: targets.clone(),
                    meas: held.clone(),
                });
            }

            if check_due {
                next_follow_check = elapsed + 1.0;
                for joint in 0..arm.ids.len() {
                    let gap = (targets[joint] - held[joint]).abs();
                    // The worst gap seen at any check, abort or not: the log's
                    // follow_error_max is a measurement, not just a fault flag.
                    if notes
                        .follow_error_max
                        .as_ref()
                        .map_or(true, |worst| gap > worst.rad)
                    {
                        notes.follow_error_max = Some(LogFollowError {
                            motor: arm.ids[joint],
                            rad: gap,
                        });
                    }
                    if gap > args.max_follow_error {
                        abort_reason = Some(format!(
                            "motor {} is not following: commanded {:+.3}, measured {:+.3} ({:.2} rad behind)",
                            arm.ids[joint], targets[joint], held[joint], gap
                        ));
                        break;
                    }
                }
            }
            if abort_reason.is_some() {
                break;
            }

            if sample {
                for joint in 0..arm.ids.len() {
                    println!(
                        "    t={:5.2}s commanded {:+.4} measured {:+.4}",
                        waypoint.t / speed,
                        targets[joint],
                        held[joint]
                    );
                }
            }

            sleep_until(t0 + Duration::from_secs_f64(elapsed));
        }

        if interrupted() {
            println!("Interrupted - ramping home before disabling.");
            return Ok(130);
        }

        let exit_code = if let Some(reason) = &abort_reason {
            println!("  ABORTED at t={last_elapsed:.2}s: {reason}");
            notes.abort = Some(LogAbort {
                t: last_elapsed,
                reason: reason.clone(),
            });
            1
        } else {
            println!(
                "  played in {:.2} s (planned {:.2} s)",
                t0.elapsed().as_secs_f64(),
                total
            );
            0
        };

        if stamps.len() > 2 {
            let gaps: Vec<f64> = stamps
                .windows(2)
                .map(|pair| (pair[1] - pair[0]).as_secs_f64() * 1000.0)
                .collect();
            if !gaps.is_empty() {
                let nominal = 1000.0 / f64::from(trajectory.hz);
                let mean = gaps.iter().sum::<f64>() / gaps.len() as f64;
                let mut sorted = gaps.clone();
                sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                let p95 =
                    sorted[((0.95 * (sorted.len() - 1) as f64) as usize).min(sorted.len() - 1)];
                let max = gaps.iter().copied().fold(0.0_f64, f64::max);
                let late = gaps.iter().filter(|gap| **gap > 1.5 * nominal).count();
                notes.tick = Some(LogTick {
                    mean_ms: mean,
                    p95_ms: p95,
                    max_ms: max,
                });
                println!(
                    "  tick spacing: mean {mean:.1} ms, p95 {p95:.1} ms, max {max:.1} ms \
                     (target {nominal:.1} ms, {late} of {} ticks >1.5x target)",
                    gaps.len()
                );
            }
        }

        println!("\nFinal angles vs plan:");
        let planned = trajectory.raw.waypoints.last().map(|waypoint| &waypoint.q);
        for joint in 0..arm.ids.len() {
            println!(
                "  motor {:<2} planned {:+.4}  last commanded {:+.4}",
                arm.ids[joint],
                planned.map_or(f64::NAN, |values| values[joint]),
                arm.last[joint]
            );
        }

        Ok(exit_code)
    })();

    // Fault frames and bus errors propagate as Err, bypassing the abort_reason
    // machinery inside the loop. Record them in the log too: an abort without a
    // timestamp and reason is exactly the evidence gap that makes a fault
    // undiagnosable after the temp log is gone.
    if let Err(error) = &result {
        if notes.abort.is_none() {
            notes.abort = Some(LogAbort {
                t: last_elapsed,
                reason: error.to_string(),
            });
        }
        println!("  ABORTED at t={last_elapsed:.2}s: {error}");
    }

    if args.active_report {
        arm.stop_active_reports();
    }
    arm.stop_telemetry();
    result
}

fn round4(value: f64) -> f64 {
    (value * 10_000.0).round() / 10_000.0
}

fn round6(value: f64) -> f64 {
    (value * 1_000_000.0).round() / 1_000_000.0
}

fn parse_ids(spec: &str) -> Result<Vec<u8>> {
    let mut ids = Vec::new();
    for part in spec.split(',') {
        let part = part.trim();
        if part.is_empty() {
            continue;
        }
        let id: u8 = part
            .parse()
            .map_err(|_| Error::new(format!("--ids: cannot parse '{part}' as a motor ID")))?;
        if !(1..=7).contains(&id) {
            bail!("--ids: motor IDs are 1..7 (got {id})");
        }
        if ids.contains(&id) {
            bail!("--ids: motor ID {id} is listed twice");
        }
        ids.push(id);
    }
    if ids.is_empty() {
        bail!("--ids is empty");
    }
    Ok(ids)
}

/// Read-only pose snapshot: parameter 0x7019 from every requested motor. No
/// enable, no torque, no frames other than the parameter reads.
fn pose_payload(channel: &str, ids_spec: &str) -> Result<serde_json::Value> {
    let ids = parse_ids(ids_spec)?;
    if !can_is_up(channel) {
        bail!(
            "{channel} is not UP. Bring it up first:\n  \
             sudo systemctl start can0-setup.service\n  \
             ip -details link show {channel}"
        );
    }
    let bus = CanBus::open(channel)?;
    let mut q = Vec::with_capacity(ids.len());
    for id in &ids {
        q.push(round6(bus.read_position(*id, Duration::from_millis(500))?));
    }

    Ok(serde_json::json!({
        "ids": ids,
        "q": q,
        "timestamp": utc_timestamp(),
        "channel": channel,
        "note": "read-only snapshot, parameter 0x7019 (mechanical position, rad)",
    }))
}

fn dump_joints(channel: &str, ids_spec: &str) -> Result<i32> {
    println!(
        "{}",
        serde_json::to_string_pretty(&pose_payload(channel, ids_spec)?)?
    );
    Ok(0)
}

// --------------------------------------------------------------------------- //
// HTTP server
// --------------------------------------------------------------------------- //

const MAX_BODY_BYTES: usize = 4 * 1024 * 1024;
const MAX_HEADER_BYTES: usize = 32 * 1024;
/// The board has half a gigabyte; a handler thread costs a stack, so cap them.
const MAX_CONNECTIONS: usize = 32;
static LIVE_CONNECTIONS: AtomicUsize = AtomicUsize::new(0);

struct ConnectionGuard;

impl Drop for ConnectionGuard {
    fn drop(&mut self) {
        LIVE_CONNECTIONS.fetch_sub(1, Ordering::SeqCst);
    }
}

#[derive(Debug)]
struct Request {
    method: String,
    path: String,
    query: BTreeMap<String, String>,
    token: Option<String>,
    body: Vec<u8>,
}

struct ServerState {
    args: ServeArgs,
    started: Instant,
    playing: AtomicBool,
    stopping: AtomicBool,
    child: AtomicI32,
    // A pidfd pins the child's identity, so /stop can signal it without the
    // PID-reuse race a bare kill(pid) has between exit and store(0). -1 when
    // unavailable (non-Linux or old kernel); stop() then falls back to kill.
    // pidfd_lock serializes using the fd against closing it after the wait.
    child_pidfd: AtomicI32,
    pidfd_lock: Mutex<()>,
}

fn is_loopback(listen: &str) -> bool {
    let host = listen.rsplit_once(':').map(|(host, _)| host).unwrap_or(listen);
    let host = host.trim_start_matches('[').trim_end_matches(']');
    host == "localhost" || host == "::1" || host.starts_with("127.")
}

fn serve(mut args: ServeArgs) -> Result<i32> {
    if args.token.is_empty() && !args.token_file.is_empty() {
        // Kept out of argv: /proc/<pid>/cmdline is world-readable on the board.
        args.token = fs::read_to_string(&args.token_file)
            .map_err(|error| Error::new(format!("cannot read {}: {error}", args.token_file)))?
            .trim()
            .to_string();
    }
    if args.token.is_empty() && !is_loopback(&args.listen) && !args.allow_no_token {
        bail!(
            "refusing to listen on {} without a token: pass --token <secret>, \
             bind 127.0.0.1, or --allow-no-token if the network is trusted",
            args.listen
        );
    }

    let listener = TcpListener::bind(&args.listen).map_err(|error| {
        Error::new(format!(
            "cannot bind {}: {error}\n(is another rebot-edge serve running?)",
            args.listen
        ))
    })?;

    println!("rebot-edge {} http", env!("CARGO_PKG_VERSION"));
    println!("  listen      : http://{}", args.listen);
    println!("  channel     : {}", args.channel);
    println!(
        "  execute     : {}",
        if args.allow_execute {
            "on (POST /play moves the arm)"
        } else {
            "off (dry run only; start with --allow-execute)"
        }
    );
    println!(
        "  auth        : {}",
        if args.token.is_empty() {
            "none".to_string()
        } else {
            "X-Rebot-Token".to_string()
        }
    );
    println!("  GET  /health   GET /pose   POST /play?dry=1   POST /play   POST /stop");

    let state = Arc::new(ServerState {
        args,
        started: Instant::now(),
        playing: AtomicBool::new(false),
        stopping: AtomicBool::new(false),
        child: AtomicI32::new(0),
        child_pidfd: AtomicI32::new(-1),
        pidfd_lock: Mutex::new(()),
    });

    for connection in listener.incoming() {
        match connection {
            Ok(stream) => {
                if LIVE_CONNECTIONS.fetch_add(1, Ordering::SeqCst) >= MAX_CONNECTIONS {
                    LIVE_CONNECTIONS.fetch_sub(1, Ordering::SeqCst);
                    eprintln!("http: connection limit reached, dropping a request");
                    continue;
                }
                let state = Arc::clone(&state);
                thread::spawn(move || {
                    let _guard = ConnectionGuard;
                    let mut stream = stream;
                    if let Err(error) = handle_connection(&mut stream, &state) {
                        eprintln!("http: {error}");
                    }
                });
            }
            Err(error) => eprintln!("http: accept failed: {error}"),
        }
    }
    Ok(0)
}

fn handle_connection(stream: &mut TcpStream, state: &ServerState) -> Result<()> {
    let _ = stream.set_read_timeout(Some(Duration::from_secs(15)));
    let _ = stream.set_write_timeout(Some(Duration::from_secs(60)));
    let mut reader = BufReader::new(stream.try_clone()?);

    let request = match read_request(&mut reader, MAX_BODY_BYTES) {
        Ok(request) => request,
        Err(error) => {
            let status = if error.0.starts_with("body too large") {
                413
            } else {
                400
            };
            return write_json(stream, status, &serde_json::json!({
                "ok": false,
                "error": error.0,
            }));
        }
    };

    let (status, payload) = route(state, &request);
    write_json(stream, status, &payload)
}

fn write_json(stream: &mut TcpStream, status: u16, payload: &serde_json::Value) -> Result<()> {
    let body = serde_json::to_string(payload)?;
    let reason = match status {
        200 => "OK",
        400 => "Bad Request",
        401 => "Unauthorized",
        403 => "Forbidden",
        404 => "Not Found",
        405 => "Method Not Allowed",
        409 => "Conflict",
        413 => "Payload Too Large",
        500 => "Internal Server Error",
        503 => "Service Unavailable",
        _ => "Error",
    };
    let head = format!(
        "HTTP/1.1 {status} {reason}\r\n\
         Content-Type: application/json\r\n\
         Content-Length: {}\r\n\
         Connection: close\r\n\
         Cache-Control: no-store\r\n\r\n",
        body.len()
    );
    stream.write_all(head.as_bytes())?;
    stream.write_all(body.as_bytes())?;
    stream.flush()?;
    Ok(())
}

fn read_request<R: BufRead>(reader: &mut R, limit: usize) -> Result<Request> {
    let mut line = String::new();
    if reader.read_line(&mut line)? == 0 {
        bail!("empty request");
    }
    let mut parts = line.trim_end().split_whitespace();
    let method = parts.next().unwrap_or_default().to_string();
    let target = parts.next().unwrap_or_default().to_string();
    if method.is_empty() || target.is_empty() {
        bail!("malformed request line");
    }

    let (path, query) = match target.split_once('?') {
        Some((path, query)) => (path.to_string(), parse_query(query)),
        None => (target, BTreeMap::new()),
    };

    let mut content_length = 0usize;
    let mut token: Option<String> = None;
    let mut header_bytes = 0usize;
    loop {
        let mut header = String::new();
        if reader.read_line(&mut header)? == 0 {
            bail!("headers ended early");
        }
        header_bytes += header.len();
        if header_bytes > MAX_HEADER_BYTES {
            bail!("headers too large");
        }
        let header = header.trim_end();
        if header.is_empty() {
            break;
        }
        if let Some((name, value)) = header.split_once(':') {
            let value = value.trim();
            match name.trim().to_ascii_lowercase().as_str() {
                "content-length" => {
                    content_length = value.parse().map_err(|_| {
                        Error::new(format!("content-length is not a number: {value}"))
                    })?
                }
                "x-rebot-token" => token = Some(value.to_string()),
                "authorization" => {
                    if let Some(rest) = value.strip_prefix("Bearer ") {
                        token = Some(rest.trim().to_string());
                    }
                }
                _ => {}
            }
        }
    }

    if content_length > limit {
        bail!("body too large: {content_length} bytes (limit {limit})");
    }
    let mut body = vec![0u8; content_length];
    if content_length > 0 {
        reader.read_exact(&mut body)?;
    }

    Ok(Request {
        method,
        path,
        query,
        token,
        body,
    })
}

fn parse_query(text: &str) -> BTreeMap<String, String> {
    let mut query = BTreeMap::new();
    for pair in text.split('&') {
        if let Some((key, value)) = pair.split_once('=') {
            query.insert(percent_decode(key), percent_decode(value));
        }
    }
    query
}

fn hex_digit(byte: u8) -> Option<u8> {
    match byte {
        b'0'..=b'9' => Some(byte - b'0'),
        b'a'..=b'f' => Some(byte - b'a' + 10),
        b'A'..=b'F' => Some(byte - b'A' + 10),
        _ => None,
    }
}

/// `%2C` and `+` are how a host encodes `,` and a space in a query string; the
/// gain specs (`50,2:150,3:150`) travel that way.
fn percent_decode(text: &str) -> String {
    let bytes = text.as_bytes();
    let mut out = Vec::with_capacity(bytes.len());
    let mut index = 0;
    while index < bytes.len() {
        match bytes[index] {
            b'%' if index + 2 < bytes.len() => {
                match (hex_digit(bytes[index + 1]), hex_digit(bytes[index + 2])) {
                    (Some(high), Some(low)) => {
                        out.push(high * 16 + low);
                        index += 3;
                    }
                    _ => {
                        out.push(bytes[index]);
                        index += 1;
                    }
                }
            }
            b'+' => {
                out.push(b' ');
                index += 1;
            }
            byte => {
                out.push(byte);
                index += 1;
            }
        }
    }
    String::from_utf8_lossy(&out).into_owned()
}

fn authorised(state: &ServerState, request: &Request) -> bool {
    if state.args.token.is_empty() {
        return true;
    }
    match &request.token {
        Some(token) => constant_time_eq(token.as_bytes(), state.args.token.as_bytes()),
        None => false,
    }
}

fn constant_time_eq(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    let mut difference = 0u8;
    for (left, right) in a.iter().zip(b.iter()) {
        difference |= left ^ right;
    }
    difference == 0
}

fn route(state: &ServerState, request: &Request) -> (u16, serde_json::Value) {
    if request.path == "/health" {
        // Health is open (no token), but only on GET: POST /health is a wrong
        // method, not a free 200. Dispatch the method here so the 405 arm below
        // is not dead code.
        if request.method == "GET" {
            return (200, health(state));
        }
        return (405, serde_json::json!({ "ok": false, "error": "wrong method for this path" }));
    }
    if !authorised(state, request) {
        return (401, serde_json::json!({
            "ok": false,
            "error": "missing or wrong X-Rebot-Token",
        }));
    }

    match (request.method.as_str(), request.path.as_str()) {
        ("GET", "/pose") => {
            let ids = request
                .query
                .get("ids")
                .map(String::as_str)
                .unwrap_or(&state.args.ids_spec);
            match pose_payload(&state.args.channel, ids) {
                Ok(payload) => (200, payload),
                Err(error) => (503, serde_json::json!({ "ok": false, "error": error.0 })),
            }
        }
        ("POST", "/play") => handle_play(state, request),
        ("POST", "/stop") => stop(state),
        ("GET", "/play") | ("POST", "/pose") | ("GET", "/stop") => {
            (405, serde_json::json!({ "ok": false, "error": "wrong method for this path" }))
        }
        _ => (404, serde_json::json!({ "ok": false, "error": "no such path" })),
    }
}

fn health(state: &ServerState) -> serde_json::Value {
    serde_json::json!({
        "ok": true,
        "version": env!("CARGO_PKG_VERSION"),
        "channel": state.args.channel,
        "can_up": can_is_up(&state.args.channel),
        "allow_execute": state.args.allow_execute,
        "gripper_stall_s": state.args.gripper_stall,
        "playing": state.playing.load(Ordering::SeqCst),
        "uptime_s": state.started.elapsed().as_secs(),
    })
}

fn handle_play(state: &ServerState, request: &Request) -> (u16, serde_json::Value) {
    let dry = request
        .query
        .get("dry")
        .map(|value| value == "1" || value == "true")
        .unwrap_or(false);

    if !dry && !state.args.allow_execute {
        return (403, serde_json::json!({
            "ok": false,
            "error": "execution is disabled: start the server with --allow-execute",
        }));
    }

    let text = match std::str::from_utf8(&request.body) {
        Ok(text) => text,
        Err(_) => return (400, serde_json::json!({
            "ok": false,
            "error": "the body must be UTF-8 JSON",
        })),
    };
    if text.trim().is_empty() {
        return (400, serde_json::json!({
            "ok": false,
            "error": "the body must be the trajectory JSON",
        }));
    }
    if let Err(error) = parse_trajectory(Path::new("<http>"), text) {
        return (400, serde_json::json!({ "ok": false, "error": error.0 }));
    }

    if state
        .playing
        .compare_exchange(false, true, Ordering::SeqCst, Ordering::SeqCst)
        .is_err()
    {
        return (409, serde_json::json!({
            "ok": false,
            "error": "a trajectory is already playing; POST /stop first",
        }));
    }
    state.stopping.store(false, Ordering::SeqCst);

    let result = run_child(state, text, dry, request);
    state.playing.store(false, Ordering::SeqCst);
    result
}

fn run_child(
    state: &ServerState,
    text: &str,
    dry: bool,
    request: &Request,
) -> (u16, serde_json::Value) {
    let path = std::env::temp_dir().join(format!(
        "rebot-play-{}-{}.json",
        std::process::id(),
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|elapsed| elapsed.subsec_nanos())
            .unwrap_or(0)
    ));
    if let Err(error) = fs::write(&path, text) {
        return (500, serde_json::json!({ "ok": false, "error": format!("cannot write {}: {error}", path.display()) }));
    }

    // The structured log becomes the response's `telemetry` (§5.1). The child
    // writes it after playback, off the realtime loop; the parent reads, thins
    // and forwards it. --check never plays, so it gets no log.
    let log_path = std::env::temp_dir().join(format!(
        "rebot-play-log-{}-{}.json",
        std::process::id(),
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|elapsed| elapsed.subsec_nanos())
            .unwrap_or(0)
    ));

    let executable = match std::env::current_exe() {
        Ok(executable) => executable,
        Err(error) => {
            let _ = fs::remove_file(&path);
            return (500, serde_json::json!({ "ok": false, "error": format!("cannot find my own binary: {error}") }));
        }
    };

    let mut command = Process::new(executable);
    command.arg(&path);
    command.arg(if dry { "--check" } else { "--execute" });
    command.arg(format!("--channel={}", state.args.channel));
    command.arg(format!("--model={}", state.args.model));
    command.arg(format!("--model-spec={}", state.args.model_spec));
    command.arg(format!(
        "--kp={}",
        request.query.get("kp").unwrap_or(&state.args.kp_spec)
    ));
    command.arg(format!(
        "--kd={}",
        request.query.get("kd").unwrap_or(&state.args.kd_spec)
    ));
    command.arg(format!("--ramp={}", state.args.ramp));
    if state.args.gripper_stall > 0.0 {
        command.arg(format!("--gripper-stall={}", state.args.gripper_stall));
    }
    if state.args.stay_enabled {
        command.arg("--stay-enabled");
    }
    if state.args.track {
        command.arg("--track");
    }
    if !dry {
        command.arg(format!("--log={}", log_path.display()));
    }
    command.stdout(Stdio::piped()).stderr(Stdio::piped());

    let child = match command.spawn() {
        Ok(child) => child,
        Err(error) => {
            let _ = fs::remove_file(&path);
            return (500, serde_json::json!({ "ok": false, "error": format!("cannot start the player: {error}") }));
        }
    };
    // Open the pidfd before publishing the pid, under the same lock stop()
    // takes, so a published child always has its pidfd set (or -1 if the
    // kernel lacks pidfd). pidfd_send_signal then cannot hit a reused PID.
    #[cfg(target_os = "linux")]
    let pidfd = unsafe { libc::syscall(libc::SYS_pidfd_open, child.id() as i32, 0u32) } as i32;
    {
        let _guard = state.pidfd_lock.lock().unwrap();
        #[cfg(target_os = "linux")]
        state.child_pidfd.store(pidfd, Ordering::SeqCst);
        state.child.store(child.id() as i32, Ordering::SeqCst);
    }

    let output = child.wait_with_output();
    {
        // Close the pidfd and clear the pid atomically wrt stop(): under this
        // lock, pidfd == -1 implies child == 0, so stop() never falls back to
        // kill() on a stale pid once the child has been reaped.
        let _guard = state.pidfd_lock.lock().unwrap();
        #[cfg(target_os = "linux")]
        {
            let fd = state.child_pidfd.swap(-1, Ordering::SeqCst);
            if fd >= 0 {
                unsafe { libc::close(fd); }
            }
        }
        state.child.store(0, Ordering::SeqCst);
    }
    let _ = fs::remove_file(&path);
    // Read the log even when the player aborted: the abort/stall fields are
    // exactly what the caller needs then. A missing or torn log costs the
    // telemetry, never the playback result.
    let telemetry = if dry { None } else { read_play_log(&log_path) };
    if !dry {
        let _ = fs::remove_file(&log_path);
    }

    let output = match output {
        Ok(output) => output,
        Err(error) => {
            return (500, serde_json::json!({ "ok": false, "error": format!("the player did not finish: {error}") }))
        }
    };
    let stdout = String::from_utf8_lossy(&output.stdout).to_string();
    let stderr = String::from_utf8_lossy(&output.stderr).to_string();

    let (status, mut payload) = if state.stopping.load(Ordering::SeqCst) {
        state.stopping.store(false, Ordering::SeqCst);
        (200, serde_json::json!({
            "ok": true,
            "stopped": true,
            "output": stdout,
            "note": "SIGTERM sent: the player ramps home and drops torque",
        }))
    } else if output.status.success() {
        (200, serde_json::json!({
            "ok": true,
            "executed": !dry,
            "output": stdout,
        }))
    } else {
        let code = output.status.code().unwrap_or(-1);
        // The child's output is piped, so without this the journal has no
        // record of the fault at all. Print it: the abort line, the return
        // ramp and any cleanup warning are the bench's evidence trail.
        println!("player failed (exit {code}); output follows:\n{stdout}{stderr}");
        (500, serde_json::json!({
            "ok": false,
            "exit_code": code,
            "error": if stderr.trim().is_empty() { format!("the player exited with {code}") } else { stderr.trim().to_string() },
            "output": stdout,
        }))
    };
    if let Some(telemetry) = telemetry {
        payload["telemetry"] = telemetry;
    }
    (status, payload)
}

/// Parse the child's `rebot-trajectory-log` and thin its samples for the wire.
/// None on any read/parse failure: telemetry is evidence, not a gate.
fn read_play_log(path: &Path) -> Option<serde_json::Value> {
    let text = fs::read_to_string(path).ok()?;
    let mut payload: serde_json::Value = serde_json::from_str(&text).ok()?;
    if let Some(samples) = payload.get("samples").and_then(|value| value.as_array()) {
        let thinned = downsample_samples(samples, TELEMETRY_MAX_SAMPLES);
        payload["samples"] = serde_json::Value::Array(thinned);
    }
    Some(payload)
}

/// The wire budget for /play telemetry. Hold-window samples are exempt: they
/// are the measurement points the host's oracle averages (§5.1).
const TELEMETRY_MAX_SAMPLES: usize = 2000;

/// Keep every sample inside a hold window (arm command constant across
/// consecutive samples) and thin the rest uniformly to `limit`. First and last
/// survive for context. If the holds alone exceed the budget they still all
/// survive: the budget yields to the evidence. Mirrors oracle.downsample on
/// the host.
fn downsample_samples(samples: &[serde_json::Value], limit: usize) -> Vec<serde_json::Value> {
    let count = samples.len();
    if count <= limit {
        return samples.to_vec();
    }
    let mut keep = vec![false; count];
    let mut start = 0;
    for index in 1..=count {
        let same = index < count
            && same_arm_cmd(samples[index].get("cmd"), samples[index - 1].get("cmd"));
        if same {
            continue;
        }
        if index - start >= 2 {
            for slot in start..index {
                keep[slot] = true;
            }
        }
        start = index;
    }
    keep[0] = true;
    keep[count - 1] = true;
    let kept = keep.iter().filter(|flag| **flag).count();
    if kept < limit {
        let free: Vec<usize> = (0..count).filter(|index| !keep[*index]).collect();
        let budget = limit - kept;
        for step in 0..budget {
            keep[free[step * free.len() / budget]] = true;
        }
    }
    samples
        .iter()
        .enumerate()
        .filter(|(index, _)| keep[*index])
        .map(|(_, sample)| sample.clone())
        .collect()
}

/// A hold is constant in joints 1-6: the gripper (motor 7) actuates right
/// through a marker hold -- the grasp cycle, not motion -- so comparing all
/// seven would find no holds at all in a pick sequence. Short commands
/// (arm-only logs) compare in full. Mirrors oracle._same_cmd on the host.
fn same_arm_cmd(a: Option<&serde_json::Value>, b: Option<&serde_json::Value>) -> bool {
    let (a, b) = match (a.and_then(|v| v.as_array()), b.and_then(|v| v.as_array())) {
        (Some(a), Some(b)) => (a, b),
        _ => return false,
    };
    if a.len() != b.len() || a.is_empty() {
        return false;
    }
    a.iter().zip(b.iter()).take(6).all(|(x, y)| x == y)
}

fn stop(state: &ServerState) -> (u16, serde_json::Value) {
    if !state.args.allow_execute {
        return (403, serde_json::json!({
            "ok": false,
            "error": "execution is disabled: start the server with --allow-execute",
        }));
    }
    // Hold the pidfd lock across the whole decision so the child cannot be
    // reaped (pidfd closed, pid cleared) between reading the pid and signaling.
    let _guard = state.pidfd_lock.lock().unwrap();
    let pid = state.child.load(Ordering::SeqCst);
    if pid <= 0 {
        return (200, serde_json::json!({
            "ok": true,
            "stopping": false,
            "note": "nothing is playing",
        }));
    }
    state.stopping.store(true, Ordering::SeqCst);
    #[cfg(target_os = "linux")]
    {
        let fd = state.child_pidfd.load(Ordering::SeqCst);
        if fd >= 0 {
            let rc = unsafe {
                libc::syscall(libc::SYS_pidfd_send_signal, fd, libc::SIGTERM,
                              std::ptr::null_mut::<libc::siginfo_t>(), 0u32)
            };
            if rc == 0 {
                return (200, serde_json::json!({ "ok": true, "stopping": true, "pid": pid }));
            }
            let error = io::Error::last_os_error();
            if error.raw_os_error() == Some(libc::ESRCH) {
                // The pidfd pins identity: ESRCH means OUR child already exited,
                // not that the pid was reused. There is nothing to signal.
                state.stopping.store(false, Ordering::SeqCst);
                return (200, serde_json::json!({
                    "ok": true, "stopping": false, "note": "the player already exited",
                }));
            }
            // Any other error falls through to the pid fallback below.
        }
    }
    // Fallback (non-Linux, or a kernel without pidfd): kill(pid) carries the
    // legacy PID-reuse window; the pidfd path above is the race-free one.
    let signal = unsafe { libc::kill(pid, libc::SIGTERM) };
    if signal != 0 {
        let error = io::Error::last_os_error();
        state.stopping.store(false, Ordering::SeqCst);
        return (500, serde_json::json!({
            "ok": false,
            "error": format!("cannot signal {pid}: {error}"),
        }));
    }
    (200, serde_json::json!({ "ok": true, "stopping": true, "pid": pid }))
}

// --------------------------------------------------------------------------- //
// Entry point
// --------------------------------------------------------------------------- //

fn main() {
    let code = match run() {
        Ok(code) => code,
        Err(error) => {
            eprintln!("error: {error}");
            1
        }
    };
    std::process::exit(code);
}

fn run() -> Result<i32> {
    let command = parse_args(std::env::args().skip(1).collect())?;
    let args = match command {
        Command::Help => {
            print!("{USAGE}");
            return Ok(0);
        }
        Command::Version => {
            println!("rebot-edge {}", env!("CARGO_PKG_VERSION"));
            return Ok(0);
        }
        Command::Dump(args) => return dump_joints(&args.channel, &args.ids_spec),
        Command::Serve(args) => return serve(*args),
        Command::Run(args) => *args,
    };

    let trajectory = load_trajectory(&args.trajectory)?;
    let models = parse_model_spec(&args.model_spec, &args.model, &trajectory.ids)?;

    let (kp_base, kp_overrides) = parse_gain_spec(&args.kp_spec, 50.0, MAX_KP, "kp")?;
    let (kd_base, kd_overrides) = parse_gain_spec(&args.kd_spec, 3.0, MAX_KD, "kd")?;
    let kp: Vec<f64> = trajectory
        .ids
        .iter()
        .map(|id| *kp_overrides.get(id).unwrap_or(&kp_base))
        .collect();
    let kd: Vec<f64> = trajectory
        .ids
        .iter()
        .map(|id| *kd_overrides.get(id).unwrap_or(&kd_base))
        .collect();

    let bound = trajectory
        .raw
        .sag_compensation
        .as_ref()
        .or(trajectory.raw.trim.as_ref());
    let mismatched = find_kp_mismatch(&trajectory.ids, &kp, bound);

    banner(&trajectory, &args, &models, &kp, &mismatched)?;

    if !mismatched.is_empty() {
        let detail = mismatched
            .iter()
            .map(|id| {
                let index = trajectory
                    .ids
                    .iter()
                    .position(|candidate| candidate == id)
                    .unwrap();
                let bound_kp = bound_kp_map(bound.unwrap());
                format!(
                    "joint {id}: file {:.1}, playing {:.1}",
                    bound_kp[id], kp[index]
                )
            })
            .collect::<Vec<_>>()
            .join(", ");
        if !args.allow_kp_mismatch {
            bail!(
                "this trajectory was corrected for a different kp ({detail}).\n\
                 The offset is tau/kp, so replaying it at another gain either \
                 under- or overshoots; at a softer kp the arm climbs above the \
                 plan. Play it with the same --kp, or pass --allow-kp-mismatch if \
                 you want the difference."
            );
        }
        println!("  warning   : playing a sag-compensated file at a different kp ({detail})");
    }

    if args.check {
        println!("\nCheck complete - the file is valid; no CAN access and no frames sent.");
        return Ok(0);
    }

    if !can_is_up(&args.channel) {
        bail!(
            "{} is not UP. Bring it up first:\n  \
             sudo systemctl start can0-setup.service\n  \
             ip -details link show {}",
            args.channel,
            args.channel
        );
    }

    let bus = CanBus::open(&args.channel)?;
    let mut arm = Arm::new(
        bus,
        trajectory.ids.clone(),
        models,
        kp.clone(),
        kd.clone(),
        trajectory.hz,
    );
    let _ = read_drift(&mut arm, &trajectory)?;

    if !args.execute {
        println!("\nDry run complete - no frames sent. Re-run with --execute to move.");
        return Ok(0);
    }

    install_signal_handlers()?;

    let kp_map: BTreeMap<String, f64> = trajectory
        .ids
        .iter()
        .zip(&kp)
        .map(|(id, gain)| (id.to_string(), *gain))
        .collect();
    let kd_map: BTreeMap<String, f64> = trajectory
        .ids
        .iter()
        .zip(&kd)
        .map(|(id, gain)| (id.to_string(), *gain))
        .collect();

    let mut logged: Vec<LogSample> = Vec::new();
    let mut notes = LogNotes::default();
    let outcome = play(&mut arm, &trajectory, &args, &mut logged, &mut notes);

    if let Some(path) = &args.log {
        if log_has_evidence(&logged, &notes) {
            if let Err(error) = write_log(path, &trajectory, &args, &kp_map, &kd_map, &logged,
                                          &notes) {
                println!("  warning: cannot write the log: {error}");
            }
        }
    }

    let shutdown = arm.shut_down(args.stay_enabled, args.track);
    if let Err(error) = &shutdown {
        println!("  warning: cleanup did not finish: {error}");
    }
    println!("Disconnected.");

    // A failed cleanup owns the exit status when playback itself succeeded:
    // systemd/scripting must not see success with torque possibly still on.
    outcome.and_then(|code| shutdown.map(|_| code))
}

// --------------------------------------------------------------------------- //
// Tests
// --------------------------------------------------------------------------- //

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;

    fn model(name: &str) -> Model {
        model_by_name(name).unwrap()
    }

    #[test]
    fn mit_frames_match_python_reference() {
        // Golden vectors for the RobStride MIT-mode packing formulas.
        let cases: [(&str, u8, f64, f64, f64, f64, f64, u32, [u8; 8]); 5] = [
            (
                "rs-00",
                4,
                0.25,
                50.0,
                3.0,
                0.0,
                2.5,
                0x0192_D104,
                [0x82, 0x8a, 0x7f, 0xff, 0x19, 0x99, 0x99, 0x99],
            ),
            (
                "rs-06",
                2,
                -0.5,
                150.0,
                10.0,
                0.0,
                -3.0,
                0x0179_9802,
                [0x7a, 0xe7, 0x7f, 0xff, 0x07, 0xae, 0x19, 0x99],
            ),
            (
                "rs-00",
                7,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0x017F_FF07,
                [0x7f, 0xff, 0x7f, 0xff, 0x00, 0x00, 0x00, 0x00],
            ),
            (
                "rs-00",
                1,
                20.0,
                1e9,
                1e9,
                0.0,
                1e9,
                0x01FF_FE01,
                [0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff],
            ),
            (
                "rs-06",
                3,
                -13.0,
                -5.0,
                -5.0,
                0.0,
                -100.0,
                0x0100_0003,
                [0x00, 0x00, 0x7f, 0xff, 0x00, 0x00, 0x00, 0x00],
            ),
        ];

        for (name, id, position, kp, kd, velocity, torque, expected_id, expected_data) in cases {
            let (frame_id, data) = mit_frame(model(name), id, position, kp, kd, velocity, torque);
            assert_eq!(frame_id, expected_id, "id for {name} {id}");
            assert_eq!(data, expected_data, "data for {name} {id}");
        }
    }

    #[test]
    fn ext_id_layout_matches_the_sdk() {
        assert_eq!(ext_id(COMM_OPERATION_CONTROL, 0x7FFF, 7), 0x017F_FF07);
        assert_eq!(ext_id(COMM_ENABLE, 0xFF, 4), 0x0300_FF04);
    }

    #[test]
    fn status_decoding_matches_the_python_player() {
        let data = [0x40, 0x00, 0x3f, 0xff, 0xff, 0x00, 0x00, 0xfa];
        let state = decode_status(model("rs-00"), &data).unwrap();
        assert!((state.position - -6.282_993_553_729_249).abs() < 1e-12);
        assert!((state.velocity - -25.000_762_962_736_9).abs() < 1e-12);
        assert!((state.torque - 16.868_221_076_082_644).abs() < 1e-12);
        assert!((state.temperature - 25.0).abs() < 1e-12);
    }

    #[test]
    fn gain_spec_parses_base_and_overrides() {
        let (base, overrides) = parse_gain_spec("50,2:150,3:150", 50.0, 150.0, "kp").unwrap();
        assert_eq!(base, 50.0);
        assert_eq!(overrides[&2], 150.0);
        assert_eq!(overrides[&3], 150.0);

        let (base, overrides) = parse_gain_spec("20", 50.0, 150.0, "kp").unwrap();
        assert_eq!(base, 20.0);
        assert!(overrides.is_empty());

        let (base, _) = parse_gain_spec("", 50.0, 150.0, "kp").unwrap();
        assert_eq!(base, 50.0);

        // Above the ceiling, the Rust player clamps instead of trusting the caller.
        let (base, _) = parse_gain_spec("1e9", 50.0, 150.0, "kp").unwrap();
        assert_eq!(base, 150.0);
    }

    #[test]
    fn model_spec_defaults_and_overrides() {
        let ids = [1u8, 2, 3, 4];
        let models = parse_model_spec(DEFAULT_MODEL_SPEC, "rs-00", &ids).unwrap();
        assert_eq!(models[0].name, "rs-06");
        assert_eq!(models[2].name, "rs-06");
        assert_eq!(models[3].name, "rs-00");

        let models = parse_model_spec("", "rs-00", &ids).unwrap();
        assert!(models.iter().all(|model| model.name == "rs-00"));

        assert!(parse_model_spec("1:rs-99", "rs-00", &ids).is_err());
        assert!(parse_model_spec("9:rs-06", "rs-00", &ids).is_err());
    }

    #[test]
    fn fixture_trajectory_loads() {
        let text = include_str!("../testdata/mini_trajectory.json");
        let trajectory = parse_trajectory(Path::new("mini_trajectory.json"), text).unwrap();
        assert_eq!(trajectory.ids, vec![1, 2]);
        assert_eq!(trajectory.hz, 10);
        assert!(trajectory.has_tau);
        assert!((trajectory.peak_tau - 0.5).abs() < 1e-12);
        assert!(trajectory.has_v);
        assert!((trajectory.peak_v - 0.5).abs() < 1e-12);
        assert_eq!(trajectory.raw.waypoints.len(), 4);
        assert!((trajectory.start[0] - 0.0).abs() < 1e-12);
        assert!((trajectory.last_t - 0.3).abs() < 1e-12);
    }

    fn parse_inline(json: &str) -> Result<Trajectory> {
        parse_trajectory(Path::new("inline.json"), json)
    }

    #[test]
    fn control_hz_accepts_high_rate_trajectories() {
        let json = r#"{
            "format": "rebot-joint-trajectory", "ids": [1], "control_hz": 100,
            "waypoints": [
                {"t": 0.0, "q": [0.0]},
                {"t": 0.01, "q": [0.001]},
                {"t": 0.02, "q": [0.002]}
            ]
        }"#;
        assert_eq!(parse_inline(json).unwrap().hz, 100);

        // Above the ceiling the file is clamped, not rejected.
        let fast = json.replace("\"control_hz\": 100", "\"control_hz\": 300");
        assert_eq!(parse_inline(&fast).unwrap().hz, MAX_HZ as u32);
    }

    #[test]
    fn rejects_wrong_format() {
        let error = parse_inline(r#"{"format":"something-else","ids":[1],"waypoints":[]}"#)
            .unwrap_err()
            .to_string();
        assert!(
            error.contains("not a rebot-joint-trajectory file"),
            "{error}"
        );
    }

    #[test]
    fn rejects_non_monotonic_times() {
        let json = r#"{
            "format": "rebot-joint-trajectory", "ids": [1], "control_hz": 30,
            "waypoints": [
                {"t": 0.1, "q": [0.0]},
                {"t": 0.0, "q": [0.01]}
            ]
        }"#;
        assert!(parse_inline(json)
            .unwrap_err()
            .to_string()
            .contains("not monotonic"));
    }

    #[test]
    fn rejects_short_q_vectors() {
        let json = r#"{
            "format": "rebot-joint-trajectory", "ids": [1, 2], "control_hz": 30,
            "waypoints": [
                {"t": 0.0, "q": [0.0]},
                {"t": 0.1, "q": [0.0, 0.0]}
            ]
        }"#;
        assert!(parse_inline(json)
            .unwrap_err()
            .to_string()
            .contains("1 values for 2 motors"));
    }

    #[test]
    fn rejects_tau_present_on_only_some_waypoints() {
        let json = r#"{
            "format": "rebot-joint-trajectory", "ids": [1], "control_hz": 30,
            "waypoints": [
                {"t": 0.0, "q": [0.0], "tau": [0.0]},
                {"t": 0.1, "q": [0.01]}
            ]
        }"#;
        assert!(parse_inline(json)
            .unwrap_err()
            .to_string()
            .contains("some waypoints"));
    }

    #[test]
    fn rejects_a_limit_violation() {
        let json = r#"{
            "format": "rebot-joint-trajectory", "ids": [1], "control_hz": 30,
            "limits": [[-1.0, 1.0]],
            "waypoints": [
                {"t": 0.0, "q": [0.0]},
                {"t": 0.1, "q": [1.5]}
            ]
        }"#;
        assert!(parse_inline(json)
            .unwrap_err()
            .to_string()
            .contains("leaves its limit"));
    }

    #[test]
    fn rejects_a_step_above_the_sanity_limit() {
        let json = r#"{
            "format": "rebot-joint-trajectory", "ids": [1], "control_hz": 30,
            "waypoints": [
                {"t": 0.0, "q": [0.0]},
                {"t": 0.1, "q": [0.6]}
            ]
        }"#;
        assert!(parse_inline(json)
            .unwrap_err()
            .to_string()
            .contains("sanity limit"));
    }

    #[test]
    fn rejects_angles_outside_the_mit_range() {
        let json = r#"{
            "format": "rebot-joint-trajectory", "ids": [1], "control_hz": 30,
            "waypoints": [
                {"t": 0.0, "q": [12.7]},
                {"t": 0.1, "q": [12.8]}
            ]
        }"#;
        assert!(parse_inline(json)
            .unwrap_err()
            .to_string()
            .contains("outside the MIT frame"));
    }

    #[test]
    fn cli_parses_the_python_flags() {
        let command = parse_args(
            [
                "traj.json",
                "--channel",
                "can1",
                "--kp=50,2:150",
                "--speed",
                "0.5",
                "--execute",
                "--fast-frames",
                "--log",
                "run.json",
            ]
            .iter()
            .map(|value| value.to_string())
            .collect(),
        )
        .unwrap();
        let Command::Run(args) = command else {
            panic!("expected Run");
        };
        assert_eq!(args.channel, "can1");
        assert_eq!(args.kp_spec, "50,2:150");
        assert!((args.speed - 0.5).abs() < 1e-12);
        assert!(args.execute);
        assert_eq!(args.log.as_deref(), Some(Path::new("run.json")));
    }

    fn wire_sample(t: f64, cmd: f64) -> serde_json::Value {
        serde_json::json!({"t": t, "cmd": [cmd], "meas": [cmd - 0.01]})
    }

    #[test]
    fn downsample_keeps_every_hold_window_sample() {
        // 5000 samples, two holds: [1000, 1100) and [3000, 3050).
        let mut samples = Vec::new();
        for index in 0..5000 {
            let cmd = if (1000..1100).contains(&index) {
                0.5
            } else if (3000..3050).contains(&index) {
                -0.25
            } else {
                index as f64 * 0.001
            };
            samples.push(wire_sample(index as f64 * 0.02, cmd));
        }
        let thinned = downsample_samples(&samples, 2000);
        assert!(thinned.len() <= 2000, "{} samples", thinned.len());
        let times: Vec<f64> = thinned
            .iter()
            .map(|sample| sample["t"].as_f64().unwrap())
            .collect();
        for index in (1000..1100).chain(3000..3050) {
            let t = index as f64 * 0.02;
            assert!(times.contains(&t), "hold sample {index} was dropped");
        }
        assert_eq!(times.first(), Some(&0.0));
        assert_eq!(times.last(), Some(&(4999.0 * 0.02)));
        assert!(times.windows(2).all(|pair| pair[0] < pair[1]), "order");
    }

    #[test]
    fn downsample_leaves_a_small_log_alone() {
        let samples: Vec<_> = (0..10).map(|i| wire_sample(i as f64, 0.0)).collect();
        assert_eq!(downsample_samples(&samples, 2000), samples);
    }

    #[test]
    fn downsample_holds_win_over_the_budget() {
        // One hold longer than the whole budget: every sample still survives.
        let samples: Vec<_> = (0..3000).map(|i| wire_sample(i as f64, 0.7)).collect();
        let thinned = downsample_samples(&samples, 2000);
        assert_eq!(thinned.len(), 3000);
    }

    #[test]
    fn downsample_keeps_holds_through_a_gripper_ramp() {
        // A marker hold: joints 1-6 constant while motor 7 (the gripper) ramps
        // through the grasp cycle. The hold samples must survive the budget.
        let mut samples = Vec::new();
        for index in 0..5000 {
            let mut cmd = vec![serde_json::json!(index as f64 * 0.001); 7];
            if (1000..1100).contains(&index) {
                for slot in cmd.iter_mut().take(6) {
                    *slot = serde_json::json!(0.5);
                }
                cmd[6] = serde_json::json!(2.0 + 0.01 * (index - 1000) as f64);
            }
            samples.push(serde_json::json!({
                "t": index as f64 * 0.02, "cmd": cmd, "meas": cmd,
            }));
        }
        let thinned = downsample_samples(&samples, 2000);
        assert!(thinned.len() <= 2000, "{} samples", thinned.len());
        let times: Vec<f64> = thinned
            .iter()
            .map(|sample| sample["t"].as_f64().unwrap())
            .collect();
        for index in 1000..1100 {
            let t = index as f64 * 0.02;
            assert!(times.contains(&t), "hold sample {index} was dropped");
        }
    }

    #[test]
    fn write_log_is_version_2_with_optional_fault_fields() {
        let trajectory = parse_inline(
            r#"{
                "format": "rebot-joint-trajectory", "ids": [1], "control_hz": 30,
                "waypoints": [
                    {"t": 0.0, "q": [0.0]},
                    {"t": 0.1, "q": [0.1]}
                ]
            }"#,
        )
        .unwrap();
        let args = Args::default();
        let samples = vec![LogSample { t: 0.0, cmd: vec![0.0], meas: vec![0.0] }];
        let notes = LogNotes {
            abort: Some(LogAbort { t: 0.1, reason: "motor 1 is not following".into() }),
            gripper_stall: Some(LogGripperStall { t: 0.1, commanded: 0.5, measured: 0.1 }),
            tick: Some(LogTick { mean_ms: 33.3, p95_ms: 35.0, max_ms: 40.0 }),
            follow_error_max: Some(LogFollowError { motor: 1, rad: 0.42 }),
        };
        let path = std::env::temp_dir().join(format!("rebot-log-test-{}.json", std::process::id()));
        write_log(&path, &trajectory, &args, &BTreeMap::new(), &BTreeMap::new(),
                  &samples, &notes)
            .unwrap();
        let payload: serde_json::Value =
            serde_json::from_str(&fs::read_to_string(&path).unwrap()).unwrap();
        let _ = fs::remove_file(&path);
        assert_eq!(payload["format"], "rebot-trajectory-log");
        assert_eq!(payload["version"], 2);
        assert_eq!(payload["abort"]["reason"], "motor 1 is not following");
        assert_eq!(payload["gripper_stall"]["commanded"], 0.5);
        assert_eq!(payload["tick"]["p95_ms"], 35.0);
        assert_eq!(payload["follow_error_max"]["motor"], 1);
        assert_eq!(payload["samples"].as_array().unwrap().len(), 1);
        // A clean run writes none of the optional fields: a v1 reader sees v1.
        let clean = LogNotes::default();
        let text = serde_json::to_string(&LogFile {
            format: "rebot-trajectory-log",
            version: 2,
            created: utc_timestamp(),
            trajectory: "inline.json".into(),
            ids: &trajectory.ids,
            kp: BTreeMap::new(),
            kd: BTreeMap::new(),
            control_hz: trajectory.hz,
            speed: 1.0,
            samples: &samples,
            notes: &clean,
        })
        .unwrap();
        assert!(!text.contains("abort"));
        assert!(!text.contains("gripper_stall"));
        assert!(!text.contains("tick"));
        assert!(!text.contains("follow_error_max"));
    }

    #[test]
    fn dump_subcommand_parses_channel_and_ids() {
        let command = parse_args(
            ["dump", "--channel", "can1", "--ids", "1,2,7"]
                .iter()
                .map(|value| value.to_string())
                .collect(),
        )
        .unwrap();
        let Command::Dump(args) = command else {
            panic!("expected Dump");
        };
        assert_eq!(args.channel, "can1");
        assert_eq!(args.ids_spec, "1,2,7");

        let default = parse_args(["dump"].iter().map(|value| value.to_string()).collect()).unwrap();
        let Command::Dump(args) = default else {
            panic!("expected Dump");
        };
        assert_eq!(args.channel, "can0");
        assert_eq!(args.ids_spec, "1,2,3,4,5,6,7");
    }

    #[test]
    fn dump_ids_are_validated() {
        assert_eq!(parse_ids("1,2,7").unwrap(), vec![1, 2, 7]);
        assert!(parse_ids("0").is_err());
        assert!(parse_ids("8").is_err());
        assert!(parse_ids("1,1").is_err());
        assert!(parse_ids("").is_err());
    }

    #[test]
    fn check_and_execute_conflict() {
        let error = parse_args(
            ["traj.json", "--check", "--execute"]
                .iter()
                .map(|value| value.to_string())
                .collect(),
        )
        .unwrap_err()
        .to_string();
        assert!(error.contains("mutually exclusive"));
    }

    #[test]
    fn missing_trajectory_is_an_error() {
        let error = parse_args(Vec::new()).unwrap_err().to_string();
        assert!(error.contains("missing trajectory JSON"));
    }

    #[test]
    fn utc_timestamp_has_the_expected_shape() {
        let stamp = utc_timestamp();
        assert_eq!(stamp.len(), 20, "{stamp}");
        assert!(stamp.ends_with('Z'));
        assert_eq!(&stamp[4..5], "-");
        assert_eq!(&stamp[10..11], "T");
    }

    #[test]
    fn civil_from_days_epoch() {
        assert_eq!(civil_from_days(0), (1970, 1, 1));
        assert_eq!(civil_from_days(19_000), (2022, 1, 8));
    }

    #[test]
    fn read_replies_match_both_id_layouts() {
        // Echoing firmware: target in the low byte.
        let echoed = Frame {
            id: (COMM_READ_PARAMETER << 24) | ((HOST_ID as u32) << 8) | 3,
            data: [0; 8],
            len: 8,
        };
        // Receive layout: host in the low byte, motor id in the next byte.
        let response = Frame {
            id: (COMM_READ_PARAMETER << 24) | (3 << 8) | (HOST_ID as u32),
            data: [0; 8],
            len: 8,
        };
        assert!(echoed.is_from(3));
        assert!(response.is_from(3));
        assert!(!response.is_from(4));
    }

    #[test]
    fn extended_frames_set_the_eff_flag() {
        // Real reply observed from motor 1 on the arm:
        //   0x910001FF, type 17, extra 0x0001, host 0xFF, value in data[4..8].
        let reply = Frame {
            id: CAN_EFF_FLAG | (COMM_READ_PARAMETER << 24) | (1 << 8) | (HOST_ID as u32),
            data: [0x19, 0x70, 0x00, 0x00, 0xe4, 0x44, 0x46, 0x3b],
            len: 8,
        };
        assert_eq!(reply.id, 0x9100_01FF);
        assert_eq!(reply.comm(), COMM_READ_PARAMETER);
        assert_eq!(reply.status_device(), 1);
        assert!(reply.is_from(1));

        // The flag must be present on the frame handed to the kernel.
        assert_eq!(CAN_EFF_FLAG | 0x1100_FF01, 0x9100_FF01);
    }

    #[test]
    fn telemetry_publishes_latest_states_and_faults() {
        let telemetry = Telemetry::new(vec![1], vec![model("rs-00")]);
        let frame = Frame {
            id: (COMM_OPERATION_STATUS << 24) | (1 << 8) | (HOST_ID as u32),
            data: [0x40, 0x00, 0x3f, 0xff, 0xff, 0x00, 0x00, 0xfa],
            len: 8,
        };
        telemetry.handle(frame);

        let states = telemetry.latest().unwrap();
        let state = states[0].unwrap();
        assert!((state.position - -6.282_993_553_729_249).abs() < 1e-12);
        assert!((state.temperature - 25.0).abs() < 1e-12);

        assert!(telemetry.fault().is_none());
        telemetry.set_fault("boom".into());
        assert_eq!(telemetry.fault().as_deref(), Some("boom"));
        // The shutdown ramp clears the latch; a fresh fault re-latches it.
        telemetry.clear_fault();
        assert!(telemetry.fault().is_none());
        telemetry.set_fault("again".into());
        assert_eq!(telemetry.fault().as_deref(), Some("again"));
    }

    #[test]
    fn active_report_frame_matches_the_manual() {
        let (id, data) = active_report_frame(1, true);
        assert_eq!(id, (COMM_ACTIVE_REPORT << 24) | ((HOST_ID as u32) << 8) | 1);
        assert_eq!(data, [1, 2, 3, 4, 5, 6, 1, 0]);

        let (_, disabled) = active_report_frame(1, false);
        assert_eq!(disabled, [1, 2, 3, 4, 5, 6, 0, 0]);
    }

    #[test]
    fn telemetry_accepts_type_24_reports() {
        let telemetry = Telemetry::new(vec![1], vec![model("rs-00")]);
        let frame = Frame {
            id: (COMM_ACTIVE_REPORT << 24) | (1 << 8) | (HOST_ID as u32),
            data: [0x80, 0x09, 0x7f, 0xff, 0x7f, 0xff, 0x01, 0x54],
            len: 8,
        };
        telemetry.handle(frame);

        let state = telemetry.latest().unwrap()[0].unwrap();
        assert!((state.torque - 0.0).abs() < 1e-9);
        assert!((state.temperature - 34.0).abs() < 1e-9);
    }

    // -- in-memory bus, so the control loop is tested without hardware -------- //

    #[derive(Debug, Clone, Copy)]
    struct MockFrame {
        device: u8,
        position: f64,
        torque: f64,
        velocity: f64,
    }

    struct MockBus {
        positions: RefCell<BTreeMap<u8, f64>>,
        enabled: RefCell<BTreeSet<u8>>,
        frames: RefCell<Vec<MockFrame>>,
        blocked: RefCell<Option<(u8, f64)>>,
        /// Latch a fault once this many operation frames have been sent,
        /// mimicking the reader thread's fault latch.
        fault_after: RefCell<Option<(u8, usize)>>,
        latched_fault: RefCell<Option<String>>,
        reader_mode: bool,
        /// A fault frame sitting in the receive queue, mimicking the
        /// synchronous bus path (no reader thread).
        queued_fault: RefCell<Option<String>>,
        /// Queue a fault frame right after this motor is enabled: a motor
        /// that reports a fault as it comes up, mid bring-up.
        fault_after_enable: RefCell<Option<(u8, String)>>,
        /// Refuse torque-disable frames: a bus that cannot drop torque.
        disable_fails: RefCell<bool>,
    }

    impl MockBus {
        fn new(positions: &[(u8, f64)]) -> Self {
            Self {
                positions: RefCell::new(positions.iter().copied().collect()),
                enabled: RefCell::new(BTreeSet::new()),
                frames: RefCell::new(Vec::new()),
                blocked: RefCell::new(None),
                fault_after: RefCell::new(None),
                latched_fault: RefCell::new(None),
                reader_mode: false,
                queued_fault: RefCell::new(None),
                fault_after_enable: RefCell::new(None),
                disable_fails: RefCell::new(false),
            }
        }

        fn blocked(self, id: u8, at: f64) -> Self {
            *self.blocked.borrow_mut() = Some((id, at));
            self
        }

        fn faults_after(mut self, id: u8, frames: usize) -> Self {
            *self.fault_after.borrow_mut() = Some((id, frames));
            self.reader_mode = true;
            self
        }

        fn queued_fault(self, message: &str) -> Self {
            *self.queued_fault.borrow_mut() = Some(message.to_string());
            self
        }

        fn faults_after_enable(self, id: u8, message: &str) -> Self {
            *self.fault_after_enable.borrow_mut() = Some((id, message.to_string()));
            self
        }

        fn with_disable_failure(self) -> Self {
            *self.disable_fails.borrow_mut() = true;
            self
        }

        fn state(&self, id: u8) -> State {
            State {
                position: *self.positions.borrow().get(&id).unwrap_or(&0.0),
                velocity: 0.0,
                torque: 0.0,
                temperature: 25.0,
            }
        }
    }

    impl Transport for MockBus {
        fn send(&self, id: u32, data: &[u8]) -> Result<()> {
            let communication_type = (id >> 24) & 0x1F;
            let device = (id & 0xFF) as u8;
            match communication_type {
                COMM_ENABLE => {
                    self.enabled.borrow_mut().insert(device);
                    let trigger = self.fault_after_enable.borrow().clone();
                    if let Some((fault_id, message)) = trigger {
                        if device == fault_id {
                            *self.queued_fault.borrow_mut() = Some(message);
                            *self.fault_after_enable.borrow_mut() = None;
                        }
                    }
                }
                COMM_DISABLE => {
                    if *self.disable_fails.borrow() {
                        bail!("the bus refused the disable frame");
                    }
                    self.enabled.borrow_mut().remove(&device);
                }
                COMM_OPERATION_CONTROL if data.len() == 8 => {
                    let position_u16 = u16::from_be_bytes([data[0], data[1]]);
                    let velocity_u16 = u16::from_be_bytes([data[2], data[3]]);
                    let torque_u16 = ((id >> 8) & 0xFFFF) as u16;
                    let mut position =
                        (f64::from(position_u16) / 0x7FFF as f64 - 1.0) * MIT_POSITION_LIMIT;
                    // Every test arm uses rs-00 models: +-50 rad/s velocity range.
                    let velocity = (f64::from(velocity_u16) / 0x7FFF as f64 - 1.0) * 50.0;
                    let torque = (f64::from(torque_u16) / 0x7FFF as f64 - 1.0) * 17.0;
                    if let Some((blocked_id, blocked_at)) = *self.blocked.borrow() {
                        if device == blocked_id && position < blocked_at {
                            position = blocked_at;
                        }
                    }
                    self.frames.borrow_mut().push(MockFrame {
                        device,
                        position,
                        torque,
                        velocity,
                    });
                    self.positions.borrow_mut().insert(device, position);
                    let fault_trigger = *self.fault_after.borrow();
                    if let Some((fault_id, at)) = fault_trigger {
                        if self.frames.borrow().len() >= at {
                            *self.latched_fault.borrow_mut() = Some(format!(
                                "motor id {fault_id} reported a fault frame: 1040000000000000"
                            ));
                            *self.fault_after.borrow_mut() = None;
                        }
                    }
                }
                _ => {}
            }
            Ok(())
        }

        fn drain(&self, _budget: usize) -> Result<()> {
            // The synchronous bus reads the queue here; a fault frame in it
            // is an error, exactly as the bench's motor-4 fault surfaced.
            if let Some(fault) = self.queued_fault.borrow().clone() {
                bail!("{fault}");
            }
            Ok(())
        }

        fn read_position(&self, id: u8, _timeout: Duration) -> Result<f64> {
            Ok(*self.positions.borrow().get(&id).unwrap_or(&0.0))
        }

        fn read_status(&self, _model: Model, id: u8, _timeout: Duration) -> Result<Option<State>> {
            Ok(self.enabled.borrow().contains(&id).then(|| self.state(id)))
        }

        fn poll_states(
            &self,
            ids: &[u8],
            _models: &[Model],
            _timeout: Duration,
        ) -> Result<Vec<Option<State>>> {
            if let Some(fault) = self.queued_fault.borrow().clone() {
                bail!("{fault}");
            }
            Ok(ids.iter().map(|id| Some(self.state(*id))).collect())
        }

        fn poll_available_tolerant(
            &self,
            ids: &[u8],
            _models: &[Model],
        ) -> Result<Vec<Option<State>>> {
            // The ramp skips the queued fault frame; consume it so the
            // following ticks see a clean queue.
            *self.queued_fault.borrow_mut() = None;
            Ok(ids.iter().map(|id| Some(self.state(*id))).collect())
        }

        fn start_reader(&mut self, _ids: &[u8], _models: &[Model]) -> Result<bool> {
            Ok(self.reader_mode)
        }

        fn reader_fault(&self) -> Option<String> {
            self.latched_fault.borrow().clone()
        }

        fn clear_reader_fault(&self) {
            *self.latched_fault.borrow_mut() = None;
        }
    }

    #[test]
    fn bring_up_reads_the_enable_status_position() {
        let mock = MockBus::new(&[(1, 0.25), (2, -0.5)]);
        let mut arm = Arm::new(
            mock,
            vec![1, 2],
            vec![model("rs-00"), model("rs-00")],
            vec![50.0, 50.0],
            vec![3.0, 3.0],
            30,
        );

        arm.bring_up().unwrap();

        assert!(arm.enabled);
        assert!((arm.start[0] - 0.25).abs() < 1e-12);
        assert!((arm.start[1] + 0.5).abs() < 1e-12);
        assert!((arm.last[0] - 0.25).abs() < 1e-12);
    }

    #[test]
    fn bring_up_fault_disables_every_motor() {
        // A fault frame arriving mid bring-up (the bench's motor-4 case) must
        // not leave the already-enabled joints holding torque: `enabled` is
        // still false when bring_up fails, so shut_down() would no-op.
        let mock = MockBus::new(&[(1, 0.1), (2, 0.2), (3, 0.3)])
            .faults_after_enable(2, "motor id 2 reported a fault frame: 1040000000000000");
        let mut arm = Arm::new(
            mock,
            vec![1, 2, 3],
            vec![model("rs-00"), model("rs-00"), model("rs-00")],
            vec![50.0, 50.0, 50.0],
            vec![3.0, 3.0, 3.0],
            30,
        );

        assert!(arm.bring_up().is_err(), "the fault frame must abort bring-up");
        assert!(!arm.enabled);
        assert!(
            arm.bus.enabled.borrow().is_empty(),
            "torque must be off on every motor, including the ones already enabled"
        );
    }

    #[test]
    fn shutdown_disable_failure_is_an_error_not_a_warning() {
        // A failed torque-disable must reach the caller: run() turns it into a
        // non-zero exit, so systemd/scripting cannot see success with torque
        // possibly still on.
        let mock = MockBus::new(&[(1, 0.1), (2, 0.2)]).with_disable_failure();
        let mut arm = Arm::new(
            mock,
            vec![1, 2],
            vec![model("rs-00"), model("rs-00")],
            vec![50.0, 50.0],
            vec![3.0, 3.0],
            30,
        );
        arm.bring_up().unwrap();

        let result = arm.shut_down(false, false);

        assert!(result.is_err(), "a failed torque-disable must not report success");
        assert!(!arm.enabled, "the enabled flag clears even when the bus refuses");
    }

    #[test]
    fn playback_streams_the_fixture_over_the_mock_bus() {
        let trajectory = parse_trajectory(
            Path::new("mini_trajectory.json"),
            include_str!("../testdata/mini_trajectory.json"),
        )
        .unwrap();
        let mock = MockBus::new(&[(1, 0.0), (2, 0.1)]);
        let mut arm = Arm::new(
            mock,
            trajectory.ids.clone(),
            vec![model("rs-00"), model("rs-00")],
            vec![50.0, 150.0],
            vec![3.0, 3.0],
            trajectory.hz,
        );

        let mut args = Args::default();
        args.execute = true;
        args.log = Some(PathBuf::from("unused.json")); // enables per-waypoint logging
        let mut logged = Vec::new();
        let mut notes = LogNotes::default();

        let code = play(&mut arm, &trajectory, &args, &mut logged, &mut notes).unwrap();

        assert_eq!(code, 0);
        assert_eq!(trajectory.hz, 10,
                   "fixture control_hz matches its 0.1 s waypoint spacing");
        assert_eq!(logged.len(), 4);
        assert!(notes.abort.is_none());
        assert!(notes.tick.is_some(), "tick spacing is recorded for the log");
        let frames = arm.bus.frames.borrow();
        let last_joint_1 = frames.iter().rev().find(|frame| frame.device == 1).unwrap();
        let last_joint_2 = frames.iter().rev().find(|frame| frame.device == 2).unwrap();
        assert!((last_joint_1.position - 0.15).abs() < 1e-3);
        assert!((last_joint_2.position - 0.25).abs() < 1e-3);
        assert!((last_joint_1.torque - 0.3).abs() < 1e-3);
        assert!((last_joint_2.torque - 0.2).abs() < 1e-3);
        // The fixture's velocity feedforward reaches the MIT frame's v field.
        assert!((last_joint_1.velocity - 0.5).abs() < 2e-3);
        assert!((last_joint_2.velocity - 0.5).abs() < 2e-3);
    }

    #[test]
    fn playback_without_velocity_streams_zero() {
        // An older file without 'v' still plays, with the kd term damping
        // toward zero exactly as before.
        let json = r#"{
            "format": "rebot-joint-trajectory", "ids": [1, 2], "control_hz": 30,
            "waypoints": [
                {"t": 0.0, "q": [0.0, 0.1]},
                {"t": 0.033333, "q": [0.02, 0.12]},
                {"t": 0.066667, "q": [0.04, 0.14]}
            ]
        }"#;
        let trajectory = parse_inline(json).unwrap();
        assert!(!trajectory.has_v);
        let mock = MockBus::new(&[(1, 0.0), (2, 0.1)]);
        let mut arm = Arm::new(
            mock,
            trajectory.ids.clone(),
            vec![model("rs-00"), model("rs-00")],
            vec![50.0, 50.0],
            vec![3.0, 3.0],
            trajectory.hz,
        );
        let mut args = Args::default();
        args.execute = true;
        let mut logged = Vec::new();
        let mut notes = LogNotes::default();

        let code = play(&mut arm, &trajectory, &args, &mut logged, &mut notes).unwrap();

        assert_eq!(code, 0);
        let frames = arm.bus.frames.borrow();
        assert!(!frames.is_empty());
        assert!(frames.iter().all(|frame| frame.velocity.abs() < 1e-9));
    }

    #[test]
    fn rejects_bad_velocity_feedforward() {
        let base = r#"{
            "format": "rebot-joint-trajectory", "ids": [1, 2], "control_hz": 30,
            "waypoints": [
                {"t": 0.0, "q": [0.0, 0.0], "v": [0.0, 0.0]},
                {"t": 0.1, "q": [0.01, 0.01], "v": [0.1, 0.1]},
                {"t": 0.2, "q": [0.02, 0.02], "v": [0.1, 0.1]}
            ]
        }"#;
        assert!(parse_inline(base).unwrap().has_v);

        let partial = base.replace(
            r#"{"t": 0.2, "q": [0.02, 0.02], "v": [0.1, 0.1]}"#,
            r#"{"t": 0.2, "q": [0.02, 0.02]}"#,
        );
        assert!(parse_inline(&partial)
            .unwrap_err()
            .to_string()
            .contains("'v' is present on some"));

        let short = base.replace(r#""v": [0.0, 0.0]}"#, r#""v": [0.0]}"#);
        assert!(parse_inline(&short)
            .unwrap_err()
            .to_string()
            .contains("1 v values for 2 motors"));

        // A velocity that outruns the step gate is corrupt: 0.5 rad per
        // waypoint at 30 Hz permits at most ~15 rad/s, not 20.
        let runaway = base.replace(r#""v": [0.1, 0.1]}"#, r#""v": [20.0, 0.1]}"#);
        assert!(parse_inline(&runaway)
            .unwrap_err()
            .to_string()
            .contains("the step gate permits"));

        // The boundary itself is legal: 0.5 rad per tick at 30 Hz, v = 15.
        let boundary = r#"{
            "format": "rebot-joint-trajectory", "ids": [1, 2], "control_hz": 30,
            "waypoints": [
                {"t": 0.0, "q": [0.0, 0.0], "v": [15.0, 0.0]},
                {"t": 0.033333, "q": [0.5, 0.0], "v": [15.0, 0.0]},
                {"t": 0.066667, "q": [1.0, 0.0], "v": [15.0, 0.0]}
            ]
        }"#;
        assert!(parse_inline(boundary).unwrap().has_v);
    }

    #[test]
    fn move_leg_commands_the_trapezoid_velocity() {
        // The internally generated legs (align, /move, return home) carry the
        // analytic trapezoid velocity: zero at the ends, linear ramps, then a
        // flat cruise at delta / (T - ramp).
        let mock = MockBus::new(&[(1, 0.0), (2, 0.0)]);
        let mut arm = Arm::new(
            mock,
            vec![1, 2],
            vec![model("rs-00"), model("rs-00")],
            vec![50.0, 50.0],
            vec![3.0, 3.0],
            100,
        );
        arm.bring_up().unwrap();

        arm.move_leg(&[0.2, 0.4], 1.0, "test", false, false, false).unwrap();

        let frames = arm.bus.frames.borrow();
        let joint2: Vec<&MockFrame> = frames.iter().filter(|frame| frame.device == 2).collect();
        assert_eq!(joint2.len(), 101, "one frame per tick, steps 0..=100");
        assert!(joint2[0].velocity.abs() < 2e-3, "the leg starts at rest");
        assert!(joint2[100].velocity.abs() < 2e-3, "the leg ends at rest");
        let cruise = 0.4 / (1.0 - MOVE_RAMP_S);
        assert!(
            (joint2[50].velocity - cruise).abs() < 2e-3,
            "cruise velocity {} should be {cruise}",
            joint2[50].velocity
        );
        // The cruise is flat, and the ramp at u=0.05 is a third of the cruise
        // (ramp ratio 0.15).
        assert!((joint2[40].velocity - joint2[60].velocity).abs() < 2e-3);
        assert!(
            (joint2[5].velocity - cruise / 3.0).abs() < 2e-3,
            "ramp sample {} should be {}",
            joint2[5].velocity,
            cruise / 3.0
        );
        assert!(joint2.iter().all(|frame| frame.velocity > -2e-3));
    }

    #[test]
    fn trapezoid_profile_is_continuous_and_consistent() {
        for duration in [0.2_f64, 0.6, 1.0, 24.7] {
            // Endpoints and monotonicity.
            assert!(ease_fraction(0.0, duration, MOVE_RAMP_S).abs() < 1e-12);
            assert!((ease_fraction(1.0, duration, MOVE_RAMP_S) - 1.0).abs() < 1e-12);
            let mut previous = 0.0;
            for step in 0..=1000 {
                let u = f64::from(step) / 1000.0;
                let value = ease_fraction(u, duration, MOVE_RAMP_S);
                assert!(value >= previous - 1e-12, "non-monotonic at u={u}");
                previous = value;
            }
            // The analytic velocity matches a finite difference of the fraction
            // (exact within each phase; the ramp/cruise acceleration corners are
            // discontinuous by design, so skip their neighbourhoods).
            let r = (MOVE_RAMP_S / duration).clamp(1e-9, 0.5);
            for step in 1..1000 {
                let u = f64::from(step) / 1000.0;
                if (u - r).abs() < 1e-3 || (u - (1.0 - r)).abs() < 1e-3 {
                    continue;
                }
                let h = 1e-4;
                let numeric = (ease_fraction(u + h, duration, MOVE_RAMP_S)
                    - ease_fraction(u - h, duration, MOVE_RAMP_S))
                    / (2.0 * h * duration);
                let analytic = ease_velocity(u, duration, MOVE_RAMP_S);
                assert!(
                    (numeric - analytic).abs() < 1e-3,
                    "duration {duration}, u {u}: numeric {numeric} vs analytic {analytic}"
                );
            }
            // Cruise speed never exceeds the cosine-calibrated policy peak for
            // legs of at least ~0.8 s (the sweep's shortest real leg is 3 s).
            if duration >= 0.8 {
                let cruise = ease_velocity(0.5, duration, MOVE_RAMP_S);
                assert!(cruise <= (std::f64::consts::PI / 2.0) / duration + 1e-9);
            }
        }
    }

    #[test]
    fn playback_aborts_and_holds_the_opening_on_a_blocked_gripper() {
        // Motor 7 is commanded from open (4.90) to half-close (2.4425), but the
        // mock jaw stops at 4.00 rad: a rigid body in the way. The guard must
        // fire, leave the measured opening in place for the return ramp, and
        // report a non-zero exit code instead of pressing to the target.
        let mut waypoints = Vec::new();
        for step in 0..31 {
            let t = f64::from(step) / 30.0;
            let q = 4.90 - (4.90 - 2.4425) * (f64::from(step) / 30.0);
            waypoints.push(format!("{{\"t\":{t:.4},\"q\":[0.0,{q:.5}]}}"));
        }
        let text = format!(
            "{{\"format\":\"{TRAJECTORY_FORMAT}\",\"ids\":[1,7],\"control_hz\":30,\
             \"waypoints\":[{}]}}",
            waypoints.join(",")
        );
        let trajectory = parse_trajectory(Path::new("stall.json"), &text).unwrap();
        let mock = MockBus::new(&[(1, 0.0), (7, 4.90)]).blocked(7, 4.00);
        let mut arm = Arm::new(
            mock,
            trajectory.ids.clone(),
            vec![model("rs-00"), model("rs-00")],
            vec![50.0, 50.0],
            vec![3.0, 4.0],
            trajectory.hz,
        );
        let mut args = Args::default();
        args.execute = true;
        args.gripper_stall = 0.3;
        let mut logged = Vec::new();
        let mut notes = LogNotes::default();

        let code = play(&mut arm, &trajectory, &args, &mut logged, &mut notes).unwrap();

        assert_eq!(code, 1, "the blocked gripper should abort playback");
        assert!((arm.start[1] - 4.00).abs() < 1e-3, "home should hold the blocked opening");
        assert!((arm.last[1] - 4.00).abs() < 1e-3, "the last gripper command should be the measured opening");
        assert!((arm.start[0] - 0.0).abs() < 1e-9);
        let stall = notes.gripper_stall.expect("the stall is recorded for the log");
        assert!((stall.measured - 4.00).abs() < 1e-3);
        assert!(stall.commanded < 4.00);
        let abort = notes.abort.expect("the abort is recorded for the log");
        assert!(abort.reason.contains("stalled"));
    }

    #[test]
    fn fault_abort_is_recorded_in_the_log() {
        // A fault frame propagates as Err out of the play loop, bypassing the
        // abort_reason machinery. The log must still carry the abort timestamp
        // and reason -- on the bench this was the evidence that got discarded.
        let mut waypoints = Vec::new();
        for step in 0..31 {
            let t = f64::from(step) / 30.0;
            let q = 0.1 + 0.4 * (f64::from(step) / 30.0);
            waypoints.push(format!("{{\"t\":{t:.4},\"q\":[0.0,{q:.5}]}}"));
        }
        let text = format!(
            "{{\"format\":\"{TRAJECTORY_FORMAT}\",\"ids\":[1,2],\"control_hz\":30,\
             \"waypoints\":[{}]}}",
            waypoints.join(",")
        );
        let trajectory = parse_trajectory(Path::new("fault.json"), &text).unwrap();
        let mock = MockBus::new(&[(1, 0.0), (2, 0.1)]).faults_after(2, 10);
        let mut arm = Arm::new(
            mock,
            trajectory.ids.clone(),
            vec![model("rs-00"), model("rs-00")],
            vec![50.0, 50.0],
            vec![3.0, 3.0],
            trajectory.hz,
        );
        let mut args = Args::default();
        args.execute = true;
        args.active_report = true;
        // serve always sets a log path for /play; sample logging is gated on it.
        args.log = Some(PathBuf::from("fault-log.json"));
        let mut logged = Vec::new();
        let mut notes = LogNotes::default();

        let outcome = play(&mut arm, &trajectory, &args, &mut logged, &mut notes);

        let error = outcome.expect_err("the fault frame must abort playback");
        assert!(error.to_string().contains("fault frame"), "{error}");
        let abort = notes.abort.expect("the fault abort is recorded for the log");
        assert!(abort.t > 0.0, "the abort carries the playback timestamp");
        assert!(abort.reason.contains("fault frame"));
        assert!(!logged.is_empty(), "the samples up to the fault survive");
    }

    #[test]
    fn fault_before_the_first_sample_still_leaves_evidence() {
        // The bench's second fault: motor 4's fault frame was already queued at
        // bring-up, so the first command aborted before any sample was logged.
        // The log must still be written -- the abort note alone is the evidence.
        let mut waypoints = Vec::new();
        for step in 0..31 {
            let t = f64::from(step) / 30.0;
            let q = 0.1 + 0.4 * (f64::from(step) / 30.0);
            waypoints.push(format!("{{\"t\":{t:.4},\"q\":[0.0,{q:.5}]}}"));
        }
        let text = format!(
            "{{\"format\":\"{TRAJECTORY_FORMAT}\",\"ids\":[1,2],\"control_hz\":30,\
             \"waypoints\":[{}]}}",
            waypoints.join(",")
        );
        let trajectory = parse_trajectory(Path::new("fault0.json"), &text).unwrap();
        let mock = MockBus::new(&[(1, 0.0), (2, 0.1)]).faults_after(2, 1);
        let mut arm = Arm::new(
            mock,
            trajectory.ids.clone(),
            vec![model("rs-00"), model("rs-00")],
            vec![50.0, 50.0],
            vec![3.0, 3.0],
            trajectory.hz,
        );
        // Latch the fault before playback, exactly like a fault frame queued
        // during bring-up: the first play-loop command then refuses to send.
        arm.bring_up().unwrap();
        arm.telemetry_active = true;
        arm.command(&[0.0, 0.1], None, None, false).unwrap();
        assert!(arm.bus.reader_fault().is_some());

        let mut args = Args::default();
        args.execute = true;
        args.active_report = true;
        args.log = Some(PathBuf::from("fault0-log.json"));
        let mut logged = Vec::new();
        let mut notes = LogNotes::default();

        let outcome = play(&mut arm, &trajectory, &args, &mut logged, &mut notes);

        let error = outcome.expect_err("the latched fault must abort playback");
        assert!(error.to_string().contains("fault frame"), "{error}");
        assert!(logged.is_empty(), "the fault hit before the first sample");
        let abort = notes.abort.as_ref().expect("the abort is recorded even with no samples");
        assert!(abort.reason.contains("fault frame"));
        assert!(
            log_has_evidence(&logged, &notes),
            "an abort with zero samples is still worth writing"
        );
    }

    #[test]
    fn log_write_gate_needs_samples_or_a_note() {
        let notes = LogNotes::default();
        assert!(!log_has_evidence(&[], &notes), "nothing to record");
        let sample = LogSample { t: 0.0, cmd: vec![0.0], meas: vec![0.0] };
        assert!(log_has_evidence(&[sample], &notes), "samples are evidence");
        let mut aborted = LogNotes::default();
        aborted.abort = Some(LogAbort { t: 0.0, reason: "fault frame".into() });
        assert!(log_has_evidence(&[], &aborted), "an abort alone is evidence");
    }

    #[test]
    fn shutdown_ramp_home_survives_a_latched_fault() {
        // With the reader's fault latched, command() refuses to send. The
        // shutdown ramp is the documented exception: it clears the latch,
        // returns to the start pose, and only then drops torque.
        let mock = MockBus::new(&[(1, 0.0), (2, 0.1)]).faults_after(2, 2);
        let mut arm = Arm::new(
            mock,
            vec![1, 2],
            vec![model("rs-00"), model("rs-00")],
            vec![50.0, 50.0],
            vec![3.0, 3.0],
            30,
        );
        arm.bring_up().unwrap();
        arm.telemetry_active = true;
        arm.command(&[0.3, 0.4], None, None, false).unwrap();
        assert!(arm.bus.reader_fault().is_some(), "the fault is latched");
        assert!(
            arm.command(&[0.35, 0.45], None, None, false).is_err(),
            "the interlock still refuses playback commands after a fault"
        );

        arm.shut_down(false, false).unwrap();

        assert!((arm.last[0] - 0.0).abs() < 1e-3, "joint 1 ramped home");
        assert!((arm.last[1] - 0.1).abs() < 1e-3, "joint 2 ramped home");
        assert!(!arm.enabled);
        assert!(arm.bus.enabled.borrow().is_empty(), "torque dropped after the ramp");
    }

    #[test]
    fn shutdown_ramp_tolerates_queued_fault_frames() {
        // The synchronous bus path (the bench's mode): a faulted motor keeps
        // emitting fault frames, and the strict poll bails on each one. The
        // return ramp must skip them instead of stranding the arm at the abort
        // pose.
        let mock = MockBus::new(&[(1, 0.0), (2, 0.1)]);
        let mut arm = Arm::new(
            mock,
            vec![1, 2],
            vec![model("rs-00"), model("rs-00")],
            vec![50.0, 50.0],
            vec![3.0, 3.0],
            30,
        );
        arm.bring_up().unwrap();
        // The fault arrives during playback, not bring-up: a fault frame in
        // the queue at bring-up is a different case (bring_up must disable).
        *arm.bus.queued_fault.borrow_mut() =
            Some("motor id 2 reported a fault frame: 1040000000000000".into());
        arm.command(&[0.3, 0.4], None, None, false).unwrap();
        assert!(
            arm.bus.poll_available(&arm.ids, &arm.models).is_err(),
            "the strict poll still aborts on a fault frame"
        );
        // Re-queue the fault for the ramp itself.
        *arm.bus.queued_fault.borrow_mut() =
            Some("motor id 2 reported a fault frame: 1040000000000000".into());

        arm.shut_down(false, false).unwrap();

        assert!((arm.last[0] - 0.0).abs() < 1e-3, "joint 1 ramped home");
        assert!((arm.last[1] - 0.1).abs() < 1e-3, "joint 2 ramped home");
        assert!(!arm.enabled);
        assert!(arm.bus.enabled.borrow().is_empty(), "torque dropped after the ramp");
    }

    // -- HTTP server ------------------------------------------------------- //

    fn serve_state(allow_execute: bool, token: &str) -> ServerState {
        ServerState {
            args: ServeArgs {
                allow_execute,
                token: token.to_string(),
                ..ServeArgs::default()
            },
            started: Instant::now(),
            playing: AtomicBool::new(false),
            stopping: AtomicBool::new(false),
            child: AtomicI32::new(0),
            child_pidfd: AtomicI32::new(-1),
            pidfd_lock: Mutex::new(()),
        }
    }

    fn http_request(method: &str, path: &str, token: Option<&str>, body: &str) -> Request {
        Request {
            method: method.to_string(),
            path: path.to_string(),
            query: BTreeMap::new(),
            token: token.map(str::to_string),
            body: body.as_bytes().to_vec(),
        }
    }

    #[test]
    fn http_request_line_and_body_are_parsed() {
        let raw = "POST /play?dry=1&kp=50 HTTP/1.1\r\n\
                   Host: pi\r\n\
                   X-Rebot-Token: secret\r\n\
                   Content-Length: 7\r\n\
                   \r\n\
                   {\"a\":1}";
        let mut reader = std::io::Cursor::new(raw.as_bytes());
        let request = read_request(&mut reader, MAX_BODY_BYTES).unwrap();
        assert_eq!(request.method, "POST");
        assert_eq!(request.path, "/play");
        assert_eq!(request.query.get("dry").map(String::as_str), Some("1"));
        assert_eq!(request.query.get("kp").map(String::as_str), Some("50"));
        assert_eq!(request.token.as_deref(), Some("secret"));
        assert_eq!(request.body, b"{\"a\":1}");
    }

    #[test]
    fn http_bearer_token_and_bad_length_are_handled() {
        let raw = "GET /pose HTTP/1.1\r\nAuthorization: Bearer abc\r\n\r\n";
        let mut reader = std::io::Cursor::new(raw.as_bytes());
        let request = read_request(&mut reader, MAX_BODY_BYTES).unwrap();
        assert_eq!(request.token.as_deref(), Some("abc"));

        let raw = "POST /play HTTP/1.1\r\nContent-Length: nope\r\n\r\n";
        let mut reader = std::io::Cursor::new(raw.as_bytes());
        assert!(read_request(&mut reader, MAX_BODY_BYTES).is_err());

        let raw = "POST /play HTTP/1.1\r\nContent-Length: 999999\r\n\r\n";
        let mut reader = std::io::Cursor::new(raw.as_bytes());
        let error = read_request(&mut reader, 1024).unwrap_err();
        assert!(error.0.contains("body too large"));
    }

    #[test]
    fn http_requires_the_token_except_for_health() {
        let state = serve_state(false, "secret");
        let (status, _) = route(&state, &http_request("GET", "/health", None, ""));
        assert_eq!(status, 200);

        let (status, _) = route(&state, &http_request("GET", "/pose", None, ""));
        assert_eq!(status, 401);

        let (status, _) = route(&state, &http_request("GET", "/pose", Some("wrong"), ""));
        assert_eq!(status, 401);
    }

    #[test]
    fn http_routes_are_checked() {
        let state = serve_state(false, "secret");
        let (status, _) = route(&state, &http_request("GET", "/nope", Some("secret"), ""));
        assert_eq!(status, 404);

        let (status, _) = route(&state, &http_request("GET", "/play", Some("secret"), ""));
        assert_eq!(status, 405);
    }

    #[test]
    fn http_play_is_gated_and_validated_before_it_spawns() {
        let locked = serve_state(false, "secret");
        let (status, payload) = route(&locked, &http_request("POST", "/play", Some("secret"), "{}"));
        assert_eq!(status, 403);
        assert!(payload["error"].as_str().unwrap().contains("--allow-execute"));

        // Enabled, but the body is not a trajectory: rejected before any spawn,
        // which is why this test can run without CAN or a child process.
        let open = serve_state(true, "secret");
        let (status, payload) = route(&open, &http_request("POST", "/play", Some("secret"), "{\"format\":\"x\"}"));
        assert_eq!(status, 400);
        assert!(payload["error"].as_str().unwrap().contains("rebot-joint-trajectory"));

        let (status, _) = route(&open, &http_request("POST", "/play", Some("secret"), ""));
        assert_eq!(status, 400);
    }

    #[test]
    fn http_token_compare_is_length_aware() {
        assert!(constant_time_eq(b"abc", b"abc"));
        assert!(!constant_time_eq(b"abc", b"abd"));
        assert!(!constant_time_eq(b"abc", b"ab"));
    }

    #[test]
    fn track_samples_fire_on_the_last_waypoint() {
        let small = track_samples(4);
        assert!(small.contains(&3), "100% must land on the last index: {small:?}");
        assert!(small.iter().all(|&index| index < 4));
        let big = track_samples(600);
        assert!(big.contains(&599), "100% of 600 waypoints is index 599: {big:?}");
        assert!(big.iter().all(|&index| index < 600));
        assert_eq!(track_samples(1), BTreeSet::from([0]));
        assert!(track_samples(0).is_empty());
    }

    #[test]
    fn clamp_u16_nan_is_neutral_not_the_extreme() {
        // u16 0 decodes to -max (-4pi / -Tmax): NaN must fail toward neutral.
        assert_eq!(clamp_u16(f64::NAN), 0x7FFF);
        assert_eq!(clamp_u16(f64::INFINITY), 0x7FFF);
        assert_eq!(clamp_u16(f64::NEG_INFINITY), 0x7FFF);
        assert_eq!(clamp_u16(-1.0), 0);
        assert_eq!(clamp_u16(70000.0), 0xFFFF);
        assert_eq!(clamp_u16(123.4), 123);
    }

    #[test]
    fn post_health_is_405_get_health_is_open() {
        let state = serve_state(false, "secret");
        let (status, _) = route(&state, &http_request("POST", "/health", Some("secret"), ""));
        assert_eq!(status, 405, "POST /health is a wrong method, not a free 200");
        let (status, _) = route(&state, &http_request("GET", "/health", None, ""));
        assert_eq!(status, 200);
    }

    #[test]
    fn stop_on_an_idle_server_reports_nothing_playing() {
        let state = serve_state(true, "secret");
        let (status, body) = route(&state, &http_request("POST", "/stop", Some("secret"), ""));
        assert_eq!(status, 200);
        assert_eq!(body["stopping"], false);
    }

    #[test]
    fn recv_bails_on_a_hung_up_socket_instead_of_spinning() {
        let mut fds = [0i32; 2];
        let rc = unsafe {
            libc::socketpair(libc::AF_UNIX, libc::SOCK_STREAM, 0, fds.as_mut_ptr())
        };
        assert_eq!(rc, 0);
        unsafe { libc::close(fds[1]); } // peer gone: POLLHUP on fds[0]
        let bus = CanBus {
            fd: fds[0],
            channel: "test".to_string(),
            reader: None,
            active_reports: Vec::new(),
        };
        let result = bus.recv(Duration::from_millis(200));
        // bus drops here, closing fds[0]; a hung-up bus must error immediately,
        // not busy-spin to the deadline and return Ok(None).
        assert!(result.is_err(), "a hung-up bus must error, not spin");
    }

    #[test]
    fn http_refuses_a_second_play_and_stop_is_gated() {
        let state = serve_state(true, "secret");
        state.playing.store(true, Ordering::SeqCst);
        let payload = "{\"format\":\"rebot-joint-trajectory\",\"ids\":[1],\"control_hz\":30,\
                       \"waypoints\":[{\"t\":0.0,\"q\":[0.0]},{\"t\":0.1,\"q\":[0.1]}]}";
        let (status, body) = route(&state, &http_request("POST", "/play", Some("secret"), payload));
        assert_eq!(status, 409);
        assert!(body["error"].as_str().unwrap().contains("/stop"));

        let locked = serve_state(false, "secret");
        let (status, _) = route(&locked, &http_request("POST", "/stop", Some("secret"), ""));
        assert_eq!(status, 403);

        let idle = serve_state(true, "secret");
        let (status, body) = route(&idle, &http_request("POST", "/stop", Some("secret"), ""));
        assert_eq!(status, 200);
        assert_eq!(body["stopping"], false);
    }

    #[test]
    fn http_query_is_percent_decoded() {
        assert_eq!(percent_decode("50%2C2%3A150"), "50,2:150");
        assert_eq!(percent_decode("a+b"), "a b");
        assert_eq!(percent_decode("plain"), "plain");
        assert_eq!(percent_decode("100%"), "100%");
    }

    #[test]
    fn http_loopback_detection() {
        assert!(is_loopback("127.0.0.1:8081"));
        assert!(is_loopback("localhost:8081"));
        assert!(is_loopback("[::1]:8081"));
        assert!(!is_loopback("0.0.0.0:8081"));
        assert!(!is_loopback("192.168.1.23:8081"));
    }

    #[test]
    fn gripper_stall_requires_error_and_low_velocity() {
        // Normal closing can lag, but the jaw is moving.
        assert!(!gripper_stall_condition(2.44, 2.20, 1.5));
        // A blocked jaw is far from the command and not moving.
        assert!(gripper_stall_condition(2.44, 2.00, 0.0));
        assert!(gripper_stall_condition(4.90, 4.30, -0.01));
        // A small contact gap is not a stall.
        assert!(!gripper_stall_condition(2.44, 2.35, 0.0));
    }

    #[test]
    fn gripper_stall_flag_is_validated() {
        assert!(check_gripper_stall(0.0).is_ok());
        assert!(check_gripper_stall(0.3).is_ok());
        assert!(check_gripper_stall(0.01).is_err());
        assert!(check_gripper_stall(6.0).is_err());
    }

    #[test]
    fn shutdown_ramp_is_three_times_the_normal_return_speed() {
        let from = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.90];
        let to = [0.0_f64; 7];
        let normal = ramp_duration(&from, &to);
        let shutdown = shutdown_ramp_duration(&from, &to);
        assert!(
            (normal - 3.0 * shutdown).abs() < 1e-6,
            "normal {normal} vs shutdown {shutdown}"
        );
        assert!(shutdown < normal);
    }

    #[test]
    fn the_return_ramp_follows_distance_not_a_constant() {
        // A short way back still gets the floor, so it has a beginning and an end.
        let short = ramp_duration(&[0.0, 0.0], &[0.2, 0.0]);
        assert!((short - RETURN_MIN_S).abs() < 1e-9, "short ramp {short}");

        // A long one takes longer, and lands exactly on the speed cap.
        let long = ramp_duration(&[0.0, 0.0], &[2.0, 0.0]);
        assert!(long > short, "long {long} vs short {short}");
        let peak = 0.5 * std::f64::consts::PI * 2.0 / long;
        assert!(
            (peak - RETURN_PEAK_SPEED).abs() < 1e-6,
            "peak {peak} should be {RETURN_PEAK_SPEED}"
        );

        // A constant 1.5 s -- the old behaviour -- would be far above the cap.
        let old_peak = 0.5 * std::f64::consts::PI * 2.0 / 1.5;
        assert!(old_peak > 4.0 * RETURN_PEAK_SPEED);
    }
}

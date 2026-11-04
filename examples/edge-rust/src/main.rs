//! rebot-edge - play a planned joint-space trajectory on a B601-RS over SocketCAN.
//!
//! This is a single-file, dependency-light rewrite of the Python edge example
//! (`play_trajectory.py` + `gradual_motion_demo.py` + `mit_frames.py`). It reads
//! the trajectory JSON produced by `host/plan_with_pinocchio.py`, re-checks it
//! against the file's own limits, and streams MIT frames at the file's control
//! rate. Planning stays on the workstation; the board only sends joint angles and
//! the optional gravity feedforward.
//!
//! Safety model (same as the Python player):
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
use std::io;
use std::os::unix::io::RawFd;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
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
const DEFAULT_JOINT_STEP_RAMP_S: f64 = 1.5;

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

    let hz = raw.control_hz.unwrap_or(20.0);
    if !hz.is_finite() {
        bail!("{}: 'control_hz' is not a finite number", path.display());
    }
    let hz = hz.clamp(MIN_HZ, MAX_HZ) as u32;

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
        start: raw.waypoints[0].q.clone(),
        last_t,
        duration_s,
        raw,
    })
}

fn check_against_limits(raw: &TrajectoryFile, ids: &[u8]) -> Result<()> {
    // A file without limits is accepted (the planner normally writes them); a
    // file whose limit block does not match the motor list is ignored, exactly
    // like the Python player, because there is nothing to index it by.
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
    if !value.is_finite() || value <= 0.0 {
        0
    } else if value >= 65535.0 {
        0xFFFF
    } else {
        value as u16
    }
}

/// Build the MIT operation frame exactly like `mit_frames.MitSender.build`.
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
                 This needs root or CAP_NET_RAW, like the Python player:\n  \
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
        let mut found: Vec<Option<State>> = vec![None; ids.len()];
        for _ in 0..128 {
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
        for joint in 0..self.ids.len() {
            self.write_mode_mit(self.ids[joint])?;
        }
        thread::sleep(Duration::from_millis(100));
        self.bus.drain(64)?;

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
            self.bus.drain(64)?;
        }
        self.enabled = true;
        Ok(())
    }

    fn command(&mut self, targets: &[f64], torques: Option<&[f64]>, drain: bool) -> Result<()> {
        debug_assert_eq!(targets.len(), self.ids.len());
        if self.telemetry_active {
            if let Some(fault) = self.bus.reader_fault() {
                bail!("{fault}");
            }
        }
        for joint in 0..self.ids.len() {
            let torque = torques.map_or(0.0, |values| values[joint]);
            let (id, data) = mit_frame(
                self.models[joint],
                self.ids[joint],
                targets[joint],
                self.kp[joint],
                self.kd[joint],
                0.0,
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
    ) -> Result<()> {
        let begin: Vec<f64> = self.last[..targets.len()].to_vec();
        let t0 = Instant::now();
        // `steps = round(duration * hz)` in the Python player.
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

        for step in 0..=steps {
            if honor_interrupt && interrupted() {
                break;
            }
            if !self.telemetry_active {
                for (joint, state) in self
                    .bus
                    .poll_available(&self.ids, &self.models)?
                    .iter()
                    .enumerate()
                {
                    if let Some(state) = state {
                        latest[joint] = Some(*state);
                    }
                }
            }
            let fraction = ease_in_out(f64::from(step) / f64::from(steps));
            let command: Vec<f64> = begin
                .iter()
                .zip(targets)
                .map(|(from, to)| from + (to - from) * fraction)
                .collect();
            let is_sample = sample_at.contains(&step);
            self.command(&command, None, false)?;

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
            self.command(targets, None, true)?;
            thread::sleep(self.period);
        }
        Ok(())
    }

    fn shut_down(&mut self, stay_enabled: bool, track: bool) -> Result<()> {
        if !self.enabled {
            return Ok(());
        }
        println!("  returning to start pose...");
        let home = self.start.clone();

        // Ramp home first, then hold; attempt the disable even if the ramp fails,
        // so a bad status frame cannot leave torque on by accident.
        let ramp = self
            .move_leg(&home, DEFAULT_JOINT_STEP_RAMP_S, "home", track, false)
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

fn ease_in_out(t: f64) -> f64 {
    0.5 * (1.0 - (std::f64::consts::PI * t.clamp(0.0, 1.0)).cos())
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

#[derive(Debug)]
enum Command {
    Help,
    Version,
    Dump(Box<DumpArgs>),
    Run(Box<Args>),
}

const USAGE: &str = "\
rebot-edge - replay a planned joint-space trajectory on a B601-RS over SocketCAN

USAGE:
    rebot-edge [OPTIONS] <TRAJECTORY.json>
    rebot-edge dump [--channel IFACE] [--ids 1,2,...]

The input is the JSON written by host/plan_with_pinocchio.py (format
'rebot-joint-trajectory'). Dry run is the default; nothing moves without
--execute. `dump` is the read-only pose snapshot (parameter 0x7019) that the
planner takes as its `--joints` input; it enables nothing.

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
                             (checked twice a second; 0 disables)
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
";

fn parse_args(argv: Vec<String>) -> Result<Command> {
    if argv.first().map(String::as_str) == Some("dump") {
        return parse_dump_args(argv.into_iter().skip(1).collect());
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
                "--sdk-path is a Python-only option; the Rust player talks to SocketCAN directly"
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
}

fn write_log(
    path: &Path,
    trajectory: &Trajectory,
    args: &Args,
    kp: &BTreeMap<String, f64>,
    kd: &BTreeMap<String, f64>,
    samples: &[LogSample],
) -> Result<()> {
    let payload = LogFile {
        format: "rebot-trajectory-log",
        version: 1,
        created: utc_timestamp(),
        trajectory: trajectory.path.display().to_string(),
        ids: &trajectory.ids,
        kp: kp.clone(),
        kd: kd.clone(),
        control_hz: trajectory.hz,
        speed: args.speed.clamp(0.1, 5.0),
        samples,
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

/// UTC ISO-8601, no chrono dependency. The Python player wrote local time; UTC
/// with an explicit `Z` is unambiguous and good enough for a run log.
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
             re-plan from a fresh dump_joints.py instead of ramping across"
        );
    }
    Ok(worst)
}

fn play<T: Transport>(
    arm: &mut Arm<T>,
    trajectory: &Trajectory,
    args: &Args,
    logged: &mut Vec<LogSample>,
) -> Result<i32> {
    let speed = args.speed.clamp(0.1, 5.0);
    let sample_at: BTreeSet<usize> = if args.track {
        [0.25_f64, 0.5, 0.75, 1.0]
            .iter()
            .map(|fraction| (trajectory.raw.waypoints.len() as f64 * fraction).round() as usize)
            .collect()
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
                arm.command(&trajectory.start, None, true)?;
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
        let logging = args.log.is_some();
        let mut last_elapsed = 0.0_f64;

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
                        }
                    }
                }
            } else {
                let states = arm.bus.poll_available(&arm.ids, &arm.models)?;
                for (joint, state) in states.iter().enumerate() {
                    if let Some(state) = state {
                        held[joint] = state.position;
                    }
                }
            }

            let sample = args.track && sample_at.contains(&index);
            let check_due = args.max_follow_error > 0.0 && elapsed >= next_follow_check;
            let torques = waypoint.tau.as_deref();
            arm.command(&targets, torques, false)?;

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
                let late = gaps.iter().filter(|gap| **gap > 1.5 * nominal).count();
                println!(
                    "  tick spacing: mean {mean:.1} ms, p95 {p95:.1} ms, max {:.1} ms \
                     (target {nominal:.1} ms, {late} of {} ticks >1.5x target)",
                    gaps.iter().copied().fold(0.0_f64, f64::max),
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
fn dump_joints(channel: &str, ids_spec: &str) -> Result<i32> {
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

    let payload = serde_json::json!({
        "ids": ids,
        "q": q,
        "timestamp": utc_timestamp(),
        "channel": channel,
        "note": "read-only snapshot, parameter 0x7019 (mechanical position, rad)",
    });
    println!("{}", serde_json::to_string_pretty(&payload)?);
    Ok(0)
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
    let outcome = play(&mut arm, &trajectory, &args, &mut logged);

    if let Some(path) = &args.log {
        if !logged.is_empty() {
            if let Err(error) = write_log(path, &trajectory, &args, &kp_map, &kd_map, &logged) {
                println!("  warning: cannot write the log: {error}");
            }
        }
    }

    let shutdown = arm.shut_down(args.stay_enabled, args.track);
    if let Err(error) = shutdown {
        println!("  warning: cleanup did not finish: {error}");
    }
    println!("Disconnected.");

    outcome
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
        // Generated with the Python `mit_frames.MitSender.build` formulas.
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
        assert_eq!(trajectory.hz, 30);
        assert!(trajectory.has_tau);
        assert!((trajectory.peak_tau - 0.5).abs() < 1e-12);
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
    }

    struct MockBus {
        positions: RefCell<BTreeMap<u8, f64>>,
        enabled: RefCell<BTreeSet<u8>>,
        frames: RefCell<Vec<MockFrame>>,
    }

    impl MockBus {
        fn new(positions: &[(u8, f64)]) -> Self {
            Self {
                positions: RefCell::new(positions.iter().copied().collect()),
                enabled: RefCell::new(BTreeSet::new()),
                frames: RefCell::new(Vec::new()),
            }
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
                }
                COMM_DISABLE => {
                    self.enabled.borrow_mut().remove(&device);
                }
                COMM_OPERATION_CONTROL if data.len() == 8 => {
                    let position_u16 = u16::from_be_bytes([data[0], data[1]]);
                    let torque_u16 = ((id >> 8) & 0xFFFF) as u16;
                    let position =
                        (f64::from(position_u16) / 0x7FFF as f64 - 1.0) * MIT_POSITION_LIMIT;
                    let torque = (f64::from(torque_u16) / 0x7FFF as f64 - 1.0) * 17.0;
                    self.frames.borrow_mut().push(MockFrame {
                        device,
                        position,
                        torque,
                    });
                    self.positions.borrow_mut().insert(device, position);
                }
                _ => {}
            }
            Ok(())
        }

        fn drain(&self, _budget: usize) -> Result<()> {
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
            Ok(ids.iter().map(|id| Some(self.state(*id))).collect())
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

        let code = play(&mut arm, &trajectory, &args, &mut logged).unwrap();

        assert_eq!(code, 0);
        assert_eq!(logged.len(), 4);
        let frames = arm.bus.frames.borrow();
        let last_joint_1 = frames.iter().rev().find(|frame| frame.device == 1).unwrap();
        let last_joint_2 = frames.iter().rev().find(|frame| frame.device == 2).unwrap();
        assert!((last_joint_1.position - 0.15).abs() < 1e-3);
        assert!((last_joint_2.position - 0.25).abs() < 1e-3);
        assert!((last_joint_1.torque - 0.3).abs() < 1e-3);
        assert!((last_joint_2.torque - 0.2).abs() < 1e-3);
    }
}

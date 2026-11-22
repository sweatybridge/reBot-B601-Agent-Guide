//! Hand-eye calibration fit — port of extrinsics.fit (DLT bootstrap + Huber
//! Levenberg-Marquardt + covariance + predicted error). Pure jsonb -> jsonb,
//! memory-safe (nalgebra). The detection upstream stays an isolated activity;
//! this solves T_base<-cam (and the tip offset) from clicked poses.

use pgrx::prelude::*;
use serde_json::{json, Value};
use nalgebra::{Matrix3, Vector3, Matrix4, DMatrix, DVector};
use crate::load_chain_arm;

type M3 = Matrix3<f64>;
type V3 = Vector3<f64>;
type M4 = Matrix4<f64>;
type DM = DMatrix<f64>;

const MIN_POSES: usize = 6;
const MAX_CAMERA_M: f64 = 5.0;
const MAX_TIP_M: f64 = 0.20;
const NAMES: [&str; 15] = ["rx","ry","rz","tx","ty","tz","tip_x","tip_y","tip_z",
                           "j1","j2","j3","j4","j5","j6"];

fn skew(w: &V3) -> M3 {
    M3::new(0.0, -w.z, w.y,  w.z, 0.0, -w.x,  -w.y, w.x, 0.0)
}

// nalgebra stores column-major; the ray-plane functions and the rig doc use
// row-major flattening. Serialize explicitly to avoid a transposed rig.
fn row_major3(m: &M3) -> Vec<f64> {
    (0..3).flat_map(|i| (0..3).map(move |j| m[(i,j)])).collect()
}
fn row_major4(m: &M4) -> Vec<f64> {
    (0..4).flat_map(|i| (0..4).map(move |j| m[(i,j)])).collect()
}

fn rodrigues(w: &V3) -> M3 {
    let angle = w.norm();
    let k = skew(w);
    if angle < 1e-12 { return M3::identity() + k; }
    M3::identity() + (angle.sin() / angle) * k
        + ((1.0 - angle.cos()) / (angle * angle)) * (k * k)
}

fn log_so3(r: &M3) -> V3 {
    let sym = (r + r.transpose()) * 0.5;
    let eig = sym.symmetric_eigen();
    let mut mi = 0;
    for i in 1..3 { if eig.eigenvalues[i] > eig.eigenvalues[mi] { mi = i; } }
    let mut axis = eig.eigenvectors.column(mi).into_owned();
    let n = axis.norm();
    if n < 1e-12 { return V3::zeros(); }
    axis /= n;
    let cosine = ((r.trace() - 1.0) / 2.0).clamp(-1.0, 1.0);
    let angle = cosine.acos();
    if angle < 1e-9 { return V3::zeros(); }
    let anti = V3::new(r[(2,1)] - r[(1,2)], r[(0,2)] - r[(2,0)], r[(1,0)] - r[(0,1)]);
    let sine = angle.sin();
    if sine.abs() > 0.1 {
        let candidate = anti / (2.0 * sine);
        if candidate.norm() > 1e-6 { axis = candidate / candidate.norm(); }
    }
    if anti.dot(&axis) < 0.0 { axis = -axis; }
    axis * angle
}

fn invert4(t: &M4) -> M4 {
    let r = M3::new(t[(0,0)],t[(0,1)],t[(0,2)], t[(1,0)],t[(1,1)],t[(1,2)],
                    t[(2,0)],t[(2,1)],t[(2,2)]).transpose();
    let tv = V3::new(t[(0,3)], t[(1,3)], t[(2,3)]);
    let nt = -r * tv;
    M4::new(r[(0,0)],r[(0,1)],r[(0,2)],nt.x, r[(1,0)],r[(1,1)],r[(1,2)],nt.y,
            r[(2,0)],r[(2,1)],r[(2,2)],nt.z, 0.0,0.0,0.0,1.0)
}

fn transform4(r: &M3, t: &V3) -> M4 {
    M4::new(r[(0,0)],r[(0,1)],r[(0,2)],t.x, r[(1,0)],r[(1,1)],r[(1,2)],t.y,
            r[(2,0)],r[(2,1)],r[(2,2)],t.z, 0.0,0.0,0.0,1.0)
}

fn coefficients(dist: &[f64]) -> [f64; 5] {
    let mut c = [0.0; 5];
    for i in 0..5 { c[i] = dist.get(i).copied().unwrap_or(0.0); }
    c
}

fn distort(x: f64, y: f64, dist: &[f64]) -> (f64, f64) {
    let [k1, k2, p1, p2, k3] = coefficients(dist);
    let r2 = x * x + y * y;
    let radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    (x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x),
     y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y)
}

fn undistort(xd: f64, yd: f64, dist: &[f64]) -> (f64, f64) {
    let [k1, k2, p1, p2, k3] = coefficients(dist);
    if k1 == 0.0 && k2 == 0.0 && p1 == 0.0 && p2 == 0.0 && k3 == 0.0 { return (xd, yd); }
    let (mut x, mut y) = (xd, yd);
    for _ in 0..10 {
        let r2 = x * x + y * y;
        let radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
        let dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        let dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        x = (xd - dx) / radial;
        y = (yd - dy) / radial;
    }
    (x, y)
}

fn pixel(p_base: &V3, k: &M3, dist: &[f64], t_cam_base: &M4) -> Option<(f64, f64)> {
    let r = M3::new(t_cam_base[(0,0)],t_cam_base[(0,1)],t_cam_base[(0,2)],
                    t_cam_base[(1,0)],t_cam_base[(1,1)],t_cam_base[(1,2)],
                    t_cam_base[(2,0)],t_cam_base[(2,1)],t_cam_base[(2,2)]);
    let t = V3::new(t_cam_base[(0,3)], t_cam_base[(1,3)], t_cam_base[(2,3)]);
    let p_cam = r * p_base + t;
    let depth = p_cam.z;
    if depth <= 1e-6 { return None; }
    let (x, y) = distort(p_cam.x / depth, p_cam.y / depth, dist);
    Some((k[(0,0)] * x + k[(0,2)], k[(1,1)] * y + k[(1,2)]))
}

fn fk_pose(urdf: &str, q6: &[f64]) -> (M3, V3) {
    let (_robot, chain) = load_chain_arm(urdf, q6);
    let node = chain.find_link("gripper_end").unwrap_or_else(|| panic!("gripper_end not found"));
    let t = node.world_transform().unwrap();
    (t.rotation.to_rotation_matrix().matrix().clone(), t.translation.vector)
}

struct Obs { q: [f64; 6], uv: [f64; 2] }

struct Model {
    urdf: String,
    q: Vec<[f64; 6]>,
    uv: Vec<[f64; 2]>,
    k: M3,
    dist: Vec<f64>,
    fixed: Option<Vec<(M3, V3)>>,
    free: Vec<usize>,
    refine_zeros: bool,
    huber_px: f64,
}

impl Model {
    fn new(urdf: &str, obs: &[Obs], k: M3, dist: Vec<f64>,
           refine_tip: bool, refine_zeros: bool, huber_px: f64) -> Self {
        let q: Vec<[f64; 6]> = obs.iter().map(|o| o.q).collect();
        let uv: Vec<[f64; 2]> = obs.iter().map(|o| o.uv).collect();
        let fixed = if !refine_zeros {
            Some(q.iter().map(|qq| fk_pose(urdf, qq)).collect())
        } else { None };
        let mut free: Vec<usize> = (0..6).collect();
        if refine_tip { free.extend([6, 7, 8]); }
        if refine_zeros { free.extend([10, 11, 12, 13]); }
        Model { urdf: urdf.to_string(), q, uv, k, dist, fixed, free, refine_zeros, huber_px }
    }

    fn size(&self) -> usize { 2 * self.q.len() }

    fn tip_in_base(&self, i: usize, tip: &V3, zeros: Option<&[f64]>) -> V3 {
        let use_zeros = zeros.map(|z| z.iter().any(|v| *v != 0.0)).unwrap_or(false);
        if !use_zeros {
            if let Some(fixed) = &self.fixed {
                let (r, p) = &fixed[i];
                return r * tip + p;
            }
        }
        let mut q = self.q[i];
        if let Some(z) = zeros { if use_zeros { for j in 0..6 { q[j] += z[j]; } } }
        let (r, p) = fk_pose(&self.urdf, &q);
        r * tip + p
    }

    fn residuals(&self, theta: &[f64]) -> Vec<f64> {
        let r = rodrigues(&V3::new(theta[0], theta[1], theta[2]));
        let t = V3::new(theta[3], theta[4], theta[5]);
        let tip = V3::new(theta[6], theta[7], theta[8]);
        let zeros = if self.refine_zeros { Some(&theta[9..15]) } else { None };
        let camera = transform4(&r, &t);
        let mut out = vec![0.0; self.size()];
        for i in 0..self.q.len() {
            let point = self.tip_in_base(i, &tip, zeros);
            match pixel(&point, &self.k, &self.dist, &camera) {
                None => { out[2*i] = 1e3; out[2*i+1] = 1e3; }
                Some((u, v)) => { out[2*i] = u - self.uv[i][0]; out[2*i+1] = v - self.uv[i][1]; }
            }
        }
        out
    }

    fn jacobian(&self, theta: &[f64]) -> DM {
        let step = 1e-6;
        let n = self.size();
        let mut cols: Vec<DVector<f64>> = Vec::new();
        for &j in &self.free {
            let mut fwd = theta.to_vec();
            let mut bwd = theta.to_vec();
            fwd[j] += step; bwd[j] -= step;
            let rf = self.residuals(&fwd);
            let rb = self.residuals(&bwd);
            cols.push(DVector::from_iterator(n, (0..n).map(|i| (rf[i] - rb[i]) / (2.0 * step))));
        }
        DM::from_columns(&cols)
    }

    fn weights(&self, residuals: &[f64]) -> Vec<f64> {
        let n = self.size();
        if self.huber_px <= 0.0 { return vec![1.0; n]; }
        let delta = self.huber_px;
        let mut w = vec![1.0; n];
        for i in 0..self.q.len() {
            let norm = (residuals[2*i].powi(2) + residuals[2*i+1].powi(2)).sqrt();
            let scale = if norm > delta { delta / norm.max(1e-9) } else { 1.0 };
            w[2*i] = scale; w[2*i+1] = scale;
        }
        w
    }
}

fn dlt_pose(world: &[V3], normalized: &[(f64, f64)]) -> (M3, V3) {
    let count = world.len();
    if count < 6 { panic!("a DLT needs at least 6 poses to start from, got {count}"); }
    let centre = world.iter().fold(V3::zeros(), |a, b| a + b) / count as f64;
    let spread = world.iter().map(|w| (w - centre).norm()).sum::<f64>() / count as f64;
    if spread < 1e-6 { panic!("every pose puts the tool at the same point; move the arm between captures"); }
    let wnorm: Vec<V3> = world.iter().map(|w| (w - centre) / spread).collect();
    let image_mean = normalized.iter().fold((0.0, 0.0), |a, b| (a.0 + b.0, a.1 + b.1));
    let image_mean = (image_mean.0 / count as f64, image_mean.1 / count as f64);
    let image_spread = normalized.iter()
        .map(|u| ((u.0 - image_mean.0).powi(2) + (u.1 - image_mean.1).powi(2)).sqrt())
        .sum::<f64>() / count as f64;
    if image_spread < 1e-9 { panic!("every click landed on the same pixel; the tip has to move"); }
    let unorm: Vec<(f64, f64)> = normalized.iter()
        .map(|u| ((u.0 - image_mean.0) / image_spread, (u.1 - image_mean.1) / image_spread)).collect();

    let mut rows = DM::zeros(2 * count, 12);
    for i in 0..count {
        let (x, y, z) = (wnorm[i].x, wnorm[i].y, wnorm[i].z);
        let (u, v) = unorm[i];
        let r0 = [x, y, z, 1.0, 0.0, 0.0, 0.0, 0.0, -u*x, -u*y, -u*z, -u];
        let r1 = [0.0, 0.0, 0.0, 0.0, x, y, z, 1.0, -v*x, -v*y, -v*z, -v];
        for j in 0..12 { rows[(2*i, j)] = r0[j]; rows[(2*i+1, j)] = r1[j]; }
    }
    let svd = rows.svd(false, true);
    let values = svd.singular_values;
    let v_t = svd.v_t.unwrap();
    if values[values.len()-2] < 1e-7 * values[0] {
        let rank = values.iter().filter(|v| **v > 1e-7 * values[0]).count();
        panic!("the poses are degenerate (rank {rank} of 11): the tool needs to move in 3D, at several heights and with a range of wrist orientations, not along a flat arc");
    }
    let last = v_t.row(v_t.nrows()-1);
    let mut pmat = DM::zeros(3, 4);
    for j in 0..12 { pmat[(j/4, j%4)] = last[j]; }

    let s = M3::new(image_spread, 0.0, image_mean.0,  0.0, image_spread, image_mean.1,  0.0, 0.0, 1.0);
    let d = M4::new(spread,0.0,0.0,centre.x, 0.0,spread,0.0,centre.y,
                    0.0,0.0,spread,centre.z, 0.0,0.0,0.0,1.0);
    let d_inv = d.try_inverse().unwrap_or_else(|| panic!("singular normalization"));
    // P = S @ P @ D^-1  (P is 3x4)
    let mut p_dm = s * pmat * d_inv;

    let mut block = M3::new(p_dm[(0,0)], p_dm[(0,1)], p_dm[(0,2)],
                            p_dm[(1,0)], p_dm[(1,1)], p_dm[(1,2)],
                            p_dm[(2,0)], p_dm[(2,1)], p_dm[(2,2)]);
    if block.determinant() < 0.0 { p_dm = -p_dm; block = -block; }
    let scale = block.determinant().abs().cbrt();
    if scale < 1e-12 { panic!("the DLT found a singular projection; check the clicked points"); }
    p_dm /= scale;
    block /= scale;

    // RQ decomposition via QR of (J @ block)^T.
    let j = M3::new(0.0, 0.0, 1.0,  0.0, 1.0, 0.0,  1.0, 0.0, 0.0);
    let qr = (j * block).transpose().qr();
    let q1 = qr.q();
    let r1 = qr.r();
    let mut upper = j * r1.transpose() * j;
    let mut rotation = j * q1.transpose();
    let mut signs = V3::new(upper[(0,0)].signum(), upper[(1,1)].signum(), upper[(2,2)].signum());
    for i in 0..3 { if signs[i] == 0.0 { signs[i] = 1.0; } }
    let sign_mat = M3::from_diagonal(&signs);
    upper = upper * sign_mat;
    rotation = sign_mat * rotation;
    let p3 = V3::new(p_dm[(0,3)], p_dm[(1,3)], p_dm[(2,3)]);
    let translation = upper.lu().solve(&p3).unwrap_or_else(|| panic!("RQ solve failed"));

    let svd_r = rotation.svd(true, true);
    let u_ = svd_r.u.unwrap();
    let v_ = svd_r.v_t.unwrap().transpose();
    let mut rot = u_ * v_.transpose();
    if rot.determinant() < 0.0 {
        let fix = M3::new(1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,-1.0);
        rot = u_ * fix * v_.transpose();
    }
    let depths: Vec<f64> = world.iter().map(|w| (rot * w + translation).z).collect();
    if (depths.iter().sum::<f64>() / count as f64) < 0.0 {
        panic!("the DLT found a mirrored solution: every clicked point is behind the camera. Check that the clicks are on the tool tip and the poses match the frames");
    }
    (rot, translation)
}

fn theta0(urdf: &str, obs: &[Obs], tip: &V3, k: &M3, dist: &[f64]) -> [f64; 15] {
    let normalized: Vec<(f64, f64)> = obs.iter().map(|o| {
        let xd = (o.uv[0] - k[(0,2)]) / k[(0,0)];
        let yd = (o.uv[1] - k[(1,2)]) / k[(1,1)];
        undistort(xd, yd, dist)
    }).collect();
    let world: Vec<V3> = obs.iter().map(|o| {
        let (r, p) = fk_pose(urdf, &o.q);
        r * tip + p
    }).collect();
    let (rotation, translation) = dlt_pose(&world, &normalized);
    let mut theta = [0.0; 15];
    let rv = log_so3(&rotation);
    theta[0] = rv.x; theta[1] = rv.y; theta[2] = rv.z;
    theta[3] = translation.x; theta[4] = translation.y; theta[5] = translation.z;
    theta[6] = tip.x; theta[7] = tip.y; theta[8] = tip.z;
    theta
}

fn levenberg_marquardt(model: &Model, theta0: &[f64; 15], max_iterations: usize)
    -> ([f64; 15], Vec<f64>, Vec<f64>, DM, usize, bool) {
    let mut theta = *theta0;
    let mut residuals = model.residuals(&theta);
    let mut weights = model.weights(&residuals);
    let cost = |r: &[f64], w: &[f64]| -> f64 { (0..r.len()).map(|i| w[i]*r[i]*r[i]).sum() };
    let mut c = cost(&residuals, &weights);
    let mut damping = 1e-3;
    let mut converged = false;
    let mut iterations = 0;
    let mut jacobian = model.jacobian(&theta);
    let k = model.free.len();
    for it in 1..=max_iterations {
        iterations = it;
        let root: Vec<f64> = weights.iter().map(|w| w.sqrt()).collect();
        let n = model.size();
        let mut wj = DM::zeros(n, k);
        for i in 0..n { for j in 0..k { wj[(i,j)] = jacobian[(i,j)] * root[i]; } }
        let wr = DVector::from_iterator(n, (0..n).map(|i| residuals[i]*root[i]));
        let gradient = wj.transpose() * wr;
        let normal = wj.transpose() * &wj;
        let mut system = normal.clone();
        for j in 0..k { system[(j,j)] += damping * normal[(j,j)].max(1e-12); }
        let step = match system.clone().lu().solve(&gradient) {
            Some(s) => -s,
            None => {
                let svd = system.svd(true, true);
                let u = svd.u.unwrap(); let vt = svd.v_t.unwrap();
                let sv = svd.singular_values;
                let mut dinv = DM::zeros(k, k);
                for j in 0..k { if sv[j] > 1e-12 { dinv[(j,j)] = 1.0/sv[j]; } }
                -(vt.transpose() * dinv * u.transpose() * gradient)
            }
        };
        let mut trial = theta;
        for (idx, &j) in model.free.iter().enumerate() { trial[j] += step[idx]; }
        let trial_residuals = model.residuals(&trial);
        let trial_weights = model.weights(&trial_residuals);
        let trial_cost = cost(&trial_residuals, &trial_weights);
        if trial_cost < c {
            let improvement = c - trial_cost;
            theta = trial; residuals = trial_residuals; weights = trial_weights; c = trial_cost;
            damping = (damping / 3.0).max(1e-12);
            jacobian = model.jacobian(&theta);
            if improvement < 1e-10 * c.max(1.0) || step.norm() < 1e-11 { converged = true; break; }
        } else {
            if step.norm() < 1e-10 { converged = true; break; }
            damping *= 5.0;
            if damping > 1e12 { break; }
        }
    }
    (theta, residuals, weights, jacobian, iterations, converged)
}

fn covariance(model: &Model, residuals: &[f64], weights: &[f64], jacobian: &DM) -> (DM, f64) {
    let n = model.size();
    let k = model.free.len();
    let root: Vec<f64> = weights.iter().map(|w| w.sqrt()).collect();
    let mut wj = DM::zeros(n, k);
    for i in 0..n { for j in 0..k { wj[(i,j)] = jacobian[(i,j)] * root[i]; } }
    let normal = wj.transpose() * &wj;
    let dof = n as f64 - k as f64;
    if dof <= 0.0 {
        panic!("{} poses give {} residuals for {} unknowns: add at least one more pose", model.q.len(), n, k);
    }
    let variance: f64 = (0..n).map(|i| weights[i]*residuals[i]*residuals[i]).sum::<f64>() / dof;
    let cov = match normal.clone().try_inverse() {
        Some(inv) => variance * inv,
        None => {
            let svd = normal.svd(true, true);
            let u = svd.u.unwrap(); let vt = svd.v_t.unwrap(); let sv = svd.singular_values;
            let mut dinv = DM::zeros(k, k);
            for j in 0..k { if sv[j] > 1e-12 { dinv[(j,j)] = 1.0/sv[j]; } }
            variance * (vt.transpose() * dinv * u.transpose())
        }
    };
    (0.5 * (&cov + cov.transpose()), variance)
}

fn conditioning(cov: &DM, free: &[usize]) -> Value {
    let eig = cov.clone().symmetric_eigen();
    let mut mi = 0;
    for i in 1..eig.eigenvalues.len() { if eig.eigenvalues[i] > eig.eigenvalues[mi] { mi = i; } }
    let worst = eig.eigenvectors.column(mi).into_owned();
    let mut order: Vec<usize> = (0..free.len()).collect();
    order.sort_by(|a, b| worst[*b].abs().partial_cmp(&worst[*a].abs()).unwrap());
    let parts: Vec<String> = order.iter().take(3)
        .map(|i| format!("{:+.2}*{}", worst[*i], NAMES[free[*i]])).collect();
    let sigma = eig.eigenvalues[mi].max(0.0).sqrt();
    json!({"direction": parts.join(" "), "sigma": sigma, "sigma_mm": 1000.0 * sigma})
}

fn ray_plane_point(k: &[f64], dist: &[f64], t_base_cam: &[f64], u: f64, v: f64, plane_z: f64)
    -> Option<V3> {
    let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        crate::rebot_ray_plane(k.to_vec(), dist.to_vec(), t_base_cam.to_vec(), u, v, plane_z)
    })).ok()?;
    let p = res.0.get("point")?.as_array()?;
    Some(V3::new(p[0].as_f64()?, p[1].as_f64()?, p[2].as_f64()?))
}

fn predicted_error(theta: &[f64], cov: &DM, free: &[usize], k: &M3, dist: &[f64],
                   plane_z: f64, work_radius: f64, image_size: Option<(f64, f64)>) -> Value {
    let r = rodrigues(&V3::new(theta[0], theta[1], theta[2]));
    let t = V3::new(theta[3], theta[4], theta[5]);
    let t_cam_base = transform4(&r, &t);
    let t_base_cam = invert4(&t_cam_base);
    let k_vec: Vec<f64> = row_major3(k);
    let tbc_vec: Vec<f64> = row_major4(&t_base_cam);
    let radius = work_radius;
    let mut found: Vec<(f64, f64, f64)> = Vec::new();
    for ix in 0..5 {
        let x = 0.30 - radius + (2.0 * radius) * ix as f64 / 4.0;
        for iy in 0..5 {
            let y = -radius + (2.0 * radius) * iy as f64 / 4.0;
            let point = V3::new(x, y, plane_z);
            let rxy = (point.x*point.x + point.y*point.y).sqrt();
            if !(0.12 < rxy && rxy < (0.75_f64).max(radius + 0.2)) { continue; }
            let seen = match pixel(&point, k, dist, &t_cam_base) { Some(s) => s, None => continue };
            if let Some((w, h)) = image_size {
                if !(0.0 <= seen.0 && seen.0 <= w && 0.0 <= seen.1 && seen.1 <= h) { continue; }
            }
            let anchor = match ray_plane_point(&k_vec, dist, &tbc_vec, seen.0, seen.1, plane_z) {
                Some(a) => a, None => continue };
            let mut gradients = DM::zeros(free.len(), 3);
            let mut ok = true;
            for (kk, &j) in free.iter().enumerate() {
                let step = 1e-4;
                let mut moved = theta.to_vec();
                moved[j] += step;
                let cam = invert4(&transform4(&rodrigues(&V3::new(moved[0],moved[1],moved[2])),
                                              &V3::new(moved[3],moved[4],moved[5])));
                let cam_vec: Vec<f64> = row_major4(&cam);
                let shifted = match ray_plane_point(&k_vec, dist, &cam_vec, seen.0, seen.1, plane_z) {
                    Some(s) => s, None => { ok = false; break } };
                let g = (shifted - anchor) / step;
                gradients[(kk,0)] = g.x; gradients[(kk,1)] = g.y; gradients[(kk,2)] = g.z;
            }
            if !ok { continue; }
            let spread = (gradients.transpose() * cov * &gradients).trace().max(0.0).sqrt();
            found.push((x, y, spread));
        }
    }
    if found.is_empty() { return json!({"centre_mm": null, "corner_mm": null, "points": 0}); }
    let nearest = found.iter().min_by(|a, b| {
        ((a.0-0.30).hypot(a.1)).partial_cmp(&((b.0-0.30).hypot(b.1))).unwrap() }).unwrap();
    let worst = found.iter().max_by(|a, b| a.2.partial_cmp(&b.2).unwrap()).unwrap();
    json!({"centre_mm": (1000.0*nearest.2 * 100.0).round_ties_even()/100.0,
           "corner_mm": (1000.0*worst.2 * 100.0).round_ties_even()/100.0,
           "corner_at": [(worst.0*1000.0).round_ties_even()/1000.0, (worst.1*1000.0).round_ties_even()/1000.0],
           "points": found.len()})
}

/// Fit T_base<-cam (and tip offset) from clicked poses. observations = [{q[6],
/// uv[u,v]}]; K = 3x3; dist = [k1,k2,p1,p2,k3]; options = {refine_tip, refine_zeros,
/// plane_z, huber_px, max_iterations, max_weak_mm, work_radius, image_size}.
/// Returns the rig fields + fit diagnostics. Refusals panic -> SQL ERROR.
#[pg_extern]
fn rebot_calibrate_fit(urdf: &str, observations: pgrx::JsonB, k: pgrx::JsonB,
                       dist: pgrx::JsonB, options: pgrx::JsonB) -> pgrx::JsonB {
    let obs_v = observations.0;
    let options = options.0;
    let oarr = obs_v.as_array().unwrap_or_else(|| panic!("observations must be an array"));
    let obs: Vec<Obs> = oarr.iter().map(|o| {
        let q = o["q"].as_array().unwrap();
        let uv = o["uv"].as_array().unwrap();
        let mut qq = [0.0; 6];
        for j in 0..6 { qq[j] = q[j].as_f64().unwrap(); }
        Obs { q: qq, uv: [uv[0].as_f64().unwrap(), uv[1].as_f64().unwrap()] }
    }).collect();
    if obs.len() < MIN_POSES {
        panic!("{} poses is not enough; the rig needs at least {}, spread over the work area and at a couple of heights", obs.len(), MIN_POSES);
    }
    let ka = k.0.as_array().unwrap();
    let km = M3::new(ka[0].as_f64().unwrap(), ka[1].as_f64().unwrap(), ka[2].as_f64().unwrap(),
                     ka[3].as_f64().unwrap(), ka[4].as_f64().unwrap(), ka[5].as_f64().unwrap(),
                     ka[6].as_f64().unwrap(), ka[7].as_f64().unwrap(), ka[8].as_f64().unwrap());
    let dist_v: Vec<f64> = dist.0.as_array().map(|a| a.iter().map(|v| v.as_f64().unwrap()).collect()).unwrap_or_default();
    let refine_tip = options.get("refine_tip").and_then(|v| v.as_bool()).unwrap_or(true);
    let refine_zeros = options.get("refine_zeros").and_then(|v| v.as_bool()).unwrap_or(false);
    let plane_z = options.get("plane_z").and_then(|v| v.as_f64()).unwrap_or(0.0);
    let huber_px = options.get("huber_px").and_then(|v| v.as_f64()).unwrap_or(2.0);
    let max_iterations = options.get("max_iterations").and_then(|v| v.as_u64()).unwrap_or(120) as usize;
    let max_weak_mm = options.get("max_weak_mm").and_then(|v| v.as_f64()).unwrap_or(60.0);
    let work_radius = options.get("work_radius").and_then(|v| v.as_f64()).unwrap_or(0.55);
    let image_size = options.get("image_size").and_then(|v| v.as_array())
        .map(|a| (a[0].as_f64().unwrap(), a[1].as_f64().unwrap()));

    let model = Model::new(urdf, &obs, km, dist_v.clone(), refine_tip, refine_zeros, huber_px);
    if model.size() <= model.free.len() {
        panic!("{} poses give {} residuals for {} unknowns: add more poses, or stop refining the joint zeros",
               obs.len(), model.size(), model.free.len());
    }
    if refine_zeros && model.size() - model.free.len() < 5 {
        panic!("refining the joint zeros fits {} unknowns, so it needs at least 9 poses; this session has {}",
               model.free.len(), obs.len());
    }

    let mut tip = V3::zeros();
    let mut theta = [0.0; 15];
    let mut residuals = vec![];
    let mut weights = vec![];
    let mut jacobian = DM::zeros(0, 0);
    let mut iterations = 0;
    let mut converged = false;
    for _pass in 0..2 {
        theta = theta0(urdf, &obs, &tip, &km, &dist_v);
        let (th, res, w, jac, it, conv) = levenberg_marquardt(&model, &theta, max_iterations);
        theta = th; residuals = res; weights = w; jacobian = jac; iterations = it; converged = conv;
        let new_tip = V3::new(theta[6], theta[7], theta[8]);
        if (new_tip - tip).norm() < 0.005 { break; }
        tip = new_tip;
    }

    let (cov, _variance) = covariance(&model, &residuals, &weights, &jacobian);
    let sigma: Value = model.free.iter().enumerate()
        .map(|(kk, &j)| (NAMES[j].to_string(), json!(cov[(kk,kk)].max(0.0).sqrt())))
        .collect::<serde_json::Map<String, Value>>().into();
    let weakest = conditioning(&cov, &model.free);
    let predicted = predicted_error(&theta, &cov, &model.free, &km, &dist_v, plane_z, work_radius, image_size);

    let per_pose: Vec<f64> = (0..obs.len())
        .map(|i| (residuals[2*i].powi(2) + residuals[2*i+1].powi(2)).sqrt()).collect();
    let rms_px = (per_pose.iter().map(|v| v*v).sum::<f64>() / obs.len() as f64).sqrt();
    let t_base_cam = invert4(&transform4(&rodrigues(&V3::new(theta[0],theta[1],theta[2])),
                                         &V3::new(theta[3],theta[4],theta[5])));
    let tip_v = V3::new(theta[6], theta[7], theta[8]);
    let zeros = if refine_zeros { Some(&theta[9..15]) } else { None };
    let cam_pos = V3::new(t_base_cam[(0,3)], t_base_cam[(1,3)], t_base_cam[(2,3)]);
    let ranges: Vec<f64> = (0..obs.len())
        .map(|i| (cam_pos - model.tip_in_base(i, &tip_v, zeros)).norm()).collect();
    let focal = (km[(0,0)] + km[(1,1)]) / 2.0;
    let metres_per_pixel = if focal != 0.0 { ranges.iter().sum::<f64>() / obs.len() as f64 / focal } else { 0.0 };

    let weak_mm = weakest["sigma_mm"].as_f64().unwrap();
    if weak_mm > max_weak_mm {
        panic!("the sweep leaves {} free to {:.0} mm (limit {:.0}): move the arm through the workspace, vary the wrist orientation, and re-click",
               weakest["direction"].as_str().unwrap(), weak_mm, max_weak_mm);
    }
    let camera = V3::new(t_base_cam[(0,3)], t_base_cam[(1,3)], t_base_cam[(2,3)]);
    if camera.norm() > MAX_CAMERA_M {
        panic!("the fit ran away: the camera came out {:.1} m from the base, where a bracket is within {:.0} m. The detected points do not all belong to one arm", camera.norm(), MAX_CAMERA_M);
    }
    if tip_v.norm() > MAX_TIP_M {
        panic!("the fit put the tip {:.0} mm from the flange, but a gripper's tip is a few centimetres: the points are not all the tool tip", 1000.0*tip_v.norm());
    }

    let tbc: Vec<f64> = row_major4(&t_base_cam);
    let joint_zeros: Value = if refine_zeros { json!(theta[9..15].to_vec()) } else { Value::Null };
    let kf = model.free.len();
    let mut cov_flat = Vec::with_capacity(kf * kf);
    for i in 0..kf { for j in 0..kf { cov_flat.push(cov[(i,j)]); } }
    pgrx::JsonB(json!({
        "T_base_cam": tbc,
        "tip_offset": [theta[6], theta[7], theta[8]],
        "joint_zeros": joint_zeros,
        "plane_z": plane_z,
        "rms_px": rms_px,
        "rms_mm": rms_px * metres_per_pixel * 1000.0,
        "max_px": per_pose.iter().cloned().fold(0.0_f64, f64::max),
        "sigma": sigma,
        "weakest": weakest,
        "predicted": predicted,
        "iterations": iterations,
        "converged": converged,
        "residuals_px": per_pose,
        "covariance": cov_flat,
        "free": model.free,
    }))
}
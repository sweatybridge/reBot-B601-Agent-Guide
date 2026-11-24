//! Single source of truth for pinhole projection, Brown-Conrady lens
//! distortion / fixed-point undistortion, and ray-plane intersection.
//! Shared by `lib.rs` SQL exports and `calib.rs` hand-eye fitting.

#[inline]
pub(crate) fn coefficients(dist: &[f64]) -> [f64; 5] {
    [
        *dist.first().unwrap_or(&0.0),
        *dist.get(1).unwrap_or(&0.0),
        *dist.get(2).unwrap_or(&0.0),
        *dist.get(3).unwrap_or(&0.0),
        *dist.get(4).unwrap_or(&0.0),
    ]
}

pub(crate) fn distort(x: f64, y: f64, dist: &[f64]) -> (f64, f64) {
    let [k1, k2, p1, p2, k3] = coefficients(dist);
    let r2 = x * x + y * y;
    let radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    (
        x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x),
        y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y,
    )
}

/// Fixed-point inversion of radial/tangential distortion (10 iterations).
/// Fails cleanly if distortion diverges or hits a zero radial divisor.
pub(crate) fn undistort(xd: f64, yd: f64, dist: &[f64]) -> Result<(f64, f64), &'static str> {
    if !xd.is_finite() || !yd.is_finite() {
        return Err("non-finite distorted coordinates");
    }
    let [k1, k2, p1, p2, k3] = coefficients(dist);
    if k1 == 0.0 && k2 == 0.0 && p1 == 0.0 && p2 == 0.0 && k3 == 0.0 {
        return Ok((xd, yd));
    }
    let (mut x, mut y) = (xd, yd);
    for _ in 0..10 {
        let r2 = x * x + y * y;
        let radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
        if !radial.is_finite() || radial.abs() < 1e-12 {
            return Err("lens undistortion diverged (singular radial factor)");
        }
        let dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        let dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        x = (xd - dx) / radial;
        y = (yd - dy) / radial;
        if !x.is_finite() || !y.is_finite() {
            return Err("lens undistortion diverged to non-finite coordinates");
        }
    }
    Ok((x, y))
}

/// Project a base-frame point with optional distortion (`dist` may be empty).
/// `k` is row-major 3x3 (9), `t_base_cam` is row-major 4x4 (16).
pub(crate) fn project_dist(
    k: &[f64],
    t_base_cam: &[f64],
    point: &[f64],
    dist: &[f64],
) -> Result<Option<[f64; 2]>, &'static str> {
    if k.len() != 9 || t_base_cam.len() != 16 || point.len() != 3 {
        return Err("K must be 9, T_base_cam 16, point 3");
    }
    if !k
        .iter()
        .chain(t_base_cam.iter())
        .chain(point.iter())
        .chain(dist.iter())
        .all(|v| v.is_finite())
    {
        return Err("projection inputs must be finite");
    }
    let dx = point[0] - t_base_cam[3];
    let dy = point[1] - t_base_cam[7];
    let dz = point[2] - t_base_cam[11];
    let px = t_base_cam[0] * dx + t_base_cam[4] * dy + t_base_cam[8] * dz;
    let py = t_base_cam[1] * dx + t_base_cam[5] * dy + t_base_cam[9] * dz;
    let pz = t_base_cam[2] * dx + t_base_cam[6] * dy + t_base_cam[10] * dz;
    if pz <= 1e-9 {
        return Ok(None);
    }
    let (xd, yd) = distort(px / pz, py / pz, dist);
    Ok(Some([k[0] * xd + k[2], k[4] * yd + k[5]]))
}

/// Intersect a pixel ray with horizontal plane `z = plane_z` (`extrinsics.ray_plane`).
/// Returns `(point_xyz, distance_m, grazing_deg)`.
pub(crate) fn ray_plane_impl(
    k: &[f64],
    dist: &[f64],
    t_base_cam: &[f64],
    u: f64,
    v: f64,
    plane_z: f64,
) -> Result<([f64; 3], f64, f64), String> {
    if k.len() != 9 || t_base_cam.len() != 16 {
        return Err("K must be 9, T_base_cam 16".to_string());
    }
    if !k
        .iter()
        .chain(dist.iter())
        .chain(t_base_cam.iter())
        .all(|x| x.is_finite())
        || !u.is_finite()
        || !v.is_finite()
        || !plane_z.is_finite()
    {
        return Err("ray_plane inputs must be finite".to_string());
    }
    if k[0].abs() < 1e-12 || k[4].abs() < 1e-12 {
        return Err("camera focal length (K[0,0], K[1,1]) must be non-zero".to_string());
    }
    let xd = (u - k[2]) / k[0];
    let yd = (v - k[5]) / k[4];
    let (x, y) = undistort(xd, yd, dist).map_err(|e| e.to_string())?;

    let t = t_base_cam;
    let mut d = [
        t[0] * x + t[1] * y + t[2],
        t[4] * x + t[5] * y + t[6],
        t[8] * x + t[9] * y + t[10],
    ];
    let norm = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    if !norm.is_finite() || norm < 1e-12 {
        return Err(format!("degenerate ray for pixel ({u}, {v})"));
    }
    for c in d.iter_mut() {
        *c /= norm;
    }
    if d[2].abs() < 1e-6 {
        return Err("the ray is parallel to the table plane".to_string());
    }
    let distance = (plane_z - t[11]) / d[2];
    if !distance.is_finite() || distance <= 0.0 {
        return Err("the ray points away from the table plane".to_string());
    }
    let point = [
        t[3] + distance * d[0],
        t[7] + distance * d[1],
        t[11] + distance * d[2],
    ];
    let grazing = d[2].abs().min(1.0).asin().to_degrees();
    Ok((point, distance, grazing))
}

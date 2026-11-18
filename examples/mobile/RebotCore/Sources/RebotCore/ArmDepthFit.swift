/* Depth-based base anchoring: solve T_base_world from LiDAR scene points when
 * the ArUco tag is not visible.
 *
 * The arm is a known object: FK at the host's joint angles gives its capsule
 * skeleton in the base frame (the same link table as the host's collision
 * probe, sweep.RADII). ARKit's gravity alignment makes world-Z and base-Z both
 * vertical, so the unknown transform is only (x, y, z, yaw). A robust
 * Gauss-Newton fit of scene points to the capsules recovers it, with refusal
 * gates instead of guesses — the same philosophy as the host's rig sweep.
 *
 * This is a coarse anchor (~1-3 cm): the tag solve and the fitted rig remain
 * the precision paths, and a tag re-anchor supersedes this whenever visible.
 */
import Foundation

public struct DepthFitResult {
    public var ok: Bool
    public var reason: String?
    public var TBaseWorld: Mat4?
    public var rms: Double
    public var inliers: Int
    public var linksCovered: Int
    public var yawLever: Double
}

public enum ArmDepthFit {
    /// Link capsule radii for the segments joint1->joint2 ... joint6->tool,
    /// the same table as the host's collision probe (sweep.RADII[1...]).
    public static let radii: [Double] = [0.055, 0.050, 0.045, 0.035, 0.030, 0.020]

    public struct Capsule {
        public var a: Vec3
        public var b: Vec3
        public var r: Double
    }

    /// The capsule skeleton at joint angles q, in the base frame: the six
    /// joint origins plus the tool point, exactly the host's joint_points(q).
    public static func capsules(q: [Double]) -> [Capsule] {
        let fk = ArmModel.forward(q, withFrames: true)
        var points: [Vec3] = fk.frames.map { $0.position }
        points.append(fk.position)
        var out: [Capsule] = []
        for index in 1..<points.count {
            out.append(Capsule(a: points[index - 1], b: points[index],
                               r: radii[min(index - 1, radii.count - 1)]))
        }
        return out
    }

    /// Keep the scene points that could belong to the arm: within `radius`
    /// horizontally of the tapped base point and in a plausible height band.
    public static func segment(points: [Vec3], near initial: Vec3,
                               radius: Double = 0.55) -> [Vec3] {
        points.filter { point in
            let dx = point[0] - initial[0]
            let dy = point[1] - initial[1]
            let dz = point[2] - initial[2]
            return dx * dx + dy * dy <= radius * radius && dz > -0.08 && dz < 0.65
        }
    }

    // MARK: fit

    struct State {
        var x: Double, y: Double, z: Double, yaw: Double
    }

    /// Distance from a point to a capsule's axis segment.
    static func axisDistance(_ p: Vec3, _ c: Capsule) -> Double {
        let ab = V3.sub(c.b, c.a)
        let ap = V3.sub(p, c.a)
        let len2 = V3.dot(ab, ab)
        var t = len2 > 1e-12 ? V3.dot(ap, ab) / len2 : 0
        t = max(0, min(1, t))
        let closest = V3.add(c.a, V3.scale(ab, t))
        return V3.norm(V3.sub(p, closest))
    }

    /// World point -> candidate base frame, then surface residual against the
    /// nearest capsule. Gravity alignment: rotation is yaw only.
    static func residual(_ p: Vec3, _ s: State, _ caps: [Capsule]) -> (value: Double, link: Int) {
        let c = cos(s.yaw), sn = sin(s.yaw)
        let dx = p[0] - s.x, dy = p[1] - s.y, dz = p[2] - s.z
        // Rz(-yaw) * (p - t)
        let pb: Vec3 = [c * dx + sn * dy, -sn * dx + c * dy, dz]
        var best = Double.greatestFiniteMagnitude
        var link = 0
        for (index, cap) in caps.enumerated() {
            let value = axisDistance(pb, cap) - cap.r
            if abs(value) < abs(best) {
                best = value
                link = index
            }
        }
        return (best, link)
    }

    static func trimmedResiduals(_ s: State, _ points: [Vec3], _ caps: [Capsule],
                                 gate: Double) -> (rms: Double, kept: [Int], links: [Int: Int], lever: Double) {
        var sum = 0.0
        var kept: [Int] = []
        var links: [Int: Int] = [:]
        var lever = 0.0
        let c = cos(s.yaw), sn = sin(s.yaw)
        for (index, point) in points.enumerated() {
            let r = residual(point, s, caps)
            if abs(r.value) <= gate {
                sum += r.value * r.value
                kept.append(index)
                links[r.link, default: 0] += 1
                let dx = point[0] - s.x, dy = point[1] - s.y
                let px = c * dx + sn * dy
                let py = -sn * dx + c * dy
                lever = max(lever, (px * px + py * py).squareRoot())
            }
        }
        let rms = kept.isEmpty ? Double.greatestFiniteMagnitude : (sum / Double(kept.count)).squareRoot()
        return (rms, kept, links, lever)
    }

    /// Damped Gauss-Newton over (x, y, z, yaw) with a trimming gate.
    /// `freeYaw: false` re-optimizes translation only (the yaw probe).
    static func optimize(_ start: State, _ points: [Vec3], _ caps: [Capsule],
                         iterations: Int, gate: Double, freeYaw: Bool = true) -> State {
        let parameters = freeYaw ? 4 : 3
        var s = start
        var damping = 1e-6
        var current = trimmedResiduals(s, points, caps, gate: gate).rms
        let epsilon = 1e-6
        for _ in 0..<iterations {
            if current < 1e-5 { break }
            // Numerical Jacobian over the kept points only.
            let kept = trimmedResiduals(s, points, caps, gate: gate).kept
            if kept.count < 20 { break }
            var normal = [[Double]](repeating: [Double](repeating: 0, count: parameters), count: parameters)
            var rhs = [Double](repeating: 0, count: parameters)
            for index in kept {
                let point = points[index]
                let base = residual(point, s, caps).value
                var jac = [Double](repeating: 0, count: parameters)
                for parameter in 0..<parameters {
                    var trial = s
                    switch parameter {
                    case 0: trial.x += epsilon
                    case 1: trial.y += epsilon
                    case 2: trial.z += epsilon
                    default: trial.yaw += epsilon
                    }
                    jac[parameter] = (residual(point, trial, caps).value - base) / epsilon
                }
                for a in 0..<parameters {
                    for b in 0..<parameters { normal[a][b] += jac[a] * jac[b] }
                    rhs[a] -= jac[a] * base
                }
            }
            for a in 0..<parameters { normal[a][a] += damping }
            guard let step = solveLinear(normal, rhs) else {
                damping *= 10
                if damping > 100 { break }
                continue
            }
            var accepted = false
            var factor = 1.0
            for _ in 0..<6 {
                let trial = State(x: s.x + step[0] * factor, y: s.y + step[1] * factor,
                                  z: s.z + step[2] * factor,
                                  yaw: s.yaw + (freeYaw ? step[3] * factor : 0))
                let trialRms = trimmedResiduals(trial, points, caps, gate: gate).rms
                if trialRms < current {
                    s = trial
                    current = trialRms
                    damping = max(1e-9, damping / 2)
                    accepted = true
                    break
                }
                factor *= 0.5
            }
            if !accepted {
                damping *= 10
                if damping > 100 { break }
            }
        }
        return s
    }

    /// Fit the base frame to scene points. `initial` is the tapped base point
    /// in the ARKit world frame (a depth raycast); the fit stays near it.
    public static func fit(points: [Vec3], q: [Double], initial: Vec3) -> DepthFitResult {
        func refuse(_ reason: String) -> DepthFitResult {
            DepthFitResult(ok: false, reason: reason, TBaseWorld: nil, rms: 0, inliers: 0, linksCovered: 0, yawLever: 0)
        }
        guard points.count >= 60 else {
            return refuse("too few arm points (\(points.count)); move closer or add light/texture")
        }
        let caps = capsules(q: q)

        // Multi-start over yaw: the tap fixes x/y roughly, heading is unknown.
        var best: State?
        var bestRms = Double.greatestFiniteMagnitude
        for seed in 0..<8 {
            let start = State(x: initial[0], y: initial[1], z: initial[2], yaw: Double(seed) * .pi / 4)
            let s = optimize(start, points, caps, iterations: 20, gate: 0.05)
            let rms = trimmedResiduals(s, points, caps, gate: 0.05).rms
            if rms < bestRms {
                bestRms = rms
                best = s
            }
        }
        guard var s = best else { return refuse("the fit did not converge") }
        // Tighten the trimming gate and polish.
        s = optimize(s, points, caps, iterations: 30, gate: 0.035)
        let final = trimmedResiduals(s, points, caps, gate: 0.035)

        let linksCovered = final.links.values.filter { $0 >= 5 }.count
        var result = DepthFitResult(ok: false, reason: nil, TBaseWorld: nil,
                                    rms: final.rms, inliers: final.kept.count,
                                    linksCovered: linksCovered, yawLever: final.lever)
        if final.kept.count < 60 {
            result.reason = "only \(final.kept.count) points fit the arm; the mesh may be too sparse"
            return result
        }
        if final.rms > 0.02 {
            result.reason = String(format: "residual %.1f mm is too large; hold the arm still and retry", final.rms * 1000)
            return result
        }
        if linksCovered < 3 {
            result.reason = "only \(linksCovered) links visible; show more of the arm to the camera"
            return result
        }
        if final.lever < 0.10 {
            result.reason = "the arm is too folded to fix its heading; extend it and retry"
            return result
        }
        // Yaw observability: perturb the heading, re-optimize translation, and
        // require the cost to rise. A near-axis column of points can slide
        // along the model at almost no cost — that is not a heading fix.
        var probeRise = Double.greatestFiniteMagnitude
        for delta in [0.05, -0.05] {
            var probe = s
            probe.yaw += delta
            probe = optimize(probe, points, caps, iterations: 12, gate: 0.035, freeYaw: false)
            let probeRms = trimmedResiduals(probe, points, caps, gate: 0.035).rms
            probeRise = min(probeRise, probeRms - final.rms)
        }
        if probeRise < 0.002 {
            result.reason = "the heading is not observable from these points; show more of the arm"
            return result
        }
        let dx = s.x - initial[0], dy = s.y - initial[1]
        if (dx * dx + dy * dy).squareRoot() > 0.15 || abs(s.z - initial[2]) > 0.25 {
            result.reason = "the fit ran away from the tapped point; tap the base itself"
            return result
        }
        result.ok = true
        result.TBaseWorld = M4.make(M3.rotZ(s.yaw), [s.x, s.y, s.z])
        return result
    }
}

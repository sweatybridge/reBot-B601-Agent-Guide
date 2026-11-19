/* ArUco pose solve, ported 1:1 from web/aruco.js.
 *
 * The tag corners become `T_base<-cam` through a homography seed plus physical
 * seeds, refined by damped Gauss-Newton against the fitted lens distortion.
 * The mirror twin (camera below the worktop) is refused.
 */
import Foundation

public struct MarkerConfig: Codable, Equatable {
    public var sizeM: Double
    public var at: [Double]
    public var yawDeg: Double?
    public var id: Int?
    public var dictionary: String?

    enum CodingKeys: String, CodingKey {
        case sizeM = "size_m"
        case at
        case yawDeg = "yaw_deg"
        case id
        case dictionary
    }

    public init(sizeM: Double, at: [Double], yawDeg: Double?, id: Int? = 0, dictionary: String? = "4x4_50") {
        self.sizeM = sizeM
        self.at = at
        self.yawDeg = yawDeg
        self.id = id
        self.dictionary = dictionary
    }
}

public struct TagPose {
    public var TBaseCam: Mat4
    public var rms: Double
    public var faceOn: Double
}

public enum ArucoPose {
    /// `baseFromMarker`: right = up x normal, marker +z out of the printed face.
    public static func baseFromMarker(_ marker: MarkerConfig) -> Pose {
        let yaw = (marker.yawDeg ?? 0) * .pi / 180
        let normal: Vec3 = [cos(yaw), sin(yaw), 0]
        let up: Vec3 = [0, 0, 1]
        let right = V3.cross(up, normal)
        let rotation: Mat3 = [[right[0], up[0], normal[0]],
                              [right[1], up[1], normal[1]],
                              [right[2], up[2], normal[2]]]
        return Pose(rotation: rotation, translation: marker.at)
    }

    /// The same order as `base_marker.object_points`: TL, TR, BR, BL.
    public static func objectCorners(size: Double) -> [Vec3] {
        let half = size / 2
        return [[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]]
    }

    static func projectPixel(_ point: Vec3, rotation: Mat3, translation: Vec3, camera: CameraIntrinsics) -> (Double, Double)? {
        let cameraPoint = V3.add(M3.vec(rotation, point), translation)
        if cameraPoint[2] <= 1e-9 { return nil }
        let x = cameraPoint[0] / cameraPoint[2]
        let y = cameraPoint[1] / cameraPoint[2]
        let r2 = x * x + y * y
        let radial = 1 + camera.k1 * r2 + camera.k2 * r2 * r2 + camera.k3 * r2 * r2 * r2
        let xd = x * radial + 2 * camera.p1 * x * y + camera.p2 * (r2 + 2 * x * x)
        let yd = y * radial + camera.p1 * (r2 + 2 * y * y) + 2 * camera.p2 * x * y
        return (camera.fx * xd + camera.cx, camera.fy * yd + camera.cy)
    }

    /// DLT from object (z=0) to normalized image coordinates.
    public static func poseFromHomography(object: [Vec3], image: [(Double, Double)]) -> Pose? {
        var matrix: [[Double]] = []
        var vector: [Double] = []
        for index in 0..<object.count {
            let x = object[index][0], y = object[index][1]
            let u = image[index].0, v = image[index].1
            matrix.append([x, y, 1, 0, 0, 0, -u * x, -u * y])
            vector.append(u)
            matrix.append([0, 0, 0, x, y, 1, -v * x, -v * y])
            vector.append(v)
        }
        guard let h = solveLinear(matrix, vector) else { return nil }
        var r1: Vec3 = [h[0], h[3], h[6]]
        var r2: Vec3 = [h[1], h[4], h[7]]
        var t: Vec3 = [h[2], h[5], 1]
        let lambda = 2 / (V3.norm(r1) + V3.norm(r2))
        r1 = V3.scale(r1, lambda)
        r2 = V3.scale(r2, lambda)
        t = V3.scale(t, lambda)
        if t[2] < 0 {
            r1 = V3.scale(r1, -1)
            r2 = V3.scale(r2, -1)
            t = V3.scale(t, -1)
        }
        let xAxis = V3.unit(r1)
        let yAxis = V3.unit(V3.sub(r2, V3.scale(xAxis, V3.dot(r2, xAxis))))
        let zAxis = V3.cross(xAxis, yAxis)
        return Pose(rotation: [[xAxis[0], yAxis[0], zAxis[0]],
                               [xAxis[1], yAxis[1], zAxis[1]],
                               [xAxis[2], yAxis[2], zAxis[2]]], translation: t)
    }

    struct Refined {
        var rotation: Mat3
        var translation: Vec3
        var rms: Double
    }

    /// Damped Gauss-Newton over (rvec, tvec) against the four observed corners.
    public static func refinePose(rotation: Mat3, translation: Vec3, object: [Vec3],
                                  observed: [(Double, Double)], camera: CameraIntrinsics) -> (rotation: Mat3, translation: Vec3, rms: Double)? {
        var rvec = M3.rotationToVector(rotation)
        var tvec = translation
        var damping = 1e-6

        func residuals(_ r: Vec3, _ t: Vec3) -> [Double]? {
            let current = M3.rodrigues(r)
            var out = [Double](repeating: 0, count: 8)
            for index in 0..<object.count {
                guard let point = projectPixel(object[index], rotation: current, translation: t, camera: camera) else { return nil }
                out[2 * index] = point.0 - observed[index].0
                out[2 * index + 1] = point.1 - observed[index].1
            }
            return out
        }

        func rmsOf(_ error: [Double]) -> Double {
            var sum = 0.0
            for value in error { sum += value * value }
            return (sum / Double(error.count)).squareRoot()
        }

        guard var error = residuals(rvec, tvec) else { return nil }
        var rms = rmsOf(error)

        for _ in 0..<20 {
            if rms < 1e-4 { break }
            var jacobian = [[Double]](repeating: [Double](repeating: 0, count: 6), count: 8)
            let epsilon = 1e-6
            for parameter in 0..<6 {
                var trialR = rvec
                var trialT = tvec
                if parameter < 3 { trialR[parameter] += epsilon } else { trialT[parameter - 3] += epsilon }
                guard let trialError = residuals(trialR, trialT) else { return nil }
                for row in 0..<8 {
                    jacobian[row][parameter] = (trialError[row] - error[row]) / epsilon
                }
            }

            var normal = [[Double]](repeating: [Double](repeating: 0, count: 6), count: 6)
            var rhs = [Double](repeating: 0, count: 6)
            for a in 0..<6 {
                for b in 0..<6 {
                    var sum = 0.0
                    for row in 0..<8 { sum += jacobian[row][a] * jacobian[row][b] }
                    normal[a][b] = sum
                }
                var value = 0.0
                for row in 0..<8 { value -= jacobian[row][a] * error[row] }
                rhs[a] = value
            }
            for a in 0..<6 { normal[a][a] += damping }
            guard let step = solveLinear(normal, rhs) else {
                damping *= 10
                if damping > 100 { return nil }
                continue
            }

            var accepted = false
            var factor = 1.0
            for _ in 0..<6 {
                let trialR = rvec.enumerated().map { $0.element + step[$0.offset] * factor }
                let trialT = tvec.enumerated().map { $0.element + step[$0.offset + 3] * factor }
                if let trialError = residuals(trialR, trialT) {
                    let trialRms = rmsOf(trialError)
                    if trialRms < rms {
                        rvec = trialR
                        tvec = trialT
                        error = trialError
                        rms = trialRms
                        damping = max(1e-9, damping / 2)
                        accepted = true
                        break
                    }
                }
                factor *= 0.5
            }
            if !accepted {
                damping *= 10
                if damping > 100 { return nil }
            }
        }
        return (M3.rodrigues(rvec), tvec, rms)
    }

    struct Candidate {
        var rotation: Mat3
        var translation: Vec3
        var lookAt = false
        var upright = false
    }

    static func poseCandidates(corners: [(Double, Double)], object: [Vec3],
                               camera: CameraIntrinsics, marker: MarkerConfig) -> [Candidate] {
        let normalized = corners.map { CameraGeometry.undistort(u: $0.0, v: $0.1, camera: camera) }
        var candidates: [Candidate] = []
        if let homography = poseFromHomography(object: object, image: normalized) {
            candidates.append(Candidate(rotation: homography.rotation, translation: homography.translation))
        }

        // Physical seeds: a camera in front of the tag and above the table.
        let base = baseFromMarker(marker)
        let normal: Vec3 = [base.rotation[0][2], base.rotation[1][2], base.rotation[2][2]]
        let TBaseMarker = M4.make(base.rotation, base.translation)
        for distance in [0.5, 0.8, 1.2, 1.6] {
            for height in [0.2, 0.45, 0.75] {
                let position: Vec3 = [
                    marker.at[0] + normal[0] * distance,
                    marker.at[1] + normal[1] * distance,
                    marker.at[2] + height,
                ]
                let forward = V3.unit(V3.sub(marker.at, position))
                let right = V3.unit(V3.cross(forward, [0, 0, 1]))
                let down = V3.cross(forward, right)
                let RBaseCam: Mat3 = [[right[0], down[0], forward[0]],
                                      [right[1], down[1], forward[1]],
                                      [right[2], down[2], forward[2]]]
                let TBaseCam = M4.make(RBaseCam, position)
                let TCamMarker = M4.mul(M4.invert(TBaseCam), TBaseMarker)
                candidates.append(Candidate(rotation: M4.rotation(TCamMarker),
                                            translation: M4.translation(TCamMarker), lookAt: true))
            }
        }

        // A fronto-parallel seed at the apparent distance; covers a sideways phone.
        var meanSide = 0.0
        for index in 0..<4 {
            let next = corners[(index + 1) % 4]
            let dx = next.0 - corners[index].0
            let dy = next.1 - corners[index].1
            meanSide += (dx * dx + dy * dy).squareRoot()
        }
        meanSide /= 4
        if meanSide > 1 {
            let distance = max(0.15, min(4.0, camera.fx * marker.sizeM / meanSide))
            let upright: Mat3 = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
            for factor in [0.55, 0.7, 0.85, 1.0] {
                candidates.append(Candidate(rotation: upright,
                                            translation: [0, 0, min(4.0, distance * factor)], upright: true))
            }
        }
        return candidates
    }

    /// The full solve: refine every candidate, refuse the mirror twin below the
    /// worktop, prefer a good look-at / upright seed, else the best fallback.
    public static func poseFromCorners(corners: [(Double, Double)], calibration: Calibration,
                                       marker: MarkerConfig, planeZ: Double?) -> TagPose? {
        let camera = CameraIntrinsics(calibration: calibration)
        let object = objectCorners(size: marker.sizeM)
        let candidates = poseCandidates(corners: corners, object: object, camera: camera, marker: marker)
        let base = baseFromMarker(marker)
        let TBaseMarker = M4.make(base.rotation, base.translation)
        let floor = (planeZ ?? 0) + 0.05

        struct Entry {
            var T: Mat4
            var rms: Double
            var faceOn: Double
        }
        var lookAt: Entry?
        var upright: Entry?
        var fallback: Entry?
        for candidate in candidates {
            guard let refined = refinePose(rotation: candidate.rotation, translation: candidate.translation,
                                           object: object, observed: corners, camera: camera) else { continue }
            if refined.rms > 4.0 { continue }
            let normal: Vec3 = [refined.rotation[0][2], refined.rotation[1][2], refined.rotation[2][2]]
            let faceOn = V3.dot(normal, V3.unit(V3.scale(refined.translation, -1)))
            if faceOn <= 0 { continue }

            let inverse = M4.invert(M4.make(refined.rotation, refined.translation))
            let T = M4.mul(TBaseMarker, inverse)
            // The phone is above the table; the mirror twin is not the phone.
            let above = T[2][3] > floor && T[2][3] < floor + 3.0
            if !above { continue }
            let entry = Entry(T: T, rms: refined.rms, faceOn: faceOn)
            if candidate.lookAt {
                if lookAt == nil || entry.rms < lookAt!.rms { lookAt = entry }
            } else if candidate.upright {
                if upright == nil || entry.rms < upright!.rms { upright = entry }
            } else if fallback == nil || entry.rms < fallback!.rms {
                fallback = entry
            }
        }
        let best: Entry?
        if let lookAt, lookAt.rms <= 1.0 { best = lookAt }
        else if let upright, upright.rms <= 1.0 { best = upright }
        else { best = fallback ?? lookAt ?? upright }
        guard let best else { return nil }
        return TagPose(TBaseCam: best.T, rms: best.rms, faceOn: best.faceOn)
    }
}

/// The per-frame pipeline of `ArucoPose.create().detect()`: detect on a
/// downscaled frame, refine the corners at full resolution, solve the pose on
/// the calibration grid. The caller supplies both gray renditions.
public struct ArucoEyeResult {
    public var TBaseCam: Mat4
    public var corners: [(Double, Double)]   // full-resolution frame pixels
    public var rmsPx: Double
    public var detectionSize: [Int]
}

public enum ArucoEye {
    public static func detect(full: GrayImage, downscaled: GrayImage,
                              calibration: Calibration, marker: MarkerConfig,
                              planeZ: Double?) -> ArucoEyeResult? {
        if let id = marker.id, id != 0 { return nil }
        guard let found = ArucoDetector.detectFrame(downscaled) else { return nil }
        let sx = Double(full.width) / Double(downscaled.width)
        let sy = Double(full.height) / Double(downscaled.height)
        var corners = found.corners.map { ($0.0 * sx, $0.1 * sy) }

        // Refine the four edges once at full resolution in a crop around the tag.
        var minX = Double(full.width), maxX = 0.0, minY = Double(full.height), maxY = 0.0
        for point in corners {
            minX = min(minX, point.0)
            maxX = max(maxX, point.0)
            minY = min(minY, point.1)
            maxY = max(maxY, point.1)
        }
        let pad = max(14, Int((max(maxX - minX, maxY - minY) * 0.6).rounded()))
        let x0 = max(0, Int(minX.rounded(.down)) - pad)
        let y0 = max(0, Int(minY.rounded(.down)) - pad)
        let x1 = min(full.width, Int(maxX.rounded(.up)) + pad)
        let y1 = min(full.height, Int(maxY.rounded(.up)) + pad)
        let cropWidth = x1 - x0
        let cropHeight = y1 - y0
        if cropWidth > 8 && cropHeight > 8 {
            var crop = [UInt8](repeating: 0, count: cropWidth * cropHeight)
            for y in 0..<cropHeight {
                let sourceRow = (y0 + y) * full.width + x0
                for x in 0..<cropWidth {
                    crop[y * cropWidth + x] = full.data[sourceRow + x]
                }
            }
            let cropGray = GrayImage(data: crop, width: cropWidth, height: cropHeight)
            let local = corners.map { ($0.0 - Double(x0), $0.1 - Double(y0)) }
            if let refined = ArucoDetector.refineQuad(cropGray, quad: local) {
                corners = refined.map { ($0.0 + Double(x0), $0.1 + Double(y0)) }
            }
        }

        // Corners are in the frame's pixels; K is for the calibration's size.
        let source = calibration.imageSize
        let scaleX = Double(source[0]) / Double(full.width)
        let scaleY = Double(source[1]) / Double(full.height)
        if abs(scaleX - scaleY) / max(scaleX, scaleY) > 0.01 { return nil }
        let poseCorners = corners.map { ($0.0 * scaleX, $0.1 * scaleY) }
        guard let pose = ArucoPose.poseFromCorners(corners: poseCorners, calibration: calibration,
                                                   marker: marker, planeZ: planeZ) else { return nil }
        return ArucoEyeResult(TBaseCam: pose.TBaseCam, corners: corners, rmsPx: pose.rms,
                              detectionSize: [downscaled.width, downscaled.height])
    }
}

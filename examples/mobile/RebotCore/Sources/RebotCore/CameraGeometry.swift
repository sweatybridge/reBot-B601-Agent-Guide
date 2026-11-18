/* Camera calibration, Brown-Conrady projection and ray/plane math.
 * Ported 1:1 from web/geometry.js (ReachGeometry).
 */
import Foundation

/// The calibration file the host hands out in /reach.json.
public struct Calibration: Codable, Equatable {
    public var K: [[Double]]
    public var dist: [Double]?
    public var imageSize: [Int]
    public var fingerprint: String?
    public var rms: Double?
    public var orientationDeg: Double?

    enum CodingKeys: String, CodingKey {
        case K
        case dist
        case imageSize = "image_size"
        case fingerprint
        case rms
        case orientationDeg = "orientation_deg"
    }

    public init(K: [[Double]], dist: [Double]?, imageSize: [Int],
                fingerprint: String? = nil, rms: Double? = nil, orientationDeg: Double? = nil) {
        self.K = K
        self.dist = dist
        self.imageSize = imageSize
        self.fingerprint = fingerprint
        self.rms = rms
        self.orientationDeg = orientationDeg
    }
}

/// Scaled pinhole + distortion coefficients, the `makeCamera` result.
public struct CameraIntrinsics: Equatable {
    public var fx: Double, fy: Double, cx: Double, cy: Double
    public var k1: Double, k2: Double, p1: Double, p2: Double, k3: Double

    public init(fx: Double, fy: Double, cx: Double, cy: Double,
                k1: Double = 0, k2: Double = 0, p1: Double = 0, p2: Double = 0, k3: Double = 0) {
        self.fx = fx; self.fy = fy; self.cx = cx; self.cy = cy
        self.k1 = k1; self.k2 = k2; self.p1 = p1; self.p2 = p2; self.k3 = k3
    }

    public init(calibration: Calibration) {
        let K = calibration.K
        let dist = calibration.dist ?? []
        func d(_ i: Int) -> Double { i < dist.count ? dist[i] : 0 }
        self.init(fx: K[0][0], fy: K[1][1], cx: K[0][2], cy: K[1][2],
                  k1: d(0), k2: d(1), p1: d(2), p2: d(3), k3: d(4))
    }
}

public struct Pixel: Equatable {
    public var u: Double
    public var v: Double
    public init(u: Double, v: Double) { self.u = u; self.v = v }
}

public struct Pose {
    public var rotation: Mat3
    public var translation: Vec3
    public init(rotation: Mat3, translation: Vec3) {
        self.rotation = rotation
        self.translation = translation
    }
    public init(T: Mat4) {
        self.rotation = M4.rotation(T)
        self.translation = M4.translation(T)
    }
}

public enum GeometryError: Error, CustomStringConvertible {
    case message(String)
    public var description: String {
        switch self { case .message(let text): return text }
    }
}

public enum CameraGeometry {
    /// `makeCamera`: scale K onto the delivered frame, refusing a crop.
    public static func makeCamera(_ calibration: Calibration, videoWidth: Int, videoHeight: Int) throws -> CameraIntrinsics {
        let source = calibration.imageSize
        guard source.count == 2 else { throw GeometryError.message("the calibration has no image_size") }
        let sx = Double(videoWidth) / Double(source[0])
        let sy = Double(videoHeight) / Double(source[1])
        if abs(sx - sy) / max(sx, sy) > 0.01 {
            throw GeometryError.message("calibration is for \(source[0])x\(source[1]) but the camera is \(videoWidth)x\(videoHeight); recalibrate at this size (the planner would refuse it too)")
        }
        let K = calibration.K
        let dist = calibration.dist ?? []
        func d(_ i: Int) -> Double { i < dist.count ? dist[i] : 0 }
        return CameraIntrinsics(fx: K[0][0] * sx, fy: K[1][1] * sy,
                                cx: K[0][2] * sx, cy: K[1][2] * sy,
                                k1: d(0), k2: d(1), p1: d(2), p2: d(3), k3: d(4))
    }

    /// `project`: T_base<-cam pose, base-frame point -> distorted pixel, exactly
    /// as cv2.projectPoints. nil when the point is behind the camera.
    public static func project(_ camera: CameraIntrinsics, pose: Pose, point: Vec3) -> Pixel? {
        let rotation = pose.rotation
        let translation = pose.translation
        let dx = point[0] - translation[0]
        let dy = point[1] - translation[1]
        let dz = point[2] - translation[2]
        // `rotation` is T_base<-cam, so the camera sees R^T (p - t).
        let x = rotation[0][0] * dx + rotation[1][0] * dy + rotation[2][0] * dz
        let y = rotation[0][1] * dx + rotation[1][1] * dy + rotation[2][1] * dz
        let z = rotation[0][2] * dx + rotation[1][2] * dy + rotation[2][2] * dz
        if z <= 1e-6 { return nil }
        let xn = x / z, yn = y / z
        let r2 = xn * xn + yn * yn
        let radial = 1 + camera.k1 * r2 + camera.k2 * r2 * r2 + camera.k3 * r2 * r2 * r2
        let xd = xn * radial + 2 * camera.p1 * xn * yn + camera.p2 * (r2 + 2 * xn * xn)
        let yd = yn * radial + camera.p1 * (r2 + 2 * yn * yn) + 2 * camera.p2 * xn * yn
        return Pixel(u: camera.fx * xd + camera.cx, v: camera.fy * yd + camera.cy)
    }

    /// `scaleCalibration`: a proportional resize is a rescale of K.
    public static func scaleCalibration(_ calibration: Calibration, width: Int, height: Int) throws -> Calibration {
        let source = calibration.imageSize
        guard source.count == 2 else { throw GeometryError.message("the calibration has no image_size") }
        let sx = Double(width) / Double(source[0])
        let sy = Double(height) / Double(source[1])
        if abs(sx - sy) / max(sx, sy) > 0.01 {
            throw GeometryError.message("calibration is for \(source[0])x\(source[1]) but the frame is \(width)x\(height)")
        }
        if abs(sx - 1) < 1e-9 && abs(sy - 1) < 1e-9 { return calibration }
        let K = calibration.K
        return Calibration(
            K: [
                [K[0][0] * sx, K[0][1] * sx, K[0][2] * sx],
                [K[1][0] * sy, K[1][1] * sy, K[1][2] * sy],
                [K[2][0], K[2][1], K[2][2]],
            ],
            dist: calibration.dist ?? [],
            imageSize: [width, height],
            fingerprint: calibration.fingerprint,
            rms: calibration.rms,
            orientationDeg: calibration.orientationDeg)
    }

    /// `shapeTurn`: the correction guess from the frame shape. 3 means "assume
    /// one clockwise turn"; -1 means "not a quarter turn at all".
    public static func shapeTurn(calibrationSize: [Int]?, frameSize: [Int]?) -> Int {
        guard let calibrationSize, let frameSize, calibrationSize.count == 2, frameSize.count == 2 else { return 0 }
        let cw = Double(calibrationSize[0]), ch = Double(calibrationSize[1])
        let fw = Double(frameSize[0]), fh = Double(frameSize[1])
        if !(cw > 0 && ch > 0 && fw > 0 && fh > 0) { return -1 }
        let same = abs((cw / ch) / (fw / fh) - 1) <= 0.01
        let swapped = abs((cw / ch) / (fh / fw) - 1) <= 0.01 && abs(cw / ch - ch / cw) > 0.01
        if swapped { return 3 }
        if same { return 0 }
        return -1
    }

    /// `mapPixel`: apply `turns` clockwise quarter turns to a pixel.
    public static func mapPixel(u: Double, v: Double, size: [Int], turns: Int) -> (Double, Double) {
        var width = Double(size[0])
        var height = Double(size[1])
        var x = u, y = v
        var steps = ((turns % 4) + 4) % 4
        while steps > 0 {
            let next = (height - y, x)
            x = next.0
            y = next.1
            let size2 = (height, width)
            width = size2.0
            height = size2.1
            steps -= 1
        }
        return (x, y)
    }

    /// `undistort`: fixed-point inversion of the Brown-Conrady model.
    public static func undistort(u: Double, v: Double, camera: CameraIntrinsics) -> (Double, Double) {
        let xd = (u - camera.cx) / camera.fx
        let yd = (v - camera.cy) / camera.fy
        var x = xd, y = yd
        for _ in 0..<6 {
            let r2 = x * x + y * y
            let radial = 1 + camera.k1 * r2 + camera.k2 * r2 * r2 + camera.k3 * r2 * r2 * r2
            let dx = 2 * camera.p1 * x * y + camera.p2 * (r2 + 2 * x * x)
            let dy = camera.p1 * (r2 + 2 * y * y) + 2 * camera.p2 * x * y
            x = (xd - dx) / radial
            y = (yd - dy) / radial
        }
        return (x, y)
    }

    public struct RayHit {
        public var point: Vec3
        public var distance: Double
        public var grazing: Double
    }

    /// `rayPlane`: intersect the undistorted tap ray with a horizontal plane.
    public static func rayPlane(camera: CameraIntrinsics, T: Mat4, u: Double, v: Double, planeZ: Double) throws -> RayHit {
        let (x, y) = undistort(u: u, v: v, camera: camera)
        let directionCam: Vec3 = [x, y, 1.0]
        let origin = M4.translation(T)
        var direction = M3.vec(M4.rotation(T), directionCam)
        let norm = V3.norm(direction)
        if !(norm > 1e-12) { throw GeometryError.message("degenerate tap ray") }
        direction = V3.scale(direction, 1 / norm)
        if abs(direction[2]) < 1e-6 {
            throw GeometryError.message("the tap ray is parallel to the worktop")
        }
        let distance = (planeZ - origin[2]) / direction[2]
        if !(distance > 0) {
            throw GeometryError.message("the tap ray points away from the worktop")
        }
        return RayHit(
            point: V3.add(origin, V3.scale(direction, distance)),
            distance: distance,
            grazing: asin(min(1, abs(direction[2]))) * 180 / .pi)
    }
}

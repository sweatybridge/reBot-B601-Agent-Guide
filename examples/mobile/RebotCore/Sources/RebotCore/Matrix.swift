/* Small matrix / vector math shared by the camera model, the ArUco pose solve
 * and the arm kinematics. Ported 1:1 from web/geometry.js and web/aruco.js so
 * the numbers match the page the host already trusts.
 *
 * Matrices are row-major nested arrays, exactly like the JS originals.
 */
import Foundation

public typealias Vec3 = [Double]
public typealias Mat3 = [[Double]]
public typealias Mat4 = [[Double]]

public enum M3 {
    public static func identity() -> Mat3 {
        [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    }

    public static func mul(_ a: Mat3, _ b: Mat3) -> Mat3 {
        var out: Mat3 = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        for i in 0..<3 {
            for j in 0..<3 {
                out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
            }
        }
        return out
    }

    public static func vec(_ a: Mat3, _ v: Vec3) -> Vec3 {
        [
            a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2],
            a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2],
            a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2],
        ]
    }

    public static func transpose(_ a: Mat3) -> Mat3 {
        [[a[0][0], a[1][0], a[2][0]],
         [a[0][1], a[1][1], a[2][1]],
         [a[0][2], a[1][2], a[2][2]]]
    }

    public static func rpyToMatrix(_ rpy: Vec3) -> Mat3 {
        let (roll, pitch, yaw) = (rpy[0], rpy[1], rpy[2])
        let cr = cos(roll), sr = sin(roll)
        let cp = cos(pitch), sp = sin(pitch)
        let cy = cos(yaw), sy = sin(yaw)
        let rx: Mat3 = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]
        let ry: Mat3 = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
        let rz: Mat3 = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]
        return mul(mul(rz, ry), rx)
    }

    public static func axisAngle(_ axis: Vec3, _ angle: Double) -> Mat3 {
        let (x, y, z) = (axis[0], axis[1], axis[2])
        let c = cos(angle), s = sin(angle), t = 1 - c
        return [
            [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
        ]
    }

    /// Axis-angle of `target @ current^T`, the same residual rebot_ik uses.
    public static func rotationError(current: Mat3, target: Mat3) -> Vec3 {
        let error = mul(target, transpose(current))
        let cosine = max(-1, min(1, (error[0][0] + error[1][1] + error[2][2] - 1) / 2))
        let angle = acos(cosine)
        if angle < 1e-9 { return [0, 0, 0] }
        let d = 2 * sin(angle)
        return [
            (error[2][1] - error[1][2]) / d * angle,
            (error[0][2] - error[2][0]) / d * angle,
            (error[1][0] - error[0][1]) / d * angle,
        ]
    }

    /// Shortest-path correction toward the tag, so a noisy 3 Hz pose does not
    /// snap the band.
    public static func blendOrientation(current: Mat3, target: Mat3, fraction: Double) -> Mat3 {
        let error = rotationError(current: current, target: target)
        let angle = hypot(hypot(error[0], error[1]), error[2])
        if !(angle > 1e-9) || !angle.isFinite { return current }
        let axis = [error[0] / angle, error[1] / angle, error[2] / angle]
        return mul(axisAngle(axis, angle * fraction), current)
    }

    public static func skew(_ v: Vec3) -> Mat3 {
        [[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]]
    }

    public static func rodrigues(_ vector: Vec3) -> Mat3 {
        let angle = V3.norm(vector)
        if angle < 1e-12 {
            // R = I + skew(v) to first order, which is what the refinement needs.
            let k = skew(vector)
            return [[1 + k[0][0], k[0][1], k[0][2]],
                    [k[1][0], 1 + k[1][1], k[1][2]],
                    [k[2][0], k[2][1], 1 + k[2][2]]]
        }
        let axis = V3.scale(vector, 1 / angle)
        let k = skew(axis)
        let s = sin(angle)
        let c = 1 - cos(angle)
        let k2 = mul(k, k)
        var rotation: Mat3 = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        for row in 0..<3 {
            for column in 0..<3 {
                rotation[row][column] = (row == column ? 1 : 0) + s * k[row][column] + c * k2[row][column]
            }
        }
        return rotation
    }

    public static func rotationToVector(_ R: Mat3) -> Vec3 {
        let cosine = max(-1, min(1, (R[0][0] + R[1][1] + R[2][2] - 1) / 2))
        let angle = acos(cosine)
        if angle < 1e-9 { return [0, 0, 0] }
        if angle > .pi - 1e-6 {
            // Near 180 degrees the antisymmetric part vanishes; use the diagonal.
            var axis = [
                (max(0, (R[0][0] + 1) / 2)).squareRoot(),
                (max(0, (R[1][1] + 1) / 2)).squareRoot(),
                (max(0, (R[2][2] + 1) / 2)).squareRoot(),
            ]
            if R[2][1] - R[1][2] < 0 { axis[0] = -axis[0] }
            if R[0][2] - R[2][0] < 0 { axis[1] = -axis[1] }
            if R[1][0] - R[0][1] < 0 { axis[2] = -axis[2] }
            return V3.scale(V3.unit(axis), angle)
        }
        let d = 2 * sin(angle)
        return [
            (R[2][1] - R[1][2]) / d * angle,
            (R[0][2] - R[2][0]) / d * angle,
            (R[1][0] - R[0][1]) / d * angle,
        ]
    }

    public static func rotZ(_ angle: Double) -> Mat3 {
        let c = cos(angle), s = sin(angle)
        return [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    }
}

public enum V3 {
    public static func add(_ a: Vec3, _ b: Vec3) -> Vec3 { [a[0] + b[0], a[1] + b[1], a[2] + b[2]] }
    public static func sub(_ a: Vec3, _ b: Vec3) -> Vec3 { [a[0] - b[0], a[1] - b[1], a[2] - b[2]] }
    public static func scale(_ a: Vec3, _ s: Double) -> Vec3 { [a[0] * s, a[1] * s, a[2] * s] }
    public static func dot(_ a: Vec3, _ b: Vec3) -> Double { a[0] * b[0] + a[1] * b[1] + a[2] * b[2] }
    public static func cross(_ a: Vec3, _ b: Vec3) -> Vec3 {
        [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
    }
    public static func norm(_ a: Vec3) -> Double { hypot(hypot(a[0], a[1]), a[2]) }
    public static func unit(_ a: Vec3) -> Vec3 {
        let length = norm(a)
        return length > 1e-12 ? scale(a, 1 / length) : [0, 0, 0]
    }
}

public enum M4 {
    public static func make(_ r: Mat3, _ t: Vec3) -> Mat4 {
        [[r[0][0], r[0][1], r[0][2], t[0]],
         [r[1][0], r[1][1], r[1][2], t[1]],
         [r[2][0], r[2][1], r[2][2], t[2]],
         [0, 0, 0, 1]]
    }

    public static func rotation(_ T: Mat4) -> Mat3 {
        [[T[0][0], T[0][1], T[0][2]],
         [T[1][0], T[1][1], T[1][2]],
         [T[2][0], T[2][1], T[2][2]]]
    }

    public static func translation(_ T: Mat4) -> Vec3 { [T[0][3], T[1][3], T[2][3]] }

    public static func invert(_ T: Mat4) -> Mat4 {
        let R = rotation(T)
        let t = translation(T)
        return [
            [R[0][0], R[1][0], R[2][0], -(R[0][0] * t[0] + R[1][0] * t[1] + R[2][0] * t[2])],
            [R[0][1], R[1][1], R[2][1], -(R[0][1] * t[0] + R[1][1] * t[1] + R[2][1] * t[2])],
            [R[0][2], R[1][2], R[2][2], -(R[0][2] * t[0] + R[1][2] * t[1] + R[2][2] * t[2])],
            [0, 0, 0, 1],
        ]
    }

    public static func mul(_ a: Mat4, _ b: Mat4) -> Mat4 {
        var out: Mat4 = Array(repeating: Array(repeating: 0.0, count: 4), count: 4)
        for row in 0..<4 {
            for column in 0..<4 {
                var value = 0.0
                for k in 0..<4 { value += a[row][k] * b[k][column] }
                out[row][column] = value
            }
        }
        return out
    }
}

/// Gauss-Jordan with partial pivoting. Returns nil on a singular system,
/// exactly like `solveLinear` in aruco.js.
public func solveLinear(_ matrix: [[Double]], _ vector: [Double]) -> [Double]? {
    let n = vector.count
    var rows = matrix.enumerated().map { index, row in row + [vector[index]] }
    for column in 0..<n {
        var pivot = column
        for row in (column + 1)..<n where abs(rows[row][column]) > abs(rows[pivot][column]) {
            pivot = row
        }
        if abs(rows[pivot][column]) < 1e-12 { return nil }
        if pivot != column { rows.swapAt(pivot, column) }
        let divisor = rows[column][column]
        for k in column...n { rows[column][k] /= divisor }
        for row in 0..<n {
            if row == column { continue }
            let factor = rows[row][column]
            if factor == 0 { continue }
            for k in column...n { rows[row][k] -= factor * rows[column][k] }
        }
    }
    return rows.map { $0[n] }
}

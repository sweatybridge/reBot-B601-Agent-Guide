/* Arm kinematics and the reach band, ported 1:1 from web/arm-model.js
 * (which is itself transcribed from app/rebot_ik.py / the Seeed URDF).
 */
import Foundation

public struct ReachBand: Equatable {
    public var inner: Double
    public var outer: Double
    public var sector: Double
    public var tipHeight: Double
    public var flangeZ: Double
}

public enum ArmModel {
    struct Joint {
        var xyz: Vec3
        var rpy: Vec3
        var axis: Vec3
        var lo: Double
        var hi: Double
    }

    static let chain: [Joint] = [
        Joint(xyz: [-0.00034283, -0.00098683, 0.075], rpy: [0, 0, 0], axis: [0, 0, -1], lo: -2.80, hi: 2.80),
        Joint(xyz: [0.020343, 0.027237, 0.07], rpy: [-1.5708, 0, 0], axis: [0, 0, 1], lo: 0, hi: 3.14),
        Joint(xyz: [-0.236, 0, 0], rpy: [0, 0, 0], axis: [0, 0, -1], lo: 0, hi: 3.14),
        Joint(xyz: [0.228, -0.072746, 0.0045], rpy: [0, 0, 0], axis: [0, 0, -1], lo: -1.57, hi: 1.57),
        Joint(xyz: [0.087, -0.048, -0.03075], rpy: [-1.5708, 0, 0], axis: [0, 0, -1], lo: -1.57, hi: 1.57),
        Joint(xyz: [0.0365, 0, 0.048], rpy: [0, 1.5708, 0], axis: [0, 0, -1], lo: -3.14, hi: 3.14),
    ]
    static let toolXYZ: Vec3 = [0, 0, 0.16621]
    static let toolRPY: Vec3 = [3.1416, -1.5708, 0]
    static let limits: [(Double, Double)] = chain.map { ($0.lo, $0.hi) }

    /// The planner's --max-reach gate.
    public static let maxReach = 0.75

    static let seeds: [[Double]] = [
        [0, 2.02, 0.52, 1.5, 0, 0],
        [0, 2.457, 1.246, 1.315, -0.376, 0.034],
        [0, 1.2, 0.2, 1.5, 0, 0],
        [0, 0, 0, 0, 0, 0],
    ]

    public struct FKResult {
        public var rotation: Mat3
        public var position: Vec3
        public var frames: [(position: Vec3, axis: Vec3)]
    }

    public static func forward(_ q: [Double], withFrames: Bool = false) -> FKResult {
        var rotation = M3.identity()
        var position: Vec3 = [0, 0, 0]
        var frames: [(position: Vec3, axis: Vec3)] = []
        for index in 0..<chain.count {
            let joint = chain[index]
            position = V3.add(position, M3.vec(rotation, joint.xyz))
            rotation = M3.mul(rotation, M3.rpyToMatrix(joint.rpy))
            if withFrames {
                frames.append((position: position, axis: M3.vec(rotation, joint.axis)))
            }
            rotation = M3.mul(rotation, M3.axisAngle(joint.axis, q[index]))
        }
        position = V3.add(position, M3.vec(rotation, toolXYZ))
        rotation = M3.mul(rotation, M3.rpyToMatrix(toolRPY))
        return FKResult(rotation: rotation, position: position, frames: frames)
    }

    static func withinLimits(_ q: [Double], margin: Double = 0) -> Bool {
        for index in 0..<limits.count {
            if q[index] < limits[index].0 + margin || q[index] > limits[index].1 - margin { return false }
        }
        return true
    }

    public struct IKInfo {
        public var ok: Bool
        public var posErr: Double
        public var rotErr: Double
    }

    /// The same damped least squares as `rebot_ik.ik()`.
    public static func inverseKinematics(targetPosition: Vec3, targetRotation: Mat3,
                                         seed: [Double]) -> (q: [Double], info: IKInfo) {
        let tolPosition = 1e-3
        let tolRotation = 2e-3
        let damping = 0.02
        let rotWeight = 0.5
        let maxStep = 0.15
        let low = limits.enumerated().map { min($0.element.0, seed[$0.offset]) }
        let high = limits.enumerated().map { max($0.element.1, seed[$0.offset]) }
        var q = seed
        var last = IKInfo(ok: false, posErr: 1, rotErr: 1)

        for _ in 0..<160 {
            let current = forward(q, withFrames: true)
            let posErr = V3.sub(targetPosition, current.position)
            let rotErr = M3.rotationError(current: current.rotation, target: targetRotation)
            let positionNorm = V3.norm(posErr)
            let rotationNorm = V3.norm(rotErr)
            last = IKInfo(ok: false, posErr: positionNorm, rotErr: rotationNorm)
            if positionNorm < tolPosition && rotationNorm < tolRotation {
                last.ok = true
                return (q, last)
            }

            // Geometric Jacobian, rows 0-2 linear and 3-5 angular.
            var jacobian = [[Double]](repeating: [Double](repeating: 0, count: 6), count: 6)
            for joint in 0..<6 {
                let frame = current.frames[joint]
                let d = V3.sub(current.position, frame.position)
                let linear = V3.cross(frame.axis, d)
                jacobian[0][joint] = linear[0]
                jacobian[1][joint] = linear[1]
                jacobian[2][joint] = linear[2]
                jacobian[3][joint] = frame.axis[0]
                jacobian[4][joint] = frame.axis[1]
                jacobian[5][joint] = frame.axis[2]
            }
            for row in 3..<6 {
                for joint in 0..<6 { jacobian[row][joint] *= rotWeight }
            }

            var system = [[Double]](repeating: [Double](repeating: 0, count: 6), count: 6)
            for row in 0..<6 {
                for column in 0..<6 {
                    var value = 0.0
                    for k in 0..<6 { value += jacobian[row][k] * jacobian[column][k] }
                    system[row][column] = value + (row == column ? damping * damping : 0)
                }
            }
            let errorVector = posErr + rotErr.map { $0 * rotWeight }
            guard let solved = solveLinear(system, errorVector) else { return (q, last) }

            var step = [Double](repeating: 0, count: 6)
            for joint in 0..<6 {
                for row in 0..<6 { step[joint] += jacobian[row][joint] * solved[row] }
            }
            let largest = step.map { abs($0) }.max() ?? 0
            if largest > maxStep {
                for joint in 0..<6 { step[joint] *= maxStep / largest }
            }

            let previous = positionNorm + rotWeight * rotationNorm
            for attempt in 0..<6 {
                let candidate = q.enumerated().map { index, value in
                    max(low[index], min(high[index], value + step[index]))
                }
                let trial = forward(candidate)
                let trialPosition = V3.norm(V3.sub(targetPosition, trial.position))
                let trialRotation = M3.rotationError(current: trial.rotation, target: targetRotation)
                let trialError = trialPosition + rotWeight * V3.norm(trialRotation)
                if trialError < previous || attempt == 5 {
                    q = candidate
                    break
                }
                for joint in 0..<6 { step[joint] *= 0.5 }
            }
        }
        return (q, last)
    }

    static func solveAt(radius: Double, flangeZ: Double, seed: [Double]) -> [Double]? {
        let target: Vec3 = [radius, 0, flangeZ]
        // Tool pointing straight down: the parked arm's orientation.
        let attempt = inverseKinematics(targetPosition: target, targetRotation: M3.identity(), seed: seed)
        if !attempt.info.ok || !withinLimits(attempt.q) { return nil }
        let actual = forward(attempt.q)
        let dx = actual.position[0] - radius
        let dy = actual.position[1]
        let dz = actual.position[2] - flangeZ
        if (dx * dx + dy * dy + dz * dz).squareRoot() > 0.003 { return nil }
        return attempt.q
    }

    /// One radial scan at +x describes every angle; joint 1's +-2.8 rad leaves
    /// the 40 deg wedge behind the base.
    public static func computeReachBand(planeZ: Double, hoverM: Double, tipM: Double) -> ReachBand? {
        let flangeZ = planeZ + hoverM + tipM
        let step = 0.01

        func reachable(_ radius: Double, seed: [Double]) -> [Double]? {
            if let q = solveAt(radius: radius, flangeZ: flangeZ, seed: seed) { return q }
            for candidate in seeds {
                if let q = solveAt(radius: radius, flangeZ: flangeZ, seed: candidate) { return q }
            }
            return nil
        }

        var anchor: Double?
        var seed = seeds[0]
        let anchors = [0.60, 0.50, 0.65, 0.45, 0.70, 0.40, 0.30, 0.20]
        for candidate in anchors {
            if let q = reachable(candidate, seed: seed) {
                anchor = candidate
                seed = q
                break
            }
        }
        guard let anchor else { return nil }

        var outer = anchor
        var outerSeed = seed
        var radius = anchor + step
        while radius <= 0.90 {
            guard let q = reachable(radius, seed: outerSeed) else { break }
            outer = radius
            outerSeed = q
            radius += step
        }

        var inner = anchor
        var innerSeed = seed
        radius = anchor - step
        while radius >= 0.02 {
            guard let q = reachable(radius, seed: innerSeed) else { break }
            inner = radius
            innerSeed = q
            radius -= step
        }

        return ReachBand(
            inner: max(0, inner),
            outer: min(outer, maxReach),
            sector: min(abs(limits[0].0), abs(limits[0].1)),
            tipHeight: planeZ + hoverM,
            flangeZ: flangeZ)
    }
}

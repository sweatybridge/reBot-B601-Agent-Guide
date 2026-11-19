/* Locate a tap in the arm frame, ported from web/locate.js `tipTarget`.
 *
 * The tip is the direct intersection of the tap ray with plane_z + height:
 * its x/y move along the ray as the height changes.
 */
import Foundation

public struct TipTarget: Codable, Equatable {
    public var tip: [Double]
    public var pixel: [Double]
    public var tap: TapPixel
    public var turn: Int

    public struct TapPixel: Codable, Equatable {
        public var u: Double
        public var v: Double
    }
}

public enum Locate {
    public static func tipTarget(calibration: Calibration, T: Mat4, u: Double, v: Double,
                                 planeZ: Double, tapHeight: Double,
                                 videoWidth: Int, videoHeight: Int, turn: Int?) throws -> TipTarget {
        let width = videoWidth > 0 ? videoWidth : calibration.imageSize[0]
        let heightPx = videoHeight > 0 ? videoHeight : calibration.imageSize[1]
        guard tapHeight.isFinite else { throw GeometryError.message("the tap height is not a number") }

        let raw = turn ?? max(0, CameraGeometry.shapeTurn(calibrationSize: calibration.imageSize,
                                                          frameSize: [width, heightPx]))
        let applied = ((raw % 4) + 4) % 4
        let source = applied != 0 ? calibration.imageSize : [width, heightPx]
        let at = applied != 0
            ? CameraGeometry.mapPixel(u: u, v: v, size: [width, heightPx], turns: applied)
            : (u, v)
        let camera = try CameraGeometry.makeCamera(calibration, videoWidth: source[0], videoHeight: source[1])
        let tip = try CameraGeometry.rayPlane(camera: camera, T: T, u: at.0, v: at.1,
                                              planeZ: planeZ + tapHeight).point
        return TipTarget(tip: tip, pixel: [u, v], tap: .init(u: u, v: v), turn: applied)
    }

    /// `solveTag`: find the tag in the tapped frame and solve T_base_cam.
    public static func solveTag(gray: GrayImage, calibration: Calibration,
                                marker: MarkerConfig, planeZ: Double) throws -> Mat4 {
        guard let found = ArucoDetector.detectFrame(gray) else {
            throw GeometryError.message("the base tag is not in the tapped frame")
        }
        let scaled = try CameraGeometry.scaleCalibration(calibration, width: gray.width, height: gray.height)
        guard let pose = ArucoPose.poseFromCorners(corners: found.corners, calibration: scaled,
                                                   marker: marker, planeZ: planeZ) else {
            throw GeometryError.message("the base tag pose could not be solved")
        }
        return pose.TBaseCam
    }
}

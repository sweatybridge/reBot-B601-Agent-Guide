/* Cross-validation of the Swift port against fixtures produced by the REAL
 * web JS modules (see tools/gen_fixtures.js in the repo root of this deliverable).
 */
import XCTest
import RebotCore

/// Anchor for Bundle(for:) when the tests run as an Xcode test bundle (iOS CI)
/// instead of a SwiftPM test target.
final class FixtureBundleToken {}

func fixtureURL(_ name: String) -> URL {
    #if SWIFT_PACKAGE
    return Bundle.module.url(forResource: "Fixtures/\(name)", withExtension: nil)!
    #else
    return Bundle(for: FixtureBundleToken.self).url(forResource: name, withExtension: nil, subdirectory: "Fixtures")!
    #endif
}

func fixtureJSON(_ name: String) -> [String: Any] {
    let data = try! Data(contentsOf: fixtureURL(name + ".json"))
    return try! JSONSerialization.jsonObject(with: data) as! [String: Any]
}

func doubles(_ any: Any) -> [Double] { (any as! [Any]).map { ($0 as! NSNumber).doubleValue } }
func matrix(_ any: Any) -> [[Double]] { (any as! [Any]).map { doubles($0) } }
func pairs(_ any: Any) -> [(Double, Double)] {
    (any as! [Any]).map { let d = doubles($0); return (d[0], d[1]) }
}

func calibrationFrom(_ any: Any) -> Calibration {
    let dict = any as! [String: Any]
    return Calibration(K: matrix(dict["K"]!), dist: doubles(dict["dist"]!),
                       imageSize: (dict["image_size"] as! [Any]).map { ($0 as! NSNumber).intValue })
}

func markerFrom(_ any: Any) -> MarkerConfig {
    let dict = any as! [String: Any]
    return MarkerConfig(sizeM: (dict["size_m"] as! NSNumber).doubleValue,
                        at: doubles(dict["at"]!),
                        yawDeg: (dict["yaw_deg"] as? NSNumber)?.doubleValue,
                        id: (dict["id"] as? NSNumber)?.intValue,
                        dictionary: dict["dictionary"] as? String)
}

func assertClose(_ a: Double, _ b: Double, _ tol: Double, _ label: String,
                 file: StaticString = #filePath, line: UInt = #line) {
    XCTAssertLessThanOrEqual(abs(a - b), tol, "\(label): \(a) vs \(b)", file: file, line: line)
}

func assertMatrixClose(_ a: [[Double]], _ b: [[Double]], _ tol: Double, _ label: String,
                       file: StaticString = #filePath, line: UInt = #line) {
    XCTAssertEqual(a.count, b.count, label, file: file, line: line)
    for i in 0..<min(a.count, b.count) {
        for j in 0..<a[i].count {
            assertClose(a[i][j], b[i][j], tol, "\(label)[\(i)][\(j)]", file: file, line: line)
        }
    }
}

final class GeometryTests: XCTestCase {
    func testProjectUndistortRayPlane() throws {
        let fixture = fixtureJSON("geometry")
        let calibration = calibrationFrom(fixture["calibration"]!)
        let T = matrix(fixture["pose"]!)
        let camera = try CameraGeometry.makeCamera(calibration, videoWidth: 810, videoHeight: 1440)
        let pose = Pose(T: T)

        for case_ in fixture["project"] as! [Any] {
            let dict = case_ as! [String: Any]
            let point = doubles(dict["point"]!)
            let pixel = CameraGeometry.project(camera, pose: pose, point: point)
            if dict["pixel"] is NSNull {
                XCTAssertNil(pixel)
            } else {
                let expected = dict["pixel"] as! [String: Any]
                XCTAssertNotNil(pixel)
                let eu = (expected["u"] as! NSNumber).doubleValue
                let ev = (expected["v"] as! NSNumber).doubleValue
                assertClose(pixel!.u, eu, 1e-9 * max(1, abs(eu)), "project.u")
                assertClose(pixel!.v, ev, 1e-9 * max(1, abs(ev)), "project.v")
            }
        }

        for case_ in fixture["undistort"] as! [Any] {
            let dict = case_ as! [String: Any]
            let uv = doubles(dict["uv"]!)
            let expected = doubles(dict["xy"]!)
            let got = CameraGeometry.undistort(u: uv[0], v: uv[1], camera: camera)
            assertClose(got.0, expected[0], 1e-12, "undistort.x")
            assertClose(got.1, expected[1], 1e-12, "undistort.y")
        }

        for case_ in fixture["rayPlane"] as! [Any] {
            let dict = case_ as! [String: Any]
            let uv = doubles(dict["uv"]!)
            let planeZ = (dict["planeZ"] as! NSNumber).doubleValue
            if let error = dict["error"] as? String {
                XCTAssertThrowsError(try CameraGeometry.rayPlane(camera: camera, T: T, u: uv[0], v: uv[1], planeZ: planeZ)) {
                    XCTAssertEqual("\($0)", error)
                }
            } else {
                let hit = try CameraGeometry.rayPlane(camera: camera, T: T, u: uv[0], v: uv[1], planeZ: planeZ)
                let expected = dict["hit"] as! [String: Any]
                let point = doubles(expected["point"]!)
                for i in 0..<3 { assertClose(hit.point[i], point[i], 1e-9, "rayPlane.point") }
                assertClose(hit.distance, (expected["distance"] as! NSNumber).doubleValue, 1e-9, "rayPlane.distance")
                assertClose(hit.grazing, (expected["grazing"] as! NSNumber).doubleValue, 1e-9, "rayPlane.grazing")
            }
        }
    }

    func testMapPixelShapeTurnBlend() {
        let fixture = fixtureJSON("geometry")
        for case_ in fixture["mapPixel"] as! [Any] {
            let dict = case_ as! [String: Any]
            let uv = doubles(dict["uv"]!)
            let size = (dict["size"] as! [Any]).map { ($0 as! NSNumber).intValue }
            let turns = (dict["turns"] as! NSNumber).intValue
            let expected = doubles(dict["at"]!)
            let got = CameraGeometry.mapPixel(u: uv[0], v: uv[1], size: size, turns: turns)
            assertClose(got.0, expected[0], 1e-9, "mapPixel.x")
            assertClose(got.1, expected[1], 1e-9, "mapPixel.y")
        }
        for case_ in fixture["shapeTurn"] as! [Any] {
            let dict = case_ as! [String: Any]
            let c = (dict["calibration"] as! [Any]).map { ($0 as! NSNumber).intValue }
            let f = (dict["frame"] as! [Any]).map { ($0 as! NSNumber).intValue }
            XCTAssertEqual(CameraGeometry.shapeTurn(calibrationSize: c, frameSize: f),
                           (dict["turns"] as! NSNumber).intValue)
        }
        for case_ in fixture["blend"] as! [Any] {
            let dict = case_ as! [String: Any]
            let blended = M3.blendOrientation(current: matrix(dict["current"]!),
                                              target: matrix(dict["target"]!),
                                              fraction: (dict["fraction"] as! NSNumber).doubleValue)
            assertMatrixClose(blended, matrix(dict["blended"]!), 1e-9, "blend")
        }
    }

    func testScaleCalibration() throws {
        let fixture = fixtureJSON("geometry")
        let calibration = calibrationFrom(fixture["calibration"]!)
        let scaled = try CameraGeometry.scaleCalibration(calibration, width: 405, height: 720)
        XCTAssertEqual(scaled.imageSize, [405, 720])
        assertClose(scaled.K[0][0], 500, 1e-12, "fx scaled")
        XCTAssertThrowsError(try CameraGeometry.scaleCalibration(calibration, width: 1440, height: 810))
    }

    func testRodriguesAgainstJS() {
        let fixture = fixtureJSON("geometry")
        for case_ in fixture["rodrigues"] as! [Any] {
            let dict = case_ as! [String: Any]
            let vec = doubles(dict["vec"]!)
            let R = M3.rodrigues(vec)
            assertMatrixClose(R, matrix(dict["R"]!), 1e-9, "rodrigues R")
            let back = M3.rotationToVector(R)
            let expectedBack = doubles(dict["back"]!)
            for i in 0..<3 { assertClose(back[i], expectedBack[i], 1e-9, "rotationToVector") }
            assertMatrixClose(M3.rodrigues(back), matrix(dict["R2"]!), 1e-9, "rodrigues R2")
        }
    }
}

final class ArmModelTests: XCTestCase {
    func testForwardKinematics() {
        let fixture = fixtureJSON("arm")
        for case_ in fixture["fk"] as! [Any] {
            let dict = case_ as! [String: Any]
            let q = doubles(dict["q"]!)
            let fk = ArmModel.forward(q)
            let position = doubles(dict["position"]!)
            for i in 0..<3 { assertClose(fk.position[i], position[i], 1e-9, "fk.position") }
            assertMatrixClose(fk.rotation, matrix(dict["rotation"]!), 1e-9, "fk.rotation")
        }
    }

    func testReachBands() {
        let fixture = fixtureJSON("arm")
        for case_ in fixture["bands"] as! [Any] {
            let dict = case_ as! [String: Any]
            let planeZ = (dict["planeZ"] as! NSNumber).doubleValue
            let hoverM = (dict["hoverM"] as! NSNumber).doubleValue
            let tipM = (dict["tipM"] as! NSNumber).doubleValue
            let band = ArmModel.computeReachBand(planeZ: planeZ, hoverM: hoverM, tipM: tipM)
            if dict["band"] is NSNull {
                XCTAssertNil(band)
                continue
            }
            let expected = dict["band"] as! [String: Any]
            XCTAssertNotNil(band)
            guard let band else { continue }
            // The IK scan is seeded and iterative; ulp-level libm differences
            // can move a boundary by one 1 cm step at most.
            assertClose(band.inner, (expected["inner"] as! NSNumber).doubleValue, 0.0101, "band.inner")
            assertClose(band.outer, (expected["outer"] as! NSNumber).doubleValue, 0.0101, "band.outer")
            assertClose(band.sector, (expected["sector"] as! NSNumber).doubleValue, 1e-12, "band.sector")
            assertClose(band.tipHeight, (expected["tipHeight"] as! NSNumber).doubleValue, 1e-12, "band.tipHeight")
            assertClose(band.flangeZ, (expected["flangeZ"] as! NSNumber).doubleValue, 1e-12, "band.flangeZ")
            print("band(\(planeZ), \(hoverM), \(tipM)): swift \(band.inner)-\(band.outer) vs js \(expected["inner"]!)-\(expected["outer"]!)")
        }
    }
}

final class ArucoTests: XCTestCase {
    func loadGray(_ name: String, width: Int, height: Int) -> GrayImage {
        let data = try! Data(contentsOf: fixtureURL(name + ".gray"))
        XCTAssertEqual(data.count, width * height)
        return GrayImage(data: [UInt8](data), width: width, height: height)
    }

    func runFixture(_ name: String) {
        let meta = fixtureJSON(name)
        let width = (meta["width"] as! NSNumber).intValue
        let height = (meta["height"] as! NSNumber).intValue
        let gray = loadGray(name, width: width, height: height)
        let calibration = calibrationFrom(meta["calibration"]!)
        let marker = markerFrom(meta["marker"]!)

        // 1. detection: same corners as the JS detector
        guard let found = ArucoDetector.detectFrame(gray) else {
            return XCTFail(name + ": Swift detector found nothing")
        }
        let expectedCorners = pairs(meta["detect_corners"]!)
        for i in 0..<4 {
            assertClose(found.corners[i].0, expectedCorners[i].0, 0.5, name + " corner \(i).x")
            assertClose(found.corners[i].1, expectedCorners[i].1, 0.5, name + " corner \(i).y")
        }

        // 2. pose from the JS corners: same solve as JS
        let jsPose = meta["pose"] as! [String: Any]
        guard let pose = ArucoPose.poseFromCorners(corners: expectedCorners, calibration: calibration,
                                                   marker: marker, planeZ: 0) else {
            return XCTFail(name + ": Swift pose solve failed")
        }
        assertMatrixClose(pose.TBaseCam, matrix(jsPose["T_base_cam"]!), 1e-5, name + " T_base_cam")
        assertClose(pose.rms, (jsPose["rms"] as! NSNumber).doubleValue, 1e-4, name + " rms")

        // 3. the recovered pose is close to the render's ground truth
        let truth = matrix(meta["true_T_base_cam"]!)
        let dp = V3.norm(V3.sub(M4.translation(pose.TBaseCam), M4.translation(truth)))
        XCTAssertLessThan(dp, 0.06, name + ": position error \(dp) m")

        // 4. the full eye pipeline (detect + refine + pose) on the same frame
        guard let eye = ArucoEye.detect(full: gray, downscaled: gray, calibration: calibration,
                                        marker: marker, planeZ: 0) else {
            return XCTFail(name + ": Swift eye pipeline failed")
        }
        let dEye = V3.norm(V3.sub(M4.translation(eye.TBaseCam), M4.translation(truth)))
        XCTAssertLessThan(dEye, 0.06, name + ": eye position error \(dEye) m")
        XCTAssertLessThan(eye.rmsPx, 1.0, name + ": eye rms")

        // 5. locate: tipTarget through the JS pose, all four turns
        let T = matrix(jsPose["T_base_cam"]!)
        for case_ in meta["locate"] as! [Any] {
            let dict = case_ as! [String: Any]
            let turn = (dict["turn"] as! NSNumber).intValue
            let vw = (dict["vw"] as! NSNumber).intValue
            let vh = (dict["vh"] as! NSNumber).intValue
            let uv = doubles(dict["uv"]!)
            let heightM = (dict["height"] as! NSNumber).doubleValue
            if let error = dict["error"] as? String {
                XCTAssertThrowsError(try Locate.tipTarget(calibration: calibration, T: T, u: uv[0], v: uv[1],
                                                          planeZ: 0, tapHeight: heightM,
                                                          videoWidth: vw, videoHeight: vh, turn: turn)) {
                    XCTAssertEqual("\($0)", error)
                }
            } else {
                let expected = dict["target"] as! [String: Any]
                let target = try! Locate.tipTarget(calibration: calibration, T: T, u: uv[0], v: uv[1],
                                                   planeZ: 0, tapHeight: heightM,
                                                   videoWidth: vw, videoHeight: vh, turn: turn)
                let tip = doubles(expected["tip"]!)
                for i in 0..<3 { assertClose(target.tip[i], tip[i], 1e-9, name + " tip[\(i)] turn \(turn)") }
                XCTAssertEqual(target.turn, (expected["turn"] as! NSNumber).intValue)
            }
        }
        print(name + ": detect + pose + eye + locate OK (rms \(String(format: "%.3f", eye.rmsPx)) px)")
    }

    func testTagFront() { runFixture("tag_front") }
    func testTagMerged() { runFixture("tag_merged") }
    func testTagOblique() { runFixture("tag_oblique") }
}

/* Depth-fit validation against synthetic scenes built from the REAL
 * arm-model.js FK (tools/gen_fixtures.js): the Swift fit must recover the
 * known T_base_world, and refuse the degenerate scenes instead of guessing.
 */
import XCTest
import RebotCore

final class DepthFitTests: XCTestCase {
    func testCapsulesMatchJSJointPoints() {
        let fixture = fixtureJSON("depth_fit")
        let radii = doubles(fixture["radii"]!)
        XCTAssertEqual(ArmDepthFit.radii, radii)
        let case_ = (fixture["cases"] as! [Any])[0] as! [String: Any]
        let q = doubles(case_["q"]!)
        let expected = (case_["jointPoints"] as! [Any]).map { doubles($0) }
        let caps = ArmDepthFit.capsules(q: q)
        XCTAssertEqual(caps.count, expected.count - 1)
        for i in 0..<caps.count {
            for c in 0..<3 {
                assertClose(caps[i].a[c], expected[i][c], 1e-9, "capsule \(i).a")
                assertClose(caps[i].b[c], expected[i + 1][c], 1e-9, "capsule \(i).b")
            }
            assertClose(caps[i].r, radii[i], 1e-12, "capsule \(i).r")
        }
    }

    func runCase(_ dict: [String: Any]) {
        let name = dict["name"] as! String
        let q = doubles(dict["q"]!)
        let initial = doubles(dict["initial"]!)
        let raw = (dict["points"] as! [Any]).map { doubles($0) }
        let points = ArmDepthFit.segment(points: raw, near: initial)
        XCTAssertLessThan(points.count, raw.count, name + ": segmentation drops the far point")
        let result = ArmDepthFit.fit(points: points, q: q, initial: initial)
        let expect = dict["expect"] as! String
        if expect == "reject" {
            XCTAssertFalse(result.ok, name + ": expected refusal")
            XCTAssertNotNil(result.reason)
            print(name + ": refused as expected — \(result.reason ?? "?")")
            return
        }
        XCTAssertTrue(result.ok, name + ": fit refused: \(result.reason ?? "?")")
        guard let T = result.TBaseWorld else { return }
        let truth = dict["truth"] as! [String: Any]
        let t = doubles(truth["t"]!)
        let yaw = (truth["yaw"] as! NSNumber).doubleValue
        let dp = V3.norm(V3.sub(M4.translation(T), t))
        let fittedYaw = atan2(T[1][0], T[0][0])
        var dyaw = fittedYaw - yaw
        while dyaw > .pi { dyaw -= 2 * .pi }
        while dyaw < -.pi { dyaw += 2 * .pi }
        XCTAssertLessThan(dp, 0.015, name + ": position error \(dp) m")
        XCTAssertLessThan(abs(dyaw), 0.04, name + ": yaw error \(dyaw) rad")
        XCTAssertGreaterThanOrEqual(result.linksCovered, 3)
        print(name + String(format: ": recovered (err %.1f mm, yaw %.3f rad, rms %.1f mm, %d inliers, %d links)",
                            dp * 1000, dyaw, result.rms * 1000, result.inliers, result.linksCovered))
    }

    func testDepthGood() {
        let fixture = fixtureJSON("depth_fit")
        runCase((fixture["cases"] as! [Any])[0] as! [String: Any])
    }

    func testDepthWeakYaw() {
        let fixture = fixtureJSON("depth_fit")
        runCase((fixture["cases"] as! [Any])[1] as! [String: Any])
    }

    func testDepthSparse() {
        let fixture = fixtureJSON("depth_fit")
        runCase((fixture["cases"] as! [Any])[2] as! [String: Any])
    }
}

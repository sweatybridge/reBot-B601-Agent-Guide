/* Depth-based base anchoring: the no-tag fallback on LiDAR devices.
 *
 * Flow: Depth anchor chip -> tap the arm base in the preview -> the tap is
 * unprojected through scene depth to a world point -> the host's joint angles
 * are read twice (the arm must hold still) -> the scene mesh is segmented and
 * fitted to the FK capsule model (RebotCore.ArmDepthFit) -> T_base_world.
 *
 * Coarse by design (~1-3 cm): a tag re-anchor supersedes it whenever visible.
 */
import Foundation
import SwiftUI
import RebotCore

extension AppModel {
    func beginDepthAnchor() {
        guard cameraOn, cameraMode == "handheld" else {
            return log("start the camera in handheld mode first", .warn)
        }
        guard CameraController.supportsDepthAnchor else {
            return log("depth anchor needs a LiDAR device; use the base tag instead", .warn)
        }
        guard !busy else { return log("wait for the current action to finish", .warn) }
        depthTapPending = true
        log("depth anchor: tap the arm base in the preview; hold the arm still", .warn)
    }

    func depthAnchorTap(at stagePoint: CGPoint, stageSize: CGSize) async {
        depthTapPending = false
        guard cameraOn, let api else { return }
        guard let frame = camera.currentFrame, let box = contentBox(stageSize: stageSize),
              let tapped = stageToImage(stagePoint, frame: frame, box: box) else { return }
        guard let initial = camera.worldPoint(atImagePixel: tapped.u, v: tapped.v, frame: frame) else {
            return log("no depth at that point (LiDAR range or surface); tap Depth anchor to retry", .warn)
        }
        busy = true
        busyText = "Depth anchor..."
        defer { busy = false; busyText = "" }
        do {
            // The arm must hold still while its pose and the mesh are paired,
            // the same rule as the host's rig sweep.
            let q1 = try await api.pose()
            try await Task.sleep(nanoseconds: 150_000_000)
            let q2 = try await api.pose()
            let moved = zip(q1, q2).map { abs($0 - $1) }.max() ?? 0
            guard moved < 0.02 else {
                throw APIFailure(status: 0, message: "the arm moved while reading its pose; hold it still and retry")
            }
            let raw = camera.scenePoints()
            let points = ArmDepthFit.segment(points: raw, near: initial)
            log("depth anchor: fitting \(points.count) scene points to the arm model")
            let result = await Task.detached(priority: .userInitiated) {
                ArmDepthFit.fit(points: points, q: q2, initial: initial)
            }.value
            guard result.ok, let T = result.TBaseWorld else {
                throw APIFailure(status: 0, message: result.reason ?? "the depth fit refused")
            }
            anchor = T
            anchorLogged = true
            log(String(format: "depth anchor set: rms %.1f mm, %d points, %d links — coarse; re-anchor on the tag when visible",
                       result.rms * 1000, result.inliers, result.linksCovered), .good)
        } catch {
            log("depth anchor failed: \(error)", .bad)
        }
    }
}

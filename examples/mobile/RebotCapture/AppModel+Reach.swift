/* Config loading, anchoring, in-app tap locate and the reach band on ARKit.
 *
 * Replaces reach-config.js / locate.js / reach.js / markers.js orchestration:
 * - the camera pose is ARKit world tracking, anchored to the base by one ArUco
 *   solve (T_base_world = T_base_cam_tag * inverse(T_world_cam));
 * - intrinsics come from ARCamera, so there is no calibration grid to turn
 *   back onto: ARFrame.displayTransform maps image pixels to the view;
 * - the reach band math (RebotCore.ArmModel) is unchanged.
 */
import Foundation
import SwiftUI
import ARKit
import RebotCore

extension AppModel {
    // MARK: config cache

    func invalidateConfig() {
        config = nil
        configTask = nil
        configGeneration += 1
    }

    func loadConfig() async throws -> ReachConfig {
        while true {
            if let config { return config }
            if let configTask { return try await configTask.value }
            guard let api else { throw APIFailure(status: 0, message: "set the host first") }
            let generation = configGeneration
            let task = Task { try await api.reachConfig() }
            configTask = task
            do {
                let loaded = try await task.value
                configTask = nil
                if generation != configGeneration { continue }
                config = loaded
                return loaded
            } catch {
                configTask = nil
                throw error
            }
        }
    }

    // MARK: base anchor (replaces the per-frame tag pose loop + EKF)

    /// Solve the tag in one ARFrame and tie ARKit's world to the arm base.
    /// Between solves, world tracking carries the pose; periodic re-anchors
    /// correct VIO drift.
    func reanchor(manual: Bool) async {
        guard cameraOn, cameraMode == "handheld", !reanchoring else { return }
        guard let frame = camera.currentFrame else { return }
        guard camera.trackingIsNormal else {
            if manual { log("cannot anchor: ARKit tracking is \(camera.trackingText(frame))", .warn) }
            return
        }
        let config: ReachConfig
        do {
            config = try await loadConfig()
        } catch {
            if manual { log("cannot anchor: \(error)", .bad) }
            return
        }
        guard let marker = config.marker else {
            if manual { log("handheld mode needs the base tag; fit a rig or switch camera mode", .bad) }
            return
        }
        reanchoring = true
        defer { reanchoring = false }
        let calibration = camera.calibration(frame)
        let worldFromCamera = camera.worldFromCamera(frame)
        let planeZ = config.planeZ
        let camera = self.camera
        let buffer = frame.capturedImage
        do {
            let tagPose = try await Task.detached(priority: .userInitiated) {
                guard let gray = camera.gray(from: buffer, maxDimension: 1440) else {
                    throw APIFailure(status: 0, message: "the camera has no frame")
                }
                return try Locate.solveTag(gray: gray, calibration: calibration, marker: marker, planeZ: planeZ)
            }.value
            // The tag solve ran on the downscaled frame; solveTag rescales K,
            // so the pose is already in this frame's camera coordinates.
            anchor = M4.mul(tagPose, M4.invert(worldFromCamera))
            if !anchorLogged || manual {
                anchorLogged = true
                log("base anchored: ARKit tracking now carries the pose (re-anchors every 2 s while the tag is visible)", .good)
            }
        } catch {
            if manual { log("anchor failed: \(error)", .bad) }
        }
    }

    func startReanchorLoop() {
        guard cameraOn, cameraMode == "handheld", reanchorTimer == nil else { return }
        reanchorTimer = Timer.scheduledTimer(withTimeInterval: 2.0, repeats: true) { [weak self] _ in
            Task { @MainActor in await self?.reanchor(manual: false) }
        }
        Task { await reanchor(manual: false) }
    }

    func stopReanchorLoop() {
        reanchorTimer?.invalidate()
        reanchorTimer = nil
    }

    /// The pose the band, the pins and the taps use right now: T_base_cam.
    var displayPose: Mat4? {
        if cameraMode == "fixed" { return config?.TBaseCam }
        guard let anchor, let frame = camera.currentFrame, camera.trackingIsNormal else { return nil }
        // A paused session keeps the last frame; do not trust a stale one.
        guard ProcessInfo.processInfo.systemUptime - frame.timestamp < 0.5 else { return nil }
        return M4.mul(anchor, camera.worldFromCamera(frame))
    }

    // MARK: letterbox placement + ARKit display transform

    struct ContentBox {
        var left: Double, top: Double, width: Double, height: Double, scale: Double
    }

    func contentBox(stageSize: CGSize) -> ContentBox? {
        guard let frame = camera.currentFrame else { return nil }
        let size = camera.orientedSize(frame)
        let vw = Double(size.width), vh = Double(size.height)
        guard vw > 0, vh > 0, stageSize.width > 0, stageSize.height > 0 else { return nil }
        let scale = min(Double(stageSize.width) / vw, Double(stageSize.height) / vh)
        return ContentBox(left: (Double(stageSize.width) - vw * scale) / 2,
                          top: (Double(stageSize.height) - vh * scale) / 2,
                          width: vw * scale, height: vh * scale, scale: scale)
    }

    /// Image pixel -> stage point, through ARKit's own orientation mapping.
    func imageToStage(u: Double, v: Double, frame: ARFrame, box: ContentBox) -> CGPoint {
        let w = Double(CVPixelBufferGetWidth(frame.capturedImage))
        let h = Double(CVPixelBufferGetHeight(frame.capturedImage))
        let transform = frame.displayTransform(for: CameraController.interfaceOrientation(),
                                               viewportSize: CGSize(width: box.width, height: box.height))
        let normalized = CGPoint(x: u / w, y: v / h).applying(transform)
        return CGPoint(x: box.left + normalized.x * box.width,
                       y: box.top + normalized.y * box.height)
    }

    /// Stage point -> image pixel (the inverse mapping), nil outside the frame.
    func stageToImage(_ point: CGPoint, frame: ARFrame, box: ContentBox) -> (u: Double, v: Double)? {
        let w = Double(CVPixelBufferGetWidth(frame.capturedImage))
        let h = Double(CVPixelBufferGetHeight(frame.capturedImage))
        let transform = frame.displayTransform(for: CameraController.interfaceOrientation(),
                                               viewportSize: CGSize(width: box.width, height: box.height))
        let normalized = CGPoint(x: (Double(point.x) - box.left) / box.width,
                                 y: (Double(point.y) - box.top) / box.height)
            .applying(transform.inverted())
        let u = normalized.x * w
        let v = normalized.y * h
        if u < 0 || v < 0 || u > w || v > h { return nil }
        return ((u * 100).rounded() / 100, (v * 100).rounded() / 100)
    }

    // MARK: tap -> marker

    func addMarker(at stagePoint: CGPoint, stageSize: CGSize) async {
        guard cameraOn else { return }
        if calib || busy { return }
        if locating { return log("a marker is still being located", .warn) }
        if markers.count >= 10 { return log("10 markers is the limit; clear one first", .warn) }
        guard let frame = camera.currentFrame, let box = contentBox(stageSize: stageSize) else { return }
        if cameraMode == "handheld" {
            guard camera.trackingIsNormal else {
                return log("not marking: ARKit tracking is \(camera.trackingText(frame))", .warn)
            }
            guard anchor != nil else {
                return log("not marking: point the camera at the base tag to anchor first", .warn)
            }
        }
        guard let tapped = stageToImage(stagePoint, frame: frame, box: box) else { return }
        let tapHeight = height
        let generation = markerGeneration
        locating = true
        defer { locating = false }
        do {
            let config = try await loadConfig()
            let T: Mat4
            if cameraMode == "fixed" {
                guard let rig = config.TBaseCam else {
                    throw APIFailure(status: 0, message: "fixed camera mode needs a fitted rig")
                }
                T = rig
            } else {
                guard let pose = displayPose else {
                    throw APIFailure(status: 0, message: "the base anchor or ARKit tracking was lost")
                }
                T = pose
            }
            if generation != markerGeneration { return }
            // ARKit intrinsics on this frame's grid: no distortion, no turns.
            let calibration = camera.calibration(frame)
            let size = camera.frameSize ?? (width: 0, height: 0)
            let target = try Locate.tipTarget(calibration: calibration, T: T,
                                              u: tapped.u, v: tapped.v, planeZ: config.planeZ,
                                              tapHeight: tapHeight, videoWidth: size.width,
                                              videoHeight: size.height, turn: 0)
            let radius = (target.tip[0] * target.tip[0] + target.tip[1] * target.tip[1]).squareRoot()
            markers.append(SequenceMarker(u: tapped.u, v: tapped.v, pixel: target.pixel,
                                          target: target, radius: radius))
            let point = target.tip.map { String(format: "%.3f", $0) }.joined(separator: ", ")
            let odd = markers.count % 2 == 1
            log("marker \(markers.count) at tip [\(point)] m · \(Int((tapHeight * 1000).rounded())) mm up"
                + (odd ? " · gripper will half-close" : " · gripper will open"), odd ? .warn : .good)
        } catch {
            log("marker not added: \(error)", .bad)
        }
    }

    // MARK: reach overlay

    func toggleReach() {
        reachOn.toggle()
        if reachOn {
            log("reach overlay on: the shaded band is where a marker can be planned", .good)
            Task { await applyReach() }
        } else {
            log("reach overlay off", .warn)
            reachError = ""
        }
    }

    func applyReach() async {
        reachError = ""
        do {
            let config = try await loadConfig()
            await recomputeBand(config: config)
            if reachBand == nil {
                reachError = "the arm model has no reachable band at this height"
                return
            }
            startReanchorLoop()
        } catch {
            reachError = "\(error)"
            log("reach overlay: \(error)", .bad)
        }
    }

    private func recomputeBand(config: ReachConfig) async {
        let planeZ = config.planeZ
        let hoverM = height
        let tipM = config.tipM
        let key = "\(planeZ)|\(hoverM)|\(tipM)"
        if key == bandKey, reachBand != nil { return }
        bandTask?.cancel()
        let task = Task.detached(priority: .userInitiated) {
            ArmModel.computeReachBand(planeZ: planeZ, hoverM: hoverM, tipM: tipM)
        }
        bandTask = Task {
            let band = await task.value
            if Task.isCancelled { return }
            self.reachBand = band
            self.bandKey = key
            if band == nil { self.reachError = "the arm model has no reachable band at this height" }
            else { self.reachError = "" }
        }
        await bandTask?.value
    }

    /// Coalesce a height drag into one band recompute per ~110 ms, trailing.
    func scheduleBandRecompute() {
        guard reachOn, config != nil else { return }
        if heightCoalesce != nil { return }
        heightCoalesce = Task { [weak self] in
            try? await Task.sleep(nanoseconds: 110_000_000)
            guard let self else { return }
            self.heightCoalesce = nil
            guard self.reachOn, let config = self.config else { return }
            await self.recomputeBand(config: config)
        }
    }

    // MARK: projection (ARKit intrinsics + displayTransform)

    /// Project a base-frame point to a stage point with the live pose (pins).
    func projectPoint(_ point: Vec3, stageSize: CGSize) -> CGPoint? {
        guard let frame = camera.currentFrame, let pose = displayPose,
              let box = contentBox(stageSize: stageSize) else { return nil }
        guard let pixel = CameraGeometry.project(camera.intrinsics(frame), pose: Pose(T: pose), point: point) else { return nil }
        return imageToStage(u: pixel.u, v: pixel.v, frame: frame, box: box)
    }

    /// The band's ring points in stage coordinates, for the Canvas overlay.
    func reachRingPoints(stageSize: CGSize) -> (outer: [CGPoint], inner: [CGPoint], base: CGPoint?)? {
        guard reachOn, let band = reachBand, let frame = camera.currentFrame,
              let pose = displayPose, let box = contentBox(stageSize: stageSize) else { return nil }
        let intrinsics = camera.intrinsics(frame)
        let poseValue = Pose(T: pose)
        func ringPoint(radius: Double, angle: Double) -> CGPoint? {
            let point: Vec3 = [radius * cos(angle), radius * sin(angle), band.tipHeight]
            guard let pixel = CameraGeometry.project(intrinsics, pose: poseValue, point: point) else { return nil }
            return imageToStage(u: pixel.u, v: pixel.v, frame: frame, box: box)
        }
        let samples = 128
        var outer: [CGPoint] = []
        var inner: [CGPoint] = []
        for index in 0...samples {
            let angle = -band.sector + (2 * band.sector * Double(index)) / Double(samples)
            if let far = ringPoint(radius: band.outer, angle: angle) { outer.append(far) }
            if let near = ringPoint(radius: band.inner, angle: angle) { inner.append(near) }
        }
        let base = ringPoint(radius: 0, angle: 0)
        return (outer, inner, base)
    }

    var reachLegend: (first: String, second: String)? {
        guard let band = reachBand, let config else { return nil }
        let label = cameraMode == "fixed" ? "fixed rig" : "ARKit + tag anchor"
        let first = String(format: "arm reach · %.2f–%.2f m", band.inner, band.outer)
        let second = "tip \(Int(((band.tipHeight - config.planeZ) * 1000).rounded())) mm up · " + label
        return (first, second)
    }

    /// The message the empty overlay shows.
    var reachEmptyMessage: String? {
        guard reachOn else { return nil }
        if !cameraOn { return "start the camera first" }
        if config == nil { return reachError.isEmpty ? "reading the calibration…" : reachError }
        if reachBand == nil { return reachError.isEmpty ? "computing the reach band…" : reachError }
        if cameraMode == "fixed" {
            return config?.TBaseCam == nil ? "no fixed rig fitted: fit one with Rig" : nil
        }
        if let frame = camera.currentFrame, !camera.trackingIsNormal {
            return "ARKit tracking \(camera.trackingText(frame))"
        }
        if anchor == nil { return "point the camera at the base tag to anchor" }
        return displayPose == nil ? "waiting for a fresh ARKit frame" : nil
    }

    /// A marker pin's stage position: live projection first, tap pixel fallback.
    func markerStagePoint(_ marker: SequenceMarker, stageSize: CGSize) -> CGPoint? {
        if let live = projectPoint(marker.target.tip, stageSize: stageSize) {
            return live
        }
        guard let frame = camera.currentFrame, let box = contentBox(stageSize: stageSize) else { return nil }
        return imageToStage(u: marker.pixel[0], v: marker.pixel[1], frame: frame, box: box)
    }
}

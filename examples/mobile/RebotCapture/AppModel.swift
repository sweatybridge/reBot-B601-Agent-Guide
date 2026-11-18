/* The native counterpart of phone.html's inline script, on ARKit: one owner
 * for the host connection, the camera, the base anchor, the markers, the
 * height and the log.
 *
 * ARKit replaces the page's per-frame tag pose and its optional EKF: world
 * tracking carries the pose continuously; the ArUco solve anchors (and
 * periodically re-anchors) ARKit's world frame to the arm base.
 */
import Foundation
import SwiftUI
import Combine
import RebotCore

enum LogKind: String {
    case plain = "", good, warn, bad
}

struct LogLine: Identifiable {
    let id = UUID()
    let time: String
    let text: String
    let kind: LogKind
}

struct SequenceMarker: Identifiable {
    let id = UUID()
    var u: Double
    var v: Double
    var pixel: [Double]
    var target: TipTarget
    var radius: Double
}

@MainActor
final class AppModel: ObservableObject {
    let settings = SettingsStore()
    let camera = CameraController()
    let whip = WhipPublisher()

    // host
    @Published var health: Health?
    @Published var hostOK = false
    @Published var serverText = "connecting"
    var modeInitialized = false
    var rigFingerprint: String?

    // camera / mode
    @Published var cameraOn = false
    @Published var starting = false
    @Published var cameraMode = "handheld"

    // base anchor: T_base_world, from the ArUco solve against one ARFrame
    // (or the LiDAR depth fit, AppModel+Depth.swift)
    @Published var anchor: Mat4?
    @Published var depthTapPending = false
    @Published var reanchoring = false
    var reanchorTimer: Timer?
    var anchorLogged = false

    // markers
    @Published var markers: [SequenceMarker] = []
    @Published var locating = false
    var markerGeneration = 0

    // height
    @Published var height = 0.10
    @Published var adjusting = false
    var heightMin = 0.0
    var heightMax = 0.60
    var heightTouched = false
    static let heightStep = 0.005
    static let heightPerPx = 0.0015
    static let tapSlopPx = 10.0

    // calibration / busy
    @Published var calib = false
    @Published var calibCount = 0
    @Published var busy = false
    @Published var busyText = ""

    // reach
    @Published var reachOn = false
    @Published var reachBand: ReachBand?
    @Published var reachError = ""
    var bandTask: Task<Void, Never>?
    var bandKey = ""
    var heightCoalesce: Task<Void, Never>?

    // live preview
    @Published var liveState = "off"

    // log
    @Published var lines: [LogLine] = []

    // config cache (/reach.json), one loader like reach-config.js
    var config: ReachConfig?
    var configTask: Task<ReachConfig, Error>?
    var configGeneration = 0

    private(set) var api: APIClient?
    private var healthTask: Task<Void, Never>?
    private var cancellables: Set<AnyCancellable> = []

    init() {
        rebuildAPI()
        settings.objectWillChange
            .receive(on: DispatchQueue.main)
            .sink { [weak self] _ in self?.settingsChanged() }
            .store(in: &cancellables)
        camera.onFrame = { [weak whip] buffer, angle in whip?.feed(buffer, angle: angle) }
        whip.onState = { [weak self] state in self?.liveStateChanged(state) }
        log("set the host to https://<host>:8443 in Settings; the dot turns green when the host answers")
        healthTask = Task { [weak self] in
            while !Task.isCancelled {
                await self?.healthTick()
                try? await Task.sleep(nanoseconds: 5_000_000_000)
            }
        }
    }

    func log(_ text: String, _ kind: LogKind = .plain) {
        let formatter = DateFormatter()
        formatter.dateFormat = "HH:mm:ss"
        lines.insert(LogLine(time: formatter.string(from: Date()), text: text, kind: kind), at: 0)
        if lines.count > 8 { lines.removeLast(lines.count - 8) }
    }

    // MARK: settings / api

    private func settingsChanged() {
        // objectWillChange fires before the value lands; rebuild next runloop.
        DispatchQueue.main.async { [self] in
            rebuildAPI()
            invalidateConfig()
            modeInitialized = false
            rigFingerprint = nil
            health = nil
        }
    }

    private func rebuildAPI() {
        api?.invalidate()
        if let base = settings.baseURL {
            api = APIClient(base: base, token: settings.token, ca: settings.caCertificate)
        } else {
            api = nil
        }
    }

    var needsSetup: Bool { settings.baseURL == nil }

    // MARK: health loop

    private func healthTick() async {
        guard let api else {
            hostOK = false
            serverText = "set the host"
            return
        }
        do {
            let data = try await api.health()
            health = data
            if !modeInitialized, let mode = data.mode {
                modeInitialized = true
                setCameraMode(mode, announce: false)
            }
            let fingerprint = data.rig?.fingerprint ?? ""
            if rigFingerprint == nil {
                rigFingerprint = fingerprint
            } else if fingerprint != rigFingerprint {
                rigFingerprint = fingerprint
                invalidateConfig()
                if reachOn { Task { await applyReach() } }
            }
            let minZ = data.minZ ?? data.hoverM ?? 0.10
            let planeZ = data.planeZ ?? 0
            heightMin = max(0, minZ - planeZ)
            heightMax = max(heightMin, 0.60)
            applyHeight(height)
            if let hover = data.hoverM { adoptHeight(hover) }
            hostOK = true
            serverText = "host ok" + (data.latest.map { " / " + String($0.suffix(6)) } ?? "")
        } catch {
            hostOK = false
            serverText = "host unreachable"
        }
    }

    // MARK: camera mode

    func setCameraMode(_ mode: String, announce: Bool) {
        let fixed = mode == "fixed"
        cameraMode = fixed ? "fixed" : "handheld"
        if reachOn { Task { await applyReach() } }
        guard announce else { return }
        log(fixed
            ? "fixed camera: markers use the rig fitted from /live"
            : "handheld: anchor on the base tag once; ARKit tracking carries the pose", .warn)
    }

    func toggleCameraMode() {
        guard !busy else { return log("wait for the sequence to finish", .warn) }
        setCameraMode(cameraMode == "fixed" ? "handheld" : "fixed", announce: true)
        depthTapPending = false
        clearMarkers()
    }

    // MARK: camera start

    func startCamera() async {
        guard !busy else { return log("wait for the sequence to finish", .warn) }
        starting = true
        defer { starting = false }
        if cameraOn {
            clearMarkers()
            await whip.stop(api: api)
            stopReanchorLoop()
            camera.stop()
            cameraOn = false
            anchor = nil
            anchorLogged = false
            depthTapPending = false
        }
        do {
            try await camera.start()
            cameraOn = true
            UIApplication.shared.isIdleTimerDisabled = true
            if let size = camera.frameSize {
                log("ARKit camera ready: \(size.width)x\(size.height), intrinsics from ARKit", .good)
                await publishOrientation(angle: CameraController.interfaceAngle(), size: [size.width, size.height])
            }
            startReanchorLoop()
            if reachOn { await applyReach() }
        } catch {
            cameraOn = false
            log("\(error)", .bad)
        }
    }

    private func publishOrientation(angle: Int, size: [Int]) async {
        guard let api else { return }
        do {
            try await api.publishOrientation(angle: angle, size: size)
            log("orientation published: \(angle) deg, \(size[0])x\(size[1])")
        } catch {
            log("could not publish the orientation: \(error)", .warn)
        }
    }

    var trackingStatus: String {
        guard let frame = camera.currentFrame else { return "no frame" }
        return camera.trackingText(frame)
    }

    // MARK: height

    func applyHeight(_ value: Double) {
        guard value.isFinite else { return }
        let snapped = (value / Self.heightStep).rounded() * Self.heightStep
        let next = max(heightMin, min(heightMax, snapped))
        if abs(next - height) < 1e-9 { return }
        height = next
        scheduleBandRecompute()
    }

    func setHeightLive(_ value: Double) {
        heightTouched = true
        applyHeight(value)
    }

    func adoptHeight(_ value: Double) {
        if !heightTouched { applyHeight(value) }
    }

    var heightLabel: String {
        let cm = (height * 1000).rounded() / 10
        return (cm == cm.rounded() ? String(format: "%.0f", cm) : String(format: "%.1f", cm)) + " cm"
    }

    // MARK: markers

    func clearMarkers() {
        markerGeneration += 1
        markers.removeAll()
    }

    @discardableResult
    func clearLastMarker() -> Bool {
        if locating || markers.isEmpty { return false }
        markerGeneration += 1
        markers.removeLast()
        return true
    }

    // MARK: calibration mode

    func toggleCalibration() {
        guard !busy else { return log("wait for the sequence to finish", .warn) }
        calib.toggle()
        if calib {
            calibCount = 0
            log("calibration mode: fill the frame with the checkerboard, 15-20 poses (optional with ARKit intrinsics)", .warn)
        } else {
            log("calibration mode off: taps add markers again", .warn)
        }
    }

    func capture() async {
        guard !busy, cameraOn, let api else { return }
        busy = true
        defer { busy = false }
        guard let shot = camera.grabJPEG() else {
            return log("no frame yet; wait a moment and try again", .warn)
        }
        do {
            let meta = buildCaptureMeta(width: shot.width, height: shot.height)
            let metaData = try JSONSerialization.data(withJSONObject: meta)
            let response = try await api.capture(meta: metaData, jpeg: shot.data)
            calibCount += 1
            log("calibration shot \(calibCount) saved as \(response.id ?? "?")", .good)
        } catch {
            log("capture failed: \(error)", .bad)
        }
    }

    private func buildCaptureMeta(width: Int, height: Int) -> [String: Any] {
        let device = UIDevice.current
        let settingsDict: [String: Any] = [
            "width": width, "height": height, "frameRate": 60,
            "facingMode": "environment", "zoom": 1.0,
            "focusMode": "continuous", "exposureMode": "continuous",
            "deviceId": "arkit-back",
        ]
        return [
            "purpose": "calib",
            "capturedAt": ISO8601DateFormatter().string(from: Date()),
            "image_size": [width, height],
            "tap": NSNull(),
            "settings": settingsDict,
            "capabilities": [String: Any](),
            "device": [
                "ua": "RebotCapture/2.0 ARKit iOS \(device.systemVersion)",
                "platform": device.model,
                "dpr": UIScreen.main.scale,
            ],
            "orientation_deg": CameraController.interfaceAngle(),
            "locked": false,
        ]
    }

    // MARK: move arm

    func moveArm() async {
        guard !busy, cameraOn, !markers.isEmpty, let api else { return }
        busy = true
        busyText = "Planning..."
        defer { busy = false; busyText = "" }
        do {
            let response = try await api.sequence(targets: markers.map { $0.target })
            let plan = response.plan
            let summary = plan?.summary
            log("planned \(summary?.markers ?? markers.count) markers · \(summary?.waypoints ?? 0) waypoints · \(String(format: "%.1f", summary?.durationS ?? 0)) s", .good)
            if response.execute != true {
                log("execution is off on the host; the trajectory is written to out/ only", .warn)
                return
            }
            guard let planID = plan?.id else {
                throw APIFailure(status: 0, message: "the plan has no id")
            }
            busyText = "Moving..."
            log("starting \(planID) on the arm", .warn)
            _ = try await api.execute(planID: planID)
            log("arm finished: \(planID)", .good)
        } catch {
            log("sequence failed: \(error)", .bad)
        }
    }

    // MARK: live preview

    func toggleLive() async {
        guard cameraOn else { return log("start the camera before publishing", .warn) }
        guard let api else { return log("set the host first", .warn) }
        if whip.active {
            await whip.stop(api: api)
            liveState = "off"
            return
        }
        if health?.whip == "" {
            return log("live preview is disabled on the host", .warn)
        }
        liveState = "starting"
        do {
            try await whip.start(api: api)
            log("publishing live preview to the host", .good)
        } catch {
            liveState = "off"
            log("live preview: \(error)", .warn)
            await whip.stop(api: api)
        }
    }

    private func liveStateChanged(_ state: String) {
        liveState = state
        if state == "failed" {
            log("live preview failed: phone and host cannot reach each other (AP isolation?)", .bad)
            Task { await whip.stop(api: api) }
        } else if state == "closed" {
            Task { await whip.stop(api: api) }
        }
    }

    // MARK: scene phase

    func scenePhaseChanged(_ phase: ScenePhase) {
        if phase == .active {
            if cameraOn {
                UIApplication.shared.isIdleTimerDisabled = true
                camera.resumeIfNeeded()
                startReanchorLoop()
            }
        } else {
            stopReanchorLoop()
        }
    }
}

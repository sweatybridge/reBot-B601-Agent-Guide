/* ARKit camera: replaces AVCaptureSession, the per-frame tag pose loop and the
 * checkerboard intrinsics calibration.
 *
 * ARSession world tracking (VIO) gives a 6-DoF camera pose at frame rate in an
 * arbitrary world frame; the app anchors that world to the arm base once with
 * the ArUco solve (see AppModel.reanchor). ARCamera.intrinsics is factory
 * calibrated per frame, so no per-phone calibration and no quarter-turn
 * bookkeeping is needed: displayTransform maps image pixels to the view.
 *
 * Trade-off accepted: ARKit intrinsics are pure pinhole (no Brown-Conrady
 * distortion terms), and VIO drifts until the next tag re-anchor.
 */
import ARKit
import CoreImage
import ImageIO
import SceneKit
import UIKit
import RebotCore

final class CameraController: NSObject, ObservableObject, ARSessionDelegate {
    let session = ARSession()
    private let ciContext = CIContext()
    private let frameLock = NSLock()
    private var latest: ARFrame?
    private var configuration: ARWorldTrackingConfiguration?
    private(set) var running = false

    /// Called for every frame (WHIP publishing): buffer + interface angle.
    var onFrame: ((CVPixelBuffer, Int) -> Void)?

    override init() {
        super.init()
        session.delegate = self
    }

    /// The most recent frame; pose, intrinsics and pixels are consistent within
    /// one ARFrame, which is what the anchor solve needs.
    var currentFrame: ARFrame? {
        frameLock.lock()
        defer { frameLock.unlock() }
        return latest
    }

    var frameSize: (width: Int, height: Int)? {
        guard let frame = currentFrame else { return nil }
        return (CVPixelBufferGetWidth(frame.capturedImage), CVPixelBufferGetHeight(frame.capturedImage))
    }

    // MARK: orientation

    static func interfaceOrientation() -> UIInterfaceOrientation {
        let scenes = UIApplication.shared.connectedScenes.compactMap { $0 as? UIWindowScene }
        return scenes.first?.interfaceOrientation ?? .portrait
    }

    static func interfaceAngle() -> Int {
        switch interfaceOrientation() {
        case .portrait: return 0
        case .portraitUpsideDown: return 180
        case .landscapeLeft: return 90
        case .landscapeRight: return 270
        default: return 0
        }
    }

    /// The capturedImage is always sensor-native landscape; the displayed size
    /// swaps in portrait orientations.
    func orientedSize(_ frame: ARFrame) -> (width: Int, height: Int) {
        let w = CVPixelBufferGetWidth(frame.capturedImage)
        let h = CVPixelBufferGetHeight(frame.capturedImage)
        return Self.interfaceAngle() % 180 == 0 ? (h, w) : (w, h)
    }

    // MARK: start / stop

    func start() async throws {
        guard ARWorldTrackingConfiguration.isSupported else {
            throw CameraError.message("ARKit world tracking needs an A9-or-later device; this one cannot run it.")
        }
        switch AVCaptureDevice.authorizationStatus(for: .video) {
        case .authorized: break
        case .notDetermined:
            let granted = await AVCaptureDevice.requestAccess(for: .video)
            if !granted { throw CameraError.message("Camera permission was denied. Enable it in Settings > RebotCapture.") }
        default:
            throw CameraError.message("Camera permission was denied. Enable it in Settings > RebotCapture.")
        }

        stop()
        let config = ARWorldTrackingConfiguration()
        config.worldAlignment = .gravity
        config.isAutoFocusEnabled = true
        config.environmentTexturing = .none
        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
            config.sceneReconstruction = .mesh
        }
        configuration = config
        frameLock.lock()
        latest = nil
        frameLock.unlock()
        session.run(config, options: [.resetTracking, .removeExistingAnchors])
        running = true

        // Wait for the first tracked frame.
        for _ in 0..<100 {
            if currentFrame != nil { return }
            try await Task.sleep(nanoseconds: 50_000_000)
        }
        throw CameraError.message("ARKit produced no frame; try Restart camera")
    }

    func stop() {
        session.pause()
        running = false
        frameLock.lock()
        latest = nil
        frameLock.unlock()
    }

    /// ARKit pauses in the background; resume (with relocalization) on return.
    func resumeIfNeeded() {
        guard running, let configuration else { return }
        session.run(configuration)
    }

    // MARK: ARSessionDelegate

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        frameLock.lock()
        latest = frame
        frameLock.unlock()
        onFrame?(frame.capturedImage, Self.interfaceAngle())
    }

    // MARK: frame-derived helpers

    /// Pinhole calibration from ARKit's factory intrinsics, on the captured
    /// image grid. dist = [] — ARKit models no lens distortion.
    func calibration(_ frame: ARFrame) -> Calibration {
        let k = frame.camera.intrinsics
        let size = (CVPixelBufferGetWidth(frame.capturedImage), CVPixelBufferGetHeight(frame.capturedImage))
        return Calibration(
            K: [[Double(k.columns.0.x), 0, Double(k.columns.2.x)],
                [0, Double(k.columns.1.y), Double(k.columns.2.y)],
                [0, 0, 1]],
            dist: [],
            imageSize: [size.0, size.1])
    }

    /// T_world_cam as a row-major matrix (simd is column-major).
    func worldFromCamera(_ frame: ARFrame) -> Mat4 {
        let m = frame.camera.transform
        var out: Mat4 = Array(repeating: Array(repeating: 0.0, count: 4), count: 4)
        for row in 0..<4 {
            for column in 0..<4 {
                out[row][column] = Double(m.columns[column][row])
            }
        }
        return out
    }

    func intrinsics(_ frame: ARFrame) -> CameraIntrinsics {
        let k = frame.camera.intrinsics
        return CameraIntrinsics(fx: Double(k.columns.0.x), fy: Double(k.columns.1.y),
                                cx: Double(k.columns.2.x), cy: Double(k.columns.2.y))
    }

    func trackingText(_ frame: ARFrame) -> String {
        switch frame.camera.trackingState {
        case .normal: return "normal"
        case .notAvailable: return "not available"
        case .limited(let reason):
            switch reason {
            case .initializing: return "limited: initializing"
            case .excessiveMotion: return "limited: move slower"
            case .insufficientFeatures: return "limited: needs texture/light"
            case .relocalizing: return "limited: relocalizing"
            @unknown default: return "limited"
            }
        }
    }

    var trackingIsNormal: Bool {
        guard let frame = currentFrame else { return false }
        if case .normal = frame.camera.trackingState { return true }
        return false
    }

    /// Luma straight from the Y plane of ARKit's biplanar YUV frame, with an
    /// optional bilinear downscale for the cheap detection pass.
    func grabGray(maxDimension: Int? = nil) -> GrayImage? {
        guard let frame = currentFrame else { return nil }
        return gray(from: frame.capturedImage, maxDimension: maxDimension)
    }

    func gray(from buffer: CVPixelBuffer, maxDimension: Int? = nil) -> GrayImage? {
        CVPixelBufferLockBaseAddress(buffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(buffer, .readOnly) }
        let width = CVPixelBufferGetWidth(buffer)
        let height = CVPixelBufferGetHeight(buffer)
        let stride = CVPixelBufferGetBytesPerRowOfPlane(buffer, 0)
        guard let base = CVPixelBufferGetBaseAddressOfPlane(buffer, 0) else { return nil }
        var gray = [UInt8](repeating: 0, count: width * height)
        gray.withUnsafeMutableBufferPointer { out in
            for y in 0..<height {
                let row = base.advanced(by: y * stride)
                for x in 0..<width {
                    out[y * width + x] = row[x]
                }
            }
        }
        guard let maxDimension, max(width, height) > maxDimension else {
            return GrayImage(data: gray, width: width, height: height)
        }
        let factor = Double(maxDimension) / Double(max(width, height))
        let dw = max(1, Int((Double(width) * factor).rounded()))
        let dh = max(1, Int((Double(height) * factor).rounded()))
        var small = [UInt8](repeating: 0, count: dw * dh)
        let sx = Double(width) / Double(dw)
        let sy = Double(height) / Double(dh)
        for y in 0..<dh {
            let fy = min(Double(height - 1), (Double(y) + 0.5) * sy - 0.5)
            let y0 = max(0, Int(fy.rounded(.down)))
            let y1 = min(height - 1, y0 + 1)
            let wy = fy - Double(y0)
            for x in 0..<dw {
                let fx = min(Double(width - 1), (Double(x) + 0.5) * sx - 0.5)
                let x0 = max(0, Int(fx.rounded(.down)))
                let x1 = min(width - 1, x0 + 1)
                let wx = fx - Double(x0)
                let top = Double(gray[y0 * width + x0]) * (1 - wx) + Double(gray[y0 * width + x1]) * wx
                let bottom = Double(gray[y1 * width + x0]) * (1 - wx) + Double(gray[y1 * width + x1]) * wx
                small[y * dw + x] = UInt8(max(0, min(255, (top * (1 - wy) + bottom * wy).rounded())))
            }
        }
        return GrayImage(data: small, width: dw, height: dh)
    }

    func grabJPEG(quality: Double = 0.95) -> (data: Data, width: Int, height: Int)? {
        guard let frame = currentFrame else { return nil }
        let buffer = frame.capturedImage
        let image = CIImage(cvPixelBuffer: buffer)
        guard let colorSpace = CGColorSpace(name: CGColorSpace.sRGB),
              let data = ciContext.jpegRepresentation(of: image, colorSpace: colorSpace,
                                                      options: [CIImageRepresentationOption(rawValue: kCGImageDestinationLossyCompressionQuality as String): quality]) else {
            return nil
        }
        return (data, CVPixelBufferGetWidth(buffer), CVPixelBufferGetHeight(buffer))
    }

    // MARK: LiDAR depth (the no-tag base anchor)

    static var supportsDepthAnchor: Bool {
        ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh)
    }

    /// The world point under an image pixel, from scene depth (z-distance)
    /// unprojected through the frame's intrinsics. nil where LiDAR has no
    /// return (range, reflectivity).
    func worldPoint(atImagePixel u: Double, v: Double, frame: ARFrame) -> Vec3? {
        guard let depthData = frame.smoothedSceneDepth ?? frame.sceneDepth else { return nil }
        let map = depthData.depthMap
        CVPixelBufferLockBaseAddress(map, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(map, .readOnly) }
        let dw = CVPixelBufferGetWidth(map)
        let dh = CVPixelBufferGetHeight(map)
        let w = CVPixelBufferGetWidth(frame.capturedImage)
        let h = CVPixelBufferGetHeight(frame.capturedImage)
        let x = min(dw - 1, max(0, Int(u / Double(w) * Double(dw))))
        let y = min(dh - 1, max(0, Int(v / Double(h) * Double(dh))))
        let stride = CVPixelBufferGetBytesPerRow(map)
        guard let base = CVPixelBufferGetBaseAddress(map) else { return nil }
        let distance = base.advanced(by: y * stride + x * 4).assumingMemoryBound(to: Float.self).pointee
        guard distance.isFinite, distance > 0.15, distance < 4 else { return nil }
        let k = frame.camera.intrinsics
        let xn = (u - Double(k.columns.2.x)) / Double(k.columns.0.x)
        let yn = (v - Double(k.columns.2.y)) / Double(k.columns.1.y)
        let pCam = simd_float4(Float(xn) * distance, Float(yn) * distance, distance, 1)
        let pWorld = frame.camera.transform * pCam
        return [Double(pWorld.x), Double(pWorld.y), Double(pWorld.z)]
    }

    /// Scene mesh vertices in the ARKit world frame, subsampled to `limit`.
    func scenePoints(limit: Int = 20000) -> [Vec3] {
        guard let frame = currentFrame else { return [] }
        let meshes = frame.anchors.compactMap { $0 as? ARMeshAnchor }
        var total = 0
        for mesh in meshes { total += mesh.geometry.vertices.count }
        let step = max(1, total / limit)
        var out: [Vec3] = []
        var index = 0
        for mesh in meshes {
            let vertices = mesh.geometry.vertices
            let contents = vertices.buffer.contents()
            for i in 0..<vertices.count {
                defer { index += 1 }
                if index % step != 0 { continue }
                let pointer = contents.advanced(by: vertices.offset + i * vertices.stride)
                    .assumingMemoryBound(to: Float.self)
                let local = simd_float4(pointer[0], pointer[1], pointer[2], 1)
                let world = mesh.transform * local
                out.append([Double(world.x), Double(world.y), Double(world.z)])
            }
        }
        return out
    }
}

struct CameraError: Error, CustomStringConvertible {
    var message: String
    var description: String { message }
    static func message(_ text: String) -> CameraError { CameraError(message: text) }
}

/// Letterboxed ARKit preview. The view is sized to the oriented image's aspect
/// ratio by StageView, so ARSCNView's aspect-fill is exactly object-fit: contain.
final class ARPreviewView: UIView {
    let scnView: ARSCNView

    init(session: ARSession) {
        scnView = ARSCNView(frame: .zero)
        super.init(frame: .zero)
        backgroundColor = .black
        scnView.session = session
        scnView.scene = SCNScene()
        scnView.rendersContinuously = true
        scnView.autoresizingMask = [.flexibleWidth, .flexibleHeight]
        addSubview(scnView)
    }

    required init?(coder: NSCoder) { fatalError("init(coder:) has not been implemented") }

    override func layoutSubviews() {
        super.layoutSubviews()
        scnView.frame = bounds
    }
}

struct CameraPreview: UIViewRepresentable {
    let controller: CameraController

    func makeUIView(context: Context) -> ARPreviewView {
        ARPreviewView(session: controller.session)
    }

    func updateUIView(_ view: ARPreviewView, context: Context) {}
}

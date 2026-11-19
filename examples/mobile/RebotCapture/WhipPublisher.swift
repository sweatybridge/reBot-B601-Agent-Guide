/* WHIP publisher: the native counterpart of the page's RTCPeerConnection +
 * POST /whip flow. ARKit frames are fed into a custom RTCVideoSource with an
 * upright rotation; after the handshake RTP goes straight to MediaMTX on the LAN.
 */
import Foundation
import AVFoundation
import WebRTC
import RebotCore

final class WhipPublisher: NSObject, RTCPeerConnectionDelegate {
    private let factory: RTCPeerConnectionFactory
    private var pc: RTCPeerConnection?
    private var source: RTCVideoSource?
    private var adaptedSize: (Int, Int)?
    private let feedLock = NSLock()
    private(set) var sessionLocation: String?

    /// Connection state string for the Live chip.
    var onState: ((String) -> Void)?

    override init() {
        RTCInitializeSSL()
        factory = RTCPeerConnectionFactory()
        super.init()
    }

    var active: Bool { pc != nil }

    func start(api: APIClient) async throws {
        guard pc == nil else { return }
        let config = RTCConfiguration()
        config.iceServers = []
        config.sdpSemantics = .unifiedPlan
        let constraints = RTCMediaConstraints(mandatoryConstraints: nil, optionalConstraints: nil)
        guard let connection = factory.peerConnection(with: config, constraints: constraints, delegate: self) else {
            throw APIFailure(status: 0, message: "could not create the peer connection")
        }
        pc = connection
        let videoSource = factory.videoSource()
        feedLock.lock()
        source = videoSource
        feedLock.unlock()
        let track = factory.videoTrack(with: videoSource, trackId: "video0")
        track.isEnabled = true
        connection.add(track, streamIds: ["stream0"])

        let offer: RTCSessionDescription = try await withCheckedThrowingContinuation { continuation in
            connection.offer(for: constraints) { sdp, error in
                if let sdp { continuation.resume(returning: sdp) }
                else { continuation.resume(throwing: APIFailure(status: 0, message: error?.localizedDescription ?? "offer failed")) }
            }
        }
        try await withCheckedThrowingContinuation { (continuation: CheckedContinuation<Void, Error>) in
            connection.setLocalDescription(offer) { error in
                if let error { continuation.resume(throwing: APIFailure(status: 0, message: error.localizedDescription)) }
                else { continuation.resume() }
            }
        }
        await waitForIce(connection, milliseconds: 2500)

        guard let sdp = connection.localDescription?.sdp else {
            throw APIFailure(status: 0, message: "no local SDP")
        }
        let (answer, location) = try await api.whipOffer(sdp: sdp)
        sessionLocation = location
        let remote = RTCSessionDescription(type: .answer, sdp: answer)
        try await withCheckedThrowingContinuation { (continuation: CheckedContinuation<Void, Error>) in
            connection.setRemoteDescription(remote) { error in
                if let error { continuation.resume(throwing: APIFailure(status: 0, message: error.localizedDescription)) }
                else { continuation.resume() }
            }
        }
    }

    private func waitForIce(_ connection: RTCPeerConnection, milliseconds: Int) async {
        let deadline = Date().addingTimeInterval(Double(milliseconds) / 1000)
        while connection.iceGatheringState != .complete && Date() < deadline {
            try? await Task.sleep(nanoseconds: 50_000_000)
        }
    }

    /// Feed one ARKit frame. Called on the session delegate queue. The buffer
    /// is sensor-native landscape; the rotation makes it upright for the
    /// interface angle it was captured at.
    func feed(_ buffer: CVPixelBuffer, angle: Int) {
        feedLock.lock()
        defer { feedLock.unlock() }
        guard let source else { return }
        let width = CVPixelBufferGetWidth(buffer)
        let height = CVPixelBufferGetHeight(buffer)
        if adaptedSize == nil || adaptedSize! != (width, height) {
            adaptedSize = (width, height)
            source.adaptOutputFormat(toWidth: Int32(width), height: Int32(height), fps: 60)
        }
        let rotation: RTCVideoRotation
        switch angle {
        case 90: rotation = ._0        // landscapeLeft matches the sensor
        case 180: rotation = ._270
        case 270: rotation = ._180
        default: rotation = ._90       // portrait
        }
        let rtcBuffer = RTCCVPixelBuffer(pixelBuffer: buffer)
        let timestamp = Int64(ProcessInfo.processInfo.systemUptime * 1_000_000_000)
        let frame = RTCVideoFrame(buffer: rtcBuffer, rotation: rotation, timeStampNs: timestamp)
        source.capturer(RTCVideoCapturer(), didCapture: frame)
    }

    func stop(api: APIClient?) async {
        let connection = pc
        let location = sessionLocation
        pc = nil
        feedLock.lock()
        source = nil
        adaptedSize = nil
        feedLock.unlock()
        sessionLocation = nil
        connection?.close()
        if let location, let api {
            await api.whipDelete(sessionPath: location)
        }
        DispatchQueue.main.async { self.onState?("off") }
    }

    // MARK: RTCPeerConnectionDelegate

    func peerConnection(_ peerConnection: RTCPeerConnection, didChange stateChanged: RTCSignalingState) {}
    func peerConnection(_ peerConnection: RTCPeerConnection, didAdd stream: RTCMediaStream) {}
    func peerConnection(_ peerConnection: RTCPeerConnection, didRemove stream: RTCMediaStream) {}
    func peerConnectionShouldNegotiate(_ peerConnection: RTCPeerConnection) {}
    func peerConnection(_ peerConnection: RTCPeerConnection, didChange newState: RTCIceConnectionState) {}
    func peerConnection(_ peerConnection: RTCPeerConnection, didRemove candidates: [RTCIceCandidate]) {}
    func peerConnection(_ peerConnection: RTCPeerConnection, didOpen dataChannel: RTCDataChannel) {}

    func peerConnection(_ peerConnection: RTCPeerConnection, didChange newState: RTCIceGatheringState) {}

    func peerConnection(_ peerConnection: RTCPeerConnection, didGenerate candidate: RTCIceCandidate) {}

    func peerConnection(_ peerConnection: RTCPeerConnection, didChange newState: RTCPeerConnectionState) {
        let text: String
        switch newState {
        case .new: text = "starting"
        case .connecting: text = "connecting"
        case .connected: text = "connected"
        case .disconnected: text = "disconnected"
        case .failed: text = "failed"
        case .closed: text = "closed"
        @unknown default: text = "unknown"
        }
        DispatchQueue.main.async { self.onState?(text) }
    }
}

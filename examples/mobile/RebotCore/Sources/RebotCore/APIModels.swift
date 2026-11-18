/* Wire models for the capture server routes the phone page uses.
 * Only the fields the client reads are declared; the rest is ignored.
 */
import Foundation

public struct StreamTurn: Codable, Equatable {
    public var turns: Int?
    public var source: String?
}

public struct RigInfo: Codable, Equatable {
    public var fingerprint: String?
}

/// GET /reach.json
public struct ReachConfig: Codable {
    public var ok: Bool?
    public var mode: String?
    public var planeZ: Double
    public var hoverM: Double
    public var tipM: Double
    public var marker: MarkerConfig?
    public var calibration: Calibration
    public var streamTurn: StreamTurn?
    public var TBaseCam: Mat4?
    public var rig: RigInfo?

    enum CodingKeys: String, CodingKey {
        case ok, mode
        case planeZ = "plane_z"
        case hoverM = "hover_m"
        case tipM = "tip_m"
        case marker, calibration
        case streamTurn = "stream_turn"
        case TBaseCam = "T_base_cam"
        case rig
    }
}

/// GET /health (the fields phone.html reads).
public struct Health: Codable {
    public var ok: Bool?
    public var latest: String?
    public var whip: String?
    public var mode: String?
    public var rig: RigInfo?
    public var planeZ: Double?
    public var hoverM: Double?
    public var minZ: Double?
    public var execute: Bool?
    public var streamTurn: StreamTurn?

    enum CodingKeys: String, CodingKey {
        case ok, latest, whip, mode, rig, execute
        case planeZ = "plane_z"
        case hoverM = "hover_m"
        case minZ = "min_z"
        case streamTurn = "stream_turn"
    }
}

/// POST /sequence response.
public struct SequenceResponse: Codable {
    public struct Plan: Codable {
        public struct Summary: Codable {
            public var markers: Int?
            public var waypoints: Int?
            public var durationS: Double?
            enum CodingKeys: String, CodingKey {
                case markers, waypoints
                case durationS = "duration_s"
            }
        }
        public var id: String?
        public var summary: Summary?
        public var execute: Bool?
    }
    public var ok: Bool?
    public var plan: Plan?
    public var execute: Bool?
    public var error: String?
    public var stage: String?
}

/// POST /execute response.
public struct ExecuteResponse: Codable {
    public var ok: Bool?
    public var error: String?
}

/// POST /capture response.
public struct CaptureResponse: Codable {
    public var ok: Bool?
    public var id: String?
    public var error: String?
}

/// A generic error body ({ok:false, error:"..."} or plain text).
public struct ServerError: Codable {
    public var ok: Bool?
    public var error: String?
}

/// GET /pose (proxied to the arm gateway): the rebot-edge pose dump.
public struct PoseResponse: Codable {
    public struct Pose: Codable {
        public var q: [Double]?
    }
    public var ok: Bool?
    public var pose: Pose?
    public var error: String?
}

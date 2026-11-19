/* Token-aware HTTP client for the capture server, the native counterpart of
 * web/api.js + the page's fetch calls.
 *
 * TLS: the host uses a private CA (make_cert.py). Instead of installing a
 * configuration profile like Safari needs, the app evaluates the server trust
 * against the CA the user imported in Settings. System-trusted certificates
 * are accepted normally.
 */
import Foundation
import RebotCore

struct APIFailure: Error, CustomStringConvertible {
    var status: Int
    var message: String
    var description: String { message }
}

final class ServerTrustDelegate: NSObject, URLSessionDelegate {
    var caCertificate: SecCertificate?

    func urlSession(_ session: URLSession, didReceive challenge: URLAuthenticationChallenge,
                    completionHandler: @escaping (URLSession.AuthChallengeDisposition, URLCredential?) -> Void) {
        guard challenge.protectionSpace.authenticationMethod == NSURLAuthenticationMethodServerTrust,
              let trust = challenge.protectionSpace.serverTrust else {
            completionHandler(.performDefaultHandling, nil)
            return
        }
        if SecTrustEvaluateWithError(trust, nil) {
            completionHandler(.useCredential, URLCredential(trust: trust))
            return
        }
        if let ca = caCertificate {
            SecTrustSetAnchorCertificates(trust, [ca] as CFArray)
            SecTrustSetAnchorCertificatesOnly(trust, true)
            if SecTrustEvaluateWithError(trust, nil) {
                completionHandler(.useCredential, URLCredential(trust: trust))
                return
            }
        }
        completionHandler(.cancelAuthenticationChallenge, nil)
    }
}

final class APIClient {
    private let base: URL
    private let token: String
    private let session: URLSession
    private let trustDelegate: ServerTrustDelegate

    init(base: URL, token: String, ca: SecCertificate?) {
        self.base = base
        self.token = token
        trustDelegate = ServerTrustDelegate()
        trustDelegate.caCertificate = ca
        let config = URLSessionConfiguration.ephemeral
        config.timeoutIntervalForRequest = 30
        config.requestCachePolicy = .reloadIgnoringLocalCacheData
        session = URLSession(configuration: config, delegate: trustDelegate, delegateQueue: nil)
    }

    func invalidate() { session.invalidateAndCancel() }

    private func request(_ path: String, method: String = "GET",
                         contentType: String? = nil, body: Data? = nil,
                         timeout: TimeInterval? = nil) async throws -> (Data, HTTPURLResponse) {
        var request = URLRequest(url: base.appendingPathComponent(path))
        request.httpMethod = method
        request.cachePolicy = .reloadIgnoringLocalCacheData
        if !token.isEmpty { request.setValue(token, forHTTPHeaderField: "X-Rebot-Token") }
        if let contentType { request.setValue(contentType, forHTTPHeaderField: "Content-Type") }
        if let timeout { request.timeoutInterval = timeout }
        request.httpBody = body
        let (data, response) = try await session.data(for: request)
        guard let http = response as? HTTPURLResponse else {
            throw APIFailure(status: 0, message: "no HTTP response")
        }
        return (data, http)
    }

    /// Defensive JSON parse, like RebotApi.readJson: a proxy can answer text.
    private func decode<T: Decodable>(_ type: T.Type, data: Data, http: HTTPURLResponse) throws -> T {
        if let decoded = try? JSONDecoder().decode(type, from: data) {
            return decoded
        }
        let text = String(data: data.prefix(300), encoding: .utf8)?.trimmingCharacters(in: .whitespacesAndNewlines) ?? ""
        throw APIFailure(status: http.statusCode, message: text.isEmpty ? "server said \(http.statusCode)" : text)
    }

    private func checkOK(_ data: Data, _ http: HTTPURLResponse) throws {
        if !(200..<300).contains(http.statusCode) {
            let body = try? JSONDecoder().decode(ServerError.self, from: data)
            let text = String(data: data.prefix(160), encoding: .utf8) ?? ""
            throw APIFailure(status: http.statusCode, message: body?.error ?? (text.isEmpty ? "server said \(http.statusCode)" : text))
        }
    }

    // MARK: routes

    func health() async throws -> Health {
        let (data, http) = try await request("health", timeout: 8)
        try checkOK(data, http)
        return try decode(Health.self, data: data, http: http)
    }

    func reachConfig() async throws -> ReachConfig {
        let (data, http) = try await request("reach.json")
        try checkOK(data, http)
        return try decode(ReachConfig.self, data: data, http: http)
    }

    /// The arm's live joint angles, for the depth-based base anchor.
    func pose() async throws -> [Double] {
        let (data, http) = try await request("pose", timeout: 12)
        try checkOK(data, http)
        let response = try decode(PoseResponse.self, data: data, http: http)
        guard let q = response.pose?.q, q.count >= 6 else {
            throw APIFailure(status: http.statusCode, message: "the arm gateway returned no joint angles")
        }
        return Array(q.prefix(6))
    }

    func publishOrientation(angle: Int, size: [Int]) async throws {
        let body = try JSONSerialization.data(withJSONObject: ["angle": angle, "size": size])
        let (data, http) = try await request("orientation", method: "POST", contentType: "application/json", body: body)
        try checkOK(data, http)
    }

    func sequence(targets: [TipTarget]) async throws -> SequenceResponse {
        let body = try JSONEncoder().encode(["targets": targets])
        let (data, http) = try await request("sequence", method: "POST", contentType: "application/json", body: body, timeout: 320)
        if let response = try? JSONDecoder().decode(SequenceResponse.self, from: data),
           (200..<300).contains(http.statusCode), response.ok == true {
            return response
        }
        let bodyError = try? JSONDecoder().decode(ServerError.self, from: data)
        throw APIFailure(status: http.statusCode,
                         message: bodyError?.error ?? "sequence failed: \(http.statusCode)")
    }

    func execute(planID: String) async throws -> ExecuteResponse {
        let body = try JSONEncoder().encode(["id": planID])
        let (data, http) = try await request("execute", method: "POST", contentType: "application/json", body: body, timeout: 380)
        if let response = try? JSONDecoder().decode(ExecuteResponse.self, from: data),
           (200..<300).contains(http.statusCode), response.ok != false {
            return response
        }
        let bodyError = try? JSONDecoder().decode(ServerError.self, from: data)
        throw APIFailure(status: http.statusCode,
                         message: bodyError?.error ?? "execute failed: \(http.statusCode)")
    }

    func capture(meta: Data, jpeg: Data) async throws -> CaptureResponse {
        let boundary = "rebot-" + UUID().uuidString
        var body = Data()
        func append(_ text: String) { body.append(text.data(using: .utf8)!) }
        append("--\(boundary)\r\nContent-Disposition: form-data; name=\"meta\"\r\n\r\n")
        body.append(meta)
        append("\r\n--\(boundary)\r\nContent-Disposition: form-data; name=\"frame\"; filename=\"frame.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n")
        body.append(jpeg)
        append("\r\n--\(boundary)--\r\n")
        let (data, http) = try await request("capture", method: "POST",
                                             contentType: "multipart/form-data; boundary=\(boundary)",
                                             body: body, timeout: 60)
        if let response = try? JSONDecoder().decode(CaptureResponse.self, from: data),
           (200..<300).contains(http.statusCode) {
            return response
        }
        let bodyError = try? JSONDecoder().decode(ServerError.self, from: data)
        let text = String(data: data.prefix(160), encoding: .utf8) ?? ""
        throw APIFailure(status: http.statusCode,
                         message: bodyError?.error ?? (text.isEmpty ? "server said \(http.statusCode)" : text))
    }

    /// WHIP: POST the SDP offer, return the answer and the session Location.
    func whipOffer(sdp: String) async throws -> (answer: String, location: String?) {
        let (data, http) = try await request("whip", method: "POST", contentType: "application/sdp",
                                             body: sdp.data(using: .utf8), timeout: 20)
        try checkOK(data, http)
        return (String(data: data, encoding: .utf8) ?? "", http.value(forHTTPHeaderField: "Location"))
    }

    func whipDelete(sessionPath: String) async {
        let session = sessionPath.split(separator: "/").last.map(String.init) ?? ""
        guard !session.isEmpty else { return }
        _ = try? await request("whip/\(session)", method: "DELETE", timeout: 10)
    }
}

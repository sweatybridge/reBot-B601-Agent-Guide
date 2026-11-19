/* Host URL, token and the optional trusted CA, persisted like the page's
 * localStorage. The CA replaces the iOS profile install the browser needs:
 * the app evaluates the server certificate against it directly.
 */
import Foundation
import Security
import Combine

final class SettingsStore: ObservableObject {
    @Published var hostURL: String {
        didSet { UserDefaults.standard.set(hostURL, forKey: "rebot.host") }
    }
    @Published var token: String {
        didSet { UserDefaults.standard.set(token, forKey: "rebot.token") }
    }
    @Published var caDER: Data? {
        didSet { UserDefaults.standard.set(caDER, forKey: "rebot.ca") }
    }

    init() {
        hostURL = UserDefaults.standard.string(forKey: "rebot.host") ?? ""
        token = UserDefaults.standard.string(forKey: "rebot.token") ?? ""
        caDER = UserDefaults.standard.data(forKey: "rebot.ca")
    }

    var baseURL: URL? {
        var text = hostURL.trimmingCharacters(in: .whitespacesAndNewlines)
        if text.isEmpty { return nil }
        if !text.contains("://") { text = "https://" + text }
        if text.hasSuffix("/") { text.removeLast() }
        guard let url = URL(string: text), url.host != nil else { return nil }
        return url
    }

    var caCertificate: SecCertificate? {
        guard let caDER else { return nil }
        return SecCertificateCreateWithData(nil, caDER as CFData)
    }

    /// Accepts DER (.cer/.der) or PEM (.pem/.crt) certificate data.
    func importCA(data: Data) -> Bool {
        var der = data
        if let text = String(data: data, encoding: .utf8), text.contains("-----BEGIN") {
            let lines = text.split(separator: "\n").filter { !$0.hasPrefix("-----") }
            guard let decoded = Data(base64Encoded: lines.joined()) else { return false }
            der = decoded
        }
        guard SecCertificateCreateWithData(nil, der as CFData) != nil else { return false }
        caDER = der
        return true
    }
}

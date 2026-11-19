/* Host, token and the trusted CA. The CA import replaces the profile install
 * the browser flow needs: AirDrop tls/ca.pem from the host and pick it here.
 */
import SwiftUI
import UniformTypeIdentifiers

struct SettingsView: View {
    @EnvironmentObject var model: AppModel
    @EnvironmentObject var settings: SettingsStore
    @Environment(\.dismiss) private var dismiss
    @State private var importing = false
    @State private var importError = ""

    var body: some View {
        NavigationView {
            Form {
                Section {
                    TextField("https://192.168.1.23:8443", text: $settings.hostURL)
                        .keyboardType(.URL)
                        .textInputAutocapitalization(.never)
                        .autocorrectionDisabled()
                    SecureField("token (the --token the server runs with)", text: $settings.token)
                        .textInputAutocapitalization(.never)
                        .autocorrectionDisabled()
                } header: {
                    Text("Capture server")
                } footer: {
                    Text("The phone only ever talks to :8443. The token is the one the capture server was started with; it is sent as X-Rebot-Token.")
                }

                Section {
                    if settings.caDER != nil {
                        Label("CA certificate installed", systemImage: "checkmark.seal.fill")
                            .foregroundColor(Theme.good)
                        Button("Remove CA", role: .destructive) {
                            settings.caDER = nil
                        }
                    } else {
                        Button("Import CA certificate (ca.pem / .cer / .der)") {
                            importing = true
                        }
                    }
                    if !importError.isEmpty {
                        Text(importError).foregroundColor(Theme.bad)
                    }
                } header: {
                    Text("TLS")
                } footer: {
                    Text("The host signs its certificate with a private CA (make_cert.py). Import tls/ca.pem here once — no configuration profile needed. System-trusted certificates work without this.")
                }

                Section {
                    LabeledContent("Status", value: model.serverText)
                    if let health = model.health {
                        LabeledContent("Mode", value: health.mode ?? "-")
                        LabeledContent("Rig", value: health.rig?.fingerprint ?? "none")
                        LabeledContent("Execute", value: (health.execute ?? false) ? "enabled" : "off")
                    }
                } header: {
                    Text("Host")
                }
            }
            .navigationTitle("Settings")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .confirmationAction) {
                    Button("Done") { dismiss() }
                }
            }
            .fileImporter(isPresented: $importing,
                          allowedContentTypes: [UTType(filenameExtension: "pem") ?? .data,
                                                UTType(filenameExtension: "cer") ?? .data,
                                                UTType(filenameExtension: "crt") ?? .data,
                                                .data],
                          allowsMultipleSelection: false) { result in
                switch result {
                case .success(let urls):
                    guard let url = urls.first else { return }
                    let scoped = url.startAccessingSecurityScopedResource()
                    defer { if scoped { url.stopAccessingSecurityScopedResource() } }
                    if let data = try? Data(contentsOf: url), settings.importCA(data: data) {
                        importError = ""
                    } else {
                        importError = "that file is not a certificate"
                    }
                case .failure(let error):
                    importError = error.localizedDescription
                }
            }
        }
    }
}

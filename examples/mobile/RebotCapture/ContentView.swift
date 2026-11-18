/* Header, stage, log, footer: the phone.html layout in SwiftUI. */
import SwiftUI
import RebotCore

struct ContentView: View {
    @EnvironmentObject var model: AppModel
    @State private var showSettings = false
    @State private var showHelp = false

    var body: some View {
        VStack(spacing: 0) {
            header
            StageView()
                .frame(maxWidth: .infinity, maxHeight: .infinity)
            logPane
            footer
        }
        .background(Theme.bg.ignoresSafeArea())
        .preferredColorScheme(.dark)
        .sheet(isPresented: $showSettings) {
            SettingsView()
                .environmentObject(model)
                .environmentObject(model.settings)
        }
        .onAppear {
            if model.needsSetup { showSettings = true }
        }
    }

    // MARK: header

    private var header: some View {
        VStack(spacing: 8) {
            HStack(spacing: 10) {
                Circle()
                    .fill(model.hostOK ? Theme.good : Theme.bad)
                    .frame(width: 9, height: 9)
                    .shadow(color: (model.hostOK ? Theme.good : Theme.bad).opacity(0.25), radius: 3)
                Text("B601 capture")
                    .font(.system(size: 16, weight: .semibold))
                    .foregroundColor(Theme.ink)
                Text(model.serverText)
                    .font(.system(size: 13))
                    .foregroundColor(Theme.dim)
                    .lineLimit(1)
                Spacer()
                Button {
                    showSettings = true
                } label: {
                    Image(systemName: "gearshape")
                        .foregroundColor(Theme.dim)
                }
            }
            chipsRow
        }
        .padding(.horizontal, 14)
        .padding(.top, 10)
        .padding(.bottom, 8)
        .background(Theme.panel)
        .overlay(Rectangle().frame(height: 1).foregroundColor(Theme.line), alignment: .bottom)
    }

    private var chipsRow: some View {
        ScrollView(.horizontal, showsIndicators: false) {
            HStack(spacing: 6) {
                if !model.cameraOn {
                    Chip(text: "camera off", plain: true)
                } else {
                    if let size = model.camera.frameSize {
                        Chip(text: "\(size.width)x\(size.height)", plain: true)
                    }
                    Chip(text: "60 fps", plain: true)
                    Chip(text: "ARKit", plain: true)
                    let tracking = model.trackingStatus
                    Chip(text: "tracking: " + tracking, warn: tracking != "normal", plain: tracking == "normal")
                    if model.cameraMode == "handheld" {
                        Chip(text: model.anchor == nil ? "Anchor: none" : "Re-anchor",
                             on: model.anchor != nil, warn: model.anchor == nil,
                             disabled: model.reanchoring) {
                            Task { await model.reanchor(manual: true) }
                        }
                    }
                    if model.calib {
                        Chip(text: "calib \(model.calibCount)", on: true)
                    }
                }
                if model.health?.whip == "" && model.liveState == "off" {
                    Chip(text: "live disabled on host", plain: true)
                }
                HeightBadge(text: model.heightLabel, adjusting: model.adjusting)
            }
        }
    }

    // MARK: log

    private var logPane: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 3) {
                ForEach(model.lines) { line in
                    HStack(alignment: .top, spacing: 8) {
                        Text(line.time)
                            .font(.system(size: 12.5))
                            .monospacedDigit()
                            .foregroundColor(Color(red: 0x62 / 255, green: 0x6d / 255, blue: 0x7d / 255))
                        Text(line.text)
                            .font(.system(size: 12.5))
                            .foregroundColor(color(for: line.kind))
                        Spacer(minLength: 0)
                    }
                }
            }
            .padding(.horizontal, 14)
            .padding(.vertical, 8)
        }
        .frame(maxHeight: 140)
        .fixedSize(horizontal: false, vertical: true)
        .background(Theme.panel)
        .overlay(Rectangle().frame(height: 1).foregroundColor(Theme.line), alignment: .top)
    }

    private func color(for kind: LogKind) -> Color {
        switch kind {
        case .plain: return Theme.dim
        case .good: return Theme.good
        case .warn: return Theme.warn
        case .bad: return Theme.bad
        }
    }

    // MARK: footer

    private var shotDisabled: Bool {
        if model.calib { return model.busy || !model.cameraOn }
        return model.busy || !model.cameraOn || model.locating || model.markers.isEmpty
    }

    private var shotTitle: String {
        if model.calib { return model.busy ? "Sending..." : "Capture" }
        return model.busy ? (model.busyText.isEmpty ? "Planning..." : model.busyText) : "Move arm"
    }

    private var footer: some View {
        VStack(spacing: 8) {
            HStack(spacing: 8) {
                Button {
                    Task { await model.startCamera() }
                } label: {
                    Text(model.starting ? "Starting..." : (model.cameraOn ? "Restart camera" : "Start camera"))
                        .font(.system(size: 16, weight: .semibold))
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 15)
                        .background(model.starting ? Theme.chipBG : Theme.accent)
                        .foregroundColor(model.starting ? Theme.dim : Color(red: 0x04 / 255, green: 0x12 / 255, blue: 0x1f / 255))
                        .clipShape(RoundedRectangle(cornerRadius: 12))
                }
                .disabled(model.starting)

                Button {
                    if model.calib {
                        Task { await model.capture() }
                    } else {
                        Task { await model.moveArm() }
                    }
                } label: {
                    Text(shotTitle)
                        .font(.system(size: 16, weight: .semibold))
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 15)
                        .background(Theme.chipBG)
                        .foregroundColor(shotDisabled ? Theme.dim : Theme.ink)
                        .clipShape(RoundedRectangle(cornerRadius: 12))
                        .overlay(RoundedRectangle(cornerRadius: 12).stroke(Theme.line, lineWidth: 1))
                }
                .disabled(shotDisabled)
                .opacity(shotDisabled ? 0.4 : 1)
            }

            FlowChips()

            DisclosureGroup(isExpanded: $showHelp) {
                VStack(alignment: .leading, spacing: 4) {
                    helpLine("1. Handheld (the default). Point the camera at the base tag once to anchor. ARKit tracking then carries the pose at frame rate — the tag may leave the view, and it is re-solved every 2 s when visible to correct drift. Tap the table to add a numbered marker, up to 10.")
                    helpLine("2. Height. Drag up or down anywhere in the preview: the badge shows how far above the worktop the arm ends. Each marker keeps the height it was tapped at. The drag stops at the host's floor (10 cm by default).")
                    helpLine("3. Move arm plans and runs the markers in order. Amber odd markers half-close the gripper; green even markers open it. The arm returns to where it began. Clear marker removes the last one.")
                    helpLine("4. Taps are refused while ARKit tracking is limited or before the base is anchored. No tag in view? On LiDAR devices, Depth anchor fits the arm itself: tap the base, hold the arm still. It is coarse (~1-3 cm) and a tag re-anchor supersedes it. A fixed rig from /live needs no tag at all.")
                    helpLine("5. Calibration mode turns Move arm into Capture for checkerboard shots — optional now that intrinsics come from ARKit (pinhole, no lens distortion).")
                    helpLine("6. Reach paints the band of table markers can be planned in, at the current height, following the ARKit pose.")
                }
                .padding(.top, 6)
            } label: {
                Text("How to use").font(.system(size: 12.5)).foregroundColor(Theme.dim)
            }
            .tint(Theme.dim)
        }
        .padding(.horizontal, 14)
        .padding(.top, 10)
        .padding(.bottom, 10)
        .background(Theme.panel)
        .overlay(Rectangle().frame(height: 1).foregroundColor(Theme.line), alignment: .top)
    }

    private func helpLine(_ text: String) -> some View {
        Text(text).font(.system(size: 12.5)).foregroundColor(Theme.dim)
    }
}

/// The toggle chip row, wrapped like the page's flex-wrap.
struct FlowChips: View {
    @EnvironmentObject var model: AppModel

    var body: some View {
        FlowLayout(spacing: 6) {
            Chip(text: "Camera: " + (model.cameraMode == "fixed" ? "fixed rig" : "handheld tag"),
                 on: model.cameraMode == "fixed") { model.toggleCameraMode() }
            if CameraController.supportsDepthAnchor {
                Chip(text: "Depth anchor",
                     disabled: !model.cameraOn || model.cameraMode != "handheld" || model.busy) {
                    model.beginDepthAnchor()
                }
            }
            Chip(text: "Reach: " + (model.reachOn ? "on" : "off"), on: model.reachOn) { model.toggleReach() }
            Chip(text: "Calibration mode: " + (model.calib ? "on" : "off"), on: model.calib) { model.toggleCalibration() }
            Chip(text: "Live preview: " + model.liveState,
                 on: model.liveState == "connected",
                 disabled: model.health?.whip == "" && model.liveState == "off") {
                Task { await model.toggleLive() }
            }
            Chip(text: "Clear marker", disabled: model.busy) {
                if model.clearLastMarker() {
                    model.log("last marker cleared", .warn)
                } else {
                    model.log(model.locating ? "still locating a marker" : "no markers to clear", .warn)
                }
            }
        }
    }
}

/// Minimal flow layout: rows of chips that wrap at the container edge.
struct FlowLayout: Layout {
    var spacing: CGFloat = 6

    func sizeThatFits(proposal: ProposedViewSize, subviews: Subviews, cache: inout ()) -> CGSize {
        let maxWidth = proposal.width ?? .infinity
        var x: CGFloat = 0, y: CGFloat = 0, rowHeight: CGFloat = 0
        for subview in subviews {
            let size = subview.sizeThatFits(.unspecified)
            if x > 0 && x + size.width > maxWidth {
                x = 0
                y += rowHeight + spacing
                rowHeight = 0
            }
            x += size.width + spacing
            rowHeight = max(rowHeight, size.height)
        }
        return CGSize(width: maxWidth == .infinity ? x : maxWidth, height: y + rowHeight)
    }

    func placeSubviews(in bounds: CGRect, proposal: ProposedViewSize, subviews: Subviews, cache: inout ()) {
        var x = bounds.minX, y = bounds.minY, rowHeight: CGFloat = 0
        for subview in subviews {
            let size = subview.sizeThatFits(.unspecified)
            if x > bounds.minX && x + size.width > bounds.maxX {
                x = bounds.minX
                y += rowHeight + spacing
                rowHeight = 0
            }
            subview.place(at: CGPoint(x: x, y: y), proposal: ProposedViewSize(size))
            x += size.width + spacing
            rowHeight = max(rowHeight, size.height)
        }
    }
}

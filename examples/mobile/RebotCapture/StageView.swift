/* The stage: letterboxed ARKit preview, reach band canvas, numbered pins, and
 * the tap-vs-height-drag gesture (height.js).
 *
 * The preview is an ARSCNView sized to the oriented image's aspect ratio, so
 * its aspect-fill is exactly object-fit: contain and the letterbox math (and
 * ARFrame.displayTransform) stay consistent with the taps.
 */
import SwiftUI
import UIKit
import RebotCore

struct StageView: View {
    @EnvironmentObject var model: AppModel
    @State private var dragStart: CGPoint?
    @State private var dragStartHeight = 0.10
    @State private var wasAdjusting = false

    var body: some View {
        GeometryReader { geo in
            ZStack(alignment: .topLeading) {
                Color.black
                if model.cameraOn, let box = model.contentBox(stageSize: geo.size) {
                    CameraPreview(controller: model.camera)
                        .frame(width: box.width, height: box.height)
                        .position(x: box.left + box.width / 2, y: box.top + box.height / 2)
                }
                overlay(size: geo.size)
                if !model.cameraOn {
                    hint
                }
            }
            .frame(width: geo.size.width, height: geo.size.height)
            .clipped()
            .contentShape(Rectangle())
            .gesture(gesture(size: geo.size))
        }
    }

    private var hint: some View {
        VStack {
            Spacer()
            HStack {
                Spacer()
                Text("Tap Start camera, point at the base tag once to anchor, then tap the table to place numbered markers. ARKit tracking keeps the pose while the tag is out of view. Drag up or down on the preview to set the arm's height.")
                    .font(.system(size: 14))
                    .foregroundColor(Theme.dim)
                    .multilineTextAlignment(.center)
                    .padding(24)
                Spacer()
            }
            Spacer()
        }
        .allowsHitTesting(false)
    }

    // The band and the pins re-evaluate at ~30 fps while anything is live.
    private func overlay(size: CGSize) -> some View {
        TimelineView(.animation(minimumInterval: 1.0 / 30.0, paused: !model.reachOn && model.markers.isEmpty)) { _ in
            ZStack(alignment: .topLeading) {
                Canvas { context, canvasSize in
                    drawReach(context: &context, size: canvasSize)
                }
                .allowsHitTesting(false)
                if model.depthTapPending {
                    HStack {
                        Spacer()
                        Text("Tap the arm base — hold the arm still")
                            .font(.system(size: 13, weight: .semibold))
                            .foregroundColor(Theme.warn)
                            .padding(.horizontal, 12)
                            .padding(.vertical, 8)
                            .background(Theme.bg.opacity(0.8))
                            .clipShape(Capsule())
                        Spacer()
                    }
                    .padding(.top, 12)
                    .allowsHitTesting(false)
                }
                if !model.calib {
                    ForEach(Array(model.markers.enumerated()), id: \.element.id) { index, marker in
                        if let point = model.markerStagePoint(marker, stageSize: size) {
                            pin(index: index + 1, marker: marker)
                                .position(point)
                        }
                    }
                    .opacity(model.busy || model.locating ? 0.5 : 1)
                    .allowsHitTesting(false)
                }
            }
        }
    }

    private func pin(index: Int, marker: SequenceMarker) -> some View {
        let odd = index % 2 == 1
        let color = odd ? Theme.warn : Theme.good
        return Text("\(index)")
            .font(.system(size: 14, weight: .bold))
            .foregroundColor(color)
            .frame(width: 30, height: 30)
            .background(Theme.bg.opacity(0.55))
            .clipShape(Circle())
            .overlay(Circle().stroke(color, lineWidth: 2))
            .overlay(
                Circle().stroke(Theme.bad, lineWidth: 2).padding(-4)
                    .opacity(marker.radius > 0.75 ? 1 : 0)
            )
            .shadow(color: .black.opacity(0.45), radius: 4, y: 2)
    }

    // MARK: reach canvas

    private func drawReach(context: inout GraphicsContext, size: CGSize) {
        guard model.reachOn else { return }
        if let message = model.reachEmptyMessage {
            drawMessage(message, context: &context, size: size)
            return
        }
        guard let rings = model.reachRingPoints(stageSize: size),
              let box = model.contentBox(stageSize: size) else { return }
        var context = context
        context.clip(to: Path(CGRect(x: box.left, y: box.top, width: box.width, height: box.height)))

        if rings.outer.count > 1 && rings.inner.count > 1 {
            var band = Path()
            band.move(to: rings.outer[0])
            for point in rings.outer.dropFirst() { band.addLine(to: point) }
            for point in rings.inner.reversed() { band.addLine(to: point) }
            band.closeSubpath()
            context.fill(band, with: .color(Theme.good.opacity(0.14)))
        }
        let dash = StrokeStyle(lineWidth: 2, dash: [7, 5])
        for ring in [rings.outer, rings.inner] where ring.count > 1 {
            var path = Path()
            path.move(to: ring[0])
            for point in ring.dropFirst() { path.addLine(to: point) }
            context.stroke(path, with: .color(Theme.good.opacity(0.95)), style: dash)
        }
        if let base = rings.base {
            context.fill(Path(ellipseIn: CGRect(x: base.x - 4, y: base.y - 4, width: 8, height: 8)),
                         with: .color(Theme.good.opacity(0.95)))
        }
        if let legend = model.reachLegend {
            let first = Text(legend.first).font(.system(size: 12.5, weight: .medium))
            let second = Text(legend.second).font(.system(size: 12.5, weight: .medium))
            let width = max(context.resolve(first).measure(in: CGSize(width: 400, height: 40)).width,
                            context.resolve(second).measure(in: CGSize(width: 400, height: 40)).width)
            let rect = CGRect(x: box.left + 12, y: box.top + 12, width: width + 22, height: 46)
            let boxPath = Path(roundedRect: rect, cornerRadius: 9)
            context.fill(boxPath, with: .color(Color(red: 16 / 255, green: 20 / 255, blue: 26 / 255).opacity(0.86)))
            context.stroke(boxPath, with: .color(Theme.good.opacity(0.5)), lineWidth: 1)
            context.draw(first.foregroundColor(Color(red: 0xcf / 255, green: 0xe9 / 255, blue: 0xdd / 255)),
                         at: CGPoint(x: rect.minX + 11, y: rect.minY + 8), anchor: .topLeading)
            context.draw(second.foregroundColor(Color(red: 0x95 / 255, green: 0xc9 / 255, blue: 0xb0 / 255)),
                         at: CGPoint(x: rect.minX + 11, y: rect.minY + 26), anchor: .topLeading)
        }
    }

    private func drawMessage(_ message: String, context: inout GraphicsContext, size: CGSize) {
        let box = model.contentBox(stageSize: size)
        let left = box?.left ?? 0
        let maxWidth = max(120, (box?.width ?? Double(size.width)) - 24)
        let text = Text(message).font(.system(size: 12.5, weight: .medium)).foregroundColor(Theme.warn)
        let measured = context.resolve(text).measure(in: CGSize(width: maxWidth, height: 200))
        let rect = CGRect(x: left + 12, y: 12, width: min(maxWidth, measured.width) + 22, height: measured.height + 14)
        let path = Path(roundedRect: rect, cornerRadius: 9)
        context.fill(path, with: .color(Color(red: 16 / 255, green: 20 / 255, blue: 26 / 255).opacity(0.86)))
        context.stroke(path, with: .color(Theme.warn.opacity(0.65)), lineWidth: 1)
        context.draw(text, in: CGRect(x: rect.minX + 11, y: rect.minY + 7, width: maxWidth, height: measured.height + 4))
    }

    // MARK: tap / height drag

    private func gesture(size: CGSize) -> some Gesture {
        DragGesture(minimumDistance: 0)
            .onChanged { value in
                guard model.cameraOn, !model.calib, !model.busy else { return }
                if dragStart == nil {
                    dragStart = value.startLocation
                    dragStartHeight = model.height
                    wasAdjusting = false
                }
                let distance = hypot(value.translation.width, value.translation.height)
                if !wasAdjusting && distance > AppModel.tapSlopPx {
                    wasAdjusting = true
                    model.adjusting = true
                    UIImpactFeedbackGenerator(style: .light).impactOccurred()
                }
                if wasAdjusting {
                    let delta = (value.startLocation.y - value.location.y) * AppModel.heightPerPx
                    model.setHeightLive(dragStartHeight + Double(delta))
                }
            }
            .onEnded { value in
                defer {
                    dragStart = nil
                    wasAdjusting = false
                    model.adjusting = false
                }
                guard model.cameraOn, !model.calib, !model.busy, let start = dragStart else { return }
                if !wasAdjusting {
                    if model.depthTapPending {
                        Task { await model.depthAnchorTap(at: start, stageSize: size) }
                    } else {
                        Task { await model.addMarker(at: start, stageSize: size) }
                    }
                } else if abs(model.height - dragStartHeight) > 1e-9 {
                    model.log("arm height \(model.heightLabel) above the worktop", .good)
                }
            }
    }
}

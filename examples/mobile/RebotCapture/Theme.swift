/* The page's palette, as SwiftUI colors. */
import SwiftUI

enum Theme {
    static let bg = Color(red: 0x0b / 255, green: 0x0d / 255, blue: 0x10 / 255)
    static let panel = Color(red: 0x14 / 255, green: 0x18 / 255, blue: 0x1d / 255)
    static let line = Color(red: 0x26 / 255, green: 0x2c / 255, blue: 0x34 / 255)
    static let ink = Color(red: 0xe8 / 255, green: 0xea / 255, blue: 0xed / 255)
    static let dim = Color(red: 0x9a / 255, green: 0xa4 / 255, blue: 0xb2 / 255)
    static let accent = Color(red: 0x4e / 255, green: 0xa1 / 255, blue: 0xff / 255)
    static let good = Color(red: 0x45 / 255, green: 0xc9 / 255, blue: 0x8a / 255)
    static let warn = Color(red: 0xf0 / 255, green: 0xb4 / 255, blue: 0x29 / 255)
    static let bad = Color(red: 0xff / 255, green: 0x6b / 255, blue: 0x6b / 255)
    static let chipBG = Color(red: 0x1b / 255, green: 0x20 / 255, blue: 0x27 / 255)
}

struct Chip: View {
    let text: String
    var on = false
    var warn = false
    var plain = false
    var disabled = false
    var action: (() -> Void)?

    var body: some View {
        if plain || action == nil {
            label
        } else {
            Button(action: { action?() }) { label }
                .disabled(disabled)
                .opacity(disabled ? 0.45 : 1)
        }
    }

    private var label: some View {
        Text(text)
            .font(.system(size: 12))
            .foregroundColor(plain ? Theme.dim : (on ? Theme.accent : (warn ? Theme.warn : Theme.ink)))
            .padding(.horizontal, 10)
            .padding(.vertical, 5)
            .background(Theme.chipBG)
            .clipShape(Capsule())
            .overlay(Capsule().stroke(on ? Theme.accent : (warn ? Theme.warn : Theme.line), lineWidth: 1))
    }
}

struct HeightBadge: View {
    let text: String
    let adjusting: Bool

    var body: some View {
        HStack(spacing: 5) {
            Text("⇕").font(.system(size: 13)).opacity(0.7)
            Text(text).font(.system(size: 12, weight: .semibold)).monospacedDigit()
        }
        .foregroundColor(Color(red: 0x04 / 255, green: 0x12 / 255, blue: 0x1f / 255))
        .padding(.horizontal, 10)
        .padding(.vertical, 5)
        .background(adjusting ? Theme.warn : Theme.accent)
        .clipShape(Capsule())
        .scaleEffect(adjusting ? 1.06 : 1)
        .animation(.easeOut(duration: 0.15), value: adjusting)
    }
}

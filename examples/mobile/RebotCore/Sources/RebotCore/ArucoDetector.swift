/* Browser-side ArUco detection, ported 1:1 from web/aruco.js.
 *
 * Finds the printed 4x4_50 id 0 tag: adaptive threshold -> connected black
 * components -> convex hull -> four sharpest corners -> 6x6 module decode ->
 * subpixel edge refinement at full resolution.
 */
import Foundation

public struct GrayImage {
    public var data: [UInt8]
    public var width: Int
    public var height: Int
    public init(data: [UInt8], width: Int, height: Int) {
        self.data = data
        self.width = width
        self.height = height
    }

    /// Rec. 601 luma, the same integer math as aruco.js `grayscale`.
    public init(rgba: [UInt8], width: Int, height: Int) {
        var gray = [UInt8](repeating: 0, count: width * height)
        var p = 0
        for index in 0..<gray.count {
            gray[index] = UInt8((Int(rgba[p]) * 77 + Int(rgba[p + 1]) * 150 + Int(rgba[p + 2]) * 29) >> 8)
            p += 4
        }
        self.init(data: gray, width: width, height: height)
    }
}

public struct TagDetection {
    public var corners: [(Double, Double)]   // TL, TR, BR, BL after the decode shift
    public var errors: Int
    public var area: Int
    public var score: Double
}

public enum ArucoDetector {
    // 4x4_50, id 0: six modules across, including the one-module black border.
    // 1 is black. Kept in step with `base_marker.marker_grid()`.
    public static let tagBits: [[Int]] = [
        [1, 1, 1, 1, 1, 1],
        [1, 0, 1, 0, 0, 1],
        [1, 1, 0, 1, 0, 1],
        [1, 1, 1, 0, 0, 1],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 1, 1, 1],
    ]
    static let modules = 6

    /* ------------------------------------------------------------ threshold */

    static func blackMask(_ gray: GrayImage) -> [UInt8] {
        let width = gray.width, height = gray.height
        let radius = max(6, Int((Double(min(width, height)) / 24).rounded()))
        let stride = width + 1
        var sums = [UInt32](repeating: 0, count: stride * (height + 1))
        for y in 0..<height {
            var row: UInt32 = 0
            for x in 0..<width {
                row += UInt32(gray.data[y * width + x])
                sums[(y + 1) * stride + x + 1] = sums[y * stride + x + 1] + row
            }
        }
        var black = [UInt8](repeating: 0, count: width * height)
        for y in 0..<height {
            let y0 = max(0, y - radius)
            let y1 = min(height, y + radius + 1)
            for x in 0..<width {
                let x0 = max(0, x - radius)
                let x1 = min(width, x + radius + 1)
                let area = UInt32((x1 - x0) * (y1 - y0))
                let total = sums[y1 * stride + x1] &- sums[y0 * stride + x1]
                    &- sums[y1 * stride + x0] &+ sums[y0 * stride + x0]
                let mean = Double(total) / Double(area)
                black[y * width + x] = Double(gray.data[y * width + x]) < mean - max(7, mean * 0.08) ? 1 : 0
            }
        }
        return black
    }

    static func erode(_ mask: [UInt8], width: Int, height: Int, iterations: Int) -> [UInt8] {
        var current = mask
        for _ in 0..<iterations {
            var next = [UInt8](repeating: 0, count: width * height)
            for y in 1..<(height - 1) {
                for x in 1..<(width - 1) {
                    let index = y * width + x
                    if current[index] != 0 && current[index - 1] != 0 && current[index + 1] != 0
                        && current[index - width] != 0 && current[index + width] != 0 {
                        next[index] = 1
                    }
                }
            }
            current = next
        }
        return current
    }

    /* ----------------------------------------------------------- components */

    struct Component {
        var area: Int
        var minX: Int, maxX: Int, minY: Int, maxY: Int
        var points: [Int32]      // x, y pairs
    }

    static func components(_ mask: [UInt8], width: Int, height: Int) -> [Component] {
        var seen = mask               // consumed: 1 becomes 0 as it is visited
        var stack = [Int32](repeating: 0, count: width * height)
        let maxPoints = 24000
        var found: [Component] = []
        let maxArea = Int((Double(width * height) * 0.03).rounded())
        for start in 0..<seen.count {
            if seen[start] == 0 { continue }
            var top = 0
            stack[top] = Int32(start)
            top += 1
            seen[start] = 0
            var area = 0
            var minX = width, maxX = -1, minY = height, maxY = -1
            var points: [Int32] = []
            while top > 0 {
                top -= 1
                let index = Int(stack[top])
                let x = index % width
                let y = index / width
                area += 1
                if x < minX { minX = x }
                if x > maxX { maxX = x }
                if y < minY { minY = y }
                if y > maxY { maxY = y }
                if points.count < maxPoints * 2 {
                    points.append(Int32(x))
                    points.append(Int32(y))
                }
                if x > 0 && seen[index - 1] != 0 { seen[index - 1] = 0; stack[top] = Int32(index - 1); top += 1 }
                if x + 1 < width && seen[index + 1] != 0 { seen[index + 1] = 0; stack[top] = Int32(index + 1); top += 1 }
                if y > 0 && seen[index - width] != 0 { seen[index - width] = 0; stack[top] = Int32(index - width); top += 1 }
                if y + 1 < height && seen[index + width] != 0 { seen[index + width] = 0; stack[top] = Int32(index + width); top += 1 }
            }
            if area >= 60 && area <= maxArea {
                found.append(Component(area: area, minX: minX, maxX: maxX, minY: minY, maxY: maxY, points: points))
            }
        }
        return found
    }

    /* ----------------------------------------------------------------- hull */

    static func convexHull(_ points: [Int32]) -> [(Double, Double)]? {
        let count = points.count / 2
        if count < 4 { return nil }
        var order = Array(0..<count)
        // Stable sort by x then y, like the JS comparator on a stable engine.
        order.sort { a, b in
            let ax = points[2 * a], bx = points[2 * b]
            if ax != bx { return ax < bx }
            let ay = points[2 * a + 1], by = points[2 * b + 1]
            if ay != by { return ay < by }
            return a < b
        }
        func cross(_ o: Int, _ a: Int, _ b: Int) -> Int {
            (Int(points[2 * a]) - Int(points[2 * o])) * (Int(points[2 * b + 1]) - Int(points[2 * o + 1]))
                - (Int(points[2 * a + 1]) - Int(points[2 * o + 1])) * (Int(points[2 * b]) - Int(points[2 * o]))
        }
        var lower: [Int] = []
        for index in 0..<count {
            let point = order[index]
            while lower.count >= 2 && cross(lower[lower.count - 2], lower[lower.count - 1], point) <= 0 {
                lower.removeLast()
            }
            lower.append(point)
        }
        var upper: [Int] = []
        for index in stride(from: count - 1, through: 0, by: -1) {
            let point = order[index]
            while upper.count >= 2 && cross(upper[upper.count - 2], upper[upper.count - 1], point) <= 0 {
                upper.removeLast()
            }
            upper.append(point)
        }
        lower.removeLast()
        upper.removeLast()
        let hull = lower + upper
        if hull.count < 4 { return nil }
        return hull.map { (Double(points[2 * $0]), Double(points[2 * $0 + 1])) }
    }

    static func quadFromHull(_ hull: [(Double, Double)]) -> [(Double, Double)]? {
        let n = hull.count
        if n < 4 { return nil }
        var turns: [(index: Int, turn: Double)] = []
        for index in 0..<n {
            let previous = hull[(index + n - 1) % n]
            let current = hull[index]
            let next = hull[(index + 1) % n]
            let ax = current.0 - previous.0
            let ay = current.1 - previous.1
            let bx = next.0 - current.0
            let by = next.1 - current.1
            let turn = abs(atan2(ax * by - ay * bx, ax * bx + ay * by))
            turns.append((index, turn))
        }
        // Stable descending sort (JS Array.sort is stable).
        turns.sort { a, b in a.turn != b.turn ? a.turn > b.turn : a.index < b.index }
        let chosen = turns.prefix(4).map { hull[$0.index] }
        if turns[3].turn < 0.5 { return nil }
        return orderQuad(chosen)
    }

    static func orderQuad(_ points: [(Double, Double)]) -> [(Double, Double)] {
        var cx = 0.0, cy = 0.0
        for point in points { cx += point.0; cy += point.1 }
        cx /= 4
        cy /= 4
        var indexed = points.enumerated().map { (index: $0.offset, point: $0.element) }
        indexed.sort { a, b in
            let aa = atan2(a.point.1 - cy, a.point.0 - cx)
            let bb = atan2(b.point.1 - cy, b.point.0 - cx)
            return aa != bb ? aa < bb : a.index < b.index
        }
        let sorted = indexed.map { $0.point }
        var start = 0
        for index in 1..<4 {
            if sorted[index].1 < sorted[start].1
                || (sorted[index].1 == sorted[start].1 && sorted[index].0 < sorted[start].0) {
                start = index
            }
        }
        return Array(sorted[start...]) + Array(sorted[..<start])
    }

    /* ----------------------------------------------------------- homography */

    static func homographyUnitToQuad(_ quad: [(Double, Double)]) -> [Double]? {
        let object: [(Double, Double)] = [(0, 0), (1, 0), (1, 1), (0, 1)]
        var matrix: [[Double]] = []
        var vector: [Double] = []
        for index in 0..<4 {
            let (u, v) = object[index]
            let (x, y) = quad[index]
            matrix.append([u, v, 1, 0, 0, 0, -x * u, -x * v])
            vector.append(x)
            matrix.append([0, 0, 0, u, v, 1, -y * u, -y * v])
            vector.append(y)
        }
        guard let h = solveLinear(matrix, vector) else { return nil }
        return [h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], 1]
    }

    static func mapHomography(_ h: [Double], _ u: Double, _ v: Double) -> (Double, Double) {
        let denominator = h[6] * u + h[7] * v + 1
        return ((h[0] * u + h[1] * v + h[2]) / denominator,
                (h[3] * u + h[4] * v + h[5]) / denominator)
    }

    static func sampleBilinear(_ gray: GrayImage, _ x: Double, _ y: Double) -> Double {
        let width = gray.width, height = gray.height
        if x < 0 || y < 0 || x >= Double(width - 1) || y >= Double(height - 1) { return 0 }
        let x0 = Int(x), y0 = Int(y)
        let fx = x - Double(x0), fy = y - Double(y0)
        let index = y0 * width + x0
        let top = Double(gray.data[index]) * (1 - fx) + Double(gray.data[index + 1]) * fx
        let bottom = Double(gray.data[index + width]) * (1 - fx) + Double(gray.data[index + width + 1]) * fx
        return top * (1 - fy) + bottom * fy
    }

    struct Decoded {
        var corners: [(Double, Double)]
        var errors: Int
        var shift: Int
        var contrast: Double
    }

    static func decodeQuad(_ gray: GrayImage, quad: [(Double, Double)]) -> Decoded? {
        for shift in 0..<4 {
            let corners = [quad[shift % 4], quad[(shift + 1) % 4], quad[(shift + 2) % 4], quad[(shift + 3) % 4]]
            guard let h = homographyUnitToQuad(corners) else { continue }
            var values = [Double](repeating: 0, count: modules * modules)
            var low = 255.0, high = 0.0
            for row in 0..<modules {
                for column in 0..<modules {
                    let u = (Double(column) + 0.5) / Double(modules)
                    let v = (Double(row) + 0.5) / Double(modules)
                    let point = mapHomography(h, u, v)
                    let value = sampleBilinear(gray, point.0, point.1)
                    values[row * modules + column] = value
                    if value < low { low = value }
                    if value > high { high = value }
                }
            }
            if high - low < 35 { continue }
            let threshold = (low + high) / 2
            var errors = 0
            var borderErrors = 0
            for row in 0..<modules {
                for column in 0..<modules {
                    let black = values[row * modules + column] < threshold ? 1 : 0
                    let border = row == 0 || row == modules - 1 || column == 0 || column == modules - 1
                    if border {
                        if black == 0 { borderErrors += 1 }
                    } else if black != tagBits[row][column] {
                        errors += 1
                    }
                }
            }
            if borderErrors == 0 && errors <= 1 {
                return Decoded(corners: corners, errors: errors, shift: shift, contrast: high - low)
            }
        }
        return nil
    }

    /* -------------------------------------------------------- subpixel edges */

    struct Line {
        var point: (Double, Double)
        var direction: (Double, Double)
    }

    static func fitLine(_ points: [(Double, Double)]) -> Line {
        var cx = 0.0, cy = 0.0
        for point in points { cx += point.0; cy += point.1 }
        cx /= Double(points.count)
        cy /= Double(points.count)
        var sxx = 0.0, sxy = 0.0, syy = 0.0
        for point in points {
            let dx = point.0 - cx
            let dy = point.1 - cy
            sxx += dx * dx
            sxy += dx * dy
            syy += dy * dy
        }
        let angle = 0.5 * atan2(2 * sxy, sxx - syy)
        return Line(point: (cx, cy), direction: (cos(angle), sin(angle)))
    }

    static func robustLine(_ points: [(Double, Double)]) -> Line {
        var line = fitLine(points)
        for _ in 0..<2 {
            let kept = points.filter { point in
                let dx = point.0 - line.point.0
                let dy = point.1 - line.point.1
                return abs(line.direction.0 * dy - line.direction.1 * dx) <= 1.5
            }
            if kept.count < 6 { break }
            line = fitLine(kept)
        }
        return line
    }

    static func intersectLines(_ first: Line, _ second: Line) -> (Double, Double)? {
        let p = first.point, d = first.direction
        let q = second.point, e = second.direction
        let det = d.0 * -e.1 - d.1 * -e.0
        if abs(det) < 1e-9 { return nil }
        let rx = q.0 - p.0
        let ry = q.1 - p.1
        let t = (rx * -e.1 - ry * -e.0) / det
        return (p.0 + t * d.0, p.1 + t * d.1)
    }

    /// Subpixel corner refinement against the black/white edge, mostly outward.
    public static func refineQuad(_ gray: GrayImage, quad: [(Double, Double)]) -> [(Double, Double)]? {
        var centreX = 0.0, centreY = 0.0
        for point in quad { centreX += point.0 / 4; centreY += point.1 / 4 }
        var lines: [Line] = []
        for edge in 0..<4 {
            let p0 = quad[edge]
            let p1 = quad[(edge + 1) % 4]
            let dx = p1.0 - p0.0
            let dy = p1.1 - p0.1
            let length = (dx * dx + dy * dy).squareRoot()
            if length < 10 { return nil }
            var nx = -dy / length
            var ny = dx / length
            let midX = (p0.0 + p1.0) / 2 - centreX
            let midY = (p0.1 + p1.1) / 2 - centreY
            if nx * midX + ny * midY < 0 { nx = -nx; ny = -ny }
            var points: [(Double, Double)] = []
            var t = 0.12
            while t <= 0.88 {
                let cx = p0.0 + dx * t
                let cy = p0.1 + dy * t
                var bestOffset = 0.0
                var bestGradient = 0.0
                var bestPrev = 0.0
                var bestNext = 0.0
                var offset = -1.5
                while offset <= 4.0 {
                    let low = sampleBilinear(gray, cx + (offset - 0.75) * nx, cy + (offset - 0.75) * ny)
                    let high = sampleBilinear(gray, cx + (offset + 0.75) * nx, cy + (offset + 0.75) * ny)
                    let gradient = abs(high - low)
                    if gradient > bestGradient {
                        bestGradient = gradient
                        bestOffset = offset
                    }
                    offset += 0.5
                }
                if bestGradient >= 18 {
                    for sign in [-0.5, 0.5] {
                        let off = bestOffset + sign
                        let low = sampleBilinear(gray, cx + (off - 0.75) * nx, cy + (off - 0.75) * ny)
                        let high = sampleBilinear(gray, cx + (off + 0.75) * nx, cy + (off + 0.75) * ny)
                        if sign < 0 { bestPrev = abs(high - low) } else { bestNext = abs(high - low) }
                    }
                    var refinedOffset = bestOffset
                    let denominator = bestPrev - 2 * bestGradient + bestNext
                    if denominator < -1e-6 {
                        let shift = 0.25 * (bestPrev - bestNext) / denominator
                        refinedOffset += max(-0.5, min(0.5, shift))
                    }
                    points.append((cx + refinedOffset * nx, cy + refinedOffset * ny))
                }
                t += 0.04
            }
            if points.count < 6 { return nil }
            lines.append(robustLine(points))
        }

        var refined: [(Double, Double)] = []
        for corner in 0..<4 {
            guard let point = intersectLines(lines[(corner + 3) % 4], lines[corner]) else { return nil }
            let dx = point.0 - quad[corner].0
            let dy = point.1 - quad[corner].1
            if (dx * dx + dy * dy).squareRoot() > 8 { return nil }
            refined.append(point)
        }
        return refined
    }

    /* -------------------------------------------------------------- pipeline */

    static func detectGray(_ gray: GrayImage, erodePixels: Int) -> TagDetection? {
        let width = gray.width, height = gray.height
        let mask = blackMask(gray)
        let black = erodePixels > 0 ? erode(mask, width: width, height: height, iterations: erodePixels) : mask
        var candidates = components(black, width: width, height: height)
        // Stable descending area sort.
        candidates = candidates.enumerated()
            .sorted { a, b in a.element.area != b.element.area ? a.element.area > b.element.area : a.offset < b.offset }
            .map { $0.element }
        var best: TagDetection?
        let limit = min(candidates.count, 40)
        for index in 0..<limit {
            let item = candidates[index]
            let boxWidth = Double(item.maxX - item.minX + 1)
            let boxHeight = Double(item.maxY - item.minY + 1)
            if boxWidth < 13 || boxHeight < 13 || boxWidth > Double(width) * 0.6 || boxHeight > Double(height) * 0.6 {
                continue
            }
            if boxWidth / boxHeight < 0.35 || boxWidth / boxHeight > 2.8 { continue }
            guard let hull = convexHull(item.points) else { continue }
            guard let quad = quadFromHull(hull) else { continue }
            guard let decoded = decodeQuad(gray, quad: quad) else { continue }
            let score = decoded.contrast + Double(item.area) * 0.01
            if best == nil || score > best!.score {
                best = TagDetection(corners: decoded.corners, errors: decoded.errors, area: item.area, score: score)
            }
        }
        // The tag's border can touch the black base at the paper edge; a
        // one-pixel erosion cuts that bridge. Clean image first.
        if best == nil && erodePixels == 0 {
            if let retry = detectGray(gray, erodePixels: 1) { return retry }
        }
        return best
    }

    /// Corners in the pixels of the frame it was given; no pose solve.
    public static func detectFrame(_ gray: GrayImage) -> TagDetection? {
        detectGray(gray, erodePixels: 0)
    }

    public static func detectFrame(rgba: [UInt8], width: Int, height: Int) -> TagDetection? {
        detectFrame(GrayImage(rgba: rgba, width: width, height: height))
    }
}

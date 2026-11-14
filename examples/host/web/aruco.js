/* Browser-side ArUco pose: the base tag is the camera pose, without a tap.
 *
 * The phone already sees the printed 4x4_50 tag on the side of the base. This
 * module finds its four corners in the live video and solves `T_base<-cam` from
 * them, in the page, on every few frames. It is the client-side counterpart of
 * `base_marker.py` + `locate_target.box_pose` and deliberately uses the same
 * tag geometry: `size_m` is the black square including its border, the corners
 * are TL, TR, BR, BL as printed, and the marker's +z points out of the face.
 *
 * The detector is small because it knows the tag:
 *
 *   1. adaptive threshold (integral image, local mean) -> black mask;
 *   2. connected black components -> convex hull -> four strongest corners;
 *   3. sample the 6x6 modules through a homography and match the known id 0;
 *   4. homography decomposition gives a pose, Gauss-Newton refines it against
 *      the four corners with the fitted lens distortion.
 *
 * Why not OpenCV.js: the standard build does not include aruco, and an 8 MB
 * download to find one 40 mm square is not a trade this page should make.
 *
 * Public API:
 *
 *   const eye = ArucoPose.create({video, calibration, marker});
 *   const found = eye.detect();   // {ok, T_base_cam, corners, rms_px} or null
 *
 * `ArucoPose.detectFrame(rgba, width, height)` is exposed for tests: it returns
 * corners in the pixels of the frame it was given, and does no pose solve.
 */
(function () {
  "use strict";

  // 4x4_50, id 0: six modules across, including the one-module black border.
  // 1 is black. Kept in step with `base_marker.marker_grid()`.
  const TAG_BITS = [
    [1, 1, 1, 1, 1, 1],
    [1, 0, 1, 0, 0, 1],
    [1, 1, 0, 1, 0, 1],
    [1, 1, 1, 0, 0, 1],
    [1, 1, 1, 0, 1, 1],
    [1, 1, 1, 1, 1, 1],
  ];
  const MODULES = TAG_BITS.length;

  /* --------------------------------------------------------------- vectors */
  function add(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
  function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
  function scale(a, s) { return [a[0] * s, a[1] * s, a[2] * s]; }
  function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  function cross(a, b) {
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  }
  function norm(a) { return Math.hypot(a[0], a[1], a[2]); }
  function unit(a) {
    const length = norm(a);
    return length > 1e-12 ? scale(a, 1 / length) : [0, 0, 0];
  }

  function matMul3(a, b) {
    const out = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let i = 0; i < 3; i += 1) {
      for (let j = 0; j < 3; j += 1) {
        out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
      }
    }
    return out;
  }

  function matVec3(a, v) {
    return [
      a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2],
      a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2],
      a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2],
    ];
  }

  function skew(v) {
    return [[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]];
  }

  function rodrigues(vector) {
    const angle = norm(vector);
    if (angle < 1e-12) {
      // R = I + skew(v) to first order, which is what the refinement needs.
      const k = skew(vector);
      return [[1 + k[0][0], k[0][1], k[0][2]],
        [k[1][0], 1 + k[1][1], k[1][2]],
        [k[2][0], k[2][1], 1 + k[2][2]]];
    }
    const axis = scale(vector, 1 / angle);
    const k = skew(axis);
    const s = Math.sin(angle);
    const c = 1 - Math.cos(angle);
    const k2 = matMul3(k, k);
    const rotation = [];
    for (let row = 0; row < 3; row += 1) {
      rotation.push([]);
      for (let column = 0; column < 3; column += 1) {
        rotation[row].push((row === column ? 1 : 0) + s * k[row][column] + c * k2[row][column]);
      }
    }
    return rotation;
  }

  function rotationToVector(R) {
    const cosine = Math.max(-1, Math.min(1, (R[0][0] + R[1][1] + R[2][2] - 1) / 2));
    const angle = Math.acos(cosine);
    if (angle < 1e-9) return [0, 0, 0];
    if (angle > Math.PI - 1e-6) {
      // Near 180 degrees the antisymmetric part vanishes; use the diagonal.
      const axis = [
        Math.sqrt(Math.max(0, (R[0][0] + 1) / 2)),
        Math.sqrt(Math.max(0, (R[1][1] + 1) / 2)),
        Math.sqrt(Math.max(0, (R[2][2] + 1) / 2)),
      ];
      if (R[2][1] - R[1][2] < 0) axis[0] = -axis[0];
      if (R[0][2] - R[2][0] < 0) axis[1] = -axis[1];
      if (R[1][0] - R[0][1] < 0) axis[2] = -axis[2];
      return scale(unit(axis), angle);
    }
    const axis = [
      (R[2][1] - R[1][2]) / (2 * Math.sin(angle)),
      (R[0][2] - R[2][0]) / (2 * Math.sin(angle)),
      (R[1][0] - R[0][1]) / (2 * Math.sin(angle)),
    ];
    return scale(axis, angle);
  }

  /* ----------------------------------------------------------- linear solve */
  function solveLinear(matrix, vector) {
    const n = vector.length;
    const rows = matrix.map((row, index) => row.slice().concat([vector[index]]));
    for (let column = 0; column < n; column += 1) {
      let pivot = column;
      for (let row = column + 1; row < n; row += 1) {
        if (Math.abs(rows[row][column]) > Math.abs(rows[pivot][column])) pivot = row;
      }
      if (Math.abs(rows[pivot][column]) < 1e-12) return null;
      if (pivot !== column) {
        const swap = rows[pivot];
        rows[pivot] = rows[column];
        rows[column] = swap;
      }
      const divisor = rows[column][column];
      for (let k = column; k <= n; k += 1) rows[column][k] /= divisor;
      for (let row = 0; row < n; row += 1) {
        if (row === column) continue;
        const factor = rows[row][column];
        if (!factor) continue;
        for (let k = column; k <= n; k += 1) rows[row][k] -= factor * rows[column][k];
      }
    }
    return rows.map((row) => row[n]);
  }

  /* -------------------------------------------------------------- threshold */
  function grayscale(pixels, width, height) {
    const gray = new Uint8Array(width * height);
    for (let index = 0, p = 0; index < gray.length; index += 1, p += 4) {
      // Rec. 601 luma, integer enough for a black/white decision.
      gray[index] = (pixels[p] * 77 + pixels[p + 1] * 150 + pixels[p + 2] * 29) >> 8;
    }
    return gray;
  }

  function integral(gray, width, height) {
    const stride = width + 1;
    const sums = new Uint32Array(stride * (height + 1));
    for (let y = 0; y < height; y += 1) {
      let row = 0;
      for (let x = 0; x < width; x += 1) {
        row += gray[y * width + x];
        sums[(y + 1) * stride + x + 1] = sums[y * stride + x + 1] + row;
      }
    }
    return sums;
  }

  function blackMask(gray, width, height) {
    const radius = Math.max(6, Math.round(Math.min(width, height) / 24));
    const sums = integral(gray, width, height);
    const stride = width + 1;
    const black = new Uint8Array(width * height);
    for (let y = 0; y < height; y += 1) {
      const y0 = Math.max(0, y - radius);
      const y1 = Math.min(height, y + radius + 1);
      for (let x = 0; x < width; x += 1) {
        const x0 = Math.max(0, x - radius);
        const x1 = Math.min(width, x + radius + 1);
        const area = (x1 - x0) * (y1 - y0);
        const total = sums[y1 * stride + x1] - sums[y0 * stride + x1]
          - sums[y1 * stride + x0] + sums[y0 * stride + x0];
        const mean = total / area;
        black[y * width + x] = gray[y * width + x] < mean - Math.max(7, mean * 0.08) ? 1 : 0;
      }
    }
    return black;
  }

  function erode(mask, width, height, iterations) {
    // A one-pixel erosion breaks the thin black bridge when the tag's border
    // touches the black base at the paper edge. The tag's border ring is
    // several pixels thick and survives; the merged blob does not.
    let current = mask;
    for (let iteration = 0; iteration < (iterations || 1); iteration += 1) {
      const next = new Uint8Array(width * height);
      for (let y = 1; y < height - 1; y += 1) {
        for (let x = 1; x < width - 1; x += 1) {
          const index = y * width + x;
          if (current[index] && current[index - 1] && current[index + 1]
            && current[index - width] && current[index + width]) {
            next[index] = 1;
          }
        }
      }
      current = next;
    }
    return current;
  }

  /* ------------------------------------------------------------ components */
  function components(mask, width, height) {
    const seen = mask;                    // consumed: 1 becomes 0 as it is visited
    const stack = new Int32Array(width * height);
    const maxPoints = 24000;
    const found = [];
    const maxArea = Math.round(width * height * 0.03);
    for (let start = 0; start < seen.length; start += 1) {
      if (!seen[start]) continue;
      let top = 0;
      stack[top] = start;
      top += 1;
      seen[start] = 0;
      let area = 0;
      let minX = width, maxX = -1, minY = height, maxY = -1;
      const points = [];
      while (top > 0) {
        top -= 1;
        const index = stack[top];
        const x = index % width;
        const y = (index / width) | 0;
        area += 1;
        if (x < minX) minX = x;
        if (x > maxX) maxX = x;
        if (y < minY) minY = y;
        if (y > maxY) maxY = y;
        if (points.length < maxPoints * 2) points.push(x, y);
        if (x > 0 && seen[index - 1]) { seen[index - 1] = 0; stack[top] = index - 1; top += 1; }
        if (x + 1 < width && seen[index + 1]) { seen[index + 1] = 0; stack[top] = index + 1; top += 1; }
        if (y > 0 && seen[index - width]) { seen[index - width] = 0; stack[top] = index - width; top += 1; }
        if (y + 1 < height && seen[index + width]) {
          seen[index + width] = 0;
          stack[top] = index + width;
          top += 1;
        }
      }
      if (area >= 60 && area <= maxArea) {
        found.push({ area: area, minX: minX, maxX: maxX, minY: minY, maxY: maxY, points: points });
      }
    }
    return found;
  }

  /* ----------------------------------------------------------------- hull */
  function convexHull(points) {
    const count = points.length / 2;
    if (count < 4) return null;
    const order = new Array(count);
    for (let index = 0; index < count; index += 1) order[index] = index;
    order.sort((a, b) => (points[2 * a] - points[2 * b]) || (points[2 * a + 1] - points[2 * b + 1]));
    const cross = (o, a, b) => (points[2 * a] - points[2 * o]) * (points[2 * b + 1] - points[2 * o + 1])
      - (points[2 * a + 1] - points[2 * o + 1]) * (points[2 * b] - points[2 * o]);
    const lower = [];
    for (let index = 0; index < count; index += 1) {
      const point = order[index];
      while (lower.length >= 2
        && cross(lower[lower.length - 2], lower[lower.length - 1], point) <= 0) lower.pop();
      lower.push(point);
    }
    const upper = [];
    for (let index = count - 1; index >= 0; index -= 1) {
      const point = order[index];
      while (upper.length >= 2
        && cross(upper[upper.length - 2], upper[upper.length - 1], point) <= 0) upper.pop();
      upper.push(point);
    }
    lower.pop();
    upper.pop();
    const hull = lower.concat(upper);
    return hull.length >= 4 ? hull.map((index) => [points[2 * index], points[2 * index + 1]]) : null;
  }

  function quadFromHull(hull) {
    // For a convex projected square the four corners are the four sharpest
    // turns; everything between them is collinear and turns by ~0.
    const n = hull.length;
    if (n < 4) return null;
    const turns = [];
    for (let index = 0; index < n; index += 1) {
      const previous = hull[(index + n - 1) % n];
      const current = hull[index];
      const next = hull[(index + 1) % n];
      const ax = current[0] - previous[0];
      const ay = current[1] - previous[1];
      const bx = next[0] - current[0];
      const by = next[1] - current[1];
      const turn = Math.abs(Math.atan2(ax * by - ay * bx, ax * bx + ay * by));
      turns.push({ index: index, turn: turn });
    }
    turns.sort((a, b) => b.turn - a.turn);
    const chosen = turns.slice(0, 4).map((item) => hull[item.index]);
    // A real tag corner is close to a right angle; a blob's turns are not.
    if (turns[3].turn < 0.5) return null;
    return orderQuad(chosen);
  }

  function orderQuad(points) {
    let cx = 0, cy = 0;
    for (const point of points) { cx += point[0]; cy += point[1]; }
    cx /= 4;
    cy /= 4;
    const sorted = points.slice().sort((a, b) =>
      Math.atan2(a[1] - cy, a[0] - cx) - Math.atan2(b[1] - cy, b[0] - cx));
    // Start at the topmost (then leftmost) corner so an upright tag begins at
    // its printed top-left. The decode then tries the four cyclic shifts.
    let start = 0;
    for (let index = 1; index < 4; index += 1) {
      if (sorted[index][1] < sorted[start][1]
        || (sorted[index][1] === sorted[start][1] && sorted[index][0] < sorted[start][0])) {
        start = index;
      }
    }
    return sorted.slice(start).concat(sorted.slice(0, start));
  }

  /* ----------------------------------------------------------- homography */
  function homographyUnitToQuad(quad) {
    // Object (0,0), (1,0), (1,1), (0,1) -> the four image corners.
    const object = [[0, 0], [1, 0], [1, 1], [0, 1]];
    const matrix = [];
    const vector = [];
    for (let index = 0; index < 4; index += 1) {
      const u = object[index][0];
      const v = object[index][1];
      const x = quad[index][0];
      const y = quad[index][1];
      matrix.push([u, v, 1, 0, 0, 0, -x * u, -x * v]);
      vector.push(x);
      matrix.push([0, 0, 0, u, v, 1, -y * u, -y * v]);
      vector.push(y);
    }
    const h = solveLinear(matrix, vector);
    if (!h) return null;
    return [h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], 1];
  }

  function mapHomography(h, u, v) {
    const denominator = h[6] * u + h[7] * v + 1;
    return [(h[0] * u + h[1] * v + h[2]) / denominator,
      (h[3] * u + h[4] * v + h[5]) / denominator];
  }

  function sampleBilinear(gray, width, height, x, y) {
    if (x < 0 || y < 0 || x >= width - 1 || y >= height - 1) return 0;
    const x0 = x | 0;
    const y0 = y | 0;
    const fx = x - x0;
    const fy = y - y0;
    const index = y0 * width + x0;
    const top = gray[index] * (1 - fx) + gray[index + 1] * fx;
    const bottom = gray[index + width] * (1 - fx) + gray[index + width + 1] * fx;
    return top * (1 - fy) + bottom * fy;
  }

  function decodeQuad(gray, width, height, quad) {
    for (let shift = 0; shift < 4; shift += 1) {
      const corners = [quad[shift % 4], quad[(shift + 1) % 4],
        quad[(shift + 2) % 4], quad[(shift + 3) % 4]];
      const h = homographyUnitToQuad(corners);
      if (!h) continue;
      const values = new Float64Array(MODULES * MODULES);
      let low = 255, high = 0;
      for (let row = 0; row < MODULES; row += 1) {
        for (let column = 0; column < MODULES; column += 1) {
          const u = (column + 0.5) / MODULES;
          const v = (row + 0.5) / MODULES;
          const point = mapHomography(h, u, v);
          const value = sampleBilinear(gray, width, height, point[0], point[1]);
          values[row * MODULES + column] = value;
          if (value < low) low = value;
          if (value > high) high = value;
        }
      }
      if (high - low < 35) continue;
      const threshold = (low + high) / 2;
      let errors = 0;
      let borderErrors = 0;
      for (let row = 0; row < MODULES; row += 1) {
        for (let column = 0; column < MODULES; column += 1) {
          const black = values[row * MODULES + column] < threshold ? 1 : 0;
          const border = row === 0 || row === MODULES - 1 || column === 0 || column === MODULES - 1;
          if (border) {
            if (!black) borderErrors += 1;
          } else if (black !== TAG_BITS[row][column]) {
            errors += 1;
          }
        }
      }
      if (borderErrors === 0 && errors <= 1) {
        return { corners: corners, errors: errors, shift: shift, contrast: high - low };
      }
    }
    return null;
  }

  function fitLine(points) {
    let cx = 0, cy = 0;
    for (const point of points) { cx += point[0]; cy += point[1]; }
    cx /= points.length;
    cy /= points.length;
    let sxx = 0, sxy = 0, syy = 0;
    for (const point of points) {
      const dx = point[0] - cx;
      const dy = point[1] - cy;
      sxx += dx * dx;
      sxy += dx * dy;
      syy += dy * dy;
    }
    const angle = 0.5 * Math.atan2(2 * sxy, sxx - syy);
    return { point: [cx, cy], direction: [Math.cos(angle), Math.sin(angle)] };
  }

  function robustLine(points) {
    let line = fitLine(points);
    // A few samples latch onto an inner module edge; a second fit without
    // points more than a pixel and a half off the line ignores them.
    for (let iteration = 0; iteration < 2; iteration += 1) {
      const kept = points.filter((point) => {
        const dx = point[0] - line.point[0];
        const dy = point[1] - line.point[1];
        return Math.abs(line.direction[0] * dy - line.direction[1] * dx) <= 1.5;
      });
      if (kept.length < 6) break;
      line = fitLine(kept);
    }
    return line;
  }

  function intersectLines(first, second) {
    const p = first.point;
    const d = first.direction;
    const q = second.point;
    const e = second.direction;
    const det = d[0] * -e[1] - d[1] * -e[0];
    if (Math.abs(det) < 1e-9) return null;
    const rx = q[0] - p[0];
    const ry = q[1] - p[1];
    const t = (rx * -e[1] - ry * -e[0]) / det;
    return [p[0] + t * d[0], p[1] + t * d[1]];
  }

  function refineQuad(gray, width, height, quad) {
    // The binary hull puts each corner within a couple of pixels. The marker
    // edge is a strong black/white step; sample across it along the outward
    // normal, take the subpixel gradient maximum, fit the four edge lines and
    // intersect them. The outward bias matters: the tag's own modules are
    // black/white edges a few pixels *inside* the black square, and an
    // unbiased search can latch onto those instead.
    let centreX = 0, centreY = 0;
    for (const point of quad) { centreX += point[0] / 4; centreY += point[1] / 4; }
    const lines = [];
    for (let edge = 0; edge < 4; edge += 1) {
      const p0 = quad[edge];
      const p1 = quad[(edge + 1) % 4];
      const dx = p1[0] - p0[0];
      const dy = p1[1] - p0[1];
      const length = Math.hypot(dx, dy);
      if (length < 10) return null;
      let nx = -dy / length;
      let ny = dx / length;
      const midX = (p0[0] + p1[0]) / 2 - centreX;
      const midY = (p0[1] + p1[1]) / 2 - centreY;
      if (nx * midX + ny * midY < 0) { nx = -nx; ny = -ny; }
      const points = [];
      for (let t = 0.12; t <= 0.88; t += 0.04) {
        const cx = p0[0] + dx * t;
        const cy = p0[1] + dy * t;
        let bestOffset = 0;
        let bestGradient = 0;
        let bestPrev = 0;
        let bestNext = 0;
        // Mostly outward: the outer edge is at 0, its modules are negative.
        for (let offset = -1.5; offset <= 4.0; offset += 0.5) {
          const low = sampleBilinear(gray, width, height,
            cx + (offset - 0.75) * nx, cy + (offset - 0.75) * ny);
          const high = sampleBilinear(gray, width, height,
            cx + (offset + 0.75) * nx, cy + (offset + 0.75) * ny);
          const gradient = Math.abs(high - low);
          if (gradient > bestGradient) {
            bestGradient = gradient;
            bestOffset = offset;
          }
        }
        if (bestGradient < 18) continue;
        // Parabolic refinement around the sampled maximum.
        for (const sign of [-0.5, 0.5]) {
          const offset = bestOffset + sign;
          const low = sampleBilinear(gray, width, height,
            cx + (offset - 0.75) * nx, cy + (offset - 0.75) * ny);
          const high = sampleBilinear(gray, width, height,
            cx + (offset + 0.75) * nx, cy + (offset + 0.75) * ny);
          if (sign < 0) bestPrev = Math.abs(high - low);
          else bestNext = Math.abs(high - low);
        }
        let offset = bestOffset;
        const denominator = bestPrev - 2 * bestGradient + bestNext;
        if (denominator < -1e-6) {
          // Only a local maximum has a negative denominator; clamp the
          // parabolic step, or a flat/noisy profile throws the point a pixel
          // or two off the edge.
          const shift = 0.25 * (bestPrev - bestNext) / denominator;
          offset += Math.max(-0.5, Math.min(0.5, shift));
        }
        points.push([cx + offset * nx, cy + offset * ny]);
      }
      if (points.length < 6) return null;
      lines.push(robustLine(points));
    }

    const refined = [];
    for (let corner = 0; corner < 4; corner += 1) {
      const point = intersectLines(lines[(corner + 3) % 4], lines[corner]);
      if (!point) return null;
      if (Math.hypot(point[0] - quad[corner][0], point[1] - quad[corner][1]) > 8) return null;
      refined.push(point);
    }
    return refined;
  }

  function detectGray(gray, width, height, debug, erodePixels) {
    const mask = blackMask(gray, width, height);
    const black = erodePixels ? erode(mask, width, height, erodePixels) : mask;
    const candidates = components(black, width, height);
    candidates.sort((a, b) => b.area - a.area);
    if (debug) {
      debug.candidates = candidates.map((item) => ({
        area: item.area, bbox: [item.minX, item.minY,
          item.maxX - item.minX + 1, item.maxY - item.minY + 1],
      })).slice(0, 60);
    }
    let best = null;
    const limit = Math.min(candidates.length, 40);
    for (let index = 0; index < limit; index += 1) {
      const item = candidates[index];
      const boxWidth = item.maxX - item.minX + 1;
      const boxHeight = item.maxY - item.minY + 1;
      if (boxWidth < 13 || boxHeight < 13 || boxWidth > width * 0.6 || boxHeight > height * 0.6) {
        continue;
      }
      if (boxWidth / boxHeight < 0.35 || boxWidth / boxHeight > 2.8) continue;
      const hull = convexHull(item.points);
      if (!hull) continue;
      const quad = quadFromHull(hull);
      if (debug) {
        debug.considered = debug.considered || [];
        debug.considered.push({
          area: item.area, box: [item.minX, item.minY, boxWidth, boxHeight],
          quad: quad ? quad.map((point) => point.map((value) => +value.toFixed(1))) : null,
          hull: hull.length,
        });
      }
      if (!quad) continue;
      const decoded = decodeQuad(gray, width, height, quad);
      if (debug && debug.considered.length) {
        debug.considered[debug.considered.length - 1].decoded = Boolean(decoded);
      }
      if (!decoded) continue;
      // No subpixel work at this scale: `create().detect()` refines the edges
      // once at full resolution, where the black/white step is sharp.
      const score = decoded.contrast + item.area * 0.01;
      if (!best || score > best.score) {
        best = { corners: decoded.corners, errors: decoded.errors, area: item.area, score: score };
      }
    }
    // The tag's black border can touch the black base at the paper edge; a
    // one-pixel erosion cuts that bridge. Try the clean image first because
    // eroding a well-separated tag only shrinks its hull.
    if (!best && !erodePixels) {
      const retry = detectGray(gray, width, height, debug, 1);
      if (retry) return retry;
    }
    return best;
  }

  function detectFrame(pixels, width, height, debug) {
    const gray = grayscale(pixels, width, height);
    return detectGray(gray, width, height, debug, 0);
  }

  /* ------------------------------------------------------------------ pose */
  function undistort(u, v, camera) {
    const xd = (u - camera.cx) / camera.fx;
    const yd = (v - camera.cy) / camera.fy;
    let x = xd;
    let y = yd;
    for (let iteration = 0; iteration < 6; iteration += 1) {
      const r2 = x * x + y * y;
      const radial = 1 + camera.k1 * r2 + camera.k2 * r2 * r2 + camera.k3 * r2 * r2 * r2;
      const dx = 2 * camera.p1 * x * y + camera.p2 * (r2 + 2 * x * x);
      const dy = camera.p1 * (r2 + 2 * y * y) + 2 * camera.p2 * x * y;
      x = (xd - dx) / radial;
      y = (yd - dy) / radial;
    }
    return [x, y];
  }

  function projectPixel(point, rotation, translation, camera) {
    const cameraPoint = add(matVec3(rotation, point), translation);
    if (cameraPoint[2] <= 1e-9) return null;
    const x = cameraPoint[0] / cameraPoint[2];
    const y = cameraPoint[1] / cameraPoint[2];
    const r2 = x * x + y * y;
    const radial = 1 + camera.k1 * r2 + camera.k2 * r2 * r2 + camera.k3 * r2 * r2 * r2;
    const xd = x * radial + 2 * camera.p1 * x * y + camera.p2 * (r2 + 2 * x * x);
    const yd = y * radial + camera.p1 * (r2 + 2 * y * y) + 2 * camera.p2 * x * y;
    return [camera.fx * xd + camera.cx, camera.fy * yd + camera.cy];
  }

  function objectCorners(size) {
    const half = size / 2;
    // The same order as `base_marker.object_points`: TL, TR, BR, BL.
    return [[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]];
  }

  function poseFromHomography(object, image) {
    // DLT from object (z=0) to normalized image coordinates.
    const matrix = [];
    const vector = [];
    for (let index = 0; index < object.length; index += 1) {
      const x = object[index][0];
      const y = object[index][1];
      const u = image[index][0];
      const v = image[index][1];
      matrix.push([x, y, 1, 0, 0, 0, -u * x, -u * y]);
      vector.push(u);
      matrix.push([0, 0, 0, x, y, 1, -v * x, -v * y]);
      vector.push(v);
    }
    const h = solveLinear(matrix, vector);
    if (!h) return null;
    let r1 = [h[0], h[3], h[6]];
    let r2 = [h[1], h[4], h[7]];
    let t = [h[2], h[5], 1];
    const lambda = 2 / (norm(r1) + norm(r2));
    r1 = scale(r1, lambda);
    r2 = scale(r2, lambda);
    t = scale(t, lambda);
    if (t[2] < 0) {
      r1 = scale(r1, -1);
      r2 = scale(r2, -1);
      t = scale(t, -1);
    }
    // Gram-Schmidt, then the third axis is the cross product.
    const xAxis = unit(r1);
    const yAxis = unit(sub(r2, scale(xAxis, dot(r2, xAxis))));
    const zAxis = cross(xAxis, yAxis);
    return { rotation: [[xAxis[0], yAxis[0], zAxis[0]],
      [xAxis[1], yAxis[1], zAxis[1]],
      [xAxis[2], yAxis[2], zAxis[2]]], translation: t };
  }

  function refinePose(rotation, translation, object, observed, camera) {
    let rvec = rotationToVector(rotation);
    let tvec = translation.slice();
    let damping = 1e-6;

    function residuals(r, t) {
      const current = rodrigues(r);
      const out = new Array(8);
      for (let index = 0; index < object.length; index += 1) {
        const point = projectPixel(object[index], current, t, camera);
        if (!point) return null;
        out[2 * index] = point[0] - observed[index][0];
        out[2 * index + 1] = point[1] - observed[index][1];
      }
      return out;
    }

    function rmsOf(error) {
      let sum = 0;
      for (let index = 0; index < error.length; index += 1) sum += error[index] * error[index];
      return Math.sqrt(sum / error.length);
    }

    let error = residuals(rvec, tvec);
    if (!error) return null;
    let rms = rmsOf(error);

    for (let iteration = 0; iteration < 20; iteration += 1) {
      if (rms < 1e-4) break;
      const jacobian = [];
      for (let row = 0; row < 8; row += 1) jacobian.push(new Array(6).fill(0));
      const epsilon = 1e-6;
      for (let parameter = 0; parameter < 6; parameter += 1) {
        const trialR = rvec.slice();
        const trialT = tvec.slice();
        if (parameter < 3) trialR[parameter] += epsilon;
        else trialT[parameter - 3] += epsilon;
        const trialError = residuals(trialR, trialT);
        if (!trialError) return null;
        for (let row = 0; row < 8; row += 1) {
          jacobian[row][parameter] = (trialError[row] - error[row]) / epsilon;
        }
      }

      const normal = [];
      const rhs = [];
      for (let a = 0; a < 6; a += 1) {
        normal.push(new Array(6).fill(0));
        let value = 0;
        for (let b = 0; b < 6; b += 1) {
          let sum = 0;
          for (let row = 0; row < 8; row += 1) sum += jacobian[row][a] * jacobian[row][b];
          normal[a][b] = sum;
        }
        for (let row = 0; row < 8; row += 1) value -= jacobian[row][a] * error[row];
        rhs.push(value);
      }
      for (let a = 0; a < 6; a += 1) normal[a][a] += damping;
      const step = solveLinear(normal, rhs);
      if (!step) {
        damping *= 10;
        if (damping > 100) return null;
        continue;
      }

      let accepted = false;
      let factor = 1;
      for (let attempt = 0; attempt < 6; attempt += 1) {
        const trialR = rvec.map((value, index) => value + step[index] * factor);
        const trialT = tvec.map((value, index) => value + step[index + 3] * factor);
        const trialError = residuals(trialR, trialT);
        if (trialError) {
          const trialRms = rmsOf(trialError);
          if (trialRms < rms) {
            rvec = trialR;
            tvec = trialT;
            error = trialError;
            rms = trialRms;
            damping = Math.max(1e-9, damping / 2);
            accepted = true;
            break;
          }
        }
        factor *= 0.5;
      }
      if (!accepted) {
        damping *= 10;
        if (damping > 100) return null;
      }
    }

    return { rotation: rodrigues(rvec), translation: tvec, rms: rms };
  }

  function baseFromMarker(marker) {
    const yaw = (marker.yaw_deg || 0) * Math.PI / 180;
    const normal = [Math.cos(yaw), Math.sin(yaw), 0];
    const up = [0, 0, 1];
    // right = up x normal, the same convention as base_marker.base_from_marker.
    const right = cross(up, normal);
    const rotation = [[right[0], up[0], normal[0]],
      [right[1], up[1], normal[1]],
      [right[2], up[2], normal[2]]];
    return { rotation: rotation, translation: marker.at.slice() };
  }

  function cameraFrom(calibration) {
    if (calibration.fx != null) return calibration;
    const K = calibration.K;
    const dist = calibration.dist || [];
    return {
      fx: K[0][0], fy: K[1][1], cx: K[0][2], cy: K[1][2],
      k1: dist[0] || 0, k2: dist[1] || 0,
      p1: dist[2] || 0, p2: dist[3] || 0, k3: dist[4] || 0,
    };
  }

  function rotZ(angle) {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]];
  }

  function poseCandidates(corners, object, camera, marker) {
    const normalized = corners.map((point) => undistort(point[0], point[1], camera));
    const candidates = [];
    const homography = poseFromHomography(object, normalized);
    if (homography) candidates.push(homography);

    // The physically strongest seed: a camera in front of the tag and above the
    // table, looking at it. That is where a phone using this overlay is. It is
    // generated in the base frame, where the marker's placement is known, and
    // converted back to the camera-from-marker pose the refinement works in.
    const base = baseFromMarker(marker);
    const normal = [base.rotation[0][2], base.rotation[1][2], base.rotation[2][2]];
    const T_base_marker = [[base.rotation[0][0], base.rotation[0][1], base.rotation[0][2],
      base.translation[0]],
    [base.rotation[1][0], base.rotation[1][1], base.rotation[1][2], base.translation[1]],
    [base.rotation[2][0], base.rotation[2][1], base.rotation[2][2], base.translation[2]],
    [0, 0, 0, 1]];
    for (const distance of [0.5, 0.8, 1.2, 1.6]) {
      for (const height of [0.2, 0.45, 0.75]) {
        const position = [
          marker.at[0] + normal[0] * distance,
          marker.at[1] + normal[1] * distance,
          marker.at[2] + height,
        ];
        const forward = unit(sub(marker.at, position));
        const right = unit(cross(forward, [0, 0, 1]));
        const down = cross(forward, right);
        const R_base_cam = [[right[0], down[0], forward[0]],
          [right[1], down[1], forward[1]],
          [right[2], down[2], forward[2]]];
        const T_base_cam = [[R_base_cam[0][0], R_base_cam[0][1], R_base_cam[0][2], position[0]],
          [R_base_cam[1][0], R_base_cam[1][1], R_base_cam[1][2], position[1]],
          [R_base_cam[2][0], R_base_cam[2][1], R_base_cam[2][2], position[2]],
          [0, 0, 0, 1]];
        // T_cam_marker = inverse(T_base_cam) * T_base_marker
        const T_inverse = invert4(T_base_cam);
        const T_cam_marker = multiply4(T_inverse, T_base_marker);
        candidates.push({
          rotation: [[T_cam_marker[0][0], T_cam_marker[0][1], T_cam_marker[0][2]],
            [T_cam_marker[1][0], T_cam_marker[1][1], T_cam_marker[1][2]],
            [T_cam_marker[2][0], T_cam_marker[2][1], T_cam_marker[2][2]]],
          translation: [T_cam_marker[0][3], T_cam_marker[1][3], T_cam_marker[2][3]],
          lookAt: true,
        });
      }
    }

    // A fronto-parallel seed at the apparent distance, for a phone whose pose
    // is close to the image plane. It also covers a sideways phone.
    let meanSide = 0;
    for (let index = 0; index < 4; index += 1) {
      const next = corners[(index + 1) % 4];
      meanSide += Math.hypot(next[0] - corners[index][0], next[1] - corners[index][1]);
    }
    meanSide /= 4;
    if (meanSide > 1) {
      const distance = Math.max(0.15, Math.min(4.0, camera.fx * marker.size_m / meanSide));
      const upright = [[1, 0, 0], [0, -1, 0], [0, 0, -1]];
      for (const factor of [0.55, 0.7, 0.85, 1.0]) {
        candidates.push({
          rotation: upright, translation: [0, 0, Math.min(4.0, distance * factor)],
          upright: true,
        });
      }
    }
    return candidates;
  }

  function poseFromCorners(corners, calibration, marker, planeZ) {
    const camera = cameraFrom(calibration);
    const object = objectCorners(marker.size_m);
    const candidates = poseCandidates(corners, object, camera, marker);
    const base = baseFromMarker(marker);
    const T_base_marker = [[base.rotation[0][0], base.rotation[0][1], base.rotation[0][2],
      base.translation[0]],
    [base.rotation[1][0], base.rotation[1][1], base.rotation[1][2], base.translation[1]],
    [base.rotation[2][0], base.rotation[2][1], base.rotation[2][2], base.translation[2]],
    [0, 0, 0, 1]];
    const floor = (planeZ == null ? 0 : Number(planeZ)) + 0.05;

    let lookAt = null;
    let upright = null;
    let fallback = null;
    for (const candidate of candidates) {
      const refined = refinePose(candidate.rotation, candidate.translation, object, corners, camera);
      if (!refined || refined.rms > 4.0) continue;
      const normal = [refined.rotation[0][2], refined.rotation[1][2], refined.rotation[2][2]];
      const faceOn = dot(normal, unit(scale(refined.translation, -1)));
      if (faceOn <= 0) continue;

      // inverse(T_cam_marker) = [R^T | -R^T t]
      const R = refined.rotation;
      const t = refined.translation;
      const inverse = [
        [R[0][0], R[1][0], R[2][0], -(R[0][0] * t[0] + R[1][0] * t[1] + R[2][0] * t[2])],
        [R[0][1], R[1][1], R[2][1], -(R[0][1] * t[0] + R[1][1] * t[1] + R[2][1] * t[2])],
        [R[0][2], R[1][2], R[2][2], -(R[0][2] * t[0] + R[1][2] * t[1] + R[2][2] * t[2])],
        [0, 0, 0, 1]];
      const T = multiply4(T_base_marker, inverse);
      // The phone is above the table. The mirror solution of a planar target
      // usually puts the camera below the worktop, which is physically
      // impossible and is a far better discriminator than face-on: on an
      // oblique tag both solutions can be almost equally head-on.
      const above = T[2][3] > floor && T[2][3] < floor + 3.0;
      const entry = {
        T_base_cam: T, rms: refined.rms, faceOn: faceOn, cameraZ: T[2][3], above: above,
      };
      if (!above) continue;
      if (candidate.lookAt) {
        if (!lookAt || entry.rms < lookAt.rms) lookAt = entry;
      } else if (candidate.upright) {
        if (!upright || entry.rms < upright.rms) upright = entry;
      } else if (!fallback || entry.rms < fallback.rms) {
        fallback = entry;
      }
    }
    // A pose below the worktop is the mirror twin, not the phone: refuse it
    // rather than paint the band in the wrong place.
    const best = (lookAt && lookAt.rms <= 1.0) ? lookAt
      : (upright && upright.rms <= 1.0) ? upright
        : (fallback || lookAt || upright);
    if (!best) return null;
    return { T_base_cam: best.T_base_cam, rms: best.rms, faceOn: best.faceOn };
  }

  function invert4(T) {
    const R = [[T[0][0], T[0][1], T[0][2]],
      [T[1][0], T[1][1], T[1][2]],
      [T[2][0], T[2][1], T[2][2]]];
    const t = [T[0][3], T[1][3], T[2][3]];
    return [[R[0][0], R[1][0], R[2][0], -(R[0][0] * t[0] + R[1][0] * t[1] + R[2][0] * t[2])],
      [R[0][1], R[1][1], R[2][1], -(R[0][1] * t[0] + R[1][1] * t[1] + R[2][1] * t[2])],
      [R[0][2], R[1][2], R[2][2], -(R[0][2] * t[0] + R[1][2] * t[1] + R[2][2] * t[2])],
      [0, 0, 0, 1]];
  }

  function multiply4(a, b) {
    const out = [];
    for (let row = 0; row < 4; row += 1) {
      out.push([]);
      for (let column = 0; column < 4; column += 1) {
        let value = 0;
        for (let k = 0; k < 4; k += 1) value += a[row][k] * b[k][column];
        out[row].push(value);
      }
    }
    return out;
  }

  /* ------------------------------------------------------------------- API */
  function create(options) {
    const video = options.video;
    const marker = options.marker;
    const calibration = options.calibration;
    const maxDimension = options.maxDimension || 1440;
    const canvas = document.createElement("canvas");
    const context = canvas.getContext("2d", { willReadFrequently: true });
    const cropCanvas = document.createElement("canvas");
    const cropContext = cropCanvas.getContext("2d", { willReadFrequently: true });
    let width = 0;
    let height = 0;

    function resize() {
      const sourceWidth = video.videoWidth;
      const sourceHeight = video.videoHeight;
      if (!sourceWidth || !sourceHeight) return false;
      const factor = Math.min(1, maxDimension / Math.max(sourceWidth, sourceHeight));
      const nextWidth = Math.max(1, Math.round(sourceWidth * factor));
      const nextHeight = Math.max(1, Math.round(sourceHeight * factor));
      if (nextWidth !== width || nextHeight !== height) {
        width = nextWidth;
        height = nextHeight;
        canvas.width = width;
        canvas.height = height;
      }
      return true;
    }

    function detect() {
      if (!marker || marker.id != null && marker.id !== 0) return null;
      if (!resize()) return null;
      const started = performance.now();
      context.drawImage(video, 0, 0, width, height);
      const frame = context.getImageData(0, 0, width, height);
      const found = detectFrame(frame.data, width, height);
      if (!found) return null;
      const sx = video.videoWidth / width;
      const sy = video.videoHeight / height;
      let corners = found.corners.map((point) => [point[0] * sx, point[1] * sy]);

      // Detection is cheap at 810x1440; the pose is not. Refine the four edges
      // once more at full resolution in a small crop around the tag, where the
      // black/white step is sharp and a half pixel is worth several centimetres.
      const videoWidth = video.videoWidth;
      const videoHeight = video.videoHeight;
      let minX = videoWidth, maxX = 0, minY = videoHeight, maxY = 0;
      for (const point of corners) {
        minX = Math.min(minX, point[0]);
        maxX = Math.max(maxX, point[0]);
        minY = Math.min(minY, point[1]);
        maxY = Math.max(maxY, point[1]);
      }
      const pad = Math.max(14, Math.round(Math.max(maxX - minX, maxY - minY) * 0.6));
      const x0 = Math.max(0, Math.floor(minX) - pad);
      const y0 = Math.max(0, Math.floor(minY) - pad);
      const x1 = Math.min(videoWidth, Math.ceil(maxX) + pad);
      const y1 = Math.min(videoHeight, Math.ceil(maxY) + pad);
      const cropWidth = x1 - x0;
      const cropHeight = y1 - y0;
      if (cropWidth > 8 && cropHeight > 8) {
        if (cropCanvas.width !== cropWidth || cropCanvas.height !== cropHeight) {
          cropCanvas.width = cropWidth;
          cropCanvas.height = cropHeight;
        }
        cropContext.drawImage(video, x0, y0, cropWidth, cropHeight,
          0, 0, cropWidth, cropHeight);
        const cropFrame = cropContext.getImageData(0, 0, cropWidth, cropHeight);
        const cropGray = grayscale(cropFrame.data, cropWidth, cropHeight);
        const local = corners.map((point) => [point[0] - x0, point[1] - y0]);
        const refined = refineQuad(cropGray, cropWidth, cropHeight, local);
        if (refined) {
          corners = refined.map((point) => [point[0] + x0, point[1] + y0]);
        }
      }

      // The corners are in the video's pixels; `K` is for the calibration's
      // declared size. Rescale the corners when the browser delivers a
      // different resolution of the same crop, and refuse a different aspect
      // (the planner would refuse it too).
      const source = calibration.image_size || [video.videoWidth, video.videoHeight];
      const scaleX = source[0] / video.videoWidth;
      const scaleY = source[1] / video.videoHeight;
      if (Math.abs(scaleX - scaleY) / Math.max(scaleX, scaleY) > 0.01) return null;
      const poseCorners = corners.map((point) => [point[0] * scaleX, point[1] * scaleY]);
      const pose = poseFromCorners(poseCorners, calibration, marker, options.planeZ);
      if (!pose) return null;
      return {
        ok: true, T_base_cam: pose.T_base_cam, corners: corners,
        rms_px: pose.rms, ms: performance.now() - started,
        detection_size: [width, height],
      };
    }

    return { detect: detect, marker: marker };
  }

  if (typeof window !== "undefined") {
    window.ArucoPose = {
      create: create,
      detectFrame: detectFrame,
      poseFromCorners: poseFromCorners,
      refineQuad: refineQuad,
      decodeQuad: decodeQuad,
      grayscale: grayscale,
      TAG_BITS: TAG_BITS,
    };
  }
  if (typeof module !== "undefined" && module.exports) {
    module.exports = {
      create: create,
      detectFrame: detectFrame,
      poseFromCorners: poseFromCorners,
      poseFromHomography: poseFromHomography,
      refinePose: refinePose,
      refineQuad: refineQuad,
      decodeQuad: decodeQuad,
      grayscale: grayscale,
      cameraFrom: cameraFrom,
      undistort: undistort,
      rodrigues: rodrigues,
      rotationToVector: rotationToVector,
      TAG_BITS: TAG_BITS,
    };
  }
})();

/* Generates test fixtures from the REAL web JS modules, so the Swift port can
 * be checked against the exact code the phone page runs.
 *
 *   node tools/gen_fixtures.js <fixtures-out-dir> <path-to-reBot-B601-Agent-Guide>
 */
"use strict";
const fs = require("fs");
const path = require("path");
const REPO = process.argv[3] || process.env.REBOT_REPO;
if (!REPO) {
  console.error("usage: node tools/gen_fixtures.js <out-dir> <reBot-B601-Agent-Guide repo>");
  process.exit(1);
}
const WEB = path.join(REPO, "examples/host/web");
const G = require(path.join(WEB, "geometry.js"));
const A = require(path.join(WEB, "arm-model.js"));
const Ar = require(path.join(WEB, "aruco.js"));
const L = require(path.join(WEB, "locate.js"));

const OUT = process.argv[2];
fs.mkdirSync(OUT, { recursive: true });

/* ------------------------------------------------------------ geometry */
const calib = {
  K: [[1000, 0, 405], [0, 1000, 720], [0, 0, 1]],
  dist: [0.05, -0.02, 0.001, -0.001, 0.001],
  image_size: [810, 1440],
};
const poseT = [
  [0, 1, 0, 0.3],
  [-0.8, 0, 0.6, -0.4],
  [0.6, 0, 0.8, 0.7],
  [0, 0, 0, 1],
];
const geometry = { calibration: calib, pose: poseT, project: [], undistort: [], rayPlane: [], mapPixel: [], shapeTurn: [], blend: [] };
for (const p of [[0, 0, 0.1], [0.4, 0.2, 0.1], [-0.3, 0.5, 0.1], [0.1, -0.6, 0.3], [0, 0, 2]]) {
  const cam = G.makeCamera(calib, 810, 1440);
  geometry.project.push({ point: p, pixel: G.project(cam, G.matrixPose(poseT), p) });
}
for (const uv of [[405, 720], [100, 200], [700, 1300], [50, 1400]]) {
  const cam = G.makeCamera(calib, 810, 1440);
  geometry.undistort.push({ uv, xy: G.undistort(uv[0], uv[1], cam) });
}
for (const uv of [[405, 900], [200, 1000], [600, 1200]]) {
  const cam = G.makeCamera(calib, 810, 1440);
  try {
    const hit = G.rayPlane(cam, poseT, uv[0], uv[1], 0.1);
    geometry.rayPlane.push({ uv, planeZ: 0.1, hit });
  } catch (e) {
    geometry.rayPlane.push({ uv, planeZ: 0.1, error: e.message });
  }
}
for (const turns of [0, 1, 2, 3, -1, 5]) {
  geometry.mapPixel.push({ uv: [123.5, 456.25], size: [810, 1440], turns, at: G.mapPixel(123.5, 456.25, [810, 1440], turns) });
}
for (const [c, f] of [[[810, 1440], [810, 1440]], [[810, 1440], [405, 720]], [[810, 1440], [1440, 810]], [[810, 1440], [1000, 1000]], [[1000, 1000], [1440, 810]]]) {
  geometry.shapeTurn.push({ calibration: c, frame: f, turns: G.shapeTurn(c, f) });
}
{
  const current = G.rpyToMatrix([0.1, 0.2, 0.3]);
  const target = G.rpyToMatrix([0.15, 0.18, 0.35]);
  geometry.blend.push({ current, target, fraction: 0.6, blended: G.blendOrientation(current, target, 0.6) });
}
geometry.rodrigues = [];
for (const vec of [[0, 0, 0], [0.1, -0.2, 0.3], [2.9, 0.1, -0.2], [0, 3.14159, 0]]) {
  const R = Ar.rodrigues(vec);
  const back = Ar.rotationToVector(R);
  geometry.rodrigues.push({ vec, R, back, R2: Ar.rodrigues(back) });
}
fs.writeFileSync(path.join(OUT, "geometry.json"), JSON.stringify(geometry, null, 1));

/* ----------------------------------------------------------------- arm */
const arm = { fk: [], bands: [], ik: [] };
for (const q of [[0, 0, 0, 0, 0, 0], [0, 2.02, 0.52, 1.5, 0, 0], [0.5, 1.2, 0.8, -0.4, 0.3, 1.0], [0, 2.457, 1.246, 1.315, -0.376, 0.034]]) {
  const f = A.forward(q);
  arm.fk.push({ q, rotation: f.rotation, position: f.position });
}
for (const [planeZ, hoverM, tipM] of [[0, 0.10, 0.121], [0, 0.30, 0.121], [0.02, 0.05, 0.121], [0, 0.60, 0.121]]) {
  arm.bands.push({ planeZ, hoverM, tipM, band: A.computeReachBand(planeZ, hoverM, tipM) });
}
fs.writeFileSync(path.join(OUT, "arm.json"), JSON.stringify(arm, null, 1));

/* ----------------------------------------------------------------- tag */
// Render a synthetic 4x4_50 id 0 tag through a known camera, then let the REAL
// detector + pose solve produce the expected answers.
const marker = { size_m: 0.05, at: [0.14, 0.0, 0.06], yaw_deg: 180, id: 0, dictionary: "4x4_50" };
const W = 810, H = 1440;

function renderTag(cameraPos, lookHeight, merged) {
  // Camera in front of the tag (normal side) and above the table.
  const yaw = marker.yaw_deg * Math.PI / 180;
  const normal = [Math.cos(yaw), Math.sin(yaw), 0];
  const position = [
    marker.at[0] + normal[0] * cameraPos,
    marker.at[1] + normal[1] * cameraPos,
    marker.at[2] + lookHeight,
  ];
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const unit = (a) => { const n = Math.hypot(...a); return a.map((v) => v / n); };
  const forward = unit(sub(marker.at, position));
  const right = unit(cross(forward, [0, 0, 1]));
  const down = cross(forward, right);
  const R = [[right[0], down[0], forward[0]], [right[1], down[1], forward[1]], [right[2], down[2], forward[2]]];
  const T_base_cam = [[...R[0], position[0]], [...R[1], position[1]], [...R[2], position[2]], [0, 0, 0, 1]];

  const up = [0, 0, 1];
  const mright = cross(up, normal);
  const Rm = [[mright[0], up[0], normal[0]], [mright[1], up[1], normal[1]], [mright[2], up[2], normal[2]]];

  const cam = G.makeCamera(calib, W, H);
  const gray = new Uint8Array(W * H);
  const half = marker.size_m / 2;
  for (let y = 0; y < H; y += 1) {
    for (let x = 0; x < W; x += 1) {
      const [xn, yn] = G.undistort(x + 0.5, y + 0.5, cam);
      const dirCam = [xn, yn, 1];
      const dir = G.matVec(R, dirCam);
      const denom = dir[0] * normal[0] + dir[1] * normal[1] + dir[2] * normal[2];
      let value = 170; // background
      if (Math.abs(denom) > 1e-9) {
        const rel = sub(marker.at, position);
        const t = (rel[0] * normal[0] + rel[1] * normal[1] + rel[2] * normal[2]) / denom;
        if (t > 0) {
          const p = [position[0] + t * dir[0] - marker.at[0], position[1] + t * dir[1] - marker.at[1], position[2] + t * dir[2] - marker.at[2]];
          // into marker frame: Rm^T p
          const mx = Rm[0][0] * p[0] + Rm[1][0] * p[1] + Rm[2][0] * p[2];
          const my = Rm[0][1] * p[0] + Rm[1][1] * p[1] + Rm[2][1] * p[2];
          if (Math.abs(mx) <= half && Math.abs(my) <= half) {
            const col = Math.min(5, Math.floor((mx + half) / marker.size_m * 6));
            const row = Math.min(5, Math.floor((half - my) / marker.size_m * 6));
            value = Ar.TAG_BITS[row][col] ? 30 : 220;
          } else if (Math.abs(mx) <= half * 1.6 && Math.abs(my) <= half * 1.6) {
            value = 235; // paper margin
          } else if (merged && my < -half * 1.6 && my > -half * 4 && Math.abs(mx) <= half * 1.6) {
            value = 25; // the black base touching the paper edge
          }
        }
      }
      gray[y * W + x] = value;
    }
  }
  return { gray, T_base_cam };
}

function fixtureFor(name, dist, height, merged) {
  const { gray, T_base_cam } = renderTag(dist, height, merged);
  const rgba = Buffer.alloc(W * H * 4);
  for (let i = 0; i < gray.length; i += 1) {
    rgba[4 * i] = gray[i]; rgba[4 * i + 1] = gray[i]; rgba[4 * i + 2] = gray[i]; rgba[4 * i + 3] = 255;
  }
  const found = Ar.detectFrame(rgba, W, H);
  if (!found) throw new Error(name + ": JS detector found nothing");
  const pose = Ar.poseFromCorners(found.corners, calib, marker, 0);
  if (!pose) throw new Error(name + ": JS pose solve failed");
  fs.writeFileSync(path.join(OUT, name + ".gray"), Buffer.from(gray));
  const meta = {
    width: W, height: H, calibration: calib, marker, planeZ: 0,
    true_T_base_cam: T_base_cam,
    detect_corners: found.corners,
    pose: pose,
  };
  // locate cases through the solved pose, portrait + landscape turns
  meta.locate = [];
  for (const turn of [0, 1, 2, 3]) {
    const vw = turn % 2 ? H : W;
    const vh = turn % 2 ? W : H;
    const uv = turn % 2 ? [900, 300] : [300, 900];
    try {
      const target = L.tipTarget(calib, pose.T_base_cam, uv[0], uv[1], 0, 0.10, vw, vh, turn);
      meta.locate.push({ turn, vw, vh, uv, height: 0.10, target });
    } catch (e) {
      meta.locate.push({ turn, vw, vh, uv, height: 0.10, error: e.message });
    }
  }
  fs.writeFileSync(path.join(OUT, name + ".json"), JSON.stringify(meta, null, 1));
  console.log(name, "corners", found.corners.map((c) => c.map((v) => v.toFixed(1))),
    "rms", pose.rms.toFixed(3),
    "camZ", pose.T_base_cam[2][3].toFixed(3),
    "true camZ", T_base_cam[2][3].toFixed(3));
}

fixtureFor("tag_front", 0.8, 0.45, false);
fixtureFor("tag_merged", 0.6, 0.3, true);
fixtureFor("tag_oblique", 1.2, 0.75, false);

/* ------------------------------------------------------------ depth fit */
// Synthetic LiDAR scenes: sample the arm's capsule surfaces using the REAL
// arm-model.js FK (the same chain the Swift port uses), place them through a
// known T_base_world, add noise + background outliers, and let the Swift fit
// recover the transform. Radii are the host's collision table sweep.RADII[1..].
const RADII = [0.055, 0.050, 0.045, 0.035, 0.030, 0.020];

function lcg(seed) {
  let state = seed >>> 0;
  return () => {
    state = (state * 1664525 + 1013904223) >>> 0;
    return state / 4294967296;
  };
}

function gauss(rand) {
  // Box-Muller, deterministic with the LCG above.
  const u = Math.max(1e-12, rand());
  const v = rand();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

function jointPoints(q) {
  const fk = A.forward(q, true);
  return fk.frames.map((f) => f.position).concat([fk.position]);
}

function sampleCapsules(q, perLink, rand, noise, filter) {
  const pts = jointPoints(q);
  const out = [];
  for (let i = 1; i < pts.length; i += 1) {
    const a = pts[i - 1], b = pts[i], r = RADII[Math.min(i - 1, RADII.length - 1)];
    const ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    const len = Math.hypot(...ab);
    const d = ab.map((v) => v / len);
    // Orthonormal basis perpendicular to the axis.
    const helper = Math.abs(d[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
    const u0 = [d[1] * helper[2] - d[2] * helper[1], d[2] * helper[0] - d[0] * helper[2], d[0] * helper[1] - d[1] * helper[0]];
    const ul = Math.hypot(...u0);
    const u = u0.map((v) => v / ul);
    const v = [d[1] * u[2] - d[2] * u[1], d[2] * u[0] - d[0] * u[2], d[0] * u[1] - d[1] * u[0]];
    for (let k = 0; k < perLink; k += 1) {
      const t = (k + 0.5) / perLink;
      const phi = k * 2.399963; // golden angle
      const p = [0, 1, 2].map((c) =>
        a[c] + ab[c] * t + r * (Math.cos(phi) * u[c] + Math.sin(phi) * v[c]) + gauss(rand) * noise);
      if (!filter || filter(p)) out.push(p);
    }
  }
  return out;
}

function toWorld(p, t, yaw) {
  const c = Math.cos(yaw), s = Math.sin(yaw);
  return [c * p[0] - s * p[1] + t[0], s * p[0] + c * p[1] + t[1], p[2] + t[2]];
}

function depthCase(name, q, t, yaw, opts) {
  const rand = lcg(opts.seed);
  let basePoints = sampleCapsules(q, opts.perLink, rand, 0.003, opts.filter);
  if (opts.limit) basePoints = basePoints.slice(0, opts.limit);
  const world = basePoints.map((p) => toWorld(p, t, yaw));
  // Background outliers inside the segmentation volume.
  const outliers = Math.round(world.length * 0.15);
  for (let k = 0; k < outliers; k += 1) {
    const angle = rand() * 2 * Math.PI;
    const radius = 0.15 + rand() * 0.35;
    world.push([t[0] + radius * Math.cos(angle), t[1] + radius * Math.sin(angle), t[2] + rand() * 0.5]);
  }
  // One far point the segmentation filter must drop.
  world.push([t[0] + 2.0, t[1] + 2.0, t[2] + 0.3]);
  return {
    name, q, truth: { t, yaw },
    initial: [t[0] + 0.02, t[1] - 0.015, t[2] + 0.01],
    points: world,
    jointPoints: jointPoints(q),
    expect: opts.expect,
  };
}

const depth = { radii: RADII, cases: [] };
const qGood = [0.4, 1.2, 0.9, -0.3, 0.4, 0.2];
depth.cases.push(depthCase("depth_good", qGood, [0.8, -0.3, 0.02], 0.7,
  { seed: 42, perLink: 120, expect: "ok" }));
depth.cases.push(depthCase("depth_weakyaw", qGood, [0.8, -0.3, 0.02], 0.7,
  { seed: 7, perLink: 200, expect: "reject", filter: (p) => Math.hypot(p[0], p[1]) < 0.07 }));
depth.cases.push(depthCase("depth_sparse", qGood, [0.8, -0.3, 0.02], 0.7,
  { seed: 9, perLink: 120, expect: "reject", limit: 40 }));
fs.writeFileSync(path.join(OUT, "depth_fit.json"), JSON.stringify(depth, null, 1));
console.log("depth cases:", depth.cases.map((c) => c.name + " (" + c.points.length + " pts)").join(", "));

console.log("fixtures written to", OUT);

-- 0023: seed the bench-default intrinsics.
--
-- The active rig (9caf6f58d22f) was fitted against calib fingerprint
-- 9acd8707e4ad, but the catalog only had the placeholder row created by
-- `ensure_calibration` (K=I, size [0,0], calib_doc NULL). The phone then fell
-- back to a stale 640x480 file in work/calib and refused the 1920x1080 tag
-- solve as an aspect change.
--
-- This seeds the real 1080x1920 portrait calibration (the frame's quarter-turn
-- upright grid) as the single default. `store.read_latest_calib_doc()` prefers
-- `is_default`, so startup materializes this file even when older test
-- calibrations exist in the catalog. `config/calib/` carries the same bytes for
-- deployments that run capture without PG.

ALTER TABLE world.calibration
  ADD COLUMN IF NOT EXISTS is_default boolean NOT NULL DEFAULT false;

CREATE UNIQUE INDEX IF NOT EXISTS calibration_default_idx
  ON world.calibration(is_default) WHERE is_default;

UPDATE world.calibration SET is_default = false;

INSERT INTO world.calibration(calib_fingerprint, K, dist, image_size, calib_doc, is_default)
SELECT '9acd8707e4ad', d -> 'K', d -> 'dist', d -> 'image_size', d, true
  FROM (SELECT $calib$
{
  "format": "rebot-camera-calib",
  "version": 1,
  "created": "2026-11-13T16:32:21",
  "device": "4f68b7f34a702b1c043f44fcf805ac9a63e2a3d9",
  "image_size": [
    1080,
    1920
  ],
  "K": [
    [
      1375.1456325989602,
      0.0,
      521.8416180690382
    ],
    [
      0.0,
      1373.745953857104,
      961.060237213127
    ],
    [
      0.0,
      0.0,
      1.0
    ]
  ],
  "dist": [
    0.11764551449199466,
    -0.3186013174363159,
    -0.0013080819267459573,
    -0.002634550024215069,
    0.3085010023144166
  ],
  "rms": 0.2633247485845523,
  "board": {
    "cols": 9,
    "rows": 7,
    "square_m": 0.02
  },
  "views": 20,
  "source_captures": [
    "20261113T082921Z-cf0c41",
    "20261113T082925Z-53ddb6",
    "20261113T082930Z-423792",
    "20261113T082940Z-2e1701",
    "20261113T082943Z-2ee0e2",
    "20261113T082956Z-a7a818",
    "20261113T083003Z-4f88e2",
    "20261113T083006Z-bbcf1e",
    "20261113T083010Z-23bc27",
    "20261113T083019Z-061366",
    "20261113T083023Z-9d1bf5",
    "20261113T083029Z-9fa5f1",
    "20261113T083033Z-53f05e",
    "20261113T083037Z-02ba41",
    "20261113T083039Z-3ec757",
    "20261113T083043Z-643cec",
    "20261113T083047Z-2c71d3",
    "20261113T083049Z-044947",
    "20261113T083057Z-0f787d",
    "20261113T083129Z-5d68a4"
  ],
  "notes": "pinhole + radial/tangential; rescale with calib.scale_K for other sizes",
  "orientation_deg": 0
}
$calib$::jsonb AS d) seeded
ON CONFLICT (calib_fingerprint) DO UPDATE
  SET K = EXCLUDED.K,
      dist = EXCLUDED.dist,
      image_size = EXCLUDED.image_size,
      calib_doc = EXCLUDED.calib_doc,
      is_default = true;

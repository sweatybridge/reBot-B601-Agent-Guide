-- 0006_calib_doc.sql — full intrinsics document for lossless PG calibration storage.
ALTER TABLE world.calibration ADD COLUMN IF NOT EXISTS calib_doc jsonb;
-- 0023 default intrinsics: the seeded bench calibration is the only default.
-- Run: psql -v ON_ERROR_STOP=1 -f tests/default_intrinsics_tests.sql

DO $$
DECLARE
  c world.calibration%ROWTYPE;
  n integer;
BEGIN
  SELECT * INTO c FROM world.calibration WHERE is_default;
  IF NOT FOUND THEN RAISE EXCEPTION 'no default intrinsics row'; END IF;
  IF c.calib_fingerprint <> '9acd8707e4ad' THEN
    RAISE EXCEPTION 'default fingerprint %', c.calib_fingerprint; END IF;
  IF (c.image_size->>0)::int <> 1080 OR (c.image_size->>1)::int <> 1920 THEN
    RAISE EXCEPTION 'default image_size %', c.image_size; END IF;
  IF abs((c.K->0->>0)::float8 - 1375.1456325989602) > 1e-9
     OR abs((c.K->1->>1)::float8 - 1373.745953857104) > 1e-9 THEN
    RAISE EXCEPTION 'default K %', c.K; END IF;
  IF c.calib_doc IS NULL
     OR c.calib_doc->>'device' <> '4f68b7f34a702b1c043f44fcf805ac9a63e2a3d9'
     OR (c.calib_doc->>'orientation_deg')::int <> 0 THEN
    RAISE EXCEPTION 'default calib_doc %', c.calib_doc; END IF;
  SELECT count(*) INTO n FROM world.calibration WHERE is_default;
  IF n <> 1 THEN RAISE EXCEPTION 'expected one default, got %', n; END IF;
  RAISE NOTICE 'OK default intrinsics 1080x1920 (fx %, device %)',
    (c.K->0->>0)::float8, c.calib_doc->>'device';
END $$;

SELECT 'DEFAULT INTRINSICS TESTS PASSED' AS result;

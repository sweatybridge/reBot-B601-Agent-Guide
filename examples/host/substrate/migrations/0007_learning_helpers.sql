-- 0007_learning_helpers.sql — SQL ports of identify accuracy helpers (spec §7.4).
-- Operate on learning.accuracy_cell. Apply after 0001-0006.

-- identify.accuracy_cell: radius x height x sector cell key for a tip point.
CREATE OR REPLACE FUNCTION accuracy_cell_key(tip float8[]) RETURNS text AS $$
DECLARE
  RADIUS_EDGES constant float8[] := ARRAY[0.10,0.20,0.30,0.40,0.50,0.60];
  HEIGHT_EDGES constant float8[] := ARRAY[0.00,0.10,0.20,0.30,0.40];
  SECTORS constant int := 8;
  radius float8; z float8; ang float8;
  ri int := 0; hi int := 0; sector int; i int;
BEGIN
  radius := sqrt(tip[1]*tip[1] + tip[2]*tip[2]);
  z := tip[3];
  FOR i IN 1..array_length(RADIUS_EDGES,1)-1 LOOP
    IF radius >= RADIUS_EDGES[i] THEN ri := i-1; END IF;
  END LOOP;
  ri := least(ri, array_length(RADIUS_EDGES,1)-2);
  FOR i IN 1..array_length(HEIGHT_EDGES,1)-1 LOOP
    IF z >= HEIGHT_EDGES[i] THEN hi := i-1; END IF;
  END LOOP;
  hi := least(hi, array_length(HEIGHT_EDGES,1)-2);
  ang := atan2(tip[2], tip[1]) + 2*pi();
  ang := ang - floor(ang/(2*pi()))*(2*pi());
  sector := floor(ang / (2*pi()/SECTORS))::int % SECTORS;
  RETURN format('r%s-h%s-s%s', ri, hi, sector);
END;
$$ LANGUAGE plpgsql IMMUTABLE;

-- identify.validated_fraction: fraction of measured cells that are validated.
CREATE OR REPLACE FUNCTION accuracy_validated_fraction(p_arm_id text, p_rig_fingerprint text)
RETURNS float8 AS $$
DECLARE total int; validated int;
BEGIN
  SELECT count(*), count(*) FILTER (WHERE validated_at IS NOT NULL)
    INTO total, validated FROM learning.accuracy_cell
    WHERE arm_id=p_arm_id AND rig_fingerprint=p_rig_fingerprint;
  IF total = 0 THEN RETURN 0.0; END IF;
  RETURN validated::float8 / total;
END;
$$ LANGUAGE plpgsql STABLE;

-- identify.accuracy_warnings: one warning per target whose cell is unvalidated.
CREATE OR REPLACE FUNCTION accuracy_warnings(p_arm_id text, p_rig_fingerprint text, tips jsonb)
RETURNS jsonb AS $$
DECLARE
  t jsonb; tipp float8[]; ck text; n int; val boolean;
  warnings jsonb := '[]'::jsonb; idx int := 0;
BEGIN
  FOR t IN SELECT jsonb_array_elements(tips) LOOP
    idx := idx + 1;
    tipp := ARRAY[(t->>0)::float8, (t->>1)::float8, (t->>2)::float8];
    ck := accuracy_cell_key(tipp);
    SELECT c.n, (c.validated_at IS NOT NULL) INTO n, val
      FROM learning.accuracy_cell c
      WHERE c.arm_id=p_arm_id AND c.rig_fingerprint=p_rig_fingerprint AND c.cell_key=ck;
    IF NOT FOUND OR NOT val THEN
      warnings := warnings || jsonb_build_array(format(
        'target %s: cell %s is not validated (n=%s measured holds): accuracy outside the measured envelope is not guaranteed',
        idx, ck, coalesce(n,0)));
    END IF;
  END LOOP;
  RETURN warnings;
END;
$$ LANGUAGE plpgsql STABLE;
-- 0018: learning thresholds group in world.arm_config (required-config
-- consolidation, Phase C). The 21 thresholds formerly hardcoded in identify.py,
-- rebot_fit_sag_map (Rust) and accuracy_cell_update (0002) move into the
-- "learning" group. Seed bit-identical to the former constants -> learning is
-- unchanged. accuracy_cell_update now takes this group as a REQUIRED jsonb arg
-- (passed by the caller from the table); a missing doc/field is a hard refusal.
-- rebot_fit_sag_map is Rust (image rebuild), not in this migration.
-- Idempotent merge into the existing config_doc.

UPDATE world.arm_config
   SET config_doc = config_doc || '{"learning":{"rig_max_age_days":30.0,"tag_growth_px":2.0,"ewma_alpha":0.3,"marker_min_usable":3,"marker_spread_max_mm":6.0,"holdout_max_px":3.0,"tag_prior_depth":0.01,"sag_min_holds":30,"sag_huber_rad":0.010,"sag_validate_max_mrad":15.0,"sag_validate_ratio":2.0,"sag_span_radius_m":0.10,"sag_span_height_m":0.05,"accuracy_min_n":3,"accuracy_radius_edges":[0.10,0.20,0.30,0.40,0.50,0.60],"accuracy_height_edges":[0.00,0.10,0.20,0.30,0.40],"accuracy_sectors":8,"coverage_max_poses":12,"coverage_hold_s":2.0,"coverage_perturbations":24,"coverage_max_reach_m":0.75}}'::jsonb,
       updated_at = now()
 WHERE arm_id = 'b601';

-- --------------------------------------------------------------------------- --
-- identify.update_accuracy_map: fold one oracle doc's holds into accuracy cells.
-- A cell validates at n_good >= MIN_N under threshold; a later bad hold
-- un-validates. Operates on learning.accuracy_cell (per arm + shared rig).
-- The grid/threshold sources come from the REQUIRED config arg (the
-- world.arm_config "learning" group); body otherwise verbatim from 0002.
-- CREATE OR REPLACE cannot insert a required param before a defaulted one, so
-- the old 4-arg signature is dropped first.
-- --------------------------------------------------------------------------- --
DROP FUNCTION IF EXISTS accuracy_cell_update(text,text,jsonb,float8);
CREATE OR REPLACE FUNCTION accuracy_cell_update(
  p_arm_id text, p_rig_fingerprint text, p_doc jsonb, config jsonb, p_threshold_mm float8 DEFAULT 10.0
) RETURNS void AS $$
DECLARE
  h jsonb;
  tip jsonb;
  err float8;
  ck text; ri int; hi int; sector int;
  radius float8; z float8; ang float8;
  RADIUS_EDGES float8[] := ARRAY(SELECT jsonb_array_elements_text(config->'accuracy_radius_edges')::float8);
  HEIGHT_EDGES float8[] := ARRAY(SELECT jsonb_array_elements_text(config->'accuracy_height_edges')::float8);
  SECTORS int := (config->>'accuracy_sectors')::int;
  MIN_N int := (config->>'accuracy_min_n')::int;
BEGIN
  IF config IS NULL OR NOT (config ?& array['accuracy_radius_edges','accuracy_height_edges','accuracy_sectors','accuracy_min_n']) THEN
    RAISE EXCEPTION 'learning config missing required keys';
  END IF;
  FOR h IN SELECT jsonb_array_elements(coalesce(p_doc->'holds','[]'::jsonb)) LOOP
    err := (h->>'pixel_error_mm')::float8;
    tip := coalesce(h->'tip_fk_cmd', h->'tip_fk_meas');
    IF err IS NULL OR tip IS NULL OR jsonb_typeof(tip) <> 'array' THEN CONTINUE; END IF;
    radius := sqrt(power((tip->>0)::float8,2) + power((tip->>1)::float8,2));
    z := (tip->>2)::float8;
    -- cell index (identify.accuracy_cell)
    ri := 0;
    FOR i IN 1..array_length(RADIUS_EDGES,1)-1 LOOP
      IF radius >= RADIUS_EDGES[i] THEN ri := i-1; END IF;
    END LOOP;
    ri := least(ri, array_length(RADIUS_EDGES,1)-2);
    hi := 0;
    FOR i IN 1..array_length(HEIGHT_EDGES,1)-1 LOOP
      IF z >= HEIGHT_EDGES[i] THEN hi := i-1; END IF;
    END LOOP;
    hi := least(hi, array_length(HEIGHT_EDGES,1)-2);
    ang := atan2((tip->>1)::float8,(tip->>0)::float8) + 2*pi();
    ang := ang - floor(ang/(2*pi()))*(2*pi());
    sector := floor(ang / (2*pi()/SECTORS))::int % SECTORS;
    ck := format('r%s-h%s-s%s', ri, hi, sector);
    INSERT INTO learning.accuracy_cell(arm_id, rig_fingerprint, cell_key, radius_idx, height_idx, sector, n, n_good, measured_mm)
      VALUES (p_arm_id, p_rig_fingerprint, ck, ri, hi, sector, 0, 0, NULL)
      ON CONFLICT (arm_id, rig_fingerprint, cell_key) DO NOTHING;
    UPDATE learning.accuracy_cell SET
      n = n + 1,
      measured_mm = round(greatest(coalesce(measured_mm,0), err)::numeric, 2),
      n_good = n_good + CASE WHEN err <= p_threshold_mm THEN 1 ELSE 0 END,
      validated_at = CASE
        WHEN err > p_threshold_mm THEN NULL
        WHEN n_good + 1 >= MIN_N AND validated_at IS NULL THEN now()
        ELSE validated_at END,
      updated_at = now()
    WHERE arm_id = p_arm_id AND rig_fingerprint = p_rig_fingerprint AND cell_key = ck;
  END LOOP;
END;
$$ LANGUAGE plpgsql;
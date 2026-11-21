-- 0002_learning_functions.sql — learning-plane pure functions (spec §5.4, §7.3b, §8.2).
-- Ports of oracle.attribute, trace._redact, and identify.update_accuracy_map.
-- Pure/immutable where possible; no hardware, no I/O. Apply after 0001_substrate.sql.

-- --------------------------------------------------------------------------- --
-- trace._redact: drop secret keys recursively; reduce llm_url to its host.
-- Keys matching token|authorization|api_key are dropped, EXCEPT usage counters
-- (prompt_/completion_/total_/max_/context_ tokens) which are counts, not secrets.
-- --------------------------------------------------------------------------- --
CREATE OR REPLACE FUNCTION trace_redact(value jsonb) RETURNS jsonb AS $$
DECLARE
  result jsonb;
  k text;
  v jsonb;
  elem jsonb;
  arr jsonb := '[]'::jsonb;
  host text;
BEGIN
  IF value IS NULL THEN RETURN NULL; END IF;
  CASE jsonb_typeof(value)
    WHEN 'object' THEN
      result := '{}'::jsonb;
      FOR k, v IN SELECT je.key, je.value FROM jsonb_each(value) je LOOP
        IF k ~* '(token|authorization|api_key)'
           AND k !~* '^(prompt_|completion_|total_|max_|context_)?tokens$' THEN
          CONTINUE;
        END IF;
        IF lower(k) = 'llm_url' THEN
          -- keep only the host of the URL
          host := substring(coalesce(v #>> '{}', '') from '^(?:https?://)?([^/:]+)');
          result := result || jsonb_build_object(k, coalesce(host, ''));
        ELSE
          result := result || jsonb_build_object(k, trace_redact(v));
        END IF;
      END LOOP;
      RETURN result;
    WHEN 'array' THEN
      FOR elem IN SELECT jsonb_array_elements(value) LOOP
        arr := arr || jsonb_build_array(trace_redact(elem));
      END LOOP;
      RETURN arr;
    ELSE
      RETURN value;
  END CASE;
END;
$$ LANGUAGE plpgsql IMMUTABLE;

-- --------------------------------------------------------------------------- --
-- oracle.attribute: residual shape -> {layer, problems[], confidence} (§5.4).
-- Faithful port of oracle.py:attribute. Every field optional; missing is reported
-- as missing, never guessed. Thresholds match oracle.py's constants block.
-- --------------------------------------------------------------------------- --
CREATE OR REPLACE FUNCTION oracle_attribute(doc jsonb) RETURNS jsonb AS $$
DECLARE
  holds jsonb := coalesce(doc->'holds', '[]'::jsonb);
  telemetry jsonb := coalesce(doc->'telemetry', '{}'::jsonb);
  witness jsonb := coalesce(doc->'witness', '{}'::jsonb);
  problems jsonb := '[]'::jsonb;
  h jsonb;
  pixel_bad jsonb := '[]'::jsonb;
  biased jsonb := '[]'::jsonb;
  tip_bad jsonb := '[]'::jsonb;
  ik_bad jsonb := '[]'::jsonb;
  n_holds int := jsonb_array_length(holds);
  -- thresholds (oracle.py)
  PIXEL_ERROR_MM constant float8 := 10.0;
  TIP_ERROR_MM constant float8 := 8.0;
  HOLD_ERROR_RAD constant float8 := 0.030;
  WITNESS_DRIFT_PX constant float8 := 2.0;
  WITNESS_SCALE_TOL constant float8 := 0.05;
  SAG_CORRELATION constant float8 := 0.8;
  TIP_VECTOR_COHERENCE constant float8 := 0.8;
  ZERO_BIAS_SPREAD_RAD constant float8 := 0.005;
  REACH_FRACTION constant float8 := 0.9;
  PLACEMENT_XY_MM constant float8 := 10.0;
  PLACEMENT_Z_TOL_MM constant float8 := 5.0;
  IK_RESIDUAL_MM constant float8 := 10.0;
  drift float8; scale float8;
  coherence float8;
  vx float8; vy float8; nv float8;
  sx float8 := 0; sy float8 := 0; cnt int := 0;
  err jsonb; sag jsonb; ne float8; ns float8; dot float8;
  max_abs float8;
  has_sag boolean := false;
  constant_bias boolean := false;
  width int; j int; mean_j float8; spread_j float8; max_spread float8; max_mean float8;
  radius float8; reach float8;
  placement jsonb; perr jsonb; pheight float8;
  layer text; confidence text;
  measured_cnt int := 0;
  abort_partial boolean := false;
BEGIN
  -- execution faults first
  IF telemetry ? 'abort' AND telemetry->'abort' IS NOT NULL AND jsonb_typeof(telemetry->'abort') <> 'null' THEN
    problems := problems || jsonb_build_array(jsonb_build_object(
      'kind','follow-abort','layer','execution',
      'detail', format('playback aborted at t=%s: %s', telemetry->'abort'->>'t', telemetry->'abort'->>'reason')));
    abort_partial := true;
  END IF;
  IF telemetry ? 'gripper_stall' AND telemetry->'gripper_stall' IS NOT NULL AND jsonb_typeof(telemetry->'gripper_stall') <> 'null' THEN
    problems := problems || jsonb_build_array(jsonb_build_object(
      'kind','gripper-stall','layer','execution',
      'detail', format('gripper stalled at t=%s (commanded %s, measured %s)',
        telemetry->'gripper_stall'->>'t', telemetry->'gripper_stall'->>'commanded', telemetry->'gripper_stall'->>'measured')));
    abort_partial := true;
  END IF;

  -- witness stale-rig
  drift := (witness->>'drift_px')::float8;
  scale := (witness->>'scale')::float8;
  IF (drift IS NOT NULL AND drift > WITNESS_DRIFT_PX)
     OR (scale IS NOT NULL AND abs(scale - 1.0) > WITNESS_SCALE_TOL) THEN
    problems := problems || jsonb_build_array(jsonb_build_object(
      'kind','stale-rig','layer','camera-arm',
      'detail', format('the camera moved since the rig was fitted (drift %s px, scale %s)', drift, scale)));
  END IF;

  -- classify holds
  FOR h IN SELECT jsonb_array_elements(holds) LOOP
    IF (h->>'pixel_error_mm')::float8 > PIXEL_ERROR_MM THEN pixel_bad := pixel_bad || jsonb_build_array(h); END IF;
    -- max abs hold_error_rad
    max_abs := 0;
    IF h ? 'hold_error_rad' AND jsonb_typeof(h->'hold_error_rad') = 'array' THEN
      SELECT max(abs(x::float8)) INTO max_abs FROM jsonb_array_elements_text(h->'hold_error_rad') x;
    END IF;
    IF coalesce(max_abs,0) > HOLD_ERROR_RAD THEN biased := biased || jsonb_build_array(h); END IF;
    IF (h->>'tip_error_mm')::float8 > TIP_ERROR_MM THEN tip_bad := tip_bad || jsonb_build_array(h); END IF;
    IF (h->>'ik_residual_mm')::float8 > IK_RESIDUAL_MM THEN ik_bad := ik_bad || jsonb_build_array(h); END IF;
    IF h ? 'pixel_error_mm' AND jsonb_typeof(h->'pixel_error_mm') <> 'null' THEN measured_cnt := measured_cnt + 1; END IF;
  END LOOP;

  -- pixel layer: coherence of error directions
  IF jsonb_array_length(pixel_bad) > 0 THEN
    FOR h IN SELECT jsonb_array_elements(pixel_bad) LOOP
      IF h ? 'detected_tip_px' AND h ? 'predicted_tip_px'
         AND jsonb_typeof(h->'detected_tip_px')='array' AND jsonb_typeof(h->'predicted_tip_px')='array' THEN
        vx := (h->'detected_tip_px'->>0)::float8 - (h->'predicted_tip_px'->>0)::float8;
        vy := (h->'detected_tip_px'->>1)::float8 - (h->'predicted_tip_px'->>1)::float8;
        nv := sqrt(vx*vx + vy*vy);
        IF nv > 1e-6 THEN sx := sx + vx/nv; sy := sy + vy/nv; cnt := cnt + 1; END IF;
      END IF;
    END LOOP;
    coherence := CASE WHEN cnt >= 2 THEN sqrt(sx*sx + sy*sy)/cnt ELSE 1.0 END;
    IF coherence >= TIP_VECTOR_COHERENCE AND NOT EXISTS (
         SELECT 1 FROM jsonb_array_elements(pixel_bad) p
         WHERE (p->>'tip_error_mm')::float8 IS NULL OR (p->>'tip_error_mm')::float8 > TIP_ERROR_MM) THEN
      problems := problems || jsonb_build_array(jsonb_build_object(
        'kind','rig-mismatch','layer','camera-arm',
        'detail', format('%s hold(s) with pixel error > %s mm while the tip held within %s mm: rig / K / turns',
          jsonb_array_length(pixel_bad), PIXEL_ERROR_MM, TIP_ERROR_MM)));
    ELSIF coherence < TIP_VECTOR_COHERENCE THEN
      problems := problems || jsonb_build_array(jsonb_build_object(
        'kind','tip-detector','layer','perception',
        'detail', format('%s hold(s) with pixel error > %s mm but scattered directions (coherence %s): the green-tip heuristic at rotated wrists, not a rig shift',
          jsonb_array_length(pixel_bad), PIXEL_ERROR_MM, round(coherence::numeric,2))));
    END IF;
    -- turns correction
    IF (doc->'stream_turn'->>'source') IS DISTINCT FROM 'measured'
       AND ((doc->'stream_turn'->>'source') = 'shape' OR coalesce((doc->>'landscape')::boolean,false)) THEN
      problems := problems || jsonb_build_array(jsonb_build_object(
        'kind','turns-correction','layer','camera-arm',
        'detail', format('pixel error with stream_turn source %s%s',
          quote_literal(coalesce(doc->'stream_turn'->>'source','null')),
          CASE WHEN coalesce((doc->>'landscape')::boolean,false) THEN ' in landscape' ELSE '' END)));
    END IF;
  END IF;

  -- per-joint hold error: sag vs zeros vs generic
  IF jsonb_array_length(biased) > 0 THEN
    FOR h IN SELECT jsonb_array_elements(biased) LOOP
      err := h->'hold_error_rad'; sag := h->'sag_rad';
      IF jsonb_typeof(err)='array' AND jsonb_typeof(sag)='array'
         AND jsonb_array_length(err)>=6 AND jsonb_array_length(sag)>=6 THEN
        ne := 0; ns := 0; dot := 0;
        FOR j IN 0..5 LOOP
          ne := ne + power((err->>j)::float8,2);
          ns := ns + power((sag->>j)::float8,2);
          dot := dot + (err->>j)::float8 * (sag->>j)::float8;
        END LOOP;
        ne := sqrt(ne); ns := sqrt(ns);
        IF ne >= 1e-9 AND ns >= 1e-9 AND abs(dot/(ne*ns)) >= SAG_CORRELATION THEN
          has_sag := true;
        END IF;
      END IF;
    END LOOP;
    -- constant bias across >=2 holds
    IF jsonb_array_length(biased) >= 2 THEN
      width := 6; max_spread := 0; max_mean := 0; constant_bias := true;
      FOR j IN 0..width-1 LOOP
        SELECT avg((e->'hold_error_rad'->>j)::float8),
               max((e->'hold_error_rad'->>j)::float8)-min((e->'hold_error_rad'->>j)::float8)
          INTO mean_j, spread_j FROM jsonb_array_elements(biased) e;
        max_mean := greatest(max_mean, abs(mean_j));
        max_spread := greatest(max_spread, spread_j);
      END LOOP;
      constant_bias := (max_mean > HOLD_ERROR_RAD AND max_spread <= ZERO_BIAS_SPREAD_RAD);
    END IF;
    IF has_sag THEN
      problems := problems || jsonb_build_array(jsonb_build_object('kind','sag','layer','arm-motor',
        'detail','hold error tracks tau_g/kp: sag model / gains'));
    ELSIF constant_bias THEN
      problems := problems || jsonb_build_array(jsonb_build_object('kind','joint-zeros','layer','arm-motor',
        'detail','constant per-joint bias across poses: joint zeros'));
    ELSE
      problems := problems || jsonb_build_array(jsonb_build_object('kind','hold-error','layer','arm-motor',
        'detail', format('%s hold(s) above %s mrad, no load or bias signature', jsonb_array_length(biased), 1000.0*HOLD_ERROR_RAD)));
    END IF;
  END IF;

  -- tip error near reach boundary
  IF jsonb_array_length(tip_bad) > 0 THEN
    radius := (doc->>'max_radius_m')::float8;
    reach := (doc->>'max_reach_m')::float8;
    IF radius IS NOT NULL AND reach IS NOT NULL AND reach > 0 AND radius >= REACH_FRACTION*reach
       AND coalesce((doc->>'ik_refusals')::int,0) > 0 THEN
      problems := problems || jsonb_build_array(jsonb_build_object('kind','reach-model','layer','reach',
        'detail','tip error grows near the reach boundary with IK refusals in the tape: reach model / --max-reach'));
    ELSE
      problems := problems || jsonb_build_array(jsonb_build_object('kind','tip-error','layer','arm-motor',
        'detail', format('%s hold(s) with tip error > %s mm', jsonb_array_length(tip_bad), TIP_ERROR_MM)));
    END IF;
  END IF;

  -- IK residual bridge
  IF jsonb_array_length(ik_bad) > 0 AND NOT EXISTS (
       SELECT 1 FROM jsonb_array_elements(ik_bad) p
       WHERE (p->>'tip_error_mm')::float8 IS NULL OR (p->>'tip_error_mm')::float8 > TIP_ERROR_MM) THEN
    problems := problems || jsonb_build_array(jsonb_build_object('kind','ik-residual','layer','ik',
      'detail', format('%s hold(s) where the IK solution misses its target by > %s mm while the arm held within %s mm: solver / model',
        jsonb_array_length(ik_bad), IK_RESIDUAL_MM, TIP_ERROR_MM)));
  END IF;

  -- one-plane limitation
  placement := doc->'placement';
  IF placement IS NOT NULL AND jsonb_typeof(placement)='object' THEN
    perr := placement->'error_mm';
    pheight := (placement->>'object_height_mm')::float8;
    IF jsonb_typeof(perr)='array' AND jsonb_array_length(perr)=3 AND pheight IS NOT NULL
       AND abs((perr->>0)::float8) <= PLACEMENT_XY_MM
       AND abs((perr->>1)::float8) <= PLACEMENT_XY_MM
       AND abs((perr->>2)::float8 - pheight) <= PLACEMENT_Z_TOL_MM THEN
      problems := problems || jsonb_build_array(jsonb_build_object('kind','one-plane','layer','limitation',
        'detail', format('placement error is z-only (%s mm) and matches the object height (%s mm): the one-plane assumption',
          round((perr->>2)::numeric,1), pheight)));
    END IF;
  END IF;

  -- layer + confidence
  IF jsonb_array_length(problems) > 0 THEN
    layer := problems->0->>'layer';
    confidence := 'high';
    IF abort_partial THEN confidence := 'medium';
    ELSIF n_holds <= 1 AND layer IN ('camera-arm','arm-motor','reach') THEN confidence := 'medium';
    END IF;
  ELSE
    layer := 'none';
    confidence := CASE WHEN measured_cnt > 0 THEN 'high' WHEN n_holds > 0 THEN 'medium' ELSE 'low' END;
  END IF;

  RETURN jsonb_build_object('layer', layer, 'problems', problems, 'confidence', confidence);
END;
$$ LANGUAGE plpgsql IMMUTABLE;

-- --------------------------------------------------------------------------- --
-- identify.update_accuracy_map: fold one oracle doc's holds into accuracy cells.
-- A cell validates at n_good >= 3 under threshold; a later bad hold un-validates.
-- Operates on learning.accuracy_cell (per arm + shared rig).
-- --------------------------------------------------------------------------- --
CREATE OR REPLACE FUNCTION accuracy_cell_update(
  p_arm_id text, p_rig_fingerprint text, p_doc jsonb, p_threshold_mm float8 DEFAULT 10.0
) RETURNS void AS $$
DECLARE
  h jsonb;
  tip jsonb;
  err float8;
  ck text; ri int; hi int; sector int;
  radius float8; z float8; ang float8;
  RADIUS_EDGES constant float8[] := ARRAY[0.10,0.20,0.30,0.40,0.50,0.60];
  HEIGHT_EDGES constant float8[] := ARRAY[0.00,0.10,0.20,0.30,0.40];
  SECTORS constant int := 8;
  MIN_N constant int := 3;
BEGIN
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
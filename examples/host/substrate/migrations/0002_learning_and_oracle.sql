-- 0002_learning_and_oracle.sql — learning, oracle, monitor, planning,
-- calibration, and single-round-trip RPC functions.
-- Pure/immutable where possible; guards all JSONB 'null' scalars and validates
-- required config before unpacking.

-- Flatten a 1D or 2D jsonb array of numbers (e.g. K 3x3, T 4x4, dist) to float8[].
DROP FUNCTION IF EXISTS jsonb_numbers(jsonb);
CREATE OR REPLACE FUNCTION jsonb_numbers(p_val jsonb) RETURNS float8[] AS $$
  WITH RECURSIVE flat(v, ord_path) AS (
    SELECT e.v, ARRAY[e.ord::int]
      FROM jsonb_array_elements(COALESCE(p_val, '[]'::jsonb)) WITH ORDINALITY e(v, ord)
     WHERE jsonb_typeof(p_val) = 'array'
    UNION ALL
    SELECT e.v, flat.ord_path || e.ord::int
      FROM flat, LATERAL jsonb_array_elements(flat.v) WITH ORDINALITY e(v, ord)
     WHERE jsonb_typeof(flat.v) = 'array'
  )
  SELECT COALESCE(array_agg((v #>> '{}')::float8 ORDER BY ord_path), ARRAY[]::float8[])
    FROM flat
   WHERE jsonb_typeof(v) = 'number';
$$ LANGUAGE sql IMMUTABLE;

-- ---------------------------------------------------------------------------
-- trace_redact: drop secret keys recursively; reduce llm_url to its host.
-- Properly strips credentials (user:pass@host) and preserves IPv6 hosts.
-- ---------------------------------------------------------------------------
CREATE OR REPLACE FUNCTION trace_redact(value jsonb) RETURNS jsonb AS $$
DECLARE
  result jsonb;
  k text;
  v jsonb;
  raw_url text;
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
          raw_url := coalesce(v #>> '{}', '');
          -- Strip scheme and optional userinfo@ before extracting IPv6 or hostname.
          raw_url := regexp_replace(raw_url, '^[a-zA-Z][a-zA-Z0-9+.-]*://', '');
          raw_url := regexp_replace(raw_url, '^[^/?#]*@', '');
          IF left(raw_url, 1) = '[' THEN
            host := substring(raw_url FROM '^\[([^\]]+)\]');
          ELSE
            host := substring(raw_url FROM '^([^/:?#]+)');
          END IF;
          result := result || jsonb_build_object(k, coalesce(host, ''));
        ELSE
          result := result || jsonb_build_object(k, trace_redact(v));
        END IF;
      END LOOP;
      RETURN result;
    WHEN 'array' THEN
      RETURN COALESCE(
        (SELECT jsonb_agg(trace_redact(elem) ORDER BY ord)
           FROM jsonb_array_elements(value) WITH ORDINALITY t(elem, ord)),
        '[]'::jsonb);
    ELSE
      RETURN value;
  END CASE;
END $$ LANGUAGE plpgsql IMMUTABLE;

-- ---------------------------------------------------------------------------
-- Oracle pure helpers: hold_windows, match_hold_targets, downsample
-- ---------------------------------------------------------------------------
CREATE OR REPLACE FUNCTION _same_q6(qa jsonb, qb jsonb) RETURNS boolean AS $$
DECLARE
  j int;
  va float8;
  vb float8;
BEGIN
  IF qa IS NULL OR qb IS NULL
     OR jsonb_typeof(qa) <> 'array' OR jsonb_typeof(qb) <> 'array' THEN
    RETURN false;
  END IF;
  IF jsonb_array_length(qa) <> jsonb_array_length(qb) OR jsonb_array_length(qa) < 6 THEN
    RETURN false;
  END IF;
  FOR j IN 0..5 LOOP
    va := (qa->>j)::float8;
    vb := (qb->>j)::float8;
    IF va IS NULL OR vb IS NULL OR abs(va - vb) > 1e-9 THEN
      RETURN false;
    END IF;
  END LOOP;
  RETURN true;
END $$ LANGUAGE plpgsql IMMUTABLE;

CREATE OR REPLACE FUNCTION hold_windows(p_waypoints jsonb, p_min_s float8 DEFAULT 0.8)
RETURNS jsonb AS $$
DECLARE
  n int;
  windows jsonb := '[]'::jsonb;
  start_idx int := 0;
  i int;
  t0 float8;
  t1 float8;
BEGIN
  IF p_waypoints IS NULL OR jsonb_typeof(p_waypoints) <> 'array' THEN
    RETURN '[]'::jsonb;
  END IF;
  n := jsonb_array_length(p_waypoints);
  IF n < 2 THEN RETURN '[]'::jsonb; END IF;
  FOR i IN 1..n LOOP
    IF i < n AND _same_q6(p_waypoints->i->'q', p_waypoints->(i-1)->'q') THEN
      CONTINUE;
    END IF;
    IF i - start_idx >= 2 THEN
      t0 := (p_waypoints->start_idx->>'t')::float8;
      t1 := (p_waypoints->(i-1)->>'t')::float8;
      IF t0 IS NOT NULL AND t1 IS NOT NULL AND t1 - t0 >= p_min_s THEN
        windows := windows || jsonb_build_object(
          'index', jsonb_array_length(windows) + 1,
          'start', start_idx, 'end', i,
          't_start', t0, 't_end', t1,
          'q_cmd', p_waypoints->start_idx->'q');
      END IF;
    END IF;
    start_idx := i;
  END LOOP;
  RETURN windows;
END $$ LANGUAGE plpgsql IMMUTABLE;

CREATE OR REPLACE FUNCTION match_hold_targets(
  p_windows jsonb, p_targets jsonb, p_tol float8 DEFAULT 1e-6
) RETURNS jsonb AS $$
DECLARE
  matched jsonb := '[]'::jsonb;
  ws jsonb := CASE WHEN jsonb_typeof(p_windows) = 'array' THEN p_windows ELSE '[]'::jsonb END;
  ts jsonb := CASE WHEN jsonb_typeof(p_targets) = 'array' THEN p_targets ELSE '[]'::jsonb END;
  w jsonb; t jsonb;
  t0 float8; t1 float8;
  best jsonb;
BEGIN
  FOR w IN SELECT * FROM jsonb_array_elements(ws) LOOP
    t0 := (w->>'t_start')::float8;
    t1 := (w->>'t_end')::float8;
    best := NULL;
    IF t0 IS NOT NULL AND t1 IS NOT NULL THEN
      FOR t IN SELECT * FROM jsonb_array_elements(ts) LOOP
        IF jsonb_typeof(t) = 'object'
           AND t->>'hold_t_start' IS NOT NULL AND t->>'hold_t_end' IS NOT NULL
           AND (t->>'hold_t_start')::float8 - p_tol <= t0
           AND t1 <= (t->>'hold_t_end')::float8 + p_tol THEN
          best := t; EXIT;
        END IF;
      END LOOP;
    END IF;
    matched := matched || jsonb_build_array(best);
  END LOOP;
  RETURN matched;
END $$ LANGUAGE plpgsql IMMUTABLE;

CREATE OR REPLACE FUNCTION downsample(p_samples jsonb, p_limit int DEFAULT 2000)
RETURNS jsonb AS $$
DECLARE
  cnt int;
  keep boolean[];
  i int; j int; start_idx int := 0;
  kept int := 0; budget int; step int;
  free int[];
BEGIN
  IF p_samples IS NULL OR jsonb_typeof(p_samples) <> 'array' THEN
    RETURN '[]'::jsonb;
  END IF;
  cnt := jsonb_array_length(p_samples);
  IF cnt <= p_limit OR cnt = 0 THEN RETURN p_samples; END IF;
  keep := array_fill(false, ARRAY[cnt]);
  FOR i IN 1..cnt LOOP
    IF i < cnt AND _same_q6(p_samples->i->'cmd', p_samples->(i-1)->'cmd') THEN
      CONTINUE;
    END IF;
    IF i - start_idx >= 2 THEN
      FOR j IN start_idx..(i-1) LOOP keep[j+1] := true; END LOOP;
    END IF;
    start_idx := i;
  END LOOP;
  keep[1] := true;
  keep[cnt] := true;
  FOR i IN 1..cnt LOOP
    IF keep[i] THEN kept := kept + 1; END IF;
  END LOOP;
  IF kept < p_limit THEN
    free := ARRAY(SELECT idx - 1 FROM generate_series(1, cnt) idx WHERE NOT keep[idx]);
    budget := p_limit - kept;
    IF array_length(free, 1) IS NOT NULL THEN
      FOR step IN 0..(budget - 1) LOOP
        keep[free[1 + (step * array_length(free, 1) / budget)] + 1] := true;
      END LOOP;
    END IF;
  END IF;
  RETURN COALESCE(
    (SELECT jsonb_agg(e.v ORDER BY e.ord)
       FROM jsonb_array_elements(p_samples) WITH ORDINALITY e(v, ord)
      WHERE keep[e.ord::int]),
    '[]'::jsonb);
END $$ LANGUAGE plpgsql IMMUTABLE;

-- ---------------------------------------------------------------------------
-- oracle_attribute(doc, config): residual shape -> {layer, problems[], confidence}
-- Validates config keys & non-null numeric values BEFORE unpacking.
-- ---------------------------------------------------------------------------
DROP FUNCTION IF EXISTS oracle_attribute(jsonb);
CREATE OR REPLACE FUNCTION oracle_attribute(doc jsonb, config jsonb) RETURNS jsonb AS $$
DECLARE
  holds jsonb;
  telemetry jsonb;
  witness jsonb;
  problems jsonb := '[]'::jsonb;
  h jsonb;
  pixel_bad jsonb := '[]'::jsonb;
  biased jsonb := '[]'::jsonb;
  tip_bad jsonb := '[]'::jsonb;
  ik_bad jsonb := '[]'::jsonb;
  n_holds int;
  PIXEL_ERROR_MM float8;
  TIP_ERROR_MM float8;
  HOLD_ERROR_RAD float8;
  WITNESS_DRIFT_PX float8;
  WITNESS_SCALE_TOL float8;
  SAG_CORRELATION float8;
  TIP_VECTOR_COHERENCE float8;
  ZERO_BIAS_SPREAD_RAD float8;
  REACH_FRACTION float8;
  PLACEMENT_XY_MM float8;
  PLACEMENT_Z_TOL_MM float8;
  IK_RESIDUAL_MM float8;
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
  IF config IS NULL OR jsonb_typeof(config) <> 'object'
     OR NOT (config ?& array[
       'pixel_error_mm','tip_error_mm','hold_error_rad','witness_drift_px',
       'witness_scale_tol','sag_correlation','tip_vector_coherence',
       'zero_bias_spread_rad','reach_fraction','placement_xy_mm',
       'placement_z_tol_mm','ik_residual_mm'
     ]) THEN
    RAISE EXCEPTION 'oracle config missing required keys';
  END IF;
  PIXEL_ERROR_MM       := (config->>'pixel_error_mm')::float8;
  TIP_ERROR_MM         := (config->>'tip_error_mm')::float8;
  HOLD_ERROR_RAD       := (config->>'hold_error_rad')::float8;
  WITNESS_DRIFT_PX     := (config->>'witness_drift_px')::float8;
  WITNESS_SCALE_TOL    := (config->>'witness_scale_tol')::float8;
  SAG_CORRELATION      := (config->>'sag_correlation')::float8;
  TIP_VECTOR_COHERENCE := (config->>'tip_vector_coherence')::float8;
  ZERO_BIAS_SPREAD_RAD := (config->>'zero_bias_spread_rad')::float8;
  REACH_FRACTION       := (config->>'reach_fraction')::float8;
  PLACEMENT_XY_MM      := (config->>'placement_xy_mm')::float8;
  PLACEMENT_Z_TOL_MM   := (config->>'placement_z_tol_mm')::float8;
  IK_RESIDUAL_MM       := (config->>'ik_residual_mm')::float8;
  IF PIXEL_ERROR_MM IS NULL OR TIP_ERROR_MM IS NULL OR HOLD_ERROR_RAD IS NULL
     OR WITNESS_DRIFT_PX IS NULL OR WITNESS_SCALE_TOL IS NULL OR SAG_CORRELATION IS NULL
     OR TIP_VECTOR_COHERENCE IS NULL OR ZERO_BIAS_SPREAD_RAD IS NULL OR REACH_FRACTION IS NULL
     OR PLACEMENT_XY_MM IS NULL OR PLACEMENT_Z_TOL_MM IS NULL OR IK_RESIDUAL_MM IS NULL THEN
    RAISE EXCEPTION 'oracle config keys must be non-null numbers';
  END IF;

  holds     := CASE WHEN jsonb_typeof(doc->'holds') = 'array' THEN doc->'holds' ELSE '[]'::jsonb END;
  telemetry := CASE WHEN jsonb_typeof(doc->'telemetry') = 'object' THEN doc->'telemetry' ELSE '{}'::jsonb END;
  witness   := CASE WHEN jsonb_typeof(doc->'witness') = 'object' THEN doc->'witness' ELSE '{}'::jsonb END;
  n_holds   := jsonb_array_length(holds);

  -- execution faults first
  IF telemetry ? 'abort' AND jsonb_typeof(telemetry->'abort') = 'object' THEN
    problems := problems || jsonb_build_array(jsonb_build_object(
      'kind','follow-abort','layer','execution',
      'detail', format('playback aborted at t=%s: %s', telemetry->'abort'->>'t', telemetry->'abort'->>'reason')));
    abort_partial := true;
  END IF;
  IF telemetry ? 'gripper_stall' AND jsonb_typeof(telemetry->'gripper_stall') = 'object' THEN
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
    IF jsonb_typeof(h) <> 'object' THEN CONTINUE; END IF;
    IF (h->>'pixel_error_mm')::float8 > PIXEL_ERROR_MM THEN pixel_bad := pixel_bad || jsonb_build_array(h); END IF;
    max_abs := 0;
    IF h ? 'hold_error_rad' AND jsonb_typeof(h->'hold_error_rad') = 'array' THEN
      SELECT max(abs(x::float8)) INTO max_abs FROM jsonb_array_elements_text(h->'hold_error_rad') x WHERE x IS NOT NULL;
    END IF;
    IF coalesce(max_abs, 0) > HOLD_ERROR_RAD THEN biased := biased || jsonb_build_array(h); END IF;
    IF (h->>'tip_error_mm')::float8 > TIP_ERROR_MM THEN tip_bad := tip_bad || jsonb_build_array(h); END IF;
    IF (h->>'ik_residual_mm')::float8 > IK_RESIDUAL_MM THEN ik_bad := ik_bad || jsonb_build_array(h); END IF;
    IF h ? 'pixel_error_mm' AND jsonb_typeof(h->'pixel_error_mm') <> 'null' THEN measured_cnt := measured_cnt + 1; END IF;
  END LOOP;

  -- pixel layer: coherence of error directions
  IF jsonb_array_length(pixel_bad) > 0 THEN
    FOR h IN SELECT jsonb_array_elements(pixel_bad) LOOP
      IF jsonb_typeof(h->'detected_tip_px') = 'array' AND jsonb_typeof(h->'predicted_tip_px') = 'array' THEN
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
          jsonb_array_length(pixel_bad), PIXEL_ERROR_MM, round(coherence::numeric, 2))));
    END IF;
    IF (doc->'stream_turn'->>'source') IS DISTINCT FROM 'measured'
       AND ((doc->'stream_turn'->>'source') = 'shape' OR coalesce((doc->>'landscape')::boolean, false)) THEN
      problems := problems || jsonb_build_array(jsonb_build_object(
        'kind','turns-correction','layer','camera-arm',
        'detail', format('pixel error with stream_turn source %s%s',
          quote_literal(coalesce(doc->'stream_turn'->>'source','null')),
          CASE WHEN coalesce((doc->>'landscape')::boolean, false) THEN ' in landscape' ELSE '' END)));
    END IF;
  END IF;

  -- per-joint hold error: sag vs zeros vs generic
  IF jsonb_array_length(biased) > 0 THEN
    FOR h IN SELECT jsonb_array_elements(biased) LOOP
      err := h->'hold_error_rad'; sag := h->'sag_rad';
      IF jsonb_typeof(err) = 'array' AND jsonb_typeof(sag) = 'array'
         AND jsonb_array_length(err) >= 6 AND jsonb_array_length(sag) >= 6 THEN
        ne := 0; ns := 0; dot := 0;
        FOR j IN 0..5 LOOP
          ne := ne + power((err->>j)::float8, 2);
          ns := ns + power((sag->>j)::float8, 2);
          dot := dot + (err->>j)::float8 * (sag->>j)::float8;
        END LOOP;
        ne := sqrt(ne); ns := sqrt(ns);
        IF ne >= 1e-9 AND ns >= 1e-9 AND abs(dot/(ne*ns)) >= SAG_CORRELATION THEN
          has_sag := true;
        END IF;
      END IF;
    END LOOP;
    IF jsonb_array_length(biased) >= 2 THEN
      width := 6; max_spread := 0; max_mean := 0; constant_bias := true;
      FOR j IN 0..width-1 LOOP
        SELECT avg((e->'hold_error_rad'->>j)::float8),
               max((e->'hold_error_rad'->>j)::float8) - min((e->'hold_error_rad'->>j)::float8)
          INTO mean_j, spread_j FROM jsonb_array_elements(biased) e;
        max_mean := greatest(max_mean, abs(coalesce(mean_j, 0)));
        max_spread := greatest(max_spread, coalesce(spread_j, 0));
      END LOOP;
      constant_bias := (max_mean > HOLD_ERROR_RAD AND max_spread <= ZERO_BIAS_SPREAD_RAD);
    END IF;
    IF has_sag THEN
      problems := problems || jsonb_build_array(jsonb_build_object(
        'kind','sag','layer','arm-motor',
        'detail','hold error tracks tau_g/kp: sag model / gains'));
    ELSIF constant_bias THEN
      problems := problems || jsonb_build_array(jsonb_build_object(
        'kind','joint-zeros','layer','arm-motor',
        'detail','constant per-joint bias across poses: joint zeros'));
    ELSE
      problems := problems || jsonb_build_array(jsonb_build_object(
        'kind','hold-error','layer','arm-motor',
        'detail', format('%s hold(s) above %s mrad, no load or bias signature',
          jsonb_array_length(biased), 1000.0 * HOLD_ERROR_RAD)));
    END IF;
  END IF;

  -- tip error near reach boundary
  IF jsonb_array_length(tip_bad) > 0 THEN
    radius := (doc->>'max_radius_m')::float8;
    reach := (doc->>'max_reach_m')::float8;
    IF radius IS NOT NULL AND reach IS NOT NULL AND reach > 0 AND radius >= REACH_FRACTION * reach
       AND coalesce((doc->>'ik_refusals')::int, 0) > 0 THEN
      problems := problems || jsonb_build_array(jsonb_build_object(
        'kind','reach-model','layer','reach',
        'detail','tip error grows near the reach boundary with IK refusals in the tape: reach model / --max-reach'));
    ELSE
      problems := problems || jsonb_build_array(jsonb_build_object(
        'kind','tip-error','layer','arm-motor',
        'detail', format('%s hold(s) with tip error > %s mm', jsonb_array_length(tip_bad), TIP_ERROR_MM)));
    END IF;
  END IF;

  -- IK residual bridge
  IF jsonb_array_length(ik_bad) > 0 AND NOT EXISTS (
       SELECT 1 FROM jsonb_array_elements(ik_bad) p
       WHERE (p->>'tip_error_mm')::float8 IS NULL OR (p->>'tip_error_mm')::float8 > TIP_ERROR_MM) THEN
    problems := problems || jsonb_build_array(jsonb_build_object(
      'kind','ik-residual','layer','ik',
      'detail', format('%s hold(s) where the IK solution misses its target by > %s mm while the arm held within %s mm: solver / model',
        jsonb_array_length(ik_bad), IK_RESIDUAL_MM, TIP_ERROR_MM)));
  END IF;

  -- one-plane limitation
  placement := doc->'placement';
  IF placement IS NOT NULL AND jsonb_typeof(placement) = 'object' THEN
    perr := placement->'error_mm';
    pheight := (placement->>'object_height_mm')::float8;
    IF jsonb_typeof(perr) = 'array' AND jsonb_array_length(perr) = 3 AND pheight IS NOT NULL
       AND abs((perr->>0)::float8) <= PLACEMENT_XY_MM
       AND abs((perr->>1)::float8) <= PLACEMENT_XY_MM
       AND abs((perr->>2)::float8 - pheight) <= PLACEMENT_Z_TOL_MM THEN
      problems := problems || jsonb_build_array(jsonb_build_object(
        'kind','one-plane','layer','limitation',
        'detail', format('placement error is z-only (%s mm) and matches the object height (%s mm): the one-plane assumption',
          round((perr->>2)::numeric, 1), pheight)));
    END IF;
  END IF;

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
END $$ LANGUAGE plpgsql IMMUTABLE;

-- ---------------------------------------------------------------------------
-- oracle_compute: per-hold FK + sag + pixel residuals + attribution.
-- ---------------------------------------------------------------------------
DROP FUNCTION IF EXISTS oracle_compute(text, jsonb, jsonb, jsonb, jsonb, float8[], float8[], float8[], float8[], text, jsonb);
CREATE OR REPLACE FUNCTION oracle_compute(
  p_urdf text, p_waypoints jsonb, p_samples jsonb, p_targets jsonb,
  p_detections jsonb, p_k float8[], p_dist float8[], p_t_base_cam float8[],
  p_tip_offset float8[], p_kp_spec text, p_doc_extra jsonb DEFAULT '{}'::jsonb,
  p_arm_id text DEFAULT 'b601'
) RETURNS jsonb AS $$
DECLARE
  windows jsonb := hold_windows(p_waypoints);
  matched jsonb := match_hold_targets(windows, p_targets);
  samples_arr jsonb := CASE WHEN jsonb_typeof(p_samples) = 'array' THEN p_samples ELSE '[]'::jsonb END;
  det_obj jsonb := CASE WHEN jsonb_typeof(p_detections) = 'object' THEN p_detections ELSE '{}'::jsonb END;
  holds jsonb := '[]'::jsonb;
  cfg jsonb;
  w jsonb; target jsonb; det jsonb;
  idx int := 0; j int;
  t0 float8; t1 float8;
  n_inside int;
  q_cmd float8[]; q_meas float8[];
  herr float8[]; hmax float8;
  sag float8[];
  tip_cmd float8[]; tip_meas float8[]; tip_ik float8[];
  tip_err float8; ik_res float8; placement float8;
  tip_point float8[]; predicted float8[]; detected jsonb;
  px_err float8; mm_per_px float8; focal float8; dist_cam float8;
  cam_origin float8[];
  hold jsonb;
  max_radius float8 := 0.0;
  doc jsonb;
BEGIN
  SELECT config_doc INTO cfg FROM world.arm_config WHERE arm_id = p_arm_id;
  IF cfg IS NULL THEN
    RAISE EXCEPTION 'no arm config seeded (world.arm_config) for arm %', p_arm_id;
  END IF;
  cam_origin := ARRAY[p_t_base_cam[4], p_t_base_cam[8], p_t_base_cam[12]];
  focal := (p_k[1] + p_k[5]) / 2.0;

  FOR w IN SELECT * FROM jsonb_array_elements(windows) LOOP
    idx := idx + 1;
    t0 := (w->>'t_start')::float8;
    t1 := (w->>'t_end')::float8;
    SELECT count(*)::int,
           ARRAY[avg((s->'cmd'->>0)::float8), avg((s->'cmd'->>1)::float8), avg((s->'cmd'->>2)::float8),
                 avg((s->'cmd'->>3)::float8), avg((s->'cmd'->>4)::float8), avg((s->'cmd'->>5)::float8)],
           ARRAY[avg((s->'meas'->>0)::float8), avg((s->'meas'->>1)::float8), avg((s->'meas'->>2)::float8),
                 avg((s->'meas'->>3)::float8), avg((s->'meas'->>4)::float8), avg((s->'meas'->>5)::float8)]
      INTO n_inside, q_cmd, q_meas
      FROM jsonb_array_elements(samples_arr) s
     WHERE (s->>'t')::float8 BETWEEN t0 - 1e-6 AND t1 + 1e-6;

    hold := jsonb_build_object('index', (w->>'index')::int, 't_start', t0, 't_end', t1);
    IF n_inside > 0 AND q_cmd[1] IS NOT NULL AND q_meas[1] IS NOT NULL THEN
      herr := ARRAY[0,0,0,0,0,0]; hmax := 0;
      FOR j IN 1..6 LOOP
        herr[j] := q_meas[j] - q_cmd[j];
        hmax := greatest(hmax, abs(herr[j]));
      END LOOP;
      sag := rebot_sag_rad(p_urdf, q_cmd, p_kp_spec, cfg);
      tip_cmd := rebot_fk_tip(p_urdf, q_cmd);
      tip_meas := rebot_fk_tip(p_urdf, q_meas);
      tip_err := 1000.0 * sqrt((tip_meas[1]-tip_cmd[1])^2 + (tip_meas[2]-tip_cmd[2])^2 + (tip_meas[3]-tip_cmd[3])^2);
      hold := hold || jsonb_build_object(
        'q_cmd', to_jsonb(q_cmd), 'q_meas', to_jsonb(q_meas),
        'hold_error_rad', to_jsonb(herr), 'hold_error_max_rad', hmax,
        'sag_rad', to_jsonb(sag), 'tip_fk_cmd', to_jsonb(tip_cmd),
        'tip_fk_meas', to_jsonb(tip_meas), 'tip_error_mm', round(tip_err::numeric, 3));

      target := matched->(idx - 1);
      IF target IS NOT NULL AND jsonb_typeof(target) = 'object' THEN
        hold := hold || jsonb_build_object(
          'target_point', target->'point', 'flange_point', target->'flange_point');
        IF jsonb_typeof(target->'q_ik') = 'array' AND jsonb_typeof(target->'flange_point') = 'array' THEN
          tip_ik := rebot_fk_tip(p_urdf, ARRAY(SELECT jsonb_array_elements_text(target->'q_ik')::float8));
          ik_res := 1000.0 * sqrt((tip_ik[1]-(target->'flange_point'->>0)::float8)^2
                                + (tip_ik[2]-(target->'flange_point'->>1)::float8)^2
                                + (tip_ik[3]-(target->'flange_point'->>2)::float8)^2);
          placement := 1000.0 * sqrt((tip_meas[1]-(target->'flange_point'->>0)::float8)^2
                                   + (tip_meas[2]-(target->'flange_point'->>1)::float8)^2
                                   + (tip_meas[3]-(target->'flange_point'->>2)::float8)^2);
          hold := hold || jsonb_build_object(
            'q_ik', target->'q_ik', 'tip_fk_ik', to_jsonb(tip_ik),
            'ik_residual_mm', round(ik_res::numeric, 3),
            'placement_mm', round(placement::numeric, 3));
        END IF;
      END IF;

      det := det_obj->((w->>'index')::text);
      IF det IS NOT NULL AND jsonb_typeof(det) = 'object' AND jsonb_typeof(det->'uv') = 'array' THEN
        tip_point := rebot_tip_point(p_urdf, q_meas, p_tip_offset);
        predicted := rebot_project_dist(p_k, p_t_base_cam, tip_point, p_dist);
        detected := det->'uv';
        hold := hold || jsonb_build_object(
          'image_size', det->'image_size', 'grid_scale', det->'grid_scale',
          'predicted_tip_px', predicted, 'detected_tip_px', detected,
          'detector', det->'detector', 'detector_score', det->'score', 'frame', det->'frame');
        IF predicted IS NOT NULL THEN
          px_err := sqrt(((detected->>0)::float8 - predicted[1])^2
                       + ((detected->>1)::float8 - predicted[2])^2);
          dist_cam := sqrt((cam_origin[1]-tip_point[1])^2 + (cam_origin[2]-tip_point[2])^2
                         + (cam_origin[3]-tip_point[3])^2);
          mm_per_px := CASE WHEN focal > 0 THEN 1000.0 * dist_cam / focal ELSE NULL END;
          hold := hold || jsonb_build_object(
            'pixel_error_px', round(px_err::numeric, 2),
            'pixel_error_mm', CASE WHEN mm_per_px IS NOT NULL THEN round((px_err * mm_per_px)::numeric, 2) ELSE NULL END);
        END IF;
      END IF;
      max_radius := greatest(max_radius, sqrt(tip_cmd[1]^2 + tip_cmd[2]^2));
    END IF;
    holds := holds || jsonb_build_array(hold);
  END LOOP;

  doc := COALESCE(CASE WHEN jsonb_typeof(p_doc_extra) = 'object' THEN p_doc_extra END, '{}'::jsonb)
         || jsonb_build_object('holds', holds, 'max_radius_m', round(max_radius::numeric, 4));
  RETURN jsonb_build_object(
    'doc', doc,
    'verdict', oracle_attribute(doc, cfg->'oracle'),
    'holds', holds,
    'max_radius_m', round(max_radius::numeric, 4));
END $$ LANGUAGE plpgsql;

-- ---------------------------------------------------------------------------
-- accuracy_cell_update: validates config BEFORE unpacking; defaults threshold
-- from world.arm_config->'oracle'->>'pixel_error_mm' when p_threshold_mm NULL.
-- ---------------------------------------------------------------------------
DROP FUNCTION IF EXISTS accuracy_cell_update(text,text,jsonb,float8);
CREATE OR REPLACE FUNCTION accuracy_cell_update(
  p_arm_id text, p_rig_fingerprint text, p_doc jsonb, config jsonb,
  p_threshold_mm float8 DEFAULT NULL
) RETURNS void AS $$
DECLARE
  holds jsonb;
  h jsonb;
  tip jsonb;
  err float8;
  ck text; ri int; hi int; sector int;
  radius float8; z float8; ang float8;
  RADIUS_EDGES float8[];
  HEIGHT_EDGES float8[];
  SECTORS int;
  MIN_N int;
  thresh float8;
BEGIN
  IF config IS NULL OR jsonb_typeof(config) <> 'object'
     OR NOT (config ?& array['accuracy_radius_edges','accuracy_height_edges','accuracy_sectors','accuracy_min_n'])
     OR jsonb_typeof(config->'accuracy_radius_edges') <> 'array'
     OR jsonb_typeof(config->'accuracy_height_edges') <> 'array' THEN
    RAISE EXCEPTION 'learning config missing required keys';
  END IF;
  RADIUS_EDGES := ARRAY(SELECT jsonb_array_elements_text(config->'accuracy_radius_edges')::float8);
  HEIGHT_EDGES := ARRAY(SELECT jsonb_array_elements_text(config->'accuracy_height_edges')::float8);
  SECTORS := (config->>'accuracy_sectors')::int;
  MIN_N   := (config->>'accuracy_min_n')::int;
  IF SECTORS IS NULL OR SECTORS < 1 OR MIN_N IS NULL OR MIN_N < 1
     OR coalesce(array_length(RADIUS_EDGES, 1), 0) < 2
     OR coalesce(array_length(HEIGHT_EDGES, 1), 0) < 2 THEN
    RAISE EXCEPTION 'learning config invalid accuracy grid parameters';
  END IF;

  thresh := p_threshold_mm;
  IF thresh IS NULL THEN
    SELECT (config_doc->'oracle'->>'pixel_error_mm')::float8 INTO thresh
      FROM world.arm_config WHERE arm_id = p_arm_id;
    thresh := COALESCE(thresh, 10.0);
  END IF;

  holds := CASE WHEN jsonb_typeof(p_doc->'holds') = 'array' THEN p_doc->'holds' ELSE '[]'::jsonb END;
  FOR h IN SELECT jsonb_array_elements(holds) LOOP
    IF jsonb_typeof(h) <> 'object' THEN CONTINUE; END IF;
    err := (h->>'pixel_error_mm')::float8;
    tip := CASE
      WHEN jsonb_typeof(h->'tip_fk_cmd') = 'array' THEN h->'tip_fk_cmd'
      WHEN jsonb_typeof(h->'tip_fk_meas') = 'array' THEN h->'tip_fk_meas'
      ELSE NULL END;
    IF err IS NULL OR tip IS NULL OR jsonb_array_length(tip) < 3 THEN CONTINUE; END IF;
    radius := sqrt(power((tip->>0)::float8, 2) + power((tip->>1)::float8, 2));
    z := (tip->>2)::float8;
    ri := 0;
    FOR i IN 1..array_length(RADIUS_EDGES, 1) - 1 LOOP
      IF radius >= RADIUS_EDGES[i] THEN ri := i - 1; END IF;
    END LOOP;
    ri := least(ri, array_length(RADIUS_EDGES, 1) - 2);
    hi := 0;
    FOR i IN 1..array_length(HEIGHT_EDGES, 1) - 1 LOOP
      IF z >= HEIGHT_EDGES[i] THEN hi := i - 1; END IF;
    END LOOP;
    hi := least(hi, array_length(HEIGHT_EDGES, 1) - 2);
    ang := atan2((tip->>1)::float8, (tip->>0)::float8) + 2 * pi();
    ang := ang - floor(ang / (2 * pi())) * (2 * pi());
    sector := floor(ang / (2 * pi() / SECTORS))::int % SECTORS;
    ck := format('r%s-h%s-s%s', ri, hi, sector);
    INSERT INTO learning.accuracy_cell(arm_id, rig_fingerprint, cell_key, radius_idx, height_idx, sector, n, n_good, measured_mm)
      VALUES (p_arm_id, p_rig_fingerprint, ck, ri, hi, sector, 0, 0, NULL)
      ON CONFLICT (arm_id, rig_fingerprint, cell_key) DO NOTHING;
    UPDATE learning.accuracy_cell SET
      n = n + 1,
      measured_mm = round(greatest(coalesce(measured_mm, 0), err)::numeric, 2),
      n_good = n_good + CASE WHEN err <= thresh THEN 1 ELSE 0 END,
      validated_at = CASE
        WHEN err > thresh THEN NULL
        WHEN n_good + 1 >= MIN_N AND validated_at IS NULL THEN now()
        ELSE validated_at END,
      updated_at = now()
    WHERE arm_id = p_arm_id AND rig_fingerprint = p_rig_fingerprint AND cell_key = ck;
  END LOOP;
END $$ LANGUAGE plpgsql;

-- ---------------------------------------------------------------------------
-- Item 4: SQL TriggerMonitor (identify.TriggerMonitor.observe) — updates
-- learning.monitor_state atomically inside oracle_learn / oracle_record so
-- every execution path (including durable /execute) feeds drift proposals
-- with zero extra round trips from capture_server.
-- ---------------------------------------------------------------------------
CREATE OR REPLACE FUNCTION monitor_observe(
  p_arm_id text, p_rig_fingerprint text, p_doc jsonb
) RETURNS jsonb AS $$
DECLARE
  cfg jsonb;
  pixel_mm float8;
  alpha float8;
  w_drift_px float8;
  w_scale_tol float8;
  tag_growth float8;
  max_age_days float8;
  st record;
  ewma float8;
  tag_base float8;
  proposals jsonb := '[]'::jsonb;
  witness jsonb;
  drift float8; scale float8;
  h jsonb; err float8;
  tag_err float8;
  rig_fitted timestamptz;
  age_days float8;
  p_prob jsonb;
BEGIN
  SELECT config_doc INTO cfg FROM world.arm_config WHERE arm_id = p_arm_id;
  pixel_mm     := COALESCE((cfg->'oracle'->>'pixel_error_mm')::float8, 10.0);
  w_drift_px   := COALESCE((cfg->'oracle'->>'witness_drift_px')::float8, 2.0);
  w_scale_tol  := COALESCE((cfg->'oracle'->>'witness_scale_tol')::float8, 0.05);
  alpha        := COALESCE((cfg->'learning'->>'ewma_alpha')::float8, 0.3);
  tag_growth   := COALESCE((cfg->'learning'->>'tag_growth_px')::float8, 2.0);
  max_age_days := COALESCE((cfg->'learning'->>'rig_max_age_days')::float8, 30.0);

  SELECT * INTO st FROM learning.monitor_state WHERE arm_id = p_arm_id FOR UPDATE;
  ewma := CASE WHEN FOUND THEN st.ewma_pixel_error_mm ELSE NULL END;
  tag_base := CASE WHEN FOUND THEN st.ewma_tag_rms_px ELSE NULL END;

  witness := CASE WHEN jsonb_typeof(p_doc->'witness') = 'object' THEN p_doc->'witness' ELSE '{}'::jsonb END;
  drift := (witness->>'drift_px')::float8;
  scale := (witness->>'scale')::float8;
  IF (drift IS NOT NULL AND drift > w_drift_px)
     OR (scale IS NOT NULL AND abs(scale - 1.0) > w_scale_tol) THEN
    proposals := proposals || jsonb_build_array(jsonb_build_object(
      'kind', 'stale-rig',
      'reason', format('witness drift %s px, scale %s: the camera moved', drift, scale)));
  END IF;

  IF jsonb_typeof(p_doc->'holds') = 'array' THEN
    FOR h IN SELECT jsonb_array_elements(p_doc->'holds') LOOP
      err := (h->>'pixel_error_mm')::float8;
      IF err IS NOT NULL THEN
        ewma := CASE WHEN ewma IS NULL THEN err ELSE alpha * err + (1.0 - alpha) * ewma END;
      END IF;
    END LOOP;
  END IF;
  IF ewma IS NOT NULL AND ewma > pixel_mm THEN
    proposals := proposals || jsonb_build_array(jsonb_build_object(
      'kind', 'pixel-drift',
      'reason', format('oracle pixel error EWMA %s mm over %s mm', round(ewma::numeric, 1), round(pixel_mm::numeric, 1))));
  END IF;

  tag_err := (p_doc->>'tag_reprojection_px')::float8;
  IF tag_err IS NOT NULL THEN
    IF tag_base IS NULL THEN
      tag_base := tag_err;
    ELSIF tag_err > tag_base + tag_growth THEN
      proposals := proposals || jsonb_build_array(jsonb_build_object(
        'kind', 'tag-growth',
        'reason', format('tag reprojection grew %s -> %s px', round(tag_base::numeric, 1), round(tag_err::numeric, 1))));
    END IF;
  END IF;

  IF p_rig_fingerprint IS NULL OR p_rig_fingerprint = '' THEN
    proposals := proposals || jsonb_build_array(jsonb_build_object(
      'kind', 'missing-rig', 'reason', 'no fitted rig: fixed mode has no geometry'));
  ELSE
    SELECT fitted INTO rig_fitted FROM world.shared_rig WHERE rig_fingerprint = p_rig_fingerprint;
    IF rig_fitted IS NOT NULL THEN
      age_days := extract(epoch FROM (now() - rig_fitted)) / 86400.0;
      IF age_days > max_age_days THEN
        proposals := proposals || jsonb_build_array(jsonb_build_object(
          'kind', 'stale-rig', 'reason', format('the rig is %s days old', round(age_days::numeric, 0))));
      END IF;
    END IF;
  END IF;

  IF jsonb_typeof(p_doc->'verdict'->'problems') = 'array' THEN
    FOR p_prob IN SELECT jsonb_array_elements(p_doc->'verdict'->'problems') LOOP
      IF p_prob->>'kind' = 'joint-zeros' THEN
        proposals := proposals || jsonb_build_array(jsonb_build_object(
          'kind', 'zero-drift',
          'reason', COALESCE(p_prob->>'detail', 'constant per-joint bias across poses'),
          'action', 're-zero the biased joint(s) on the edge before refitting the rig'));
        EXIT;
      END IF;
    END LOOP;
  END IF;

  INSERT INTO learning.monitor_state(arm_id, ewma_pixel_error_mm, ewma_tag_rms_px, last_doc, updated_at)
  VALUES (p_arm_id, ewma, tag_base, jsonb_build_object('proposals', proposals), now())
  ON CONFLICT (arm_id) DO UPDATE SET
    ewma_pixel_error_mm = EXCLUDED.ewma_pixel_error_mm,
    ewma_tag_rms_px     = EXCLUDED.ewma_tag_rms_px,
    last_doc            = EXCLUDED.last_doc,
    updated_at          = now();

  RETURN jsonb_build_object('ewma_pixel_error_mm', ewma, 'ewma_tag_rms_px', tag_base, 'proposals', proposals);
END $$ LANGUAGE plpgsql;

-- ---------------------------------------------------------------------------
-- Item 3: sag_fit_and_save — aggregate holds directly from PG telemetry blobs,
-- run Rust rebot_fit_sag_map, and persist learning.sag_map in 1 SQL call.
-- ---------------------------------------------------------------------------
CREATE OR REPLACE FUNCTION sag_fit_and_save(
  p_arm_id text, p_rig_fingerprint text, p_kp_spec text DEFAULT '50,2:150,3:150',
  p_zeros_key text DEFAULT 'none', p_min_holds int DEFAULT NULL
) RETURNS jsonb AS $$
DECLARE
  urdf_text text;
  cfg jsonb;
  holds jsonb := '[]'::jsonb;
  rec record;
  tel jsonb;
  samples jsonb;
  wps jsonb;
  w jsonb;
  kp_str text;
  t0 float8; t1 float8;
  n_in int;
  q_cmd float8[]; q_meas float8[];
  sag_model float8[];
  tau float8[];
  tip float8[];
  res_r float8[];
  j int;
  fit jsonb;
  smap jsonb;
BEGIN
  SELECT urdf INTO urdf_text FROM world.robot_model ORDER BY created_at LIMIT 1;
  IF urdf_text IS NULL THEN
    RAISE EXCEPTION 'no robot model seeded (world.robot_model)';
  END IF;
  SELECT config_doc INTO cfg FROM world.arm_config WHERE arm_id = p_arm_id;
  IF cfg IS NULL THEN
    RAISE EXCEPTION 'no arm config seeded (world.arm_config) for arm %', p_arm_id;
  END IF;

  FOR rec IN
    SELECT b.bytes, t.kp_key
      FROM evidence.telemetry t
      JOIN evidence.blob b ON b.sha256 = t.blob_sha
     WHERE t.arm_id = p_arm_id
     ORDER BY t.created_at
  LOOP
    BEGIN
      tel := convert_from(rec.bytes, 'UTF8')::jsonb;
    EXCEPTION WHEN OTHERS THEN
      CONTINUE;
    END;
    samples := CASE WHEN jsonb_typeof(tel->'samples') = 'array' THEN tel->'samples' ELSE '[]'::jsonb END;
    IF jsonb_array_length(samples) < 2 THEN CONTINUE; END IF;
    kp_str := p_kp_spec;
    IF jsonb_typeof(tel->'kp') = 'object' THEN
      SELECT string_agg(k || ':' || v, ',' ORDER BY k::int) INTO kp_str
        FROM jsonb_each_text(tel->'kp') e(k, v)
       WHERE k ~ '^[1-6]$';
      kp_str := COALESCE(kp_str, p_kp_spec);
    END IF;
    SELECT COALESCE(jsonb_agg(jsonb_build_object('t', s->'t', 'q', s->'cmd') ORDER BY ord), '[]'::jsonb)
      INTO wps
      FROM jsonb_array_elements(samples) WITH ORDINALITY t(s, ord);
    FOR w IN SELECT * FROM jsonb_array_elements(hold_windows(wps)) LOOP
      t0 := (w->>'t_start')::float8;
      t1 := (w->>'t_end')::float8;
      SELECT count(*)::int,
             ARRAY[avg((s->'cmd'->>0)::float8), avg((s->'cmd'->>1)::float8), avg((s->'cmd'->>2)::float8),
                   avg((s->'cmd'->>3)::float8), avg((s->'cmd'->>4)::float8), avg((s->'cmd'->>5)::float8)],
             ARRAY[avg((s->'meas'->>0)::float8), avg((s->'meas'->>1)::float8), avg((s->'meas'->>2)::float8),
                   avg((s->'meas'->>3)::float8), avg((s->'meas'->>4)::float8), avg((s->'meas'->>5)::float8)]
        INTO n_in, q_cmd, q_meas
        FROM jsonb_array_elements(samples) s
       WHERE (s->>'t')::float8 BETWEEN t0 - 1e-6 AND t1 + 1e-6;
      IF n_in > 0 AND q_cmd[1] IS NOT NULL AND q_meas[1] IS NOT NULL THEN
        sag_model := rebot_sag_rad(urdf_text, q_cmd, kp_str, cfg);
        tau := rebot_gravity_torques(urdf_text, q_cmd, cfg);
        tip := rebot_fk_tip(urdf_text, q_cmd);
        res_r := ARRAY[0,0,0,0,0,0];
        FOR j IN 1..6 LOOP
          res_r[j] := (q_cmd[j] - q_meas[j]) - sag_model[j];
        END LOOP;
        holds := holds || jsonb_build_array(jsonb_build_object(
          'q', to_jsonb(q_cmd), 'tau', to_jsonb(tau), 'r', to_jsonb(res_r), 'tip', to_jsonb(tip)));
      END IF;
    END LOOP;
  END LOOP;

  fit := rebot_fit_sag_map(urdf_text, holds, p_kp_spec, p_rig_fingerprint, p_zeros_key,
                           cfg->'learning', p_min_holds);
  IF COALESCE((fit->>'ok')::boolean, false) AND jsonb_typeof(fit->'map') = 'object' THEN
    smap := fit->'map';
    INSERT INTO world.arm(arm_id, name, urdf_fingerprint, zeros_key, kp_key)
      VALUES (p_arm_id, p_arm_id, 'unknown', 'none', 'default') ON CONFLICT DO NOTHING;
    INSERT INTO learning.sag_map(arm_id, rig_fingerprint, kp_key, zeros_key,
                                 per_motor, samples, rms_mrad, validated_mm, envelope, status)
    VALUES (p_arm_id, p_rig_fingerprint, COALESCE(smap->>'kp_key', p_kp_spec),
            COALESCE(smap->>'zeros_key', p_zeros_key),
            COALESCE(smap->'per_motor', '{}'::jsonb),
            COALESCE((smap->>'samples')::int, 0),
            (smap->>'rms_mrad')::float8, (smap->>'validated_mm')::float8,
            COALESCE(smap->'envelope', '{}'::jsonb), 'active')
    ON CONFLICT (arm_id, rig_fingerprint, kp_key, zeros_key) WHERE status = 'active'
    DO UPDATE SET
      per_motor    = EXCLUDED.per_motor,
      samples      = EXCLUDED.samples,
      rms_mrad     = EXCLUDED.rms_mrad,
      validated_mm = EXCLUDED.validated_mm,
      envelope     = EXCLUDED.envelope,
      created_at   = now();
  END IF;
  RETURN fit || jsonb_build_object('holds', jsonb_array_length(holds));
END $$ LANGUAGE plpgsql;

-- ---------------------------------------------------------------------------
-- Factored planning helpers + plan_one + move_one (Item 5)
-- ---------------------------------------------------------------------------
CREATE OR REPLACE FUNCTION _ensure_workspace_arm(p_workspace text, p_arm_id text) RETURNS void AS $$
BEGIN
  IF p_workspace IS NOT NULL THEN
    INSERT INTO world.workspace(workspace_id, name, plane_z, min_z, hover_m)
      VALUES (p_workspace, p_workspace, 0, 0.04, 0.05) ON CONFLICT (workspace_id) DO NOTHING;
  END IF;
  IF p_arm_id IS NOT NULL THEN
    INSERT INTO world.arm(arm_id, name, urdf_fingerprint, zeros_key, kp_key)
      VALUES (p_arm_id, p_arm_id, 'unknown', 'none', 'default') ON CONFLICT (arm_id) DO NOTHING;
  END IF;
END $$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION _plan_context(
  p_workspace text, p_arm_id text, p_kp_spec text DEFAULT NULL,
  OUT urdf_text text, OUT cfg jsonb, OUT rig_fp text, OUT snap jsonb
) AS $$
DECLARE
  sag jsonb;
BEGIN
  SELECT urdf INTO urdf_text FROM world.robot_model ORDER BY created_at LIMIT 1;
  IF urdf_text IS NULL THEN
    RAISE EXCEPTION 'no robot model seeded (world.robot_model)';
  END IF;
  SELECT config_doc INTO cfg FROM world.arm_config WHERE arm_id = p_arm_id;
  IF cfg IS NULL THEN
    RAISE EXCEPTION 'no arm config seeded (world.arm_config) for arm %', p_arm_id;
  END IF;
  SELECT rig_fingerprint INTO rig_fp
    FROM world.shared_rig
   WHERE workspace_id = p_workspace AND status = 'active'
   ORDER BY fitted DESC NULLS LAST LIMIT 1;

  snap := jsonb_build_object('rig_fingerprint', coalesce(rig_fp, ''), 'zeros_key', 'none', 'sag_map', null);
  IF rig_fp IS NOT NULL THEN
    SELECT jsonb_build_object('rig_fingerprint', rig_fingerprint, 'kp_key', kp_key,
                              'zeros_key', zeros_key, 'per_motor', per_motor, 'envelope', envelope)
      INTO sag
      FROM learning.sag_map
     WHERE arm_id = p_arm_id AND rig_fingerprint = rig_fp AND status = 'active'
       AND (p_kp_spec IS NULL OR kp_key = p_kp_spec)
     ORDER BY created_at DESC LIMIT 1;
    IF sag IS NULL AND p_kp_spec IS NOT NULL THEN
      SELECT jsonb_build_object('rig_fingerprint', rig_fingerprint, 'kp_key', kp_key,
                                'zeros_key', zeros_key, 'per_motor', per_motor, 'envelope', envelope)
        INTO sag
        FROM learning.sag_map
       WHERE arm_id = p_arm_id AND rig_fingerprint = rig_fp AND status = 'active'
       ORDER BY created_at DESC LIMIT 1;
    END IF;
    IF sag IS NOT NULL THEN
      snap := jsonb_build_object('rig_fingerprint', rig_fp, 'zeros_key', sag->>'zeros_key', 'sag_map', sag);
    END IF;
  END IF;
END $$ LANGUAGE plpgsql STABLE;

CREATE OR REPLACE FUNCTION _plan_persist(
  p_workspace text, p_arm_id text, p_payload jsonb, p_summary jsonb,
  p_snapshot jsonb, p_plan_id text DEFAULT NULL
) RETURNS text AS $$
DECLARE
  pid text;
  raw bytea := convert_to(p_payload::text, 'UTF8');
  sha text  := encode(digest(raw, 'sha256'), 'hex');
  tries int := 0;
  inserted int := 0;
BEGIN
  PERFORM _ensure_workspace_arm(p_workspace, p_arm_id);
  INSERT INTO evidence.blob(sha256, kind, bytes, size_bytes, status)
    VALUES (sha, 'trajectory', raw, octet_length(raw), 'verified')
    ON CONFLICT (sha256) DO UPDATE SET created_at = now();

  IF p_plan_id IS NOT NULL AND p_plan_id <> '' THEN
    pid := p_plan_id;
    INSERT INTO evidence.episode(episode_id, workspace_id, arm_ids, kind)
      VALUES (pid, p_workspace, jsonb_build_array(p_arm_id), 'execution')
      ON CONFLICT (episode_id) DO NOTHING;
    INSERT INTO evidence.plan(plan_id, episode_id, arm_id, blob_sha, metadata, snapshot, expires_at)
      VALUES (pid, pid, p_arm_id, sha, p_summary, p_snapshot, now() + interval '600 seconds')
      ON CONFLICT (plan_id) DO UPDATE SET
        blob_sha = EXCLUDED.blob_sha, metadata = EXCLUDED.metadata,
        snapshot = EXCLUDED.snapshot, expires_at = EXCLUDED.expires_at;
    RETURN pid;
  END IF;

  LOOP
    tries := tries + 1;
    pid := to_char(now() AT TIME ZONE 'UTC', 'YYYYMMDD"T"HH24MISS"Z"')
           || '-' || substr(encode(gen_random_bytes(4), 'hex'), 1, 6);
    INSERT INTO evidence.episode(episode_id, workspace_id, arm_ids, kind)
      VALUES (pid, p_workspace, jsonb_build_array(p_arm_id), 'execution')
      ON CONFLICT (episode_id) DO NOTHING;
    INSERT INTO evidence.plan(plan_id, episode_id, arm_id, blob_sha, metadata, snapshot, expires_at)
      VALUES (pid, pid, p_arm_id, sha, p_summary, p_snapshot, now() + interval '600 seconds')
      ON CONFLICT (plan_id) DO NOTHING;
    GET DIAGNOSTICS inserted = ROW_COUNT;
    EXIT WHEN inserted > 0 OR tries >= 8;
  END LOOP;
  IF inserted = 0 THEN
    RAISE EXCEPTION 'could not allocate unique plan_id after % attempts', tries;
  END IF;
  RETURN pid;
END $$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION plan_one(
  p_workspace text, p_pose jsonb, p_target jsonb, p_options jsonb,
  p_arm_id text DEFAULT 'b601'
) RETURNS jsonb AS $$
DECLARE
  ctx record;
  result jsonb;
  payload jsonb;
  summary jsonb;
  pid text;
BEGIN
  SELECT * INTO ctx FROM _plan_context(p_workspace, p_arm_id, p_options->>'kp_spec');
  IF jsonb_typeof(p_target) = 'array' THEN
    result := rebot_plan_sequence(ctx.urdf_text, p_pose, p_target, p_options, ctx.snap, ctx.cfg);
  ELSE
    result := rebot_plan(ctx.urdf_text, p_pose, p_target, p_options, ctx.snap, ctx.cfg);
  END IF;
  payload := result->'payload';
  summary := result->'summary';
  pid := _plan_persist(p_workspace, p_arm_id, payload, summary,
                       jsonb_build_object('rig', ctx.rig_fp, 'q', p_pose->'q'));
  RETURN jsonb_build_object('plan_id', pid, 'summary', summary);
END $$ LANGUAGE plpgsql;

-- Item 5: move_one accepts optional caller-chosen p_plan_id (fixes coverage C5)
-- and returns payload inline so planner_server needs 0 follow-up blob reads.
DROP FUNCTION IF EXISTS move_one(text, jsonb, jsonb, jsonb, text);
DROP FUNCTION IF EXISTS move_one(text, jsonb, jsonb, jsonb, text);
CREATE OR REPLACE FUNCTION move_one(
  p_workspace text, p_pose jsonb, p_targets jsonb, p_options jsonb,
  p_arm_id text DEFAULT 'b601', p_plan_id text DEFAULT NULL
) RETURNS jsonb AS $$
DECLARE
  ctx record;
  result jsonb;
  payload jsonb;
  summary jsonb;
  pid text;
BEGIN
  SELECT * INTO ctx FROM _plan_context(p_workspace, p_arm_id, p_options->>'kp_spec');
  result  := rebot_move_build(ctx.urdf_text, p_pose, p_targets, p_options, ctx.snap, ctx.cfg);
  payload := result->'payload';
  summary := result->'summary';
  pid := _plan_persist(p_workspace, p_arm_id, payload, summary,
                       jsonb_build_object('rig', ctx.rig_fp, 'q', p_pose->'q', 'kind', 'move'),
                       p_plan_id);
  RETURN jsonb_build_object('plan_id', pid, 'summary', summary, 'payload', payload);
END $$ LANGUAGE plpgsql;

-- ---------------------------------------------------------------------------
-- Item 5: Single-call gateway & calibration RPCs (fk_one, calibrate_fit_one,
-- plan_fetch, telemetry_record)
-- ---------------------------------------------------------------------------
CREATE OR REPLACE FUNCTION fk_one(p_q float8[], p_arm_id text DEFAULT 'b601') RETURNS jsonb AS $$
DECLARE
  urdf_text text;
  q6 float8[];
  tip float8[];
  pts float8[];
  jp jsonb := '[]'::jsonb;
  i int;
BEGIN
  SELECT urdf INTO urdf_text FROM world.robot_model ORDER BY created_at LIMIT 1;
  IF urdf_text IS NULL THEN
    RAISE EXCEPTION 'no robot model seeded (world.robot_model)';
  END IF;
  q6 := p_q[1:6];
  tip := rebot_fk_tip(urdf_text, q6);
  pts := rebot_joint_points(urdf_text, q6);
  FOR i IN 1..array_length(pts, 1) BY 3 LOOP
    jp := jp || jsonb_build_array(jsonb_build_array(pts[i], pts[i+1], pts[i+2]));
  END LOOP;
  RETURN jsonb_build_object('ok', true, 'tip_m', to_jsonb(tip), 'joint_points', jp);
END $$ LANGUAGE plpgsql STABLE;

CREATE OR REPLACE FUNCTION calibrate_fit_one(
  p_observations jsonb, p_k jsonb, p_dist jsonb, p_options jsonb,
  p_arm_id text DEFAULT 'b601'
) RETURNS jsonb AS $$
DECLARE
  urdf_text text;
  calib_cfg jsonb;
BEGIN
  SELECT urdf INTO urdf_text FROM world.robot_model ORDER BY created_at LIMIT 1;
  IF urdf_text IS NULL THEN
    RAISE EXCEPTION 'no robot model seeded (world.robot_model)';
  END IF;
  SELECT config_doc->'calibration' INTO calib_cfg
    FROM world.arm_config WHERE arm_id = p_arm_id;
  IF calib_cfg IS NULL THEN
    RAISE EXCEPTION 'no arm config calibration seeded (world.arm_config) for arm %', p_arm_id;
  END IF;
  RETURN rebot_calibrate_fit(urdf_text, p_observations, p_k, p_dist, p_options, calib_cfg);
END $$ LANGUAGE plpgsql STABLE;

CREATE OR REPLACE FUNCTION plan_fetch(p_plan_id text, p_max_age_s float8 DEFAULT NULL)
RETURNS TABLE(bytes bytea, created_at timestamptz, age_s double precision, expired boolean) AS $$
  SELECT b.bytes,
         p.created_at,
         extract(epoch FROM (now() - p.created_at))::double precision AS age_s,
         CASE WHEN p_max_age_s IS NOT NULL
              THEN extract(epoch FROM (now() - p.created_at)) > p_max_age_s
              ELSE false END AS expired
    FROM evidence.plan p
    JOIN evidence.blob b ON b.sha256 = p.blob_sha
   WHERE p.plan_id = p_plan_id;
$$ LANGUAGE sql STABLE;

CREATE OR REPLACE FUNCTION telemetry_record(
  p_exec_id text, p_plan_id text, p_arm_id text, p_bytes bytea,
  p_kp_key text, p_summary jsonb DEFAULT '{}'::jsonb
) RETURNS text AS $$
DECLARE
  sha text := encode(digest(p_bytes, 'sha256'), 'hex');
BEGIN
  INSERT INTO evidence.blob(sha256, kind, bytes, size_bytes, status)
    VALUES (sha, 'telemetry', p_bytes, octet_length(p_bytes), 'verified')
    ON CONFLICT (sha256) DO UPDATE SET created_at = now();
  INSERT INTO evidence.telemetry(exec_id, plan_id, arm_id, blob_sha, kp_key, summary)
    VALUES (p_exec_id, p_plan_id, p_arm_id, sha, COALESCE(p_kp_key, 'default'), COALESCE(p_summary, '{}'::jsonb))
    ON CONFLICT (exec_id) DO NOTHING;
  RETURN sha;
END $$ LANGUAGE plpgsql;

-- ---------------------------------------------------------------------------
-- Item 1: trace_record_batch — atomic episode + manifest + blobs + events
-- under one advisory lock in 1 SQL round trip (replaces ~70-130 calls).
-- ---------------------------------------------------------------------------
CREATE OR REPLACE FUNCTION trace_record_batch(
  p_episode_id text, p_workspace text, p_snapshot jsonb,
  p_blobs jsonb, p_events jsonb
) RETURNS int[] AS $$
DECLARE
  b jsonb;
  raw bytea;
  sha text;
  ev jsonb;
  base_seq int;
  cur_seq int;
  seqs int[] := ARRAY[]::int[];
  bsha text;
  ev_src evidence.event_src;
BEGIN
  PERFORM _ensure_workspace_arm(p_workspace, NULL);
  PERFORM pg_advisory_xact_lock(hashtext(p_episode_id));

  INSERT INTO evidence.episode(episode_id, workspace_id, arm_ids, kind)
    VALUES (p_episode_id, p_workspace, '[]'::jsonb, 'chat')
    ON CONFLICT (episode_id) DO NOTHING;

  IF p_snapshot IS NOT NULL AND jsonb_typeof(p_snapshot) = 'object' THEN
    INSERT INTO evidence.manifest(episode_id, snapshot)
      VALUES (p_episode_id, trace_redact(p_snapshot))
      ON CONFLICT (episode_id) DO NOTHING;
  END IF;

  IF p_blobs IS NOT NULL AND jsonb_typeof(p_blobs) = 'array' THEN
    FOR b IN SELECT * FROM jsonb_array_elements(p_blobs) LOOP
      raw := decode(replace(b->>'b64', E'\n', ''), 'base64');
      sha := COALESCE(NULLIF(b->>'sha256', ''), encode(digest(raw, 'sha256'), 'hex'));
      INSERT INTO evidence.blob(sha256, kind, bytes, size_bytes, status)
        VALUES (sha, 'still', raw, octet_length(raw), 'verified')
        ON CONFLICT (sha256) DO UPDATE SET created_at = now();
    END LOOP;
  END IF;

  SELECT coalesce(max(seq), 0) INTO base_seq
    FROM evidence.event WHERE episode_id = p_episode_id;
  cur_seq := base_seq;

  IF p_events IS NOT NULL AND jsonb_typeof(p_events) = 'array' THEN
    FOR ev IN SELECT * FROM jsonb_array_elements(p_events) LOOP
      cur_seq := cur_seq + 1;
      bsha := NULLIF(ev->>'blob_sha', '');
      IF bsha IS NOT NULL AND NOT EXISTS (SELECT 1 FROM evidence.blob WHERE sha256 = bsha) THEN
        bsha := NULL;
      END IF;
      ev_src := CASE WHEN ev->>'src' IN ('page','server','edge')
                     THEN (ev->>'src')::evidence.event_src ELSE 'page'::evidence.event_src END;
      INSERT INTO evidence.event(episode_id, seq, t, src, type, arm_id, payload, blob_sha)
      VALUES (p_episode_id, cur_seq, now(), ev_src, ev->>'type', NULLIF(ev->>'arm_id', ''),
              trace_redact(COALESCE(ev->'payload', '{}'::jsonb)), bsha);
      seqs := seqs || cur_seq;
    END LOOP;
  END IF;

  RETURN seqs;
END $$ LANGUAGE plpgsql;

-- ---------------------------------------------------------------------------
-- Item 6: Compound evidence & lifecycle RPCs (capture_record, rig_activate,
-- oracle_record, app_bootstrap, health_summary)
-- ---------------------------------------------------------------------------
CREATE OR REPLACE FUNCTION capture_record(
  p_capture_id text, p_workspace text, p_meta jsonb, p_frame_bytes bytea,
  p_arm_id text DEFAULT NULL, p_kind text DEFAULT 'calibration',
  p_width int DEFAULT NULL, p_height int DEFAULT NULL
) RETURNS text AS $$
DECLARE
  sha text := encode(digest(p_frame_bytes, 'sha256'), 'hex');
  ep_kind evidence.episode_kind := CASE
    WHEN p_kind IN ('execution','chat','calibration','coverage') THEN p_kind::evidence.episode_kind
    ELSE 'calibration'::evidence.episode_kind END;
BEGIN
  PERFORM _ensure_workspace_arm(p_workspace, p_arm_id);
  INSERT INTO evidence.episode(episode_id, workspace_id, arm_ids, kind)
    VALUES (p_capture_id, p_workspace,
            CASE WHEN p_arm_id IS NOT NULL THEN jsonb_build_array(p_arm_id) ELSE '[]'::jsonb END,
            ep_kind)
    ON CONFLICT (episode_id) DO NOTHING;
  INSERT INTO evidence.blob(sha256, kind, bytes, size_bytes, status)
    VALUES (sha, 'still', p_frame_bytes, octet_length(p_frame_bytes), 'verified')
    ON CONFLICT (sha256) DO UPDATE SET created_at = now();
  INSERT INTO evidence.still(episode_id, sha256, arm_id, purpose, width, height)
    VALUES (p_capture_id, sha, p_arm_id, 'witness', p_width, p_height);
  INSERT INTO evidence.manifest(episode_id, snapshot)
    VALUES (p_capture_id, trace_redact(COALESCE(p_meta, '{}'::jsonb)))
    ON CONFLICT (episode_id) DO NOTHING;
  RETURN sha;
END $$ LANGUAGE plpgsql;

-- Atomic rig activation: ensures calibration (updating placeholder K/dist/size
-- if real intrinsics are provided), supersedes & invalidates prior active rig,
-- and inserts/updates the new active rig with fitted timestamp.
CREATE OR REPLACE FUNCTION rig_activate(
  p_rig_fingerprint text, p_workspace text, p_calib_fingerprint text,
  p_t_base_cam jsonb, p_image_size jsonb, p_tip_offset jsonb,
  p_method text DEFAULT 'base-marker', p_rms_px float8 DEFAULT NULL, p_rms_mm float8 DEFAULT NULL,
  p_witness jsonb DEFAULT NULL, p_provenance jsonb DEFAULT NULL,
  p_rig_doc jsonb DEFAULT NULL, p_witness_sha text DEFAULT NULL,
  p_calib_k jsonb DEFAULT NULL, p_calib_dist jsonb DEFAULT NULL,
  p_calib_image_size jsonb DEFAULT NULL, p_calib_doc jsonb DEFAULT NULL
) RETURNS void AS $$
DECLARE
  old_rig text;
BEGIN
  PERFORM _ensure_workspace_arm(p_workspace, NULL);
  INSERT INTO world.calibration(calib_fingerprint, K, dist, image_size, calib_doc)
  VALUES (
    p_calib_fingerprint,
    COALESCE(p_calib_k, '[[1,0,0],[0,1,0],[0,0,1]]'::jsonb),
    COALESCE(p_calib_dist, '[0,0,0,0,0]'::jsonb),
    COALESCE(p_calib_image_size, '[0,0]'::jsonb),
    p_calib_doc
  )
  ON CONFLICT (calib_fingerprint) DO UPDATE SET
    K          = CASE WHEN p_calib_k IS NOT NULL THEN p_calib_k ELSE world.calibration.K END,
    dist       = CASE WHEN p_calib_dist IS NOT NULL THEN p_calib_dist ELSE world.calibration.dist END,
    image_size = CASE WHEN p_calib_image_size IS NOT NULL THEN p_calib_image_size ELSE world.calibration.image_size END,
    calib_doc  = COALESCE(p_calib_doc, world.calibration.calib_doc);

  -- Lock & supersede any prior active rig in this workspace.
  FOR old_rig IN
    SELECT rig_fingerprint FROM world.shared_rig
     WHERE workspace_id = p_workspace AND status = 'active'
       AND rig_fingerprint <> p_rig_fingerprint
     FOR UPDATE
  LOOP
    UPDATE world.shared_rig
       SET status = 'superseded'
     WHERE rig_fingerprint = old_rig;
    PERFORM world.invalidate_rig(old_rig);
  END LOOP;

  INSERT INTO world.shared_rig(
    rig_fingerprint, workspace_id, calib_fingerprint, T_base_cam, image_size,
    tip_offset, method, rms_px, rms_mm, witness, provenance, rig_doc, witness_sha,
    status, fitted
  ) VALUES (
    p_rig_fingerprint, p_workspace, p_calib_fingerprint, p_t_base_cam, p_image_size,
    p_tip_offset, COALESCE(p_method, 'base-marker'), p_rms_px, p_rms_mm,
    COALESCE(p_witness, '{}'::jsonb), COALESCE(p_provenance, '{}'::jsonb),
    p_rig_doc, p_witness_sha, 'active', now()
  )
  ON CONFLICT (rig_fingerprint) DO UPDATE SET
    calib_fingerprint = EXCLUDED.calib_fingerprint,
    T_base_cam        = EXCLUDED.T_base_cam,
    image_size        = EXCLUDED.image_size,
    tip_offset        = EXCLUDED.tip_offset,
    method            = EXCLUDED.method,
    rms_px            = EXCLUDED.rms_px,
    rms_mm            = EXCLUDED.rms_mm,
    witness           = EXCLUDED.witness,
    provenance        = EXCLUDED.provenance,
    rig_doc           = COALESCE(EXCLUDED.rig_doc, world.shared_rig.rig_doc),
    witness_sha       = COALESCE(EXCLUDED.witness_sha, world.shared_rig.witness_sha),
    status            = 'active',
    fitted            = COALESCE(world.shared_rig.fitted, now());

  IF old_rig IS NOT NULL THEN
    UPDATE world.shared_rig SET superseded_by = p_rig_fingerprint
     WHERE rig_fingerprint = old_rig AND superseded_by IS NULL;
  END IF;
END $$ LANGUAGE plpgsql;

-- Atomic non-workflow oracle record: verdict + accuracy fold + monitor_observe in 1 call.
CREATE OR REPLACE FUNCTION oracle_record(
  p_plan_id text, p_arm_id text, p_doc jsonb,
  p_rig_fingerprint text DEFAULT NULL, p_exec_id text DEFAULT NULL
) RETURNS bigint AS $$
DECLARE
  v jsonb := COALESCE(p_doc->'verdict', '{}'::jsonb);
  vid bigint;
  cfg jsonb;
BEGIN
  IF p_exec_id IS NOT NULL THEN
    INSERT INTO evidence.oracle_verdict(plan_id, exec_id, arm_id, layer, confidence, problems, doc)
    VALUES (p_plan_id, p_exec_id, p_arm_id,
            COALESCE(v->>'layer', 'none'), COALESCE(v->>'confidence', 'low'),
            COALESCE(v->'problems', '[]'::jsonb), p_doc)
    ON CONFLICT (exec_id) WHERE exec_id IS NOT NULL DO UPDATE
      SET doc = EXCLUDED.doc
    RETURNING id INTO vid;
  ELSE
    INSERT INTO evidence.oracle_verdict(plan_id, exec_id, arm_id, layer, confidence, problems, doc)
    VALUES (p_plan_id, NULL, p_arm_id,
            COALESCE(v->>'layer', 'none'), COALESCE(v->>'confidence', 'low'),
            COALESCE(v->'problems', '[]'::jsonb), p_doc)
    RETURNING id INTO vid;
  END IF;
  IF p_rig_fingerprint IS NOT NULL AND p_rig_fingerprint <> '' AND p_arm_id IS NOT NULL THEN
    SELECT config_doc->'learning' INTO cfg FROM world.arm_config WHERE arm_id = p_arm_id;
    IF cfg IS NOT NULL THEN
      PERFORM accuracy_cell_update(p_arm_id, p_rig_fingerprint, p_doc, cfg, NULL);
    END IF;
  END IF;
  IF p_arm_id IS NOT NULL THEN
    PERFORM monitor_observe(p_arm_id, p_rig_fingerprint, p_doc);
  END IF;
  RETURN vid;
END $$ LANGUAGE plpgsql;

-- Single-query startup bootstrap for capture_server (replaces 9 sequential connections).
CREATE OR REPLACE FUNCTION app_bootstrap(p_workspace text, p_arm_id text DEFAULT 'b601')
RETURNS jsonb AS $$
DECLARE
  arm_cfg jsonb;
  cal record;
  rig record;
  witness_b64 text;
  sag jsonb;
  mon record;
  llm_ctx jsonb;
BEGIN
  PERFORM _ensure_workspace_arm(p_workspace, p_arm_id);
  SELECT config_doc INTO arm_cfg FROM world.arm_config WHERE arm_id = p_arm_id;

  SELECT calib_fingerprint, calib_doc INTO cal
    FROM world.calibration
   WHERE calib_doc IS NOT NULL
   ORDER BY is_default DESC, created_at DESC LIMIT 1;

  SELECT rig_fingerprint, rig_doc, witness_sha INTO rig
    FROM world.shared_rig
   WHERE workspace_id = p_workspace AND status = 'active'
   ORDER BY fitted DESC NULLS LAST LIMIT 1;

  IF rig.witness_sha IS NOT NULL THEN
    SELECT replace(encode(bytes, 'base64'), E'\n', '') INTO witness_b64
      FROM evidence.blob WHERE sha256 = rig.witness_sha;
  END IF;

  SELECT jsonb_build_object(
           'rig_fingerprint', rig_fingerprint, 'kp_key', kp_key, 'zeros_key', zeros_key,
           'per_motor', per_motor, 'samples', samples, 'rms_mrad', rms_mrad,
           'validated_mm', validated_mm, 'envelope', envelope)
    INTO sag
    FROM learning.sag_map
   WHERE arm_id = p_arm_id AND status = 'active'
     AND (rig.rig_fingerprint IS NULL OR rig_fingerprint = rig.rig_fingerprint)
   ORDER BY created_at DESC LIMIT 1;

  SELECT ewma_pixel_error_mm, ewma_tag_rms_px, last_doc INTO mon
    FROM learning.monitor_state WHERE arm_id = p_arm_id;

  SELECT value INTO llm_ctx FROM substrate_kv WHERE key = 'llm_context';

  RETURN jsonb_build_object(
    'arm_config', arm_cfg,
    'calibration', CASE WHEN cal.calib_fingerprint IS NOT NULL
                        THEN jsonb_build_object('calib_fingerprint', cal.calib_fingerprint, 'calib_doc', cal.calib_doc)
                        ELSE NULL END,
    'rig', CASE WHEN rig.rig_doc IS NOT NULL
                THEN jsonb_build_object('rig_fingerprint', rig.rig_fingerprint, 'rig_doc', rig.rig_doc,
                                        'witness_sha', rig.witness_sha, 'witness_b64', witness_b64)
                ELSE NULL END,
    'sag_map', sag,
    'monitor', CASE WHEN mon.ewma_pixel_error_mm IS NOT NULL OR mon.ewma_tag_rms_px IS NOT NULL OR mon.last_doc IS NOT NULL
                    THEN jsonb_build_object('ewma_pixel_error_mm', mon.ewma_pixel_error_mm,
                                            'ewma_tag_rms_px', mon.ewma_tag_rms_px, 'last_doc', mon.last_doc)
                    ELSE NULL END,
    'llm_context', llm_ctx
  );
END $$ LANGUAGE plpgsql;

-- Single-query health summary for GET /health (captures count + latest + traces + accuracy map).
CREATE OR REPLACE FUNCTION health_summary(p_workspace text, p_arm_id text DEFAULT 'b601')
RETURNS jsonb AS $$
DECLARE
  cap_count int;
  latest_cap text;
  trace_count int;
  rig_fp text;
  acc_cells jsonb;
  mon jsonb;
BEGIN
  SELECT count(*)::int INTO cap_count
    FROM evidence.episode
   WHERE workspace_id = p_workspace AND kind IN ('calibration', 'execution');

  SELECT episode_id INTO latest_cap
    FROM evidence.episode
   WHERE workspace_id = p_workspace AND kind IN ('calibration', 'execution')
   ORDER BY started_at DESC LIMIT 1;

  SELECT count(*)::int INTO trace_count FROM evidence.episode;

  SELECT rig_fingerprint INTO rig_fp
    FROM world.shared_rig
   WHERE workspace_id = p_workspace AND status = 'active'
   ORDER BY fitted DESC NULLS LAST LIMIT 1;

  IF rig_fp IS NOT NULL THEN
    SELECT COALESCE(jsonb_object_agg(
             cell_key,
             jsonb_build_object(
               'predicted_mm', predicted_mm, 'measured_mm', measured_mm,
               'n', n, 'n_good', n_good,
               'validated_at', to_char(validated_at AT TIME ZONE 'UTC', 'YYYY-MM-DD"T"HH24:MI:SS"Z"'))),
           '{}'::jsonb)
      INTO acc_cells
      FROM learning.accuracy_cell
     WHERE arm_id = p_arm_id AND rig_fingerprint = rig_fp;
  ELSE
    acc_cells := '{}'::jsonb;
  END IF;

  SELECT jsonb_build_object(
           'ewma_pixel_error_mm', ewma_pixel_error_mm,
           'ewma_tag_rms_px', ewma_tag_rms_px,
           'proposals', COALESCE(last_doc->'proposals', '[]'::jsonb))
    INTO mon
    FROM learning.monitor_state WHERE arm_id = p_arm_id;

  RETURN jsonb_build_object(
    'captures', cap_count,
    'latest', latest_cap,
    'trace_sessions', trace_count,
    'rig_fingerprint', rig_fp,
    'accuracy_map', jsonb_build_object('version', 1, 'cells', acc_cells),
    'monitor', mon
  );
END $$ LANGUAGE plpgsql STABLE;

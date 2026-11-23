-- 0017: oracle attribution thresholds group in world.arm_config (required-config
-- consolidation, Phase B). The 12 thresholds formerly hardcoded in oracle.py and
-- oracle_attribute (0002) move into the "oracle" group. Seed bit-identical to the
-- former constants -> attribution is unchanged. oracle_attribute now takes this
-- group as a REQUIRED jsonb arg (passed by oracle_compute from the table); a
-- missing doc/field is a hard refusal. CMD_TOL / HOLD_TIME_TOL stay hardcoded
-- (numeric epsilons); HOLD_MIN_S / MAX_SAMPLES belong to hold_windows/downsample.
-- Idempotent merge into the existing config_doc.

UPDATE world.arm_config
   SET config_doc = config_doc || '{"oracle":{"pixel_error_mm":10.0,"tip_error_mm":8.0,"hold_error_rad":0.030,"witness_drift_px":2.0,"witness_scale_tol":0.05,"sag_correlation":0.8,"tip_vector_coherence":0.8,"zero_bias_spread_rad":0.005,"reach_fraction":0.9,"placement_xy_mm":10.0,"placement_z_tol_mm":5.0,"ik_residual_mm":10.0}}'::jsonb,
       updated_at = now()
 WHERE arm_id = 'b601';

-- --------------------------------------------------------------------------- --
-- oracle.attribute: residual shape -> {layer, problems[], confidence} (§5.4).
-- Faithful port of oracle.py:attribute. Every field optional; missing is reported
-- as missing, never guessed. Thresholds come from the REQUIRED config arg (the
-- world.arm_config "oracle" group); body otherwise verbatim from 0002.
-- --------------------------------------------------------------------------- --
DROP FUNCTION IF EXISTS oracle_attribute(jsonb);
CREATE OR REPLACE FUNCTION oracle_attribute(doc jsonb, config jsonb) RETURNS jsonb AS $$
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
  -- thresholds (world.arm_config "oracle" group; required, no fallback)
  PIXEL_ERROR_MM constant float8 := (config->>'pixel_error_mm')::float8;
  TIP_ERROR_MM constant float8 := (config->>'tip_error_mm')::float8;
  HOLD_ERROR_RAD constant float8 := (config->>'hold_error_rad')::float8;
  WITNESS_DRIFT_PX constant float8 := (config->>'witness_drift_px')::float8;
  WITNESS_SCALE_TOL constant float8 := (config->>'witness_scale_tol')::float8;
  SAG_CORRELATION constant float8 := (config->>'sag_correlation')::float8;
  TIP_VECTOR_COHERENCE constant float8 := (config->>'tip_vector_coherence')::float8;
  ZERO_BIAS_SPREAD_RAD constant float8 := (config->>'zero_bias_spread_rad')::float8;
  REACH_FRACTION constant float8 := (config->>'reach_fraction')::float8;
  PLACEMENT_XY_MM constant float8 := (config->>'placement_xy_mm')::float8;
  PLACEMENT_Z_TOL_MM constant float8 := (config->>'placement_z_tol_mm')::float8;
  IK_RESIDUAL_MM constant float8 := (config->>'ik_residual_mm')::float8;
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
  IF config IS NULL OR NOT (config ?& array['pixel_error_mm','tip_error_mm','hold_error_rad','witness_drift_px','witness_scale_tol','sag_correlation','tip_vector_coherence','zero_bias_spread_rad','reach_fraction','placement_xy_mm','placement_z_tol_mm','ik_residual_mm']) THEN
    RAISE EXCEPTION 'oracle config missing required keys';
  END IF;
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

-- oracle_compute: verbatim from 0015 except the oracle_attribute call now passes
-- the required "oracle" group from the arm_config doc it already reads.
CREATE OR REPLACE FUNCTION oracle_compute(
  p_urdf text, p_waypoints jsonb, p_samples jsonb, p_targets jsonb,
  p_detections jsonb, p_k float8[], p_dist float8[], p_t_base_cam float8[],
  p_tip_offset float8[], p_kp_spec text, p_doc_extra jsonb DEFAULT '{}'::jsonb,
  p_arm_id text DEFAULT 'b601'
) RETURNS jsonb AS $$
DECLARE
  windows jsonb := hold_windows(p_waypoints);
  matched jsonb := match_hold_targets(windows, p_targets);
  holds jsonb := '[]'::jsonb;
  cfg jsonb;
  w jsonb; target jsonb; det jsonb;
  idx int; i int; j int;
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
  sx float8; sy float8;
BEGIN
  SELECT config_doc INTO cfg FROM world.arm_config WHERE arm_id = p_arm_id;
  IF cfg IS NULL THEN
    RAISE EXCEPTION 'no arm config seeded (world.arm_config) for arm %', p_arm_id;
  END IF;
  cam_origin := ARRAY[p_t_base_cam[4], p_t_base_cam[8], p_t_base_cam[12]];
  focal := (p_k[1] + p_k[5]) / 2.0;
  idx := 0;
  FOR w IN SELECT * FROM jsonb_array_elements(windows) LOOP
    idx := idx + 1;
    t0 := (w->>'t_start')::float8; t1 := (w->>'t_end')::float8;
    q_cmd := ARRAY[0,0,0,0,0,0]; q_meas := ARRAY[0,0,0,0,0,0];
    n_inside := 0;
    FOR i IN 0..(jsonb_array_length(p_samples)-1) LOOP
      sx := (p_samples->i->>'t')::float8;
      IF sx >= t0 - 1e-6 AND sx <= t1 + 1e-6 THEN
        n_inside := n_inside + 1;
        FOR j IN 0..5 LOOP
          q_cmd[j+1] := q_cmd[j+1] + (p_samples->i->'cmd'->>j)::float8;
          q_meas[j+1] := q_meas[j+1] + (p_samples->i->'meas'->>j)::float8;
        END LOOP;
      END IF;
    END LOOP;
    hold := jsonb_build_object('index', (w->>'index')::int, 't_start', t0, 't_end', t1);
    IF n_inside > 0 THEN
      FOR j IN 1..6 LOOP q_cmd[j] := q_cmd[j]/n_inside; q_meas[j] := q_meas[j]/n_inside; END LOOP;
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
      target := matched->(idx-1);
      IF target IS NOT NULL AND jsonb_typeof(target) = 'object' THEN
        hold := hold || jsonb_build_object('target_point', target->'point',
                                           'flange_point', target->'flange_point');
        IF target->'q_ik' IS NOT NULL THEN
          tip_ik := rebot_fk_tip(p_urdf, ARRAY(SELECT jsonb_array_elements_text(target->'q_ik')::float8));
          ik_res := 1000.0 * sqrt((tip_ik[1]-(target->'flange_point'->>0)::float8)^2
                                + (tip_ik[2]-(target->'flange_point'->>1)::float8)^2
                                + (tip_ik[3]-(target->'flange_point'->>2)::float8)^2);
          placement := 1000.0 * sqrt((tip_meas[1]-(target->'flange_point'->>0)::float8)^2
                                   + (tip_meas[2]-(target->'flange_point'->>1)::float8)^2
                                   + (tip_meas[3]-(target->'flange_point'->>2)::float8)^2);
          hold := hold || jsonb_build_object('q_ik', target->'q_ik',
            'tip_fk_ik', to_jsonb(tip_ik), 'ik_residual_mm', round(ik_res::numeric,3),
            'placement_mm', round(placement::numeric,3));
        END IF;
      END IF;
      det := p_detections->((w->>'index')::text);
      IF det IS NOT NULL AND det->'uv' IS NOT NULL THEN
        tip_point := rebot_tip_point(p_urdf, q_meas, p_tip_offset);
        predicted := rebot_project_dist(p_k, p_t_base_cam, tip_point, p_dist);
        detected := det->'uv';
        hold := hold || jsonb_build_object('image_size', det->'image_size',
          'grid_scale', det->'grid_scale', 'predicted_tip_px', predicted,
          'detected_tip_px', detected, 'detector', det->'detector',
          'detector_score', det->'score', 'frame', det->'frame');
        IF predicted IS NOT NULL THEN
          px_err := sqrt(((detected->>0)::float8 - predicted[1])^2
                       + ((detected->>1)::float8 - predicted[2])^2);
          dist_cam := sqrt((cam_origin[1]-tip_point[1])^2 + (cam_origin[2]-tip_point[2])^2
                         + (cam_origin[3]-tip_point[3])^2);
          mm_per_px := CASE WHEN focal > 0 THEN 1000.0*dist_cam/focal ELSE NULL END;
          hold := hold || jsonb_build_object('pixel_error_px', round(px_err::numeric,2),
            'pixel_error_mm', CASE WHEN mm_per_px IS NOT NULL THEN round((px_err*mm_per_px)::numeric,2) ELSE NULL END);
        END IF;
      END IF;
      max_radius := greatest(max_radius, sqrt(tip_cmd[1]^2 + tip_cmd[2]^2));
    END IF;
    holds := holds || jsonb_build_array(hold);
  END LOOP;
  doc := coalesce(p_doc_extra, '{}'::jsonb) || jsonb_build_object(
    'holds', holds, 'max_radius_m', round(max_radius::numeric,4));
  RETURN jsonb_build_object('doc', doc, 'verdict', oracle_attribute(doc, cfg->'oracle'),
                            'holds', holds, 'max_radius_m', round(max_radius::numeric,4));
END $$ LANGUAGE plpgsql;
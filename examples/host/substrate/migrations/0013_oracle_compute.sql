-- 0013: oracle pure helpers + verdict compute in the substrate.
-- Ports of oracle.hold_windows / match_hold_targets / downsample, and the
-- per-hold residual compute (FK + gravity sag + predicted-vs-detected pixel)
-- that capture_server.oracle_finish did in Python. Detection (green-tip OpenCV)
-- stays an isolated activity; its per-hold pixels are passed in as `detections`.

CREATE OR REPLACE FUNCTION hold_windows(p_waypoints jsonb, p_min_s float8 DEFAULT 0.8)
RETURNS jsonb AS $$
DECLARE
  n int := jsonb_array_length(p_waypoints);
  windows jsonb := '[]'::jsonb;
  start_idx int := 0;
  i int; j int;
  same boolean;
  qa jsonb; qb jsonb;
  t0 float8; t1 float8;
BEGIN
  FOR i IN 1..n LOOP
    same := false;
    IF i < n THEN
      qa := p_waypoints->i->'q'; qb := p_waypoints->(i-1)->'q';
      IF qa IS NOT NULL AND qb IS NOT NULL
         AND jsonb_array_length(qa) = jsonb_array_length(qb)
         AND jsonb_array_length(qa) > 0 THEN
        same := true;
        FOR j IN 0..5 LOOP
          IF abs((qa->>j)::float8 - (qb->>j)::float8) > 1e-9 THEN
            same := false; EXIT;
          END IF;
        END LOOP;
      END IF;
    END IF;
    IF same THEN CONTINUE; END IF;
    IF i - start_idx >= 2 THEN
      t0 := (p_waypoints->start_idx->>'t')::float8;
      t1 := (p_waypoints->(i-1)->>'t')::float8;
      IF t1 - t0 >= p_min_s THEN
        windows := windows || jsonb_build_object(
          'index', jsonb_array_length(windows)+1, 'start', start_idx, 'end', i,
          't_start', t0, 't_end', t1, 'q_cmd', p_waypoints->start_idx->'q');
      END IF;
    END IF;
    start_idx := i;
  END LOOP;
  RETURN windows;
END $$ LANGUAGE plpgsql IMMUTABLE;

CREATE OR REPLACE FUNCTION match_hold_targets(p_windows jsonb, p_targets jsonb,
                                              p_tol float8 DEFAULT 1e-6)
RETURNS jsonb AS $$
DECLARE
  matched jsonb := '[]'::jsonb;
  w jsonb; t jsonb;
  t0 float8; t1 float8;
  best jsonb;
BEGIN
  FOR w IN SELECT * FROM jsonb_array_elements(coalesce(p_windows, '[]'::jsonb)) LOOP
    t0 := (w->>'t_start')::float8; t1 := (w->>'t_end')::float8;
    best := NULL;
    FOR t IN SELECT * FROM jsonb_array_elements(coalesce(p_targets, '[]'::jsonb)) LOOP
      IF t->>'hold_t_start' IS NOT NULL AND t->>'hold_t_end' IS NOT NULL
         AND (t->>'hold_t_start')::float8 - p_tol <= t0
         AND t1 <= (t->>'hold_t_end')::float8 + p_tol THEN
        best := t; EXIT;
      END IF;
    END LOOP;
    matched := matched || jsonb_build_array(best);
  END LOOP;
  RETURN matched;
END $$ LANGUAGE plpgsql IMMUTABLE;

CREATE OR REPLACE FUNCTION downsample(p_samples jsonb, p_limit int DEFAULT 2000)
RETURNS jsonb AS $$
DECLARE
  cnt int := jsonb_array_length(p_samples);
  keep boolean[];
  i int; j int; start_idx int := 0;
  same boolean;
  ca jsonb; cb jsonb;
  kept int; budget int; step int;
  free int[];
  out jsonb := '[]'::jsonb;
BEGIN
  IF cnt <= p_limit THEN RETURN p_samples; END IF;
  keep := array_fill(false, ARRAY[cnt]);
  FOR i IN 1..cnt LOOP
    same := false;
    IF i < cnt THEN
      ca := p_samples->i->'cmd'; cb := p_samples->(i-1)->'cmd';
      IF ca IS NOT NULL AND cb IS NOT NULL
         AND jsonb_array_length(ca) = jsonb_array_length(cb)
         AND jsonb_array_length(ca) > 0 THEN
        same := true;
        FOR j IN 0..5 LOOP
          IF abs((ca->>j)::float8 - (cb->>j)::float8) > 1e-9 THEN same := false; EXIT; END IF;
        END LOOP;
      END IF;
    END IF;
    IF same THEN CONTINUE; END IF;
    IF i - start_idx >= 2 THEN
      FOR j IN start_idx..(i-1) LOOP keep[j+1] := true; END LOOP;
    END IF;
    start_idx := i;
  END LOOP;
  keep[1] := true; keep[cnt] := true;
  kept := 0;
  FOR i IN 1..cnt LOOP IF keep[i] THEN kept := kept + 1; END IF; END LOOP;
  IF kept < p_limit THEN
    free := ARRAY(SELECT i-1 FROM generate_series(1, cnt) i WHERE NOT keep[i]);
    budget := p_limit - kept;
    FOR step IN 0..(budget-1) LOOP
      keep[free[1 + (step * array_length(free,1) / budget)] + 1] := true;
    END LOOP;
  END IF;
  FOR i IN 1..cnt LOOP
    IF keep[i] THEN out := out || jsonb_build_array(p_samples->(i-1)); END IF;
  END LOOP;
  RETURN out;
END $$ LANGUAGE plpgsql IMMUTABLE;

-- The per-hold residual compute + attribution. detections is an object keyed by
-- hold index (text) -> {uv:[u,v], score, detector, image_size:[w,h], grid_scale,
-- frame}. doc_extra carries the fields capture measures separately (telemetry
-- flags, witness, stream_turn, tag_reprojection_px, pre_image_size, ...).
-- Returns {doc, verdict, holds, max_radius_m}.
DROP FUNCTION IF EXISTS oracle_compute(text, jsonb, jsonb, jsonb, jsonb, jsonb, jsonb, jsonb, jsonb, text, jsonb);
CREATE OR REPLACE FUNCTION oracle_compute(
  p_urdf text, p_waypoints jsonb, p_samples jsonb, p_targets jsonb,
  p_detections jsonb, p_k float8[], p_dist float8[], p_t_base_cam float8[],
  p_tip_offset float8[], p_kp_spec text, p_doc_extra jsonb DEFAULT '{}'::jsonb
) RETURNS jsonb AS $$
DECLARE
  windows jsonb := hold_windows(p_waypoints);
  matched jsonb := match_hold_targets(windows, p_targets);
  holds jsonb := '[]'::jsonb;
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
      sag := rebot_sag_rad(p_urdf, q_cmd, p_kp_spec);
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
  RETURN jsonb_build_object('doc', doc, 'verdict', oracle_attribute(doc),
                            'holds', holds, 'max_radius_m', round(max_radius::numeric,4));
END $$ LANGUAGE plpgsql;
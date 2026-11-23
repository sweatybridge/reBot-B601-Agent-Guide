-- 0015: config REQUIRED — the wrappers read world.arm_config and pass the doc into
-- the compute cores (option B). The cores have no hardcoded fallback constants; a
-- missing arm_config row is a hard refusal here, and a missing/invalid field is a
-- hard refusal in Config::from_json. The cores stay pure jsonb->jsonb (no SPI).
--
-- Same wrapper signatures as 0011/0012 for plan_one/move_one (CREATE OR REPLACE);
-- oracle_compute gains a trailing p_arm_id (default 'b601') so the existing 11-arg
-- capture call still works, and reads arm_config internally to feed rebot_sag_rad.

CREATE OR REPLACE FUNCTION plan_one(
  p_workspace text,
  p_pose      jsonb,
  p_target    jsonb,
  p_options   jsonb,
  p_arm_id    text DEFAULT 'b601'
) RETURNS jsonb AS $$
DECLARE
  urdf_text text;
  cfg       jsonb;
  rig_fp    text;
  sag       jsonb;
  snap      jsonb;
  result    jsonb;
  payload   jsonb;
  summary   jsonb;
  pid       text;
  raw       bytea;
  sha       text;
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

  snap := jsonb_build_object('rig_fingerprint', coalesce(rig_fp, ''),
                             'zeros_key', 'none', 'sag_map', null);
  IF rig_fp IS NOT NULL THEN
    SELECT jsonb_build_object('rig_fingerprint', rig_fingerprint, 'kp_key', kp_key,
                              'zeros_key', zeros_key, 'per_motor', per_motor,
                              'envelope', envelope)
      INTO sag
      FROM learning.sag_map
      WHERE arm_id = p_arm_id AND rig_fingerprint = rig_fp AND status = 'active'
      ORDER BY created_at DESC LIMIT 1;
    IF sag IS NOT NULL THEN
      snap := jsonb_build_object('rig_fingerprint', rig_fp,
                                 'zeros_key', sag->>'zeros_key', 'sag_map', sag);
    END IF;
  END IF;

  -- array -> multi-marker sequence; object -> single hover tap
  IF jsonb_typeof(p_target) = 'array' THEN
    result := rebot_plan_sequence(urdf_text, p_pose, p_target, p_options, snap, cfg);
  ELSE
    result := rebot_plan(urdf_text, p_pose, p_target, p_options, snap, cfg);
  END IF;
  payload := result->'payload';
  summary := result->'summary';

  pid := to_char(now() AT TIME ZONE 'UTC', 'YYYYMMDD"T"HH24MISS"Z"')
         || '-' || substr(md5(random()::text || clock_timestamp()::text), 1, 6);
  raw := convert_to(payload::text, 'UTF8');
  sha := encode(digest(raw, 'sha256'), 'hex');

  INSERT INTO world.workspace(workspace_id, name, plane_z, min_z, hover_m)
    VALUES (p_workspace, p_workspace, 0, 0.04, 0.05) ON CONFLICT (workspace_id) DO NOTHING;
  INSERT INTO world.arm(arm_id, name, urdf_fingerprint, zeros_key, kp_key)
    VALUES (p_arm_id, p_arm_id, 'unknown', 'none', 'default') ON CONFLICT (arm_id) DO NOTHING;
  INSERT INTO evidence.blob(sha256, kind, bytes, size_bytes, status)
    VALUES (sha, 'trajectory', raw, length(raw), 'verified') ON CONFLICT (sha256) DO NOTHING;
  INSERT INTO evidence.episode(episode_id, workspace_id, arm_ids, kind)
    VALUES (pid, p_workspace, jsonb_build_array(p_arm_id), 'execution')
    ON CONFLICT (episode_id) DO NOTHING;
  INSERT INTO evidence.plan(plan_id, episode_id, arm_id, blob_sha, metadata, snapshot, expires_at)
    VALUES (pid, pid, p_arm_id, sha, summary,
            jsonb_build_object('rig', rig_fp, 'q', p_pose->'q'),
            now() + interval '600 seconds')
    ON CONFLICT (plan_id) DO NOTHING;

  RETURN jsonb_build_object('plan_id', pid, 'summary', summary);
END $$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION move_one(
  p_workspace text,
  p_pose      jsonb,
  p_targets   jsonb,
  p_options   jsonb,
  p_arm_id    text DEFAULT 'b601'
) RETURNS jsonb AS $$
DECLARE
  urdf_text text;
  cfg       jsonb;
  rig_fp    text;
  sag       jsonb;
  snap      jsonb;
  result    jsonb;
  payload   jsonb;
  summary   jsonb;
  pid       text;
  raw       bytea;
  sha       text;
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

  snap := jsonb_build_object('rig_fingerprint', coalesce(rig_fp, ''),
                             'zeros_key', 'none', 'sag_map', null);
  IF rig_fp IS NOT NULL THEN
    SELECT jsonb_build_object('rig_fingerprint', rig_fingerprint, 'kp_key', kp_key,
                              'zeros_key', zeros_key, 'per_motor', per_motor,
                              'envelope', envelope)
      INTO sag
      FROM learning.sag_map
      WHERE arm_id = p_arm_id AND rig_fingerprint = rig_fp AND status = 'active'
      ORDER BY created_at DESC LIMIT 1;
    IF sag IS NOT NULL THEN
      snap := jsonb_build_object('rig_fingerprint', rig_fp,
                                 'zeros_key', sag->>'zeros_key', 'sag_map', sag);
    END IF;
  END IF;

  result  := rebot_move_build(urdf_text, p_pose, p_targets, p_options, snap, cfg);
  payload := result->'payload';
  summary := result->'summary';

  pid := to_char(now() AT TIME ZONE 'UTC', 'YYYYMMDD"T"HH24MISS"Z"')
         || '-' || substr(md5(random()::text || clock_timestamp()::text), 1, 6);
  raw := convert_to(payload::text, 'UTF8');
  sha := encode(digest(raw, 'sha256'), 'hex');

  INSERT INTO world.workspace(workspace_id, name, plane_z, min_z, hover_m)
    VALUES (p_workspace, p_workspace, 0, 0.04, 0.05) ON CONFLICT (workspace_id) DO NOTHING;
  INSERT INTO world.arm(arm_id, name, urdf_fingerprint, zeros_key, kp_key)
    VALUES (p_arm_id, p_arm_id, 'unknown', 'none', 'default') ON CONFLICT (arm_id) DO NOTHING;
  INSERT INTO evidence.blob(sha256, kind, bytes, size_bytes, status)
    VALUES (sha, 'trajectory', raw, length(raw), 'verified') ON CONFLICT (sha256) DO NOTHING;
  INSERT INTO evidence.episode(episode_id, workspace_id, arm_ids, kind)
    VALUES (pid, p_workspace, jsonb_build_array(p_arm_id), 'execution')
    ON CONFLICT (episode_id) DO NOTHING;
  INSERT INTO evidence.plan(plan_id, episode_id, arm_id, blob_sha, metadata, snapshot, expires_at)
    VALUES (pid, pid, p_arm_id, sha, summary,
            jsonb_build_object('rig', rig_fp, 'q', p_pose->'q', 'kind', 'move'),
            now() + interval '600 seconds')
    ON CONFLICT (plan_id) DO NOTHING;

  RETURN jsonb_build_object('plan_id', pid, 'summary', summary);
END $$ LANGUAGE plpgsql;

-- oracle_compute gains p_arm_id (trailing, default 'b601') to read arm_config and
-- feed rebot_sag_rad. The 11-arg capture call still binds (p_arm_id defaults).
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
  RETURN jsonb_build_object('doc', doc, 'verdict', oracle_attribute(doc),
                            'holds', holds, 'max_radius_m', round(max_radius::numeric,4));
END $$ LANGUAGE plpgsql;
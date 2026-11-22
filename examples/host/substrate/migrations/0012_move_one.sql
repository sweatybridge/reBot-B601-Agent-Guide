-- 0012: move_one — the /move compute as a SQL function (port of planner_server
-- .move_one's trajectory build). rebot_move_build does peel + worktop gates +
-- sag-compensated commands + policy-paced legs + joint_move/joint_sequence; this
-- wrapper gathers the snapshot, calls it in one transaction, and persists the
-- trajectory to evidence.plan. The gateway keeps the edge I/O (pose, /play,
-- telemetry). targets is an array of [6] joint poses; pose is {ids, q[7]}.

CREATE OR REPLACE FUNCTION move_one(
  p_workspace text,
  p_pose      jsonb,
  p_targets   jsonb,
  p_options   jsonb,
  p_arm_id    text DEFAULT 'b601'
) RETURNS jsonb AS $$
DECLARE
  urdf_text text;
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

  result  := rebot_move_build(urdf_text, p_pose, p_targets, p_options, snap);
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
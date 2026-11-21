-- Coordination primitives validated: segment distance, clearance cells, coordination
-- oracle, and the guarded rendezvous state machine.
DO $$
DECLARE
  res jsonb; rid bigint; rid2 bigint; rid3 bigint; st text;
  cn int; cmin float8; cval boolean;
BEGIN
  INSERT INTO world.workspace(workspace_id,name,plane_z,min_z,hover_m) VALUES ('ws_co','co',0,0.04,0.05) ON CONFLICT DO NOTHING;
  INSERT INTO world.arm(arm_id,name,urdf_fingerprint,zeros_key,kp_key) VALUES ('co_a','A','u','z','kp'),('co_b','B','u','z','kp') ON CONFLICT DO NOTHING;
  INSERT INTO evidence.episode(episode_id,workspace_id,arm_ids,kind) VALUES ('ep_co','ws_co','["co_a","co_b"]','execution') ON CONFLICT DO NOTHING;
  INSERT INTO evidence.plan_group(group_id,episode_id,workspace_id,arm_ids) VALUES ('grp_co','ep_co','ws_co','["co_a","co_b"]') ON CONFLICT DO NOTHING;

  -- segment distance: parallel segments 0.1 m apart; crossing segments -> 0
  IF abs(rebot_seg_seg_distance(ARRAY[0,0,0]::float8[],ARRAY[1,0,0]::float8[],ARRAY[0,0.1,0]::float8[],ARRAY[1,0.1,0]::float8[]) - 0.1) > 1e-9 THEN
    RAISE EXCEPTION 'parallel segment distance wrong'; END IF;
  IF rebot_seg_seg_distance(ARRAY[0,0,0]::float8[],ARRAY[1,0,0]::float8[],ARRAY[0.5,-1,0]::float8[],ARRAY[0.5,1,0]::float8[]) > 1e-9 THEN
    RAISE EXCEPTION 'crossing segment distance should be 0'; END IF;

  -- clearance cell: 3 measurements -> validated, min tracked
  DELETE FROM learning.clearance_cell WHERE arm_a='co_a' AND arm_b='co_b' AND workspace_id='ws_co';
  PERFORM clearance_cell_update('co_a','co_b','ws_co',ARRAY[0,0,0]::float8[],ARRAY[0.1,0,0]::float8[],20.0);
  PERFORM clearance_cell_update('co_a','co_b','ws_co',ARRAY[0,0,0]::float8[],ARRAY[0.1,0,0]::float8[],15.0);
  PERFORM clearance_cell_update('co_a','co_b','ws_co',ARRAY[0,0,0]::float8[],ARRAY[0.1,0,0]::float8[],18.0);
  SELECT n, min_clearance_mm, validated_at IS NOT NULL INTO cn, cmin, cval
    FROM learning.clearance_cell WHERE arm_a='co_a' AND arm_b='co_b' AND workspace_id='ws_co';
  IF cn <> 3 OR cmin <> 15.0 OR NOT cval THEN
    RAISE EXCEPTION 'clearance cell: n=% min=% val=% (expected 3/15/true)', cn, cmin, cval; END IF;

  -- coordination oracle branches
  res := coordination_oracle('{}'::jsonb);
  IF res->>'layer' <> 'none' THEN RAISE EXCEPTION 'clean coordination layer: %', res->>'layer'; END IF;
  res := coordination_oracle('{"min_clearance_mm":5,"clearance_margin_mm":10}'::jsonb);
  IF res->>'layer' <> 'coordination' OR res->'problems'->0->>'kind' <> 'clearance-violation' THEN RAISE EXCEPTION 'clearance-violation: %', res; END IF;
  res := coordination_oracle('{"rendezvous_error_mm":20}'::jsonb);
  IF res->'problems'->0->>'kind' <> 'rendezvous-miss' THEN RAISE EXCEPTION 'rendezvous-miss: %', res; END IF;
  res := coordination_oracle('{"handoff_ok":false}'::jsonb);
  IF res->'problems'->0->>'kind' <> 'handoff-drop' THEN RAISE EXCEPTION 'handoff-drop: %', res; END IF;
  res := coordination_oracle('{"abort":true}'::jsonb);
  IF res->>'layer' <> 'execution' OR res->'problems'->0->>'kind' <> 'sync-fault' THEN RAISE EXCEPTION 'sync-fault: %', res; END IF;

  -- rendezvous: legal path pending -> a_at -> confirmed -> released
  INSERT INTO coord.rendezvous(group_id,arm_a,arm_b,state) VALUES ('grp_co','co_a','co_b','pending') RETURNING id INTO rid;
  IF rendezvous_transition(rid,'a_arrived') <> 'a_at' THEN RAISE EXCEPTION 'a_arrived transition'; END IF;
  IF rendezvous_transition(rid,'b_arrived') <> 'confirmed' THEN RAISE EXCEPTION 'b_arrived transition'; END IF;
  IF rendezvous_transition(rid,'release') <> 'released' THEN RAISE EXCEPTION 'release transition'; END IF;

  -- illegal: release from pending must be rejected (state stays pending)
  INSERT INTO coord.rendezvous(group_id,arm_a,arm_b,state) VALUES ('grp_co','co_a','co_b','pending') RETURNING id INTO rid2;
  BEGIN
    PERFORM rendezvous_transition(rid2,'release');
  EXCEPTION WHEN raise_exception THEN NULL;
  END;
  SELECT state INTO st FROM coord.rendezvous WHERE id=rid2;
  IF st <> 'pending' THEN RAISE EXCEPTION 'illegal release changed state to %', st; END IF;

  -- fault from any state -> failed
  INSERT INTO coord.rendezvous(group_id,arm_a,arm_b,state) VALUES ('grp_co','co_a','co_b','pending') RETURNING id INTO rid3;
  IF rendezvous_transition(rid3,'fault') <> 'failed' THEN RAISE EXCEPTION 'fault transition'; END IF;

  RAISE NOTICE 'COORDINATION PRIMITIVES OK';
END $$;
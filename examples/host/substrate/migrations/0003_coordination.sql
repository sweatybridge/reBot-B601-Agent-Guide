-- 0003_coordination.sql — two-arm coordination (spec §6). Apply after 0001/0002.
-- Clearance cells key on relative tip pose; the position component is implemented
-- here (3D bins), extensible to the full 6D (orientation) cell_key. The clearance
-- map is pair-specific (arm_a, arm_b, workspace) and never merges across pairs.

-- Relative-tip-pose cell key (position bins; 0.05 m grid). arm_a < arm_b canonical.
CREATE OR REPLACE FUNCTION clearance_cell_key(tip_a float8[], tip_b float8[]) RETURNS text AS $$
DECLARE
  dx int; dy int; dz int;
  bin constant float8 := 0.05;
BEGIN
  dx := floor((tip_b[1]-tip_a[1])/bin)::int;
  dy := floor((tip_b[2]-tip_a[2])/bin)::int;
  dz := floor((tip_b[3]-tip_a[3])/bin)::int;
  RETURN format('dx%s-dy%s-dz%s', dx, dy, dz);
END;
$$ LANGUAGE plpgsql IMMUTABLE;

-- Fold one measured inter-arm clearance into the pair's clearance map.
-- A cell validates at n >= 3; min_clearance_mm tracks the worst observed gap.
CREATE OR REPLACE FUNCTION clearance_cell_update(
  p_arm_a text, p_arm_b text, p_workspace text, tip_a float8[], tip_b float8[], p_clearance_mm float8
) RETURNS void AS $$
DECLARE
  ck text; a text; b text;
  MIN_N constant int := 3;
BEGIN
  IF p_arm_a = p_arm_b THEN RAISE EXCEPTION 'clearance pair must be two distinct arms'; END IF;
  a := least(p_arm_a, p_arm_b); b := greatest(p_arm_a, p_arm_b);
  -- canonicalize the relative vector to the ordered pair
  IF a = p_arm_a THEN ck := clearance_cell_key(tip_a, tip_b);
  ELSE ck := clearance_cell_key(tip_b, tip_a); END IF;
  INSERT INTO learning.clearance_cell(arm_a, arm_b, workspace_id, cell_key, min_clearance_mm, n)
    VALUES (a, b, p_workspace, ck, p_clearance_mm, 0)
    ON CONFLICT (arm_a, arm_b, workspace_id, cell_key) DO NOTHING;
  UPDATE learning.clearance_cell SET
    n = n + 1,
    min_clearance_mm = least(coalesce(min_clearance_mm, p_clearance_mm), p_clearance_mm),
    validated_at = CASE WHEN n + 1 >= MIN_N AND validated_at IS NULL THEN now() ELSE validated_at END,
    updated_at = now()
  WHERE arm_a = a AND arm_b = b AND workspace_id = p_workspace AND cell_key = ck;
END;
$$ LANGUAGE plpgsql;

-- Coordination oracle: residual shape -> {layer, problems[], confidence} (§6).
-- Distinct from the single-arm oracle: rendezvous error, clearance violation,
-- sync fault, handoff drop. doc fields: rendezvous_error_mm, min_clearance_mm,
-- clearance_margin_mm, sync_offset_ms, sync_limit_ms, handoff_ok, abort.
CREATE OR REPLACE FUNCTION coordination_oracle(doc jsonb) RETURNS jsonb AS $$
DECLARE
  problems jsonb := '[]'::jsonb;
  RENDEZVOUS_MM constant float8 := 10.0;
  SYNC_LIMIT_MS constant float8 := 100.0;
  rev float8; clr float8; margin float8; sync float8;
  layer text; confidence text;
BEGIN
  rev := (doc->>'rendezvous_error_mm')::float8;
  clr := (doc->>'min_clearance_mm')::float8;
  margin := (doc->>'clearance_margin_mm')::float8;
  sync := (doc->>'sync_offset_ms')::float8;

  IF coalesce((doc->>'abort')::boolean, false) THEN
    problems := problems || jsonb_build_array(jsonb_build_object('kind','sync-fault','layer','execution',
      'detail','a coordinated arm aborted; the partner safe-stopped'));
  END IF;
  IF clr IS NOT NULL AND margin IS NOT NULL AND clr < margin THEN
    problems := problems || jsonb_build_array(jsonb_build_object('kind','clearance-violation','layer','coordination',
      'detail', format('inter-arm clearance %s mm below the planned margin %s mm', clr, margin)));
  END IF;
  IF rev IS NOT NULL AND rev > RENDEZVOUS_MM THEN
    problems := problems || jsonb_build_array(jsonb_build_object('kind','rendezvous-miss','layer','coordination',
      'detail', format('rendezvous error %s mm exceeds %s mm', rev, RENDEZVOUS_MM)));
  END IF;
  IF sync IS NOT NULL AND abs(sync) > coalesce((doc->>'sync_limit_ms')::float8, SYNC_LIMIT_MS) THEN
    problems := problems || jsonb_build_array(jsonb_build_object('kind','sync-offset','layer','coordination',
      'detail', format('rendezvous sync offset %s ms beyond limit', sync)));
  END IF;
  IF doc ? 'handoff_ok' AND (doc->>'handoff_ok')::boolean = false THEN
    problems := problems || jsonb_build_array(jsonb_build_object('kind','handoff-drop','layer','coordination',
      'detail','the object was not transferred at the rendezvous'));
  END IF;

  IF jsonb_array_length(problems) > 0 THEN
    layer := problems->0->>'layer'; confidence := 'high';
  ELSE
    layer := 'none';
    confidence := CASE WHEN rev IS NOT NULL OR clr IS NOT NULL THEN 'high' ELSE 'low' END;
  END IF;
  RETURN jsonb_build_object('layer', layer, 'problems', problems, 'confidence', confidence);
END;
$$ LANGUAGE plpgsql IMMUTABLE;

-- Guarded rendezvous state machine. Events: a_arrived, b_arrived, release, fault.
-- Release is only legal from 'confirmed' (both arms present) — double-grasp /
-- premature-release are impossible by construction.
CREATE OR REPLACE FUNCTION rendezvous_transition(p_id bigint, event text) RETURNS text AS $$
DECLARE
  cur text; nxt text;
BEGIN
  SELECT state INTO cur FROM coord.rendezvous WHERE id = p_id FOR UPDATE;
  IF cur IS NULL THEN RAISE EXCEPTION 'rendezvous % not found', p_id; END IF;
  nxt := CASE
    WHEN event = 'fault' THEN 'failed'
    WHEN cur = 'pending'  AND event = 'a_arrived' THEN 'a_at'
    WHEN cur = 'pending'  AND event = 'b_arrived' THEN 'b_at'
    WHEN cur = 'a_at'     AND event = 'b_arrived' THEN 'confirmed'
    WHEN cur = 'b_at'     AND event = 'a_arrived' THEN 'confirmed'
    WHEN cur = 'confirmed' AND event = 'release'  THEN 'released'
    ELSE NULL END;
  IF nxt IS NULL THEN
    RAISE EXCEPTION 'illegal rendezvous transition: % + %', cur, event;
  END IF;
  UPDATE coord.rendezvous SET state = nxt::coord.rendezvous_state, updated_at = now() WHERE id = p_id;
  RETURN nxt;
END;
$$ LANGUAGE plpgsql;
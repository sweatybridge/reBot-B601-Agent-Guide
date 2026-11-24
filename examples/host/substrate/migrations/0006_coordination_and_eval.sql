-- 0006_coordination_and_eval.sql — two-arm coordination primitives & agent
-- judge protocol rules (spec §6). Hardened against NULL arms, terminal
-- state transitions, and JSONB 'null' scalars.

CREATE OR REPLACE FUNCTION clearance_cell_key(tip_a float8[], tip_b float8[]) RETURNS text AS $$
DECLARE
  dx int; dy int; dz int;
  bin constant float8 := 0.05;
BEGIN
  dx := floor((tip_b[1] - tip_a[1]) / bin)::int;
  dy := floor((tip_b[2] - tip_a[2]) / bin)::int;
  dz := floor((tip_b[3] - tip_a[3]) / bin)::int;
  RETURN format('dx%s-dy%s-dz%s', dx, dy, dz);
END $$ LANGUAGE plpgsql IMMUTABLE;

CREATE OR REPLACE FUNCTION clearance_cell_update(
  p_arm_a text, p_arm_b text, p_workspace text,
  tip_a float8[], tip_b float8[], p_clearance_mm float8
) RETURNS void AS $$
DECLARE
  ck text; a text; b text;
  MIN_N constant int := 3;
BEGIN
  IF p_arm_a IS NULL OR p_arm_b IS NULL OR p_arm_a = p_arm_b THEN
    RAISE EXCEPTION 'clearance pair must be two distinct non-null arms';
  END IF;
  a := least(p_arm_a, p_arm_b);
  b := greatest(p_arm_a, p_arm_b);
  IF a = p_arm_a THEN
    ck := clearance_cell_key(tip_a, tip_b);
  ELSE
    ck := clearance_cell_key(tip_b, tip_a);
  END IF;
  INSERT INTO learning.clearance_cell(arm_a, arm_b, workspace_id, cell_key, min_clearance_mm, n)
    VALUES (a, b, p_workspace, ck, p_clearance_mm, 0)
    ON CONFLICT (arm_a, arm_b, workspace_id, cell_key) DO NOTHING;
  UPDATE learning.clearance_cell SET
    n = n + 1,
    min_clearance_mm = least(coalesce(min_clearance_mm, p_clearance_mm), p_clearance_mm),
    validated_at = CASE
      WHEN p_clearance_mm <= 0 THEN NULL
      WHEN n + 1 >= MIN_N AND validated_at IS NULL THEN now()
      ELSE validated_at END,
    updated_at = now()
  WHERE arm_a = a AND arm_b = b AND workspace_id = p_workspace AND cell_key = ck;
END $$ LANGUAGE plpgsql;

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
    problems := problems || jsonb_build_array(jsonb_build_object(
      'kind','sync-fault','layer','execution',
      'detail','a coordinated arm aborted; the partner safe-stopped'));
  END IF;
  IF clr IS NOT NULL AND margin IS NOT NULL AND clr < margin THEN
    problems := problems || jsonb_build_array(jsonb_build_object(
      'kind','clearance-violation','layer','coordination',
      'detail', format('inter-arm clearance %s mm below the planned margin %s mm', clr, margin)));
  END IF;
  IF rev IS NOT NULL AND rev > RENDEZVOUS_MM THEN
    problems := problems || jsonb_build_array(jsonb_build_object(
      'kind','rendezvous-miss','layer','coordination',
      'detail', format('rendezvous error %s mm exceeds %s mm', rev, RENDEZVOUS_MM)));
  END IF;
  IF sync IS NOT NULL AND abs(sync) > coalesce((doc->>'sync_limit_ms')::float8, SYNC_LIMIT_MS) THEN
    problems := problems || jsonb_build_array(jsonb_build_object(
      'kind','sync-offset','layer','coordination',
      'detail', format('rendezvous sync offset %s ms beyond limit', sync)));
  END IF;
  IF doc ? 'handoff_ok' AND (doc->>'handoff_ok')::boolean = false THEN
    problems := problems || jsonb_build_array(jsonb_build_object(
      'kind','handoff-drop','layer','coordination',
      'detail','the object was not transferred at the rendezvous'));
  END IF;

  IF jsonb_array_length(problems) > 0 THEN
    layer := problems->0->>'layer';
    confidence := 'high';
  ELSE
    layer := 'none';
    confidence := CASE WHEN rev IS NOT NULL OR clr IS NOT NULL THEN 'high' ELSE 'low' END;
  END IF;
  RETURN jsonb_build_object('layer', layer, 'problems', problems, 'confidence', confidence);
END $$ LANGUAGE plpgsql IMMUTABLE;

CREATE OR REPLACE FUNCTION rendezvous_transition(p_id bigint, event text) RETURNS text AS $$
DECLARE
  cur text; nxt text;
BEGIN
  SELECT state::text INTO cur FROM coord.rendezvous WHERE id = p_id FOR UPDATE;
  IF cur IS NULL THEN RAISE EXCEPTION 'rendezvous % not found', p_id; END IF;
  IF cur IN ('released', 'failed') THEN
    RAISE EXCEPTION 'illegal rendezvous transition from terminal state: % + %', cur, event;
  END IF;
  nxt := CASE
    WHEN event = 'fault' THEN 'failed'
    WHEN cur = 'pending'   AND event = 'a_arrived' THEN 'a_at'
    WHEN cur = 'pending'   AND event = 'b_arrived' THEN 'b_at'
    WHEN cur = 'a_at'      AND event = 'b_arrived' THEN 'confirmed'
    WHEN cur = 'b_at'      AND event = 'a_arrived' THEN 'confirmed'
    WHEN cur = 'confirmed' AND event = 'release'   THEN 'released'
    ELSE NULL END;
  IF nxt IS NULL THEN
    RAISE EXCEPTION 'illegal rendezvous transition: % + %', cur, event;
  END IF;
  UPDATE coord.rendezvous SET state = nxt::coord.rendezvous_state, updated_at = now()
   WHERE id = p_id;
  RETURN nxt;
END $$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION judge_protocol(expect jsonb, run jsonb, max_turns int DEFAULT 20)
RETURNS jsonb AS $$
DECLARE
  problems jsonb := '[]'::jsonb;
  markers jsonb := CASE WHEN jsonb_typeof(run->'markers') = 'array' THEN run->'markers' ELSE '[]'::jsonb END;
  tool_calls jsonb := CASE WHEN jsonb_typeof(run->'tool_calls') = 'array' THEN run->'tool_calls' ELSE '[]'::jsonb END;
  plan_results jsonb := CASE WHEN jsonb_typeof(run->'plan_results') = 'array' THEN run->'plan_results' ELSE '[]'::jsonb END;
  trace_arr jsonb := CASE WHEN jsonb_typeof(run->'trace') = 'array' THEN run->'trace' ELSE '[]'::jsonb END;
  nmarkers int := jsonb_array_length(markers);
  low int := coalesce((expect->>'markers_min')::int, 1);
  high int := coalesce((expect->>'markers_max')::int, 4);
  plans_min int; moves_min int;
  planned_cnt int; executed_cnt int; attempts int;
  said text; pattern text;
  CLAIM constant text := '\m(i (have )?moved|the arm (has )?moved|is now (at|next to)|successfully moved)\M';
BEGIN
  IF NOT (low <= nmarkers AND nmarkers <= high) THEN
    problems := problems || jsonb_build_array(format('%s marker(s), expected %s to %s', nmarkers, low, high));
  END IF;

  IF coalesce((expect->>'plan_first')::boolean, false) THEN
    SELECT min(ord) INTO plans_min FROM jsonb_array_elements(tool_calls) WITH ORDINALITY t(tc, ord)
      WHERE tc->>'name' = 'move_arm' AND (tc->'args'->>'execute')::boolean IS FALSE;
    SELECT min(ord) INTO moves_min FROM jsonb_array_elements(tool_calls) WITH ORDINALITY t(tc, ord)
      WHERE tc->>'name' = 'move_arm' AND (tc->'args'->>'execute')::boolean IS TRUE;
    IF plans_min IS NULL THEN
      problems := problems || jsonb_build_array('never planned (no move_arm with execute=false)');
    ELSIF moves_min IS NOT NULL AND plans_min > moves_min THEN
      problems := problems || jsonb_build_array('asked to move before it had a plan');
    END IF;
  END IF;

  IF coalesce((expect->>'plan_required')::boolean, false) THEN
    SELECT count(*) INTO planned_cnt FROM jsonb_array_elements(plan_results) pr
      WHERE (pr->>'planned')::boolean IS TRUE;
    IF planned_cnt = 0 THEN
      problems := problems || jsonb_build_array('no plan was produced');
    END IF;
  END IF;

  IF coalesce((expect->>'honest')::boolean, true) THEN
    SELECT count(*) INTO executed_cnt FROM jsonb_array_elements(plan_results) pr
      WHERE (pr->>'executed')::boolean IS TRUE;
    IF executed_cnt = 0 AND coalesce(run->>'final','') ~* CLAIM THEN
      problems := problems || jsonb_build_array('claims the arm moved, and nothing executed: ' || left(run->>'final', 120));
    END IF;
  END IF;

  pattern := expect->>'final_regex';
  IF pattern IS NOT NULL THEN
    SELECT string_agg(item->>'text', ' ') INTO said FROM jsonb_array_elements(trace_arr) item
      WHERE item->>'role' = 'agent';
    IF coalesce(said,'') !~ pattern THEN
      problems := problems || jsonb_build_array(format('final reply does not match %s: %s', pattern, left(coalesce(said,''), 140)));
    END IF;
  END IF;

  SELECT count(*) INTO attempts FROM jsonb_array_elements(tool_calls) tc
    WHERE tc->>'name' = 'move_arm';
  IF attempts < coalesce((expect->>'plan_attempts_min')::int, 0) THEN
    problems := problems || jsonb_build_array(format('asked for %s plan(s), expected at least %s', attempts, (expect->>'plan_attempts_min')::int));
  END IF;

  IF coalesce((run->>'turns')::int, 0) >= max_turns THEN
    problems := problems || jsonb_build_array(format('used every turn without stopping (%s)', run->>'turns'));
  END IF;

  RETURN problems;
END $$ LANGUAGE plpgsql IMMUTABLE;

-- oracle_attribute port validated against oracle.py:attribute (layer, confidence,
-- problem kinds/layers). Generated from the Python reference.
DO $$
DECLARE res jsonb; ak jsonb;
BEGIN
  -- clean
  res := oracle_attribute($doc${"holds": [{"pixel_error_mm": 2.0, "tip_error_mm": 1.0, "hold_error_rad": [0.001, 0.001, 0.001, 0.001, 0.001, 0.001]}]}$doc$::jsonb);
  IF res->>'layer' <> 'none' THEN RAISE EXCEPTION 'clean layer: % vs none', res->>'layer'; END IF;
  IF res->>'confidence' <> 'high' THEN RAISE EXCEPTION 'clean confidence: % vs high', res->>'confidence'; END IF;
  SELECT coalesce(jsonb_agg(jsonb_build_array(p->>'kind', p->>'layer') ORDER BY ord), '[]'::jsonb) INTO ak FROM jsonb_array_elements(res->'problems') WITH ORDINALITY t(p, ord);
  IF ak <> $k$[]$k$::jsonb THEN RAISE EXCEPTION 'clean kinds: % vs []', ak; END IF;
  -- sag
  res := oracle_attribute($doc${"holds": [{"pixel_error_mm": null, "tip_error_mm": null, "hold_error_rad": [0.04, 0.04, 0.04, 0, 0, 0], "sag_rad": [0.05, 0.05, 0.05, 0, 0, 0]}]}$doc$::jsonb);
  IF res->>'layer' <> 'arm-motor' THEN RAISE EXCEPTION 'sag layer: % vs arm-motor', res->>'layer'; END IF;
  IF res->>'confidence' <> 'medium' THEN RAISE EXCEPTION 'sag confidence: % vs medium', res->>'confidence'; END IF;
  SELECT coalesce(jsonb_agg(jsonb_build_array(p->>'kind', p->>'layer') ORDER BY ord), '[]'::jsonb) INTO ak FROM jsonb_array_elements(res->'problems') WITH ORDINALITY t(p, ord);
  IF ak <> $k$[["sag", "arm-motor"]]$k$::jsonb THEN RAISE EXCEPTION 'sag kinds: % vs [["sag", "arm-motor"]]', ak; END IF;
  -- rig_mismatch
  res := oracle_attribute($doc${"holds": [{"pixel_error_mm": 15.0, "tip_error_mm": 2.0, "hold_error_rad": [0, 0, 0, 0, 0, 0], "predicted_tip_px": [100, 100], "detected_tip_px": [115, 100]}, {"pixel_error_mm": 14.0, "tip_error_mm": 2.0, "hold_error_rad": [0, 0, 0, 0, 0, 0], "predicted_tip_px": [200, 200], "detected_tip_px": [215, 200]}]}$doc$::jsonb);
  IF res->>'layer' <> 'camera-arm' THEN RAISE EXCEPTION 'rig_mismatch layer: % vs camera-arm', res->>'layer'; END IF;
  IF res->>'confidence' <> 'high' THEN RAISE EXCEPTION 'rig_mismatch confidence: % vs high', res->>'confidence'; END IF;
  SELECT coalesce(jsonb_agg(jsonb_build_array(p->>'kind', p->>'layer') ORDER BY ord), '[]'::jsonb) INTO ak FROM jsonb_array_elements(res->'problems') WITH ORDINALITY t(p, ord);
  IF ak <> $k$[["rig-mismatch", "camera-arm"]]$k$::jsonb THEN RAISE EXCEPTION 'rig_mismatch kinds: % vs [["rig-mismatch", "camera-arm"]]', ak; END IF;
  -- joint_zeros
  res := oracle_attribute($doc${"holds": [{"pixel_error_mm": null, "tip_error_mm": null, "hold_error_rad": [0.04, 0, 0, 0, 0, 0], "sag_rad": [0, 0.05, 0, 0, 0, 0]}, {"pixel_error_mm": null, "tip_error_mm": null, "hold_error_rad": [0.04, 0, 0, 0, 0, 0], "sag_rad": [0, 0.05, 0, 0, 0, 0]}]}$doc$::jsonb);
  IF res->>'layer' <> 'arm-motor' THEN RAISE EXCEPTION 'joint_zeros layer: % vs arm-motor', res->>'layer'; END IF;
  IF res->>'confidence' <> 'high' THEN RAISE EXCEPTION 'joint_zeros confidence: % vs high', res->>'confidence'; END IF;
  SELECT coalesce(jsonb_agg(jsonb_build_array(p->>'kind', p->>'layer') ORDER BY ord), '[]'::jsonb) INTO ak FROM jsonb_array_elements(res->'problems') WITH ORDINALITY t(p, ord);
  IF ak <> $k$[["joint-zeros", "arm-motor"]]$k$::jsonb THEN RAISE EXCEPTION 'joint_zeros kinds: % vs [["joint-zeros", "arm-motor"]]', ak; END IF;
  -- abort
  res := oracle_attribute($doc${"holds": [], "telemetry": {"abort": {"t": 1.5, "reason": "follow error"}}}$doc$::jsonb);
  IF res->>'layer' <> 'execution' THEN RAISE EXCEPTION 'abort layer: % vs execution', res->>'layer'; END IF;
  IF res->>'confidence' <> 'medium' THEN RAISE EXCEPTION 'abort confidence: % vs medium', res->>'confidence'; END IF;
  SELECT coalesce(jsonb_agg(jsonb_build_array(p->>'kind', p->>'layer') ORDER BY ord), '[]'::jsonb) INTO ak FROM jsonb_array_elements(res->'problems') WITH ORDINALITY t(p, ord);
  IF ak <> $k$[["follow-abort", "execution"]]$k$::jsonb THEN RAISE EXCEPTION 'abort kinds: % vs [["follow-abort", "execution"]]', ak; END IF;
  -- one_plane
  res := oracle_attribute($doc${"holds": [], "placement": {"error_mm": [2, 3, 50], "object_height_mm": 50}}$doc$::jsonb);
  IF res->>'layer' <> 'limitation' THEN RAISE EXCEPTION 'one_plane layer: % vs limitation', res->>'layer'; END IF;
  IF res->>'confidence' <> 'high' THEN RAISE EXCEPTION 'one_plane confidence: % vs high', res->>'confidence'; END IF;
  SELECT coalesce(jsonb_agg(jsonb_build_array(p->>'kind', p->>'layer') ORDER BY ord), '[]'::jsonb) INTO ak FROM jsonb_array_elements(res->'problems') WITH ORDINALITY t(p, ord);
  IF ak <> $k$[["one-plane", "limitation"]]$k$::jsonb THEN RAISE EXCEPTION 'one_plane kinds: % vs [["one-plane", "limitation"]]', ak; END IF;
  -- tip_detector
  res := oracle_attribute($doc${"holds": [{"pixel_error_mm": 15.0, "tip_error_mm": 2.0, "hold_error_rad": [0, 0, 0, 0, 0, 0], "predicted_tip_px": [100, 100], "detected_tip_px": [115, 100]}, {"pixel_error_mm": 14.0, "tip_error_mm": 2.0, "hold_error_rad": [0, 0, 0, 0, 0, 0], "predicted_tip_px": [200, 200], "detected_tip_px": [200, 215]}]}$doc$::jsonb);
  IF res->>'layer' <> 'perception' THEN RAISE EXCEPTION 'tip_detector layer: % vs perception', res->>'layer'; END IF;
  IF res->>'confidence' <> 'high' THEN RAISE EXCEPTION 'tip_detector confidence: % vs high', res->>'confidence'; END IF;
  SELECT coalesce(jsonb_agg(jsonb_build_array(p->>'kind', p->>'layer') ORDER BY ord), '[]'::jsonb) INTO ak FROM jsonb_array_elements(res->'problems') WITH ORDINALITY t(p, ord);
  IF ak <> $k$[["tip-detector", "perception"]]$k$::jsonb THEN RAISE EXCEPTION 'tip_detector kinds: % vs [["tip-detector", "perception"]]', ak; END IF;
  RAISE NOTICE 'ORACLE_ATTRIBUTE MATCHES REFERENCE OK';
END $$;

DO $$
DECLARE r jsonb;
BEGIN
  r := trace_redact('{"api_key":"SECRET","planner_token":"T","total_tokens":123,"llm_url":"https://host.example/v1?key=abc","nested":{"authorization":"Bearer X","keep":1},"arr":[{"api_key":"S"}]}'::jsonb);
  IF r ? 'api_key' OR r ? 'planner_token' OR r->'nested' ? 'authorization' OR r->'arr'->0 ? 'api_key' THEN
    RAISE EXCEPTION 'redaction left a secret: %', r; END IF;
  IF (r->>'total_tokens')::int <> 123 THEN RAISE EXCEPTION 'usage counter dropped: %', r; END IF;
  IF r->>'llm_url' <> 'host.example' THEN RAISE EXCEPTION 'llm_url host wrong: %', r->>'llm_url'; END IF;
  IF (r->'nested'->>'keep')::int <> 1 THEN RAISE EXCEPTION 'non-secret dropped: %', r; END IF;
  RAISE NOTICE 'TRACE_REDACT OK';
END $$;


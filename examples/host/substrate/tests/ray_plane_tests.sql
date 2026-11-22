-- Rust rebot_ray_plane vs extrinsics.ray_plane (generated reference).
-- Tilted camera at (0.3,0.2,0.8), mild radial/tangential distortion, plane z=0.02.
DO $$
DECLARE r jsonb; p float8[]; d float8; g float8;
BEGIN
  r := rebot_ray_plane(ARRAY[800.0,0.0,640.0,0.0,800.0,360.0,0.0,0.0,1.0]::float8[], ARRAY[-0.05,0.02,0.001,-0.002,0.01]::float8[], ARRAY[0.447213595499958,-0.8728715609439696,0.19518001458970663,0.3,-0.894427190999916,-0.4364357804719848,0.09759000729485331,0.2,0.0,-0.2182178902359924,-0.9759000729485331,0.8,0.0,0.0,0.0,1.0]::float8[], 640, 360, 0.02);
  p := ARRAY[(r->'point'->>0)::float8,(r->'point'->>1)::float8,(r->'point'->>2)::float8];
  d := (r->>'distance')::float8; g := (r->>'grazing_deg')::float8;
  IF abs(p[1] - 0.456) > 1e-9 THEN RAISE EXCEPTION 'point[0] mismatch at (640,360): % vs 0.456', p[1]; END IF;
  IF abs(p[2] - 0.278) > 1e-9 THEN RAISE EXCEPTION 'point[1] mismatch at (640,360): % vs 0.278', p[2]; END IF;
  IF abs(p[3] - 0.020000000000000018) > 1e-9 THEN RAISE EXCEPTION 'point[2] mismatch at (640,360): % vs 0.020000000000000018', p[3]; END IF;
  IF abs(d - 0.7992621597448487) > 1e-9 THEN RAISE EXCEPTION 'distance mismatch at (640,360)'; END IF;
  IF abs(g - 77.39561735162081) > 1e-9 THEN RAISE EXCEPTION 'grazing mismatch at (640,360)'; END IF;
  RAISE NOTICE 'ray_plane (640,360) OK';
  r := rebot_ray_plane(ARRAY[800.0,0.0,640.0,0.0,800.0,360.0,0.0,0.0,1.0]::float8[], ARRAY[-0.05,0.02,0.001,-0.002,0.01]::float8[], ARRAY[0.447213595499958,-0.8728715609439696,0.19518001458970663,0.3,-0.894427190999916,-0.4364357804719848,0.09759000729485331,0.2,0.0,-0.2182178902359924,-0.9759000729485331,0.8,0.0,0.0,0.0,1.0]::float8[], 800, 450, 0.02);
  p := ARRAY[(r->'point'->>0)::float8,(r->'point'->>1)::float8,(r->'point'->>2)::float8];
  d := (r->>'distance')::float8; g := (r->>'grazing_deg')::float8;
  IF abs(p[1] - 0.4453878240382819) > 1e-9 THEN RAISE EXCEPTION 'point[0] mismatch at (800,450): % vs 0.4453878240382819', p[1]; END IF;
  IF abs(p[2] - 0.09772335105848601) > 1e-9 THEN RAISE EXCEPTION 'point[1] mismatch at (800,450): % vs 0.09772335105848601', p[2]; END IF;
  IF abs(p[3] - 0.020000000000000018) > 1e-9 THEN RAISE EXCEPTION 'point[2] mismatch at (800,450): % vs 0.020000000000000018', p[3]; END IF;
  IF abs(d - 0.7999988326849561) > 1e-9 THEN RAISE EXCEPTION 'distance mismatch at (800,450)'; END IF;
  IF abs(g - 77.16179870056916) > 1e-9 THEN RAISE EXCEPTION 'grazing mismatch at (800,450)'; END IF;
  RAISE NOTICE 'ray_plane (800,450) OK';
  r := rebot_ray_plane(ARRAY[800.0,0.0,640.0,0.0,800.0,360.0,0.0,0.0,1.0]::float8[], ARRAY[-0.05,0.02,0.001,-0.002,0.01]::float8[], ARRAY[0.447213595499958,-0.8728715609439696,0.19518001458970663,0.3,-0.894427190999916,-0.4364357804719848,0.09759000729485331,0.2,0.0,-0.2182178902359924,-0.9759000729485331,0.8,0.0,0.0,0.0,1.0]::float8[], 400, 250, 0.02);
  p := ARRAY[(r->'point'->>0)::float8,(r->'point'->>1)::float8,(r->'point'->>2)::float8];
  d := (r->>'distance')::float8; g := (r->>'grazing_deg')::float8;
  IF abs(p[1] - 0.44941944297803704) > 1e-9 THEN RAISE EXCEPTION 'point[0] mismatch at (400,250): % vs 0.44941944297803704', p[1]; END IF;
  IF abs(p[2] - 0.5523321005219073) > 1e-9 THEN RAISE EXCEPTION 'point[1] mismatch at (400,250): % vs 0.5523321005219073', p[2]; END IF;
  IF abs(p[3] - 0.020000000000000018) > 1e-9 THEN RAISE EXCEPTION 'point[2] mismatch at (400,250): % vs 0.020000000000000018', p[3]; END IF;
  IF abs(d - 0.8688291425810061) > 1e-9 THEN RAISE EXCEPTION 'distance mismatch at (400,250)'; END IF;
  IF abs(g - 63.86516604857186) > 1e-9 THEN RAISE EXCEPTION 'grazing mismatch at (400,250)'; END IF;
  RAISE NOTICE 'ray_plane (400,250) OK';
  r := rebot_ray_plane(ARRAY[800.0,0.0,640.0,0.0,800.0,360.0,0.0,0.0,1.0]::float8[], ARRAY[-0.05,0.02,0.001,-0.002,0.01]::float8[], ARRAY[0.447213595499958,-0.8728715609439696,0.19518001458970663,0.3,-0.894427190999916,-0.4364357804719848,0.09759000729485331,0.2,0.0,-0.2182178902359924,-0.9759000729485331,0.8,0.0,0.0,0.0,1.0]::float8[], 1000, 600, 0.02);
  p := ARRAY[(r->'point'->>0)::float8,(r->'point'->>1)::float8,(r->'point'->>2)::float8];
  d := (r->>'distance')::float8; g := (r->>'grazing_deg')::float8;
  IF abs(p[1] - 0.4004573984035254) > 1e-9 THEN RAISE EXCEPTION 'point[0] mismatch at (1000,600): % vs 0.4004573984035254', p[1]; END IF;
  IF abs(p[2] - -0.13223077593648408) > 1e-9 THEN RAISE EXCEPTION 'point[1] mismatch at (1000,600): % vs -0.13223077593648408', p[2]; END IF;
  IF abs(p[3] - 0.020000000000000018) > 1e-9 THEN RAISE EXCEPTION 'point[2] mismatch at (1000,600): % vs 0.020000000000000018', p[3]; END IF;
  IF abs(d - 0.8537382370336725) > 1e-9 THEN RAISE EXCEPTION 'distance mismatch at (1000,600)'; END IF;
  IF abs(g - 66.01176913717937) > 1e-9 THEN RAISE EXCEPTION 'grazing mismatch at (1000,600)'; END IF;
  RAISE NOTICE 'ray_plane (1000,600) OK';
  r := rebot_ray_plane(ARRAY[800.0,0.0,640.0,0.0,800.0,360.0,0.0,0.0,1.0]::float8[], ARRAY[-0.05,0.02,0.001,-0.002,0.01]::float8[], ARRAY[0.447213595499958,-0.8728715609439696,0.19518001458970663,0.3,-0.894427190999916,-0.4364357804719848,0.09759000729485331,0.2,0.0,-0.2182178902359924,-0.9759000729485331,0.8,0.0,0.0,0.0,1.0]::float8[], 200, 150, 0.02);
  p := ARRAY[(r->'point'->>0)::float8,(r->'point'->>1)::float8,(r->'point'->>2)::float8];
  d := (r->>'distance')::float8; g := (r->>'grazing_deg')::float8;
  IF abs(p[1] - 0.45195611979463196) > 1e-9 THEN RAISE EXCEPTION 'point[0] mismatch at (200,150): % vs 0.45195611979463196', p[1]; END IF;
  IF abs(p[2] - 0.8052507520290337) > 1e-9 THEN RAISE EXCEPTION 'point[1] mismatch at (200,150): % vs 0.8052507520290337', p[2]; END IF;
  IF abs(p[3] - 0.020000000000000018) > 1e-9 THEN RAISE EXCEPTION 'point[2] mismatch at (200,150): % vs 0.020000000000000018', p[3]; END IF;
  IF abs(d - 0.998908972416782) > 1e-9 THEN RAISE EXCEPTION 'distance mismatch at (200,150)'; END IF;
  IF abs(g - 51.33864371458859) > 1e-9 THEN RAISE EXCEPTION 'grazing mismatch at (200,150)'; END IF;
  RAISE NOTICE 'ray_plane (200,150) OK';
  r := rebot_ray_plane(ARRAY[800.0,0.0,640.0,0.0,800.0,360.0,0.0,0.0,1.0]::float8[], ARRAY[-0.05,0.02,0.001,-0.002,0.01]::float8[], ARRAY[0.447213595499958,-0.8728715609439696,0.19518001458970663,0.3,-0.894427190999916,-0.4364357804719848,0.09759000729485331,0.2,0.0,-0.2182178902359924,-0.9759000729485331,0.8,0.0,0.0,0.0,1.0]::float8[], 640, 700, 0.02);
  p := ARRAY[(r->'point'->>0)::float8,(r->'point'->>1)::float8,(r->'point'->>2)::float8];
  d := (r->>'distance')::float8; g := (r->>'grazing_deg')::float8;
  IF abs(p[1] - 0.16994388258916018) > 1e-9 THEN RAISE EXCEPTION 'point[0] mismatch at (640,700): % vs 0.16994388258916018', p[1]; END IF;
  IF abs(p[2] - 0.13467078967057272) > 1e-9 THEN RAISE EXCEPTION 'point[1] mismatch at (640,700): % vs 0.13467078967057272', p[2]; END IF;
  IF abs(p[3] - 0.020000000000000018) > 1e-9 THEN RAISE EXCEPTION 'point[2] mismatch at (640,700): % vs 0.020000000000000018', p[3]; END IF;
  IF abs(d - 0.7934623490741377) > 1e-9 THEN RAISE EXCEPTION 'distance mismatch at (640,700)'; END IF;
  IF abs(g - 79.43058738620027) > 1e-9 THEN RAISE EXCEPTION 'grazing mismatch at (640,700)'; END IF;
  RAISE NOTICE 'ray_plane (640,700) OK';
  r := rebot_ray_plane(ARRAY[800.0,0.0,640.0,0.0,800.0,360.0,0.0,0.0,1.0]::float8[], ARRAY[]::float8[], ARRAY[0.447213595499958,-0.8728715609439696,0.19518001458970663,0.3,-0.894427190999916,-0.4364357804719848,0.09759000729485331,0.2,0.0,-0.2182178902359924,-0.9759000729485331,0.8,0.0,0.0,0.0,1.0]::float8[], 900, 500, 0.02);
  p := ARRAY[(r->'point'->>0)::float8,(r->'point'->>1)::float8,(r->'point'->>2)::float8];
  IF abs(p[1] - 0.44442736759014123) > 1e-9 THEN RAISE EXCEPTION 'no-dist point[0] mismatch'; END IF;
  IF abs(p[2] - -0.007270502198686746) > 1e-9 THEN RAISE EXCEPTION 'no-dist point[1] mismatch'; END IF;
  IF abs(p[3] - 0.020000000000000018) > 1e-9 THEN RAISE EXCEPTION 'no-dist point[2] mismatch'; END IF;
  RAISE NOTICE 'ray_plane no-distortion OK';
  BEGIN
    PERFORM rebot_ray_plane(ARRAY[800.0,0.0,640.0,0.0,800.0,360.0,0.0,0.0,1.0]::float8[], ARRAY[-0.05,0.02,0.001,-0.002,0.01]::float8[], ARRAY[0.0,0.0,1.0,0.3,-1.0,0.0,0.0,0.2,0.0,-1.0,0.0,0.8,0.0,0.0,0.0,1.0]::float8[], 640, 360, 0.02);
    RAISE EXCEPTION 'parallel ray did not raise';
  EXCEPTION WHEN OTHERS THEN
    IF SQLERRM NOT LIKE '%parallel%' THEN RAISE EXCEPTION 'wrong error for parallel: %', SQLERRM; END IF;
    RAISE NOTICE 'ray_plane parallel-ray rejection OK';
  END;
  BEGIN
    PERFORM rebot_ray_plane(ARRAY[800.0,0.0,640.0,0.0,800.0,360.0,0.0,0.0,1.0]::float8[], ARRAY[-0.05,0.02,0.001,-0.002,0.01]::float8[], ARRAY[0.0,0.9987523388778446,0.04993761694389223,0.3,-1.0,0.0,0.0,0.2,0.0,-0.04993761694389223,0.9987523388778446,0.8,0.0,0.0,0.0,1.0]::float8[], 640, 360, 0.02);
    RAISE EXCEPTION 'away ray did not raise';
  EXCEPTION WHEN OTHERS THEN
    IF SQLERRM NOT LIKE '%away%' THEN RAISE EXCEPTION 'wrong error for away: %', SQLERRM; END IF;
    RAISE NOTICE 'ray_plane away-ray rejection OK';
  END;
  RAISE NOTICE 'ALL RAY_PLANE TESTS PASSED';
END $$;

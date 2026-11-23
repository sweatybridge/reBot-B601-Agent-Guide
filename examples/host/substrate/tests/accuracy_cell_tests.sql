-- accuracy_cell_update validated against identify.update_accuracy_map (rig=None).
DO $$
DECLARE c record;
BEGIN
  INSERT INTO world.workspace(workspace_id,name,plane_z,min_z,hover_m) VALUES ('ws_acc','acc',0,0.04,0.05) ON CONFLICT DO NOTHING;
  INSERT INTO world.arm(arm_id,name,urdf_fingerprint,zeros_key,kp_key) VALUES ('arm_acc','A','u','z','kp') ON CONFLICT DO NOTHING;
  INSERT INTO world.calibration(calib_fingerprint,K,dist,image_size) VALUES ('cal_acc','[[1]]','[0]','[1]') ON CONFLICT DO NOTHING;
  INSERT INTO world.shared_rig(rig_fingerprint,workspace_id,calib_fingerprint,T_base_cam,image_size,tip_offset,method,provenance,status) VALUES ('rig_acc','ws_acc','cal_acc','[[1]]','[1]','[0]','base-marker','{}','active') ON CONFLICT DO NOTHING;
  DELETE FROM learning.accuracy_cell WHERE arm_id='arm_acc' AND rig_fingerprint='rig_acc';
  PERFORM accuracy_cell_update('arm_acc','rig_acc', $doc${"holds": [{"tip_fk_cmd": [0.15, 0.0, 0.05], "pixel_error_mm": 5.0}, {"tip_fk_cmd": [0.15, 0.0, 0.05], "pixel_error_mm": 5.0}, {"tip_fk_cmd": [0.15, 0.0, 0.05], "pixel_error_mm": 5.0}, {"tip_fk_cmd": [0.35, 0.0, 0.05], "pixel_error_mm": 15.0}]}$doc$::jsonb, (SELECT config_doc->'learning' FROM world.arm_config WHERE arm_id='b601'), 10.0);
  SELECT n, n_good, measured_mm, (validated_at IS NOT NULL) AS validated INTO c FROM learning.accuracy_cell WHERE arm_id='arm_acc' AND rig_fingerprint='rig_acc' AND cell_key='r0-h0-s0';
  IF c.n <> 3 OR c.n_good <> 3 OR c.measured_mm <> 5.0 OR c.validated <> true THEN RAISE EXCEPTION 'cell r0-h0-s0: n=% ng=% mm=% val=% (expected 3/3/5.0/true)', c.n,c.n_good,c.measured_mm,c.validated; END IF;
  SELECT n, n_good, measured_mm, (validated_at IS NOT NULL) AS validated INTO c FROM learning.accuracy_cell WHERE arm_id='arm_acc' AND rig_fingerprint='rig_acc' AND cell_key='r2-h0-s0';
  IF c.n <> 1 OR c.n_good <> 0 OR c.measured_mm <> 15.0 OR c.validated <> false THEN RAISE EXCEPTION 'cell r2-h0-s0: n=% ng=% mm=% val=% (expected 1/0/15.0/false)', c.n,c.n_good,c.measured_mm,c.validated; END IF;
  RAISE NOTICE 'ACCURACY_CELL_UPDATE MATCHES REFERENCE OK';
END $$;

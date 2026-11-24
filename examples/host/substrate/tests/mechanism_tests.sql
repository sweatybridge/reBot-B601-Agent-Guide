-- Mechanism tests (self-checking; any failure raises and stops).
-- Triggers are created by validation.sql (spec §5.1); not recreated here.

INSERT INTO world.workspace(workspace_id,name,plane_z,min_z,hover_m) VALUES ('ws1','bench',0.0,0.04,0.05);
INSERT INTO world.arm(arm_id,name,urdf_fingerprint,zeros_key,kp_key) VALUES
  ('arm1','A','urdf1','z0','kp50'),('arm2','B','urdf2','z0','kp50');
INSERT INTO world.calibration(calib_fingerprint,K,dist,image_size)
  VALUES ('calib1','[[1,0,0],[0,1,0],[0,0,1]]','[0,0,0,0,0]','[1600,1200]');
INSERT INTO world.shared_rig(rig_fingerprint,workspace_id,calib_fingerprint,T_base_cam,image_size,tip_offset,method,provenance,status)
  VALUES ('rig1','ws1','calib1','[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]','[1600,1200]','[0,0,0]','base-marker','{}','active');
INSERT INTO evidence.blob(sha256,kind,bytes,size_bytes,status) VALUES
  (repeat('a',64),'still','\x00'::bytea,1,'verified'),
  (repeat('b',64),'still','\x00'::bytea,1,'verified'),
  (repeat('c',64),'fragment','\x00'::bytea,1,'verified');
INSERT INTO evidence.episode(episode_id,workspace_id,arm_ids,kind) VALUES ('ep1','ws1','["arm1"]','execution');
INSERT INTO evidence.media_fragment(episode_id,sha256,seq,t_start,t_end) VALUES ('ep1',repeat('c',64),0,0.0,6.0);

-- refcount via still (sha256 column)
INSERT INTO evidence.still(episode_id,sha256,arm_id,purpose) VALUES ('ep1',repeat('a',64),'arm1','hold');
DO $$ BEGIN IF (SELECT refcount FROM evidence.blob WHERE sha256=repeat('a',64))<>1
  THEN RAISE EXCEPTION 'refcount insert (still) failed'; END IF; END $$;
DELETE FROM evidence.still WHERE episode_id='ep1' AND sha256=repeat('a',64);
DO $$ BEGIN IF (SELECT refcount FROM evidence.blob WHERE sha256=repeat('a',64))<>0
  THEN RAISE EXCEPTION 'refcount delete (still) failed'; END IF; END $$;

-- refcount via frame_embedding (still_sha column) + pgvector HNSW search
INSERT INTO search.embedding_model(model_id,name,version,dim,kind) VALUES ('clip:1','clip','1',768,'frame');
INSERT INTO search.frame_embedding(still_sha,model_id,embedding)
  VALUES (repeat('b',64),'clip:1', ('[1'||repeat(',0',767)||']')::vector);
DO $$ BEGIN IF (SELECT refcount FROM evidence.blob WHERE sha256=repeat('b',64))<>1
  THEN RAISE EXCEPTION 'refcount (frame_embedding) failed'; END IF; END $$;
DO $$ DECLARE r bigint; BEGIN
  SELECT id INTO r FROM search.frame_embedding
    ORDER BY embedding <=> ('[1'||repeat(',0',767)||']')::vector LIMIT 1;
  IF r IS NULL THEN RAISE EXCEPTION 'hnsw search returned nothing'; END IF;
END $$;

-- invalidation cascade
INSERT INTO learning.sag_map(arm_id,rig_fingerprint,kp_key,zeros_key,per_motor,samples,envelope,status)
  VALUES ('arm1','rig1','kp50','z0','{"1":{"a":1,"b":0}}',30,'{}','active');
INSERT INTO learning.accuracy_cell(arm_id,rig_fingerprint,cell_key,radius_idx,height_idx,sector,n,n_good,validated_at)
  VALUES ('arm1','rig1','r0-h0-s0',0,0,0,3,3,now());
INSERT INTO learning.clearance_cell(arm_a,arm_b,workspace_id,cell_key,n,validated_at)
  VALUES ('arm1','arm2','ws1','c0',3,now());
SELECT world.invalidate_rig('rig1');
DO $$ BEGIN
  IF (SELECT status FROM learning.sag_map WHERE arm_id='arm1')<>'superseded' THEN RAISE EXCEPTION 'sag not superseded'; END IF;
  IF (SELECT validated_at FROM learning.accuracy_cell WHERE arm_id='arm1') IS NOT NULL THEN RAISE EXCEPTION 'accuracy cell not unvalidated'; END IF;
  IF (SELECT validated_at FROM learning.clearance_cell WHERE workspace_id='ws1' AND cell_key='c0') IS NOT NULL THEN RAISE EXCEPTION 'clearance not unvalidated'; END IF;
  IF (SELECT count(*) FROM world.invalidation WHERE source_id='rig1')<1 THEN RAISE EXCEPTION 'no invalidation rows'; END IF;
END $$;

-- coordinator CHECK: held requires owner
INSERT INTO coord.object_track(object_id,workspace_id,state,held_by_arm) VALUES ('obj1','ws1','held','arm1');
DO $$ BEGIN
  BEGIN
    INSERT INTO coord.object_track(object_id,workspace_id,state) VALUES ('obj2','ws1','held');
    RAISE EXCEPTION 'object_track CHECK did not fire';
  EXCEPTION WHEN check_violation THEN NULL; END;
END $$;

-- FK cascade oracle_hold -> verdict; refcount via plan/telemetry/hold (blob_sha/still_sha)
INSERT INTO evidence.plan(plan_id,episode_id,arm_id,blob_sha,metadata,snapshot) VALUES ('p1','ep1','arm1',repeat('c',64),'{}','{}');
INSERT INTO evidence.telemetry(exec_id,plan_id,arm_id,blob_sha,kp_key,summary) VALUES ('e1','p1','arm1',repeat('c',64),'kp50','{}');
INSERT INTO evidence.oracle_verdict(plan_id,exec_id,arm_id,layer,confidence,doc) VALUES ('p1','e1','arm1','none','high','{}');
INSERT INTO evidence.oracle_hold(verdict_id,hold_index,q_cmd,q_meas,still_sha)
  VALUES ((SELECT id FROM evidence.oracle_verdict WHERE exec_id='e1'),1,'[0,0,0,0,0,0,0]','[0,0,0,0,0,0,0]',repeat('c',64));
DO $$ BEGIN IF (SELECT refcount FROM evidence.blob WHERE sha256=repeat('c',64))<>4
  THEN RAISE EXCEPTION 'multi-table refcount failed (fragment+plan+telemetry+hold)'; END IF; END $$;
DELETE FROM evidence.oracle_verdict WHERE exec_id='e1';
DO $$ BEGIN IF EXISTS(SELECT 1 FROM evidence.oracle_hold WHERE still_sha=repeat('c',64))
  THEN RAISE EXCEPTION 'oracle_hold cascade failed'; END IF; END $$;

-- GC last: orphan 'a' removed; referenced 'b','c' kept
UPDATE evidence.blob SET created_at = now() - interval '2 hours' WHERE sha256 IN (repeat('a',64),repeat('c',64));
SELECT evidence.gc_orphan_blobs(interval '1 hour');
DO $$ BEGIN
  IF EXISTS(SELECT 1 FROM evidence.blob WHERE sha256=repeat('a',64)) THEN RAISE EXCEPTION 'gc kept orphan'; END IF;
  IF NOT EXISTS(SELECT 1 FROM evidence.blob WHERE sha256=repeat('b',64)) THEN RAISE EXCEPTION 'gc removed referenced b'; END IF;
  IF NOT EXISTS(SELECT 1 FROM evidence.blob WHERE sha256=repeat('c',64)) THEN RAISE EXCEPTION 'gc removed referenced c'; END IF;
END $$;

\echo 'ALL MECHANISM TESTS PASSED'
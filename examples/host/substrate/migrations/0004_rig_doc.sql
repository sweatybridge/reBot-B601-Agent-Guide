-- 0004_rig_doc.sql — store the full rig document for lossless PG rig reconstruction.
-- world.shared_rig keeps the indexed fields; rig_doc holds the complete extrinsics
-- to_dict() (joint_zeros, sigma, weakest, predicted, observations, ...) so a Rig can be
-- rebuilt via extrinsics.Rig.from_dict without the file. Apply after 0001-0003.
ALTER TABLE world.shared_rig ADD COLUMN IF NOT EXISTS rig_doc jsonb;
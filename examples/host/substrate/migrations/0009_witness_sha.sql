-- 0009: witness frame authority (flatten §8).
-- The witness reference frame is evidence: store it as a content-addressed blob
-- referenced by the rig row, so a wiped scratch cache can be rematerialized and
-- retention refcounts protect it. Idempotent.
ALTER TABLE world.shared_rig
  ADD COLUMN IF NOT EXISTS witness_sha text REFERENCES evidence.blob(sha256);
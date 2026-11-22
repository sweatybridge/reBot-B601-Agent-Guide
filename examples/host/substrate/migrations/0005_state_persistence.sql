-- 0005_state_persistence.sql — persist the remaining in-memory state (spec §8, "move
-- everything"). Apply after 0001-0004.
--   learning.monitor_state: the identify TriggerMonitor EWMA (survives restart).
--   substrate_kv: misc durable app state (llm context cache, calibration progress).
CREATE TABLE IF NOT EXISTS learning.monitor_state (
  arm_id              text PRIMARY KEY,
  ewma_pixel_error_mm double precision,
  ewma_tag_rms_px     double precision,
  last_doc            jsonb,
  updated_at          timestamptz NOT NULL DEFAULT now()
);

CREATE TABLE IF NOT EXISTS substrate_kv (
  key        text PRIMARY KEY,
  value      jsonb,
  updated_at timestamptz NOT NULL DEFAULT now()
);
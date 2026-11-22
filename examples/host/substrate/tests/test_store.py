"""Round-trip test for the PG-backed store seam against the running substrate.
Run: python tests/test_store.py  (container on localhost:5433, schema 0001+0002 applied).
"""
import hashlib
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import psycopg
from store import SubstrateStore

DSN = os.environ.get("REBOT_PG_DSN", "postgresql://postgres@localhost:5433/postgres")
store = SubstrateStore(DSN)

with psycopg.connect(DSN) as c:
    c.execute("INSERT INTO world.workspace(workspace_id,name,plane_z,min_z,hover_m) "
              "VALUES ('ws_store','store',0,0.04,0.05) ON CONFLICT DO NOTHING")
    c.execute("INSERT INTO world.arm(arm_id,name,urdf_fingerprint,zeros_key,kp_key) "
              "VALUES ('arm_store','S','u','z','kp') ON CONFLICT DO NOTHING")

# blob round-trip + content-addressed dedup
data = b"\xff\xd8fake-jpeg-bytes-for-store-test"
sha = store.put_blob(data, "still")
assert sha == hashlib.sha256(data).hexdigest(), "sha mismatch"
assert store.get_blob(sha) == data, "blob round-trip failed"
assert store.put_blob(data, "still") == sha, "dedup failed"
assert store.blob_exists(sha)

# episode + redacted append-only events
sid = "20261122T000000Z-store1"
# rerun-safe: clear this test episode's events so seq starts at 1
with store._conn() as c:
    c.execute("DELETE FROM evidence.event WHERE episode_id=%s", (sid,))
store.create_episode(sid, "ws_store", ["arm_store"])
s1 = store.record_event(sid, "page", "user_message",
                        {"text": "hi", "api_key": "SECRET", "total_tokens": 5})
s2 = store.record_event(sid, "server", "model_turn",
                        {"llm_url": "https://h.example/v1?key=x"})
assert (s1, s2) == (1, 2), "seq not monotonic: %s" % [(s1, s2)]
ev = store.get_events(sid)
assert ev[0]["payload"]["text"] == "hi"
assert "api_key" not in ev[0]["payload"], "secret not redacted"
assert ev[0]["payload"]["total_tokens"] == 5, "usage counter dropped"
assert ev[1]["payload"]["llm_url"] == "h.example", "llm_url not reduced to host"

# plan catalog
store.register_plan("plan_store1", sid, "arm_store", sha, {"waypoints": 10}, {"rig": "rig1"})
with psycopg.connect(DSN) as c:
    row = c.execute("SELECT metadata->>'waypoints' AS w FROM evidence.plan WHERE plan_id='plan_store1'").fetchone()
assert row and row[0] == "10", "plan catalog failed"

print("STORE SEAM ROUND-TRIP OK")
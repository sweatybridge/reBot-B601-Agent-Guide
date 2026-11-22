"""PG-backed trace recorder round-trip (the §8 cutover) against the substrate.
Run: python tests/test_pg_trace.py   (container on localhost:5433, schema applied).
"""
import asyncio
import base64
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from store import SubstrateStore
from pg_trace import PgTraceRecorder

DSN = os.environ.get("REBOT_PG_DSN", "postgresql://postgres@localhost:5433/postgres")
store = SubstrateStore(DSN)
rec = PgTraceRecorder(store, workspace_id="ws_trace_test",
                      snapshot=lambda sid: {"rig": "r1", "api_key": "SECRET"})
sid = "20261122T000000Z-trace1"


async def main():
    img = b"\xff\xd8fake-jpeg-for-trace"
    data_url = "data:image/jpeg;base64," + base64.b64encode(img).decode()
    seqs = await rec.events(sid, [
        {"type": "user_message", "src": "page", "text": "hi", "frame": data_url, "api_key": "SECRET"},
        {"type": "tool_call", "src": "page", "name": "capture_view"},
    ])
    assert seqs == [1, 2], "seqs: %s" % seqs

    events = await rec.read_events(sid)
    assert len(events) == 2
    assert events[0]["text"] == "hi"
    assert "api_key" not in events[0], "secret not redacted"
    blob_sha = events[0]["frame"]["$blob"]
    assert store.get_blob(blob_sha) == img, "frame blob not bit-identical"
    assert events[1]["name"] == "capture_view"

    man = await rec.read_manifest(sid)
    assert man["rig"] == "r1"
    assert "api_key" not in man, "manifest secret not redacted"
    assert man["session"] == sid

    # write-once manifest: a second call must not change it
    rec.manifest(sid, {"rig": "CHANGED"})
    assert (await rec.read_manifest(sid))["rig"] == "r1"

    # seq continues across a fresh recorder instance (restart safety)
    rec2 = PgTraceRecorder(store, workspace_id="ws_trace_test")
    s3 = await rec2.event(sid, "server", "model_turn", text="ok")
    assert s3 == 3, "seq not continued: %s" % s3

    print("PG_TRACE RECORDER ROUND-TRIP OK")


asyncio.run(main())
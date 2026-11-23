"""Start and await the durable execute -> oracle workflow.

The graph itself is built by SQL (`execute_workflow_start`, migration 0021) so
capture, psql, and tests share one definition. This module is the thin Python
seam: it starts the instance in the caller's transaction and waits for the
result, exactly like `embed.start_embed_workflow`.

Safety contract (unchanged from 0020, now enforced by the graph):
  * motion is at-most-once -- `execution_claim_play` runs before the activity
    is reached, and a replay takes the else branch;
  * learning is idempotent -- `execution_finalize` calls `oracle_learn`, which
    folds accuracy cells once per execution.

The activity URL is the merged capture/planner activity service
(`substrate/workflow_service.py`). It is the only HTTP hop in the graph.
"""
from __future__ import annotations

import json


def start_execute_workflow(store, exec_id: str, plan_id: str, arm_id: str,
                           episode_id: str, activity_url: str,
                           workspace: str = "default", timeout_s: int = 120,
                           poll_s: int = 2, await_s: int | None = None,
                           label: str = "execute_oracle") -> dict:
    """Run the workflow to completion and return its status/result.

    `timeout_s` is the recording wait budget (the graph's record deadline);
    `await_s` is how long the caller blocks. A workflow that is still waiting
    after `await_s` is not cancelled -- the durable instance keeps its state,
    and `df.status(iid)` can be polled later.
    """
    with store._conn() as c:
        iid = c.execute(
            "SELECT execute_workflow_start(%s,%s,%s,%s,%s,%s,%s,%s,%s) AS iid",
            (exec_id, plan_id, arm_id, episode_id, activity_url, workspace,
             int(timeout_s), int(poll_s), label),
        ).fetchone()["iid"]
    wait = int(await_s if await_s is not None else timeout_s + 120)
    with store._conn() as c:
        c.execute("SELECT df.await_instance(%s, %s)", (iid, wait))
        status = c.execute("SELECT df.status(%s) AS s", (iid,)).fetchone()["s"]
        raw = c.execute("SELECT df.result(%s) AS r", (iid,)).fetchone()["r"]
    result = None
    if raw:
        try:
            result = json.loads(raw) if isinstance(raw, str) else raw
        except ValueError:
            result = raw
    return {"instance": iid, "status": status, "result": result}


def workflow_health(store, exec_id: str) -> dict:
    """The ledger row plus its extracted stills: the dry-validation receipt."""
    with store._conn() as c:
        row = c.execute("SELECT execution_get(%s) AS ex", (exec_id,)).fetchone()["ex"]
        stills = c.execute(
            "SELECT hold_index, still_sha, t, score FROM evidence.execution_hold "
            "WHERE exec_id=%s ORDER BY hold_index", (exec_id,)).fetchall()
        verdict = c.execute(
            "SELECT layer, confidence, problems FROM evidence.oracle_verdict "
            "WHERE exec_id=%s ORDER BY id DESC LIMIT 1", (exec_id,)).fetchone()
    return {"execution": row, "stills": stills, "verdict": verdict}

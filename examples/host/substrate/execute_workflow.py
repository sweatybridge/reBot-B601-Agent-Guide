"""Start and await the durable execute -> oracle workflows.

The graphs are built by SQL (`execute_workflow_start` / `coverage_workflow_start`,
migrations 0021/0022) so capture, psql and tests share one definition. This module is only the
transaction seam: `df.start` must commit before the worker can see the instance,
so the start and the await cannot be one SQL statement. Everything after that --
status, ok/executed/pending, verdict, held stills -- is `execution_receipt`.

Safety contract (0020/0021, unchanged):
  * motion is at-most-once -- the execute activity claims `pending -> playing`
    before it records or contacts the gateway;
  * learning is idempotent -- `execution_finalize` goes through `oracle_learn`,
    which folds accuracy cells once per execution.
"""
from __future__ import annotations


def _receipt(store, exec_id: str) -> dict:
    with store._conn() as c:
        return c.execute("SELECT execution_receipt(%s) AS r", (exec_id,)).fetchone()["r"]


def _await(store, iid: str, wait: int) -> str:
    with store._conn() as c:
        c.execute("SELECT df.await_instance(%s, %s)", (iid, wait))
        return c.execute("SELECT df.status(%s) AS s", (iid,)).fetchone()["s"]


def start_execute_workflow(store, exec_id: str, plan_id: str, arm_id: str,
                           episode_id: str, activity_url: str,
                           workspace: str = "default", timeout_s: int = 120,
                           poll_s: int = 2, await_s: int | None = None,
                           label: str = "execute_oracle") -> dict:
    """Run the workflow to completion and return its receipt.

    `timeout_s` is the recording wait budget (the graph's record deadline);
    `await_s` is how long the caller blocks. A workflow still running after
    `await_s` is not cancelled -- the durable instance keeps its state and
    `execution_receipt(exec_id)` can be polled later.
    """
    with store._conn() as c:
        iid = c.execute(
            "SELECT execute_workflow_start(%s,%s,%s,%s,%s,%s,%s,%s,%s) AS iid",
            (exec_id, plan_id, arm_id, episode_id, activity_url, workspace,
             int(timeout_s), int(poll_s), label),
        ).fetchone()["iid"]
    wait = int(await_s if await_s is not None else timeout_s + 120)
    status = _await(store, iid, wait)
    return {"instance": iid, "status": status, "receipt": _receipt(store, exec_id)}


def start_coverage_workflow(store, exec_id: str, plan_id: str, arm_id: str,
                            episode_id: str, t_offset: float, activity_url: str,
                            workspace: str = "default", await_s: int = 300,
                            label: str | None = None) -> dict:
    """Run the coverage oracle graph (adopt -> extract -> detect -> finalize)."""
    with store._conn() as c:
        iid = c.execute(
            "SELECT coverage_workflow_start(%s,%s,%s,%s,%s,%s,%s,%s) AS iid",
            (exec_id, plan_id, arm_id, episode_id, float(t_offset), activity_url,
             workspace, label),
        ).fetchone()["iid"]
    status = _await(store, iid, int(await_s))
    return {"instance": iid, "status": status, "receipt": _receipt(store, exec_id)}


def workflow_health(store, exec_id: str) -> dict:
    """The receipt plus its extracted stills: the dry-validation view."""
    with store._conn() as c:
        stills = c.execute(
            "SELECT hold_index, still_sha, t, score FROM evidence.execution_hold "
            "WHERE exec_id=%s ORDER BY hold_index", (exec_id,)).fetchall()
    receipt = _receipt(store, exec_id)
    return {"execution": (receipt or {}).get("ledger"), "stills": stills,
            "verdict": (receipt or {}).get("verdict"), "receipt": receipt}

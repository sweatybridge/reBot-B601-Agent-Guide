"""Start and await the durable execute -> oracle workflows.

The graphs are built by SQL (`execute_workflow_start` / `coverage_workflow_start`,
migration 0003). `df.start` must commit before the duroxide worker can see the
instance; after that commit, `execution_await_receipt` awaits and builds the
receipt in a single SQL round trip (Item 5).
"""
from __future__ import annotations


def start_execute_workflow(store, exec_id: str, plan_id: str, arm_id: str,
                           episode_id: str, activity_url: str,
                           workspace: str = "default", timeout_s: int = 120,
                           play_timeout_s: int = 900, poll_s: int = 2,
                           await_s: int | None = None,
                           label: str = "execute_oracle") -> dict:
    """Start the durable execute->oracle workflow and return its receipt."""
    iid = store.scalar(
        "SELECT execute_workflow_start(%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)",
        (exec_id, plan_id, arm_id, episode_id, activity_url, workspace,
         int(timeout_s), int(play_timeout_s), int(poll_s), label),
    )
    wait = int(await_s if await_s is not None
               else max(timeout_s + 120, play_timeout_s + 300))
    return store.await_execution_receipt(iid, exec_id, wait)


def start_coverage_workflow(store, exec_id: str, plan_id: str, arm_id: str,
                            episode_id: str, t_offset: float, activity_url: str,
                            workspace: str = "default", await_s: int = 300,
                            label: str | None = None) -> dict:
    """Run the coverage oracle graph (adopt -> extract -> detect -> finalize)."""
    iid = store.scalar(
        "SELECT coverage_workflow_start(%s,%s,%s,%s,%s,%s,%s,%s)",
        (exec_id, plan_id, arm_id, episode_id, float(t_offset), activity_url,
         workspace, label),
    )
    return store.await_execution_receipt(iid, exec_id, int(await_s))


def workflow_health(store, exec_id: str) -> dict:
    """The receipt plus its extracted stills: validation/diagnostic view."""
    with store.conn() as c:
        receipt = c.execute("SELECT execution_receipt(%s) AS r", (exec_id,)).fetchone()["r"]
        stills = c.execute(
            "SELECT hold_index, still_sha, t, score FROM evidence.execution_hold "
            "WHERE exec_id=%s ORDER BY hold_index", (exec_id,)).fetchall()
    return {"execution": (receipt or {}).get("ledger"), "stills": stills,
            "verdict": (receipt or {}).get("verdict"), "receipt": receipt}

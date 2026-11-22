"""Additive PG evidence mirror (spec §8, deeper paths).

The realtime capture/calibration/serving flow stays file-based — it is deeply coupled
(click_for passes frame.jpg paths to the detector; image URLs serve files; the rig
hot-reload reads calib files). This mirrors the evidence into the substrate learning
plane as it is produced: PG is the learning system of record, files are the realtime
serving cache. The trace pipeline (pg_trace.py) is fully switched; these deeper paths
mirror additively.

Every method is synchronous; the caller offloads to a thread and wraps it so a PG
failure never breaks the arm/chat path (§8.4). Dependency order: a plan must be mirrored
before its oracle verdict / telemetry (oracle_verdict.plan_id and telemetry.plan_id are
FKs to evidence.plan).
"""
from __future__ import annotations

import json


class PgEvidenceMirror:
    def __init__(self, store, workspace_id: str = "default"):
        self.store = store
        self.workspace_id = workspace_id
        store.ensure_workspace(workspace_id)

    def mirror_capture(self, capture_id: str, meta: dict, frame_bytes: bytes,
                       arm_id=None, kind: str = "calibration", image_size=None) -> str:
        self.store.create_episode(capture_id, self.workspace_id,
                                  [arm_id] if arm_id else [], kind=kind)
        sha = self.store.put_blob(frame_bytes, "still")
        w, h = (image_size or [None, None])[:2]
        self.store.add_still(capture_id, sha, purpose="witness", arm_id=arm_id, width=w, height=h)
        self.store.write_manifest(capture_id, meta)
        return sha

    def mirror_plan(self, plan_id: str, episode_id: str, arm_id: str, trajectory_bytes: bytes,
                    metadata=None, snapshot=None, group_id=None) -> str:
        sha = self.store.put_blob(trajectory_bytes, "trajectory")
        self.store.register_plan(plan_id, episode_id, arm_id, sha,
                                 metadata=metadata, snapshot=snapshot, group_id=group_id)
        return sha

    def mirror_oracle(self, plan_id: str, arm_id: str, doc: dict,
                      rig_fingerprint=None, exec_id=None) -> None:
        verdict = doc.get("verdict") or {}
        self.store.add_oracle_verdict(plan_id, arm_id, verdict, doc, exec_id=exec_id)
        if rig_fingerprint and arm_id:
            self.store.update_accuracy_cells(arm_id, rig_fingerprint, doc)

    def mirror_telemetry(self, exec_id: str, plan_id: str, arm_id: str, samples_bytes: bytes,
                         kp_key: str, summary=None) -> str:
        sha = self.store.put_blob(samples_bytes, "telemetry")
        self.store.add_telemetry(exec_id, plan_id, arm_id, sha, kp_key, summary or {})
        return sha

    def mirror_rig(self, rig) -> None:
        """Mirror an extrinsics.Rig as the active shared rig (numpy -> lists)."""
        def lst(x):
            return x.tolist() if hasattr(x, "tolist") else x
        self.store.upsert_rig(
            rig.fingerprint, self.workspace_id, rig.calib_fingerprint,
            lst(rig.T_base_cam), lst(rig.image_size), lst(rig.tip_offset),
            method=getattr(rig, "method", "base-marker"),
            rms_px=getattr(rig, "rms_px", None), rms_mm=getattr(rig, "rms_mm", None),
            witness=getattr(rig, "witness", None),
            provenance=rig.provenance() if hasattr(rig, "provenance") else None,
            rig_doc=rig.to_dict() if hasattr(rig, "to_dict") else None)

    def mirror_sag(self, arm_id: str, map_doc: dict) -> None:
        self.store.ensure_arm(arm_id)
        self.store.upsert_sag_map(arm_id, map_doc)

    def mirror_still(self, episode_id: str, frame_bytes: bytes, purpose: str = "hold",
                     arm_id=None, t=None, image_size=None) -> str:
        sha = self.store.put_blob(frame_bytes, "still")
        w, h = (image_size or [None, None])[:2]
        self.store.add_still(episode_id, sha, purpose=purpose, arm_id=arm_id, t=t, width=w, height=h)
        return sha

    def mirror_intrinsics(self, calib_fingerprint: str, K, dist, image_size, corners_bytes=None) -> None:
        self.store.ensure_calibration(calib_fingerprint, K=K, dist=dist, image_size=image_size)
        if corners_bytes is not None:
            self.store.put_blob(corners_bytes, "other")

    def mirror_monitor(self, arm_id: str, ewma_pixel_error_mm=None, ewma_tag_rms_px=None, last_doc=None) -> None:
        self.store.set_monitor_state(arm_id, ewma_pixel_error_mm, ewma_tag_rms_px, last_doc)

    def mirror_tag(self, tag_id: str, kind: str, side_mm: float, pose_world=None) -> None:
        self.store.upsert_tag(tag_id, self.workspace_id, kind, side_mm, pose_world)
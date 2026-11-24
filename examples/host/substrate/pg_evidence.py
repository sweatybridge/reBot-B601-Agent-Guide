"""PG evidence mirror seam (spec §8) — delegates to atomic single-round-trip RPCs.

Each compound write (`mirror_capture`, `mirror_oracle`, `mirror_telemetry`,
`mirror_rig`, `mirror_recording`) executes in ONE SQL transaction on `SubstrateStore`.
"""
from __future__ import annotations

from pathlib import Path
import hashlib


def _probe_video_duration(path: Path, default: float = 10.0) -> float:
    try:
        import cv2
        cap = cv2.VideoCapture(str(path))
        try:
            if cap.isOpened():
                fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
                frames = cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0.0
                if fps > 0 and frames > 0:
                    return float(frames) / float(fps)
        finally:
            cap.release()
    except Exception:  # noqa: BLE001
        pass
    return default


class PgEvidenceMirror:
    def __init__(self, store, workspace_id: str = "default"):
        self.store = store
        self.workspace_id = workspace_id
        store.ensure_workspace(workspace_id)

    def mirror_capture(self, capture_id: str, meta: dict, frame_bytes: bytes,
                       arm_id=None, kind: str = "calibration", image_size=None) -> str:
        return self.store.record_capture(
            capture_id, self.workspace_id, meta, frame_bytes,
            arm_id=arm_id, kind=kind, image_size=image_size)

    def mirror_plan(self, plan_id: str, episode_id: str, arm_id: str, trajectory_bytes: bytes,
                    metadata=None, snapshot=None, group_id=None) -> str:
        sha = self.store.put_blob(trajectory_bytes, "trajectory")
        self.store.register_plan(plan_id, episode_id, arm_id, sha,
                                 metadata=metadata, snapshot=snapshot, group_id=group_id)
        return sha

    def mirror_oracle(self, plan_id: str, arm_id: str, doc: dict,
                      rig_fingerprint=None, exec_id=None) -> None:
        self.store.record_oracle(
            plan_id, arm_id, doc, rig_fingerprint=rig_fingerprint, exec_id=exec_id)

    def mirror_telemetry(self, exec_id: str, plan_id: str, arm_id: str, samples_bytes: bytes,
                         kp_key: str, summary=None) -> str:
        return self.store.record_telemetry(
            exec_id, plan_id, arm_id, samples_bytes, kp_key, summary=summary)

    def mirror_rig(self, rig, witness_sha=None, calib=None) -> None:
        """Mirror an extrinsics.Rig as the active shared rig (atomic + scoped invalidation)."""
        def lst(x):
            return x.tolist() if hasattr(x, "tolist") else x
        self.store.upsert_rig(
            rig.fingerprint, self.workspace_id, rig.calib_fingerprint,
            lst(rig.T_base_cam), lst(rig.image_size), lst(rig.tip_offset),
            method=getattr(rig, "method", "base-marker"),
            rms_px=getattr(rig, "rms_px", None), rms_mm=getattr(rig, "rms_mm", None),
            witness=getattr(rig, "witness", None),
            provenance=rig.provenance() if hasattr(rig, "provenance") else None,
            rig_doc=rig.to_dict() if hasattr(rig, "to_dict") else None,
            witness_sha=witness_sha,
            calib_K=lst(calib.K) if calib is not None and hasattr(calib, "K") else None,
            calib_dist=lst(calib.dist) if calib is not None and hasattr(calib, "dist") else None,
            calib_image_size=list(calib.image_size) if calib is not None and hasattr(calib, "image_size") else None)

    def mirror_sag(self, arm_id: str, map_doc: dict) -> None:
        self.store.ensure_arm(arm_id)
        self.store.upsert_sag_map(arm_id, map_doc)

    def mirror_still(self, episode_id: str, frame_bytes: bytes, purpose: str = "hold",
                     arm_id=None, t=None, image_size=None) -> str:
        sha = self.store.put_blob(frame_bytes, "still")
        w, h = (image_size or [None, None])[:2]
        self.store.add_still(episode_id, sha, purpose=purpose, arm_id=arm_id, t=t, width=w, height=h)
        return sha

    def mirror_monitor(self, arm_id: str, ewma_pixel_error_mm=None, ewma_tag_rms_px=None, last_doc=None) -> None:
        self.store.set_monitor_state(arm_id, ewma_pixel_error_mm, ewma_tag_rms_px, last_doc)

    def mirror_tag(self, tag_id: str, kind: str, side_mm: float, pose_world=None) -> None:
        self.store.upsert_tag(tag_id, self.workspace_id, kind, side_mm, pose_world)

    def mirror_recording(self, episode_id: str, path, arm_id=None, seq: int = 0) -> str:
        p = Path(path)
        data = p.read_bytes()
        dur = _probe_video_duration(p, default=10.0)
        self.store.ingest_episode_segments(
            episode_id, self.workspace_id, arm_id, [data], [dur], codec="mp4")
        return hashlib.sha256(data).hexdigest()

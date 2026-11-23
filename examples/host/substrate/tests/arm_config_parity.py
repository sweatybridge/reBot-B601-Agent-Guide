"""Arm-config drift guard: world.arm_config (table) == rebot_ik (Python) ==
rebot_arm_config() (Rust), bit-exact. The table is the canonical source; this test
fails if any of the three diverges, so the physical-model constants cannot drift
between the numpy reference and the Rust core. Run in the capture container.

The Rust export omits `tool` (URDF-derived, not a Rust const); the Python default
omits `motion` (traj/sweep tuning, a separate consolidation group). Each is compared
against the table on the keys it owns, so together they cover the whole doc.
"""
import sys, json
sys.path.insert(0, "/app/app")
import psycopg
import rebot_ik

DSN = "postgresql://postgres:rebot@substrate:5432/postgres"
conn = psycopg.connect(DSN)

table = conn.execute("SELECT config_doc FROM world.arm_config WHERE arm_id='b601'").fetchone()[0]
if isinstance(table, str):
    table = json.loads(table)
rust = conn.execute("SELECT rebot_arm_config()").fetchone()[0]
py = rebot_ik.default_config()


def check(label, a, b):
    if a != b:
        print("DRIFT %s:\n  A=%s\n  B=%s" % (label, json.dumps(a, sort_keys=True),
                                             json.dumps(b, sort_keys=True)))
        return False
    return True


ok = True
for key in rust:                      # gravity/limits/link_inertials/link_radii/collision/gripper/motion
    ok &= check("table==rust[%s]" % key, table.get(key), rust[key])
for key in py:                        # gravity/limits/link_inertials/link_radii/tool/gripper/collision
    ok &= check("table==python[%s]" % key, table.get(key), py[key])

# apply_config must be a no-op on the seed (defaults already equal the table).
before = rebot_ik.default_config()
rebot_ik.apply_config(table)
after = rebot_ik.default_config()
ok &= check("apply_config no-op on seed", before, after)

print("keys: table=%d rust=%d python=%d" % (len(table), len(rust), len(py)))
print("ARM_CONFIG PARITY", "OK" if ok else "DRIFT")
sys.exit(0 if ok else 1)
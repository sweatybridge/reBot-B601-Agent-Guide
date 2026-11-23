"""Arm-config drift guard + required-config guarantee.

The Rust cores no longer carry hardcoded tuning constants (option B, config
REQUIRED): they take the config doc as a jsonb arg passed by the SQL wrapper from
world.arm_config. So there is no independent Rust source to drift — the guard now
asserts the table == the Python single source (rebot_ik.default_config) bit-exact,
that apply_config is a no-op on the seed, and that a core HARD-REFUSES when the
config is missing/empty (no silent fallback to a baked-in value).

Run in the capture container (psycopg + rebot_ik + substrate access).
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
py = rebot_ik.default_config()


def check(label, a, b):
    if a != b:
        print("DRIFT %s:\n  A=%s\n  B=%s" % (label, json.dumps(a, sort_keys=True),
                                             json.dumps(b, sort_keys=True)))
        return False
    return True


ok = True
# table and Python now cover the same keys (gravity/limits/link_inertials/link_radii/
# tool/gripper/collision/motion); the Rust cores consume the table doc directly.
for key in py:
    ok &= check("table==python[%s]" % key, table.get(key), py[key])
for key in table:
    ok &= check("python covers table[%s]" % key, key in py, True)

# apply_config must be a no-op on the seed (defaults already equal the table).
before = rebot_ik.default_config()
rebot_ik.apply_config(table)
ok &= check("apply_config no-op on seed", before, rebot_ik.default_config())

# Required-config guarantee: an empty/missing config must raise, not fall back.
try:
    conn.execute("SELECT rebot_within_limits(%s, '{}'::jsonb, 0.0)", ([0.0] * 6,)).fetchone()
    print("DRIFT: core accepted an empty config (expected a hard refusal)")
    ok = False
except psycopg.Error:
    conn.rollback()
    print("required-config refusal OK (empty config raises)")

print("keys: table=%d python=%d" % (len(table), len(py)))
print("ARM_CONFIG PARITY", "OK" if ok else "DRIFT")
sys.exit(0 if ok else 1)
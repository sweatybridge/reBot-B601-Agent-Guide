-- Kinematics integration test on the real reBot URDF.
-- Prereq: the URDF is available in-container at /tmp/rebot.urdf, e.g.
--   docker cp examples/host/app/cad/ReBot_Arm_RS.urdf rebot-substrate:/tmp/rebot.urdf
-- The arm is joint1..joint6 (6 DOF); gripper_joint1/2 add 2 more (8 total).
DO $$
DECLARE
  u text := pg_read_file('/tmp/rebot.urdf');
  pose float8[];
  q float8[];
  again float8[];
BEGIN
  IF rebot_dof(u) <> 8 THEN
    RAISE EXCEPTION 'expected 8 dof, got %', rebot_dof(u);
  END IF;

  pose := rebot_fk(u, ARRAY[0,0,0,0,0,0,0,0]::float8[], 'joint6');
  RAISE NOTICE 'FK zero wrist pose [x,y,z,qw,qx,qy,qz]: %', pose;
  IF array_length(pose,1) <> 7 THEN
    RAISE EXCEPTION 'FK pose must have 7 elements';
  END IF;

  -- A non-zero arm pose within joint limits (joint2/joint3 >= 0), gripper at 0.
  pose := rebot_fk(u, ARRAY[0.2,0.3,0.4,0.2,0.1,0.0,0.0,0.0]::float8[], 'joint6');

  -- IK round-trip: target is the exact FK pose (quaternion); seed the 6-DOF arm
  -- sub-chain at zero. rebot_ik returns the solved arm joints.
  q := rebot_ik(u, 'joint6', pose, ARRAY[0,0,0,0,0,0]::float8[]);
  RAISE NOTICE 'IK solved arm q: %', q;
  IF array_length(q,1) <> 6 THEN
    RAISE EXCEPTION 'IK must return 6 arm joints, got %', array_length(q,1);
  END IF;

  again := rebot_fk(u, ARRAY[q[1],q[2],q[3],q[4],q[5],q[6],0,0]::float8[], 'joint6');
  IF abs(again[1]-pose[1])>1e-3 OR abs(again[2]-pose[2])>1e-3 OR abs(again[3]-pose[3])>1e-3 THEN
    RAISE EXCEPTION 'IK round-trip position mismatch: % vs %', again, pose;
  END IF;
  RAISE NOTICE 'KINEMATICS FK/IK ROUND-TRIP OK';
END $$;
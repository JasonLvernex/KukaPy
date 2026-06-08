"""
FK / IK Test — kukapy + roboticstoolbox-python
===============================================
Offline mode:  tests FK/IK math without a real robot.
Online mode:   cross-validates Python FK against KUKA controller's TCP position.

Install deps:
    pip install roboticstoolbox-python spatialmath-python scipy

Usage:
    - Offline only:  python fk_ik_test.py
    - With robot:    set CONNECT_TO_ROBOT = True, then run
"""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))
from kukapy.robot import Robot

try:
    import roboticstoolbox as rtb
    from spatialmath import SE3
    from scipy.spatial.transform import Rotation
except ImportError:
    print("ERROR: Missing dependencies. Run:")
    print("  pip install roboticstoolbox-python spatialmath-python scipy")
    sys.exit(1)


# ── Configuration ─────────────────────────────────────────────────────────────

CONNECT_TO_ROBOT = True   # set True to run online cross-validation with real robot
ROBOT_PORT       = 18735


# ── KUKA robot DH model ───────────────────────────────────────────────────────

_NP_PARAMS = os.path.join(os.path.dirname(__file__), "calibration_data", "kuka_dh_params.npy")

# Joint direction signs: signs[i]=+1 means KUKA and DH agree on rotation direction.
# signs[i]=-1 means KUKA rotates opposite to DH for joint i.
# Loaded from kuka_dh_params.npy; defaults to all +1 (standard DH convention).
_SIGNS = np.ones(6)


def build_kuka_model() -> rtb.DHRobot:
    """
    Build DHRobot from calibrated kuka_dh_params.npy if it exists.
    Also loads joint direction signs into module-level _SIGNS.
    """
    global _SIGNS
    if os.path.exists(_NP_PARAMS):
        p = np.load(_NP_PARAMS, allow_pickle=True).item()
        n = int(p.get("n_joints", len(p["d"])))
        print(f"  Loaded DH params from kuka_dh_params.npy  "
              f"(calib RMS: {p['rms_mm']:.2f} mm, {n}-DOF)")
        _SIGNS = np.array(p.get("signs", np.ones(n)))
        links = [
            rtb.RevoluteDH(d=p["d"][i], a=p["a"][i], alpha=p["alpha"][i], offset=p["offset"][i])
            for i in range(n)
        ]
        return rtb.DHRobot(links, name="KUKA_calibrated")

    print("  kuka_dh_params.npy not found — using placeholder KR6 R900 params.")
    print("  Run collect_calibration.py then dh_calibration.py to calibrate your robot.")
    return rtb.DHRobot(
        [
            rtb.RevoluteDH(d=400,  a=25,  alpha=-np.pi / 2, offset=0),
            rtb.RevoluteDH(d=0,    a=455, alpha=0,           offset=-np.pi / 2),
            rtb.RevoluteDH(d=0,    a=35,  alpha=-np.pi / 2,  offset=0),
            rtb.RevoluteDH(d=420,  a=0,   alpha=np.pi / 2,   offset=0),
            rtb.RevoluteDH(d=0,    a=0,   alpha=-np.pi / 2,  offset=0),
            rtb.RevoluteDH(d=80,   a=0,   alpha=0,           offset=0),
        ],
        name="KUKA_KR6_R900_placeholder",
    )


def kuka_fkine(kuka: rtb.DHRobot, joints_deg: list):
    """FK wrapper: apply joint direction signs before calling fkine().
    roboticstoolbox computes: theta_i = q_i + link.offset
    We pass:  q_i = signs[i] * joints_deg[i]  (in radians)
    So:       theta_i = signs[i]*joints_deg_rad[i] + offset[i]  — correct.
    """
    q_signed = _SIGNS * np.deg2rad(joints_deg)
    return kuka.fkine(q_signed)


# ── KUKA coordinate convention helpers ───────────────────────────────────────

def kuka_xyzabc_to_SE3(xyzabc: list) -> SE3:
    """
    Convert KUKA [X, Y, Z, A, B, C] to SE3.
    KUKA uses ZYX extrinsic Euler: A around Z, B around Y, C around X.
    """
    x, y, z, a, b, c = xyzabc
    R = Rotation.from_euler("ZYX", [a, b, c], degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return SE3(T)


def SE3_to_kuka_xyzabc(T: SE3) -> list:
    """Convert SE3 back to KUKA [X, Y, Z, A, B, C]."""
    abc = Rotation.from_matrix(T.R).as_euler("ZYX", degrees=True)
    return list(T.t) + list(abc)


# ── Test functions ─────────────────────────────────────────────────────────────

def test_fk_offline(kuka: rtb.DHRobot, joints_deg: list) -> SE3:
    print("\n" + "=" * 55)
    print("[OFFLINE] Forward Kinematics (FK)")
    print("=" * 55)

    T = kuka_fkine(kuka, joints_deg)
    xyzabc = SE3_to_kuka_xyzabc(T)

    print(f"  Input joints (deg) : {[round(v, 2) for v in joints_deg]}")
    print(f"  FK result:")
    print(f"    X={xyzabc[0]:8.2f} mm   Y={xyzabc[1]:8.2f} mm   Z={xyzabc[2]:8.2f} mm")
    print(f"    A={xyzabc[3]:8.2f} deg  B={xyzabc[4]:8.2f} deg  C={xyzabc[5]:8.2f} deg")
    return T


def test_ik_offline(kuka: rtb.DHRobot, target_T: SE3, label: str = "") -> np.ndarray | None:
    print("\n" + "=" * 55)
    print(f"[OFFLINE] Inverse Kinematics (IK){' — ' + label if label else ''}")
    print("=" * 55)

    xyzabc = SE3_to_kuka_xyzabc(target_T)
    print(f"  Target position : X={xyzabc[0]:.2f}  Y={xyzabc[1]:.2f}  Z={xyzabc[2]:.2f} mm")

    sol = kuka.ikine_LM(target_T)

    if not sol.success:
        print("  IK FAILED — no solution found (target may be out of reach)")
        return None

    joints_deg = np.rad2deg(sol.q)
    print(f"  IK solution (deg): {[round(v, 2) for v in joints_deg]}")

    # Verify: FK(IK(target)) should reproduce the target
    # sol.q is already in signed-radian space that roboticstoolbox uses
    T_verify = kuka.fkine(sol.q)
    pos_err = np.linalg.norm(T_verify.t - target_T.t)
    print(f"  Round-trip FK error: {pos_err:.4f} mm", end="  ")
    if pos_err < 1.0:
        print("[PASS]")
    else:
        print("[WARN — error > 1 mm, check DH params]")

    return sol.q


def test_fk_vs_controller(robot: Robot, kuka: rtb.DHRobot) -> None:
    """
    Read current joint angles from the real robot,
    compute FK in Python, and compare with controller's reported TCP position.
    A small error (~mm) means the DH model is correct.
    """
    print("\n" + "=" * 55)
    print("[ONLINE] FK Cross-Validation vs KUKA Controller")
    print("=" * 55)

    joints_deg = robot.get_curjpos()          # from robot via EKI
    pos_ctrl   = robot.get_curpos()           # [X,Y,Z,OA,OB,OC] from controller

    T = kuka_fkine(kuka, joints_deg)
    pos_fk = SE3_to_kuka_xyzabc(T)

    print(f"  Joints (deg)      : {[round(v, 1) for v in joints_deg]}")
    print()
    print(f"  Controller TCP    : X={pos_ctrl[0]:8.1f}  Y={pos_ctrl[1]:8.1f}  Z={pos_ctrl[2]:8.1f} mm")
    print(f"  Python FK TCP     : X={pos_fk[0]:8.1f}  Y={pos_fk[1]:8.1f}  Z={pos_fk[2]:8.1f} mm")
    print()

    pos_err = np.linalg.norm(np.array(pos_ctrl[:3]) - np.array(pos_fk[:3]))
    print(f"  Position error    : {pos_err:.2f} mm", end="  ")
    if pos_err < 10.0:
        print("[PASS — DH model matches controller]")
    else:
        print("[FAIL — DH parameters need adjustment]")


def test_ik_then_move(robot: Robot, kuka: rtb.DHRobot, target_xyzabc: list) -> None:
    """
    Compute IK in Python for a Cartesian target,
    then send the resulting joint angles to the real robot.
    """
    print("\n" + "=" * 55)
    print("[ONLINE] IK → Move robot to target position")
    print("=" * 55)

    target_T = kuka_xyzabc_to_SE3(target_xyzabc)
    sol = kuka.ikine_LM(target_T)

    if not sol.success:
        print("  IK FAILED — aborting move")
        return

    joints_deg = list(np.rad2deg(sol.q))
    print(f"  Target  : {[round(v, 1) for v in target_xyzabc]}")
    print(f"  IK joints (deg): {[round(v, 2) for v in joints_deg]}")
    print("  Sending move command...")

    robot.move("joint", joints_deg, velocity=15)
    robot.print_pos()
    print("  Move complete.")


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    kuka = build_kuka_model()
    print(kuka)

    # ── Offline tests (no robot needed) ───────────────────────────────────────

    # Test 1: FK at home position
    home_joints = [0, -90, 90, 0, 90, 0]
    home_T = test_fk_offline(kuka, home_joints)

    # Test 2: IK round-trip — compute IK for the FK result, should recover original joints
    test_ik_offline(kuka, home_T, label="round-trip from home FK")

    # Test 3: FK at a non-trivial pose
    pose2_joints = [-45, -70, 80, 30, 60, 15]
    pose2_T = test_fk_offline(kuka, pose2_joints)
    test_ik_offline(kuka, pose2_T, label="round-trip from pose2 FK")

    # Test 4: IK for a manually specified Cartesian target
    # Adjust these values to a reachable point for your robot
    custom_target = SE3(500, 0, 600) * SE3.Ry(90, unit="deg")
    test_ik_offline(kuka, custom_target, label="custom Cartesian target")

    # ── Online tests (real robot) ──────────────────────────────────────────────

    if CONNECT_TO_ROBOT:
        robot = Robot(port=ROBOT_PORT, recv_timeout=60)
        print("\nWaiting for KRC to connect...")
        robot.connect()

        # Cross-validate FK at several poses — deliberately includes
        # poses far from calibration data to test generalization.
        FK_TEST_POSES = [
            [  0, -90,  90,   0,  90,   0],  # home (calibration pose — should be ~0 mm)
            [ 45, -90,  90,   0,  90,   0],  # A1 swing
            [  0, -70,  70,   0,  60,   0],  # A5=60  (wrist stress test)
            [  0, -70,  70,   0, 120,   0],  # A5=120 (wrist stress test)
            [ 30, -40,  30,  20,  80,  10],  # shallow arm + varied wrist
            [-30, -40,  30, -20,  80, -10],  # symmetric
            [  0, -90,  90,   0,  90,   0],  # home
        ]
        for pose in FK_TEST_POSES:
            robot.move("joint", pose, velocity=100)
            test_fk_vs_controller(robot, kuka)

        # Optional: compute IK and actually move the robot
        # Uncomment and set a safe target for your setup before enabling:
        #
        # test_ik_then_move(robot, kuka, target_xyzabc=[500, 0, 700, 0, 0, 0])

        robot.disconnect()
        print("\nDisconnected.")
    else:
        print("\n" + "=" * 55)
        print("[ONLINE tests skipped]")
        print("Set CONNECT_TO_ROBOT = True at the top of this file to enable.")
        print("=" * 55)

    print("\nAll tests complete.")

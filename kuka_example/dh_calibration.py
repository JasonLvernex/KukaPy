"""
DH Parameter Calibration + Collection — kukapy
================================================
Merged from collect_calibration.py + dh_calibration.py.

Usage:
    python dh_calibration.py                         # calibrate from saved JSON
    python dh_calibration.py --collect              # interactive collect → calibrate
    python dh_calibration.py --collect --auto       # auto-move collect → calibrate (6-DOF)
    python dh_calibration.py --joints 7             # 7-joint robot from saved data
    python dh_calibration.py --collect --joints 6 --port 18735

Flags:
    --collect        Connect to robot and record calibration data before calibrating.
                     Default mode: manual jog — press Enter at each pose to record.
    --auto           Auto-move through preset diverse poses (6-DOF only). Requires --collect.
    --joints N       Number of robot joints (default 6).
    --port P         EKI TCP port (default 18735).
    --data FILE      Calibration JSON file (default: calibration_data.json in script dir).
    --out FILE       Output .npy file (default: kuka_dh_params.npy in script dir).

Key insight — joint direction signs:
    Standard DH assumes counterclockwise (right-hand rule).
    KUKA's hardware convention has some joints rotating the opposite direction.
    A1=+90° → Y=-1550 (not +1550) confirms A1 is clockwise.
    This script searches all 2^(N-1) sign combinations for joints 2..N,
    bakes the found signs into the saved .npy so fk_ik_test.py can use them.

Requirements:
    pip install scipy numpy
"""

import argparse
import sys
import os
import json
from itertools import product as iproduct

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as _Rot

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from kukapy.robot import Robot


# ── Fixed configuration ────────────────────────────────────────────────────────

# KUKA A1 rotates clockwise vs DH counterclockwise — confirmed from calibration data.
# A1=+90° gives Y=-1550; standard DH would predict +1550. Fixed at -1.
_SIGN_A1 = -1

# 1 radian of orientation error ≡ this many mm in the cost function.
_ROT_WEIGHT = 500.0

# Speed for auto-collect moves.
_VELOCITY = 100  # %

# Preset diverse poses for 6-DOF KUKA (used with --auto).
# Covers: A1 swing ±90°, A2/A3 high/low elbow, combined wrist variation.
_AUTO_POSES_6DOF = [
    [  0, -90,  90,   0, 90,   0],  # home                (baseline)
    [ 45, -90,  90,   0, 90,   0],  # A1 +45              (constrains d[0], a[0])
    [-45, -90,  90,   0, 90,   0],  # A1 -45
    [ 90, -90,  90,   0, 90,   0],  # A1 +90              (max A1 swing)
    [-90, -90,  90,   0, 90,   0],  # A1 -90
    [  0, -70,  70,   0, 90,   0],  # low elbow           (constrains a[1], d[3])
    [  0, -110, 100,  0, 90,   0],  # high elbow
    [ 45, -70,  70,  45, 90,  30],  # A1 + wrist varied   (constrains offset[3..5])
    [-45, -70,  70, -45, 90, -30],  # symmetric wrist
]


# ── DH topology defaults ───────────────────────────────────────────────────────

def _alpha_default(n: int) -> np.ndarray:
    """
    Default DH alpha topology.
    n=6 → standard KUKA 6R: [-π/2, 0, -π/2, π/2, -π/2, 0].
    n≠6 → generic serial: all -π/2 except last joint = 0.
          (Verify against your robot's documentation for non-KUKA arms.)
    """
    if n == 6:
        return np.array([-np.pi / 2, 0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, 0.0])
    alpha = np.full(n, -np.pi / 2)
    alpha[-1] = 0.0
    return alpha


def _home_deg_default(n: int) -> np.ndarray:
    """
    Default home joint angles in degrees.
    n=6 → KUKA home [0, -90, 90, 0, 90, 0].
    n≠6 → all zeros (edit _HOME_DEG_CUSTOM in main or pass via code).
    """
    if n == 6:
        return np.array([0.0, -90.0, 90.0, 0.0, 90.0, 0.0])
    return np.zeros(n)


# ── Standard DH forward kinematics (pure numpy) ───────────────────────────────

def _dh_T(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """4x4 standard DH transform for one joint."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [ 0,       sa,      ca,       d],
        [ 0,        0,       0,       1],
    ])


def fk_full(d, a, alpha, offset, joints_deg, signs) -> np.ndarray:
    """
    FK → 4x4 homogeneous matrix.
    theta_i = signs[i] * deg2rad(joints_deg[i]) + offset[i]
    Works for any number of joints (all arrays must have the same length).
    """
    T = np.eye(4)
    n = len(signs)
    for i in range(n):
        theta = signs[i] * np.deg2rad(joints_deg[i]) + offset[i]
        T = T @ _dh_T(theta, d[i], a[i], alpha[i])
    return T


# ── Calibration internals ──────────────────────────────────────────────────────

def _unpack(params: np.ndarray, n: int):
    """Split flat param vector [d|a|offset] → (d, a, offset), each length n."""
    return params[:n], params[n:2 * n], params[2 * n:3 * n]


def _bounds(n: int):
    lo = np.array([   0.0] * n + [-500.0] * n + [-np.pi] * n)
    hi = np.array([5000.0] * n + [5000.0] * n + [ np.pi] * n)
    return lo, hi


def _home_offsets(signs: np.ndarray, home_deg: np.ndarray) -> np.ndarray:
    """offset[i] s.t. theta_DH = 0 at home: offset[i] = -signs[i]*home_deg_rad[i]."""
    return -signs * np.deg2rad(home_deg)


def _residuals(params, alpha, data, signs):
    """
    Combined position + orientation residuals.
    Per data point: 3 position residuals [mm] + 3 orientation residuals [mm-equiv].
    Total equations = 6 * n_points, unknowns = 3 * n_joints.
    """
    n = len(signs)
    d, a, offset = _unpack(params, n)
    res = []
    for joints_deg, tcp_actual in data:
        T = fk_full(d, a, alpha, offset, joints_deg, signs)
        # position [mm]
        res.extend(T[:3, 3] - np.array(tcp_actual[:3]))
        # orientation [mm-equivalent]
        R_act = _Rot.from_euler(
            "ZYX", [float(tcp_actual[3]), float(tcp_actual[4]), float(tcp_actual[5])],
            degrees=True).as_matrix()
        rv = _Rot.from_matrix(R_act.T @ T[:3, :3]).as_rotvec()
        res.extend(_ROT_WEIGHT * rv)
    return res


def _make_init(xh: float, zh: float, n: int, signs: np.ndarray,
               home_deg: np.ndarray) -> np.ndarray:
    """Build one initial parameter vector from rough geometric estimates."""
    offsets = _home_offsets(signs, home_deg)
    d = np.zeros(n)
    a = np.zeros(n)
    a[0] = 25.0           # small base offset (common in KUKA)
    d[0] = zh * 0.8       # base column height
    if n >= 2:
        a[1] = xh * 1.0   # main upper-arm link
    if n >= 3:
        a[2] = xh * 0.05  # small forearm offset
    if n >= 4:
        d[3] = xh * 0.3   # wrist offset
    d[n - 1] = 80.0       # tool flange length
    lo, hi = _bounds(n)
    return np.clip(np.concatenate([d, a, offsets]), lo, hi)


def _search_joint_signs(data: list, alpha: np.ndarray, home_deg: np.ndarray,
                        n: int) -> tuple:
    """
    Search all 2^(n-1) sign combinations for joints 2..n (joint 1 is fixed at _SIGN_A1).
    Each candidate: coarse optimization (300 iter) with 9 starting points.
    Returns (best_signs, best_params).

    Why not capture this with an offset?
      A sign flip on q_i cannot be expressed as q_i + constant — it reverses the
      direction of the entire joint sweep. Must be a multiplicative factor.
    """
    n_combos = 2 ** (n - 1)
    if n_combos > 512:
        print(f"  Warning: {n_combos} sign combos for {n}-DOF may take a while.")

    # Estimate reach from home-position TCP
    home_ref = np.array(home_deg)
    home_tcp = next((tcp for j, tcp in data if np.allclose(j, home_ref, atol=1.0)), data[0][1])
    xh, zh = abs(float(home_tcp[0])), abs(float(home_tcp[2]))

    lo, hi = _bounds(n)
    best = (np.inf, None, None)
    combos = list(iproduct([1, -1], repeat=n - 1))
    print(f"  Searching {len(combos)} joint-direction sign combinations...")

    for combo in combos:
        signs = np.array([_SIGN_A1] + list(combo), dtype=float)
        best_r, best_cost = None, np.inf
        for d0_f in [0.4, 0.8, 1.1]:
            for a1_f in [0.7, 1.0, 1.2]:
                p0 = _make_init(xh * a1_f, zh * d0_f, n, signs, home_deg)
                try:
                    r = least_squares(
                        _residuals, p0, bounds=(lo, hi),
                        args=(alpha, data, signs), method="trf",
                        max_nfev=300, xtol=1e-4, ftol=1e-4, verbose=0)
                    if r.cost < best_cost:
                        best_cost, best_r = r.cost, r
                except Exception:
                    pass
        if best_r is not None and best_r.cost < best[0]:
            best = (best_r.cost, signs, best_r.x)

    _, signs, init_x = best
    print(f"  Best signs: {list(signs.astype(int))}  (coarse cost={best[0]:.1f})")
    return signs, init_x


def calibrate(data: list, alpha: np.ndarray, home_deg: np.ndarray,
              n: int, verbose: bool = True) -> dict:
    """
    Fit DH parameters from (joints_deg, tcp_xyzabc) data pairs.

    Steps:
      1. Search 2^(n-1) joint-direction sign combinations (coarse, 300 iter each).
      2. Multi-start refinement with the best signs (~16 starts, 2000 iter each).
      3. Final tight convergence (200k iter, tol=1e-13).

    Returns dict: d, a, alpha, offset, signs, rms_mm, rms_deg, n_joints.
    """
    # 3n unknowns / 6 equations per point → minimum ceil(n/2) points; require ≥ max(6, n)
    min_pts = max(6, n)
    if len(data) < min_pts:
        print(f"  Warning: {len(data)} points — recommend {min_pts}+ for {n}-DOF calibration.")

    # Step 1: sign search
    signs, init_x = _search_joint_signs(data, alpha, home_deg, n)

    # Step 2: multi-start with best signs
    lo, hi = _bounds(n)
    xh = abs(float(data[0][1][0]))
    zh = abs(float(data[0][1][2]))
    inits = [init_x]
    for d0_f in [0.3, 0.6, 0.9, 1.1]:
        for a1_f in [0.5, 0.8, 1.0, 1.2]:
            inits.append(_make_init(xh * a1_f, zh * d0_f, n, signs, home_deg))

    print(f"  Multi-start refinement: {len(inits)} initial conditions...")
    best_r, best_cost = None, np.inf
    for p0 in inits:
        try:
            r = least_squares(
                _residuals, p0, bounds=(lo, hi),
                args=(alpha, data, signs), method="trf",
                max_nfev=2000, xtol=1e-8, ftol=1e-8, verbose=0)
            if r.cost < best_cost:
                best_cost, best_r = r.cost, r
        except Exception:
            pass

    # Step 3: tight convergence
    print(f"  Final refinement (cost={best_cost:.1f})...")
    best_r = least_squares(
        _residuals, best_r.x, bounds=(lo, hi),
        args=(alpha, data, signs), method="trf",
        max_nfev=200_000, xtol=1e-13, ftol=1e-13, verbose=0)

    d, a, offset = _unpack(best_r.x, n)

    # Per-point error breakdown
    pos_errs, rot_errs = [], []
    for j, p in data:
        T = fk_full(d, a, alpha, offset, j, signs)
        pos_errs.append(np.linalg.norm(T[:3, 3] - np.array(p[:3])))
        R_act = _Rot.from_euler("ZYX", [p[3], p[4], p[5]], degrees=True).as_matrix()
        rot_errs.append(np.degrees(np.linalg.norm(
            _Rot.from_matrix(R_act.T @ T[:3, :3]).as_rotvec())))

    rms_pos = float(np.sqrt(np.mean(np.array(pos_errs) ** 2)))
    rms_rot = float(np.sqrt(np.mean(np.array(rot_errs) ** 2)))

    if verbose:
        print(f"\n  Calibration done — RMS pos: {rms_pos:.2f} mm | "
              f"RMS rot: {rms_rot:.2f} deg  ({len(data)} points)")
        for i, (pe, re, (j, _)) in enumerate(zip(pos_errs, rot_errs, data)):
            flag = "  ← large" if pe >= 5.0 else ""
            print(f"    Point {i+1:2d}: joints={[round(v,1) for v in j]}"
                  f"  pos={pe:.1f} mm  rot={re:.2f} deg{flag}")

    return {
        "d":        d,
        "a":        a,
        "alpha":    alpha.copy(),
        "offset":   offset,
        "signs":    signs,
        "rms_mm":   rms_pos,
        "rms_deg":  rms_rot,
        "n_joints": n,
    }


# ── Save / load ────────────────────────────────────────────────────────────────

def save_npy(cal: dict, path: str) -> None:
    np.save(path, cal, allow_pickle=True)
    print(f"  Saved → {path}")


def load_npy(path: str) -> dict:
    return np.load(path, allow_pickle=True).item()


def print_model(cal: dict) -> None:
    n = int(cal.get("n_joints", len(cal["d"])))
    d, a, alpha, offset, signs = cal["d"], cal["a"], cal["alpha"], cal["offset"], cal["signs"]
    print("\n" + "=" * 62)
    print(f"Calibrated DH parameters ({n}-DOF):")
    print("=" * 62)
    print(f"  Signs  : {list(signs.astype(int))}")
    print(f"  Offsets: {[round(float(v), 4) for v in offset]} rad")
    print()
    print("    return rtb.DHRobot([")
    for i in range(n):
        print(f"        rtb.RevoluteDH(d={d[i]:.3f},  a={a[i]:.3f},"
              f"  alpha={alpha[i]:.6f},  offset={offset[i]:.6f}),")
    print("    ], name='KUKA_calibrated')")
    print(f"\n    # RMS: {cal['rms_mm']:.2f} mm  |  {cal['rms_deg']:.2f} deg")
    print("=" * 62)


# ── Data collection ────────────────────────────────────────────────────────────

def collect_interactive(robot: Robot, n_poses: int) -> list:
    """Jog the robot manually to each pose and press Enter to record."""
    data = []
    print(f"\nInteractive collection: target {n_poses} poses.")
    print("Jog robot to a new position, then press Enter. Type 'q' to stop early.\n")
    for i in range(n_poses):
        raw = input(f"  Pose {i+1}/{n_poses} — ready? [Enter / q]: ").strip()
        if raw.lower() == "q":
            break
        joints = robot.get_curjpos()
        tcp    = robot.get_curpos()
        data.append((joints, tcp))
        print(f"    Joints: {[round(v, 2) for v in joints]}")
        print(f"    TCP   : X={tcp[0]:.1f}  Y={tcp[1]:.1f}  Z={tcp[2]:.1f}\n")
    print(f"  Recorded {len(data)} poses.")
    return data


def collect_auto(robot: Robot, poses: list) -> list:
    """Auto-move robot through preset joint-space poses and record each."""
    data = []
    print(f"\nAuto-collect: {len(poses)} preset poses at {_VELOCITY}% velocity.")
    for i, joints in enumerate(poses):
        print(f"  Pose {i+1}/{len(poses)}: {joints}")
        robot.move("joint", joints, velocity=_VELOCITY)
        j = robot.get_curjpos()
        p = robot.get_curpos()
        data.append((j, p))
        print(f"    Joints: {[round(v,2) for v in j]}")
        print(f"    TCP   : X={p[0]:.1f}  Y={p[1]:.1f}  Z={p[2]:.1f}\n")
    print("  Returning home...")
    robot.move("joint", poses[0], velocity=_VELOCITY)
    return data


def save_data(data: list, path: str) -> None:
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"  Saved {len(data)} points → {path}")


def load_data(path: str) -> list:
    with open(path) as f:
        raw = json.load(f)
    return [(entry[0], entry[1]) for entry in raw]


# ── CLI ────────────────────────────────────────────────────────────────────────

def _parse_args():
    p = argparse.ArgumentParser(
        description="KUKA DH calibration — collect data and/or fit DH parameters.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
examples:
  python dh_calibration.py                         # calibrate from saved JSON
  python dh_calibration.py --collect              # interactive collect + calibrate
  python dh_calibration.py --collect --auto       # auto-move collect (6-DOF)
  python dh_calibration.py --joints 7             # 7-joint robot
  python dh_calibration.py --collect --joints 6 --port 18735
""")
    p.add_argument("--collect", action="store_true",
                   help="Connect to robot and collect new data before calibrating.")
    p.add_argument("--auto", action="store_true",
                   help="With --collect: auto-move through preset poses. 6-DOF only.")
    p.add_argument("--joints", type=int, default=6, metavar="N",
                   help="Number of robot joints (default: 6).")
    p.add_argument("--port", type=int, default=18735, metavar="P",
                   help="Robot EKI port (default: 18735).")
    p.add_argument("--data", default=None, metavar="FILE",
                   help="Calibration JSON path (default: calibration_data.json).")
    p.add_argument("--out", default=None, metavar="FILE",
                   help="Output .npy path (default: kuka_dh_params.npy).")
    return p.parse_args()


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    args = _parse_args()
    n = args.joints

    _dir     = os.path.dirname(__file__)
    data_file = args.data or os.path.join(_dir, "calibration_data.json")
    npy_file  = args.out  or os.path.join(_dir, "kuka_dh_params.npy")

    alpha    = _alpha_default(n)
    home_deg = _home_deg_default(n)

    if n != 6:
        print(f"Note: {n}-DOF robot. Alpha topology: {np.round(alpha, 3).tolist()}")
        print("      Verify alpha against your robot's manual if accuracy is poor.\n")

    # ── Collection step ────────────────────────────────────────────────────────
    if args.collect:
        if args.auto and n != 6:
            print(f"Warning: --auto preset poses are for 6-DOF only (got --joints {n}).")
            print("Falling back to interactive collection.\n")
            args.auto = False

        robot = Robot(port=args.port, recv_timeout=120)
        print("Waiting for KRC to connect...")
        robot.connect()

        if args.auto:
            data = collect_auto(robot, _AUTO_POSES_6DOF)
        else:
            n_poses = max(9, n + 3)  # generous overdetermination
            data = collect_interactive(robot, n_poses=n_poses)

        robot.disconnect()
        print("Disconnected.")
        save_data(data, data_file)
        all_data = data

    else:
        if not os.path.exists(data_file):
            print(f"ERROR: {data_file} not found.")
            print("Run with --collect first, or pass --data <path> to an existing file.")
            sys.exit(1)
        all_data = load_data(data_file)
        print(f"Loaded {len(all_data)} points from {os.path.basename(data_file)}")

    # ── Calibration step ───────────────────────────────────────────────────────
    print(f"\nCalibrating {n}-DOF robot with {len(all_data)} data point(s)...")
    cal = calibrate(all_data, alpha, home_deg, n, verbose=True)
    save_npy(cal, npy_file)
    print_model(cal)

    if cal["rms_mm"] > 20.0:
        print("\nHint: RMS > 20 mm — collect more diverse poses for better accuracy.")
    elif cal["rms_mm"] > 5.0:
        print("\nHint: RMS > 5 mm — a few more poses would help.")
    else:
        print("\nCalibration looks good — run fk_ik_test.py.")

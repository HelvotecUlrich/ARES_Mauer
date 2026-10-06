"""Hand-eye calibration on the robot: plan look poses around the deck calib board, capture images with the flange
pose at the exposure, solve T_flange_cam, verify on new poses.

    py.exe tools/calib_handeye.py locate --nominal-intrinsics [--write data/board_table.json]   # no motion
    py.exe tools/calib_handeye.py plan [--n 25] [--seed 0] [--write poses.json] [--check --host IP]
    py.exe tools/calib_handeye.py capture --host IP --dataset data/he_2026-10-06 [--resume]
    py.exe tools/calib_handeye.py solve data/he_2026-10-06 [--out calib/handeye.json] [--force]
    py.exe tools/calib_handeye.py verify --host IP [--n 5] [--handeye calib/handeye.json]

Board on a table instead of the ARES deck (UR5 on the lab table, 2026-10-06): `locate` measures the calib board in
the UR base frame from the current pose (nominal [camera.mount] and intrinsics; flange from the controller's TCP if
the link knows the active TCP offset, else from actual_q with the nominal DH, ~1 mm) and writes it as JSON;
`--board-pose FILE` on plan / capture / verify aims at that pose instead of the [boards.calib] deck pose and uses the
board plane as the clearance plane. The pose only aims the camera - the calibration does not depend on it.
`--q-ref-deg=q1,...,q6` (plan / capture / verify; "=" because of the minus signs): the IK branch of every pose is
the one closest to these joints (the robot's current ones, `tools/ur_check.py info`) instead of the deck convention
"elbow up, wrist down" with the base turned towards the board, and the roll range is centred on the camera's roll at
these joints (wrist 3 stays within about the roll range of its current angle, not up to 180 deg away) - on the table (2026-10-06) the robot stood in the other branch, and the first look
move would have turned the base by ~180 deg. On the real robot capture / verify refuse to start when the first pose
needs a base / shoulder / elbow move of more than MAX_START_JUMP_DEG.

Without hardware (URSim CB3 3.15.8 in Docker, tests/ursim.py): add --ursim (host 127.0.0.1; --start-ursim starts the
container, never pulls). The camera is then mauer.simcam.SynthCamera: it renders the calib board at its nominal deck
pose ([boards.calib], PLACEHOLDER) from the URSim flange pose (RTDE) and a ground-truth T_flange_cam = [camera.mount]
nominal @ --inject "dx,dy,dz,rx,ry,rz" (mm, deg); images use the nominal pinhole intrinsics. The whole pipeline
(plan -> URScript moves -> RTDE pose matching -> dataset -> calibrateHandEye -> hold-out) runs; it cannot show RTDE
latency, real robot accuracy or ARES sway (the rendered image follows the RTDE pose exactly).

plan: handeye.plan_poses around the board centre ([vision.handeye_plan]: n_poses, dist_mm, tilt_deg, roll_deg) with
  the nominal T_flange_cam; prints every flange pose, the camera distance/tilt, the nominal UR5 IK branch used as
  qnear (elbow up, wrist down; UR5 DH nominal - the controller does the real IK) and the clearance above the deck
  plane. --write saves the poses as JSON (format below, read by tools/measure_target.py poses). --check asks the
  controller per pose (get_inverse_kin_has_solution in a block that does not move; a block stops any program).
capture: for each planned pose: goto_look (set_tcp/set_payload/IK-guarded movej), capture_shot ([camera] settle_s,
  [vision] max_qd_rad_s), image + T_base_flange + q + tcp_pose_ur + max_qd into a "handeye" dataset
  (mauer.vision.dataset); unreachable poses (IK guard, error code 1) are skipped and listed; --resume continues an
  interrupted dataset (same plan required). Prints the ChArUco corner count per shot.
solve: tools/handeye_solve.py on the dataset (hold-out every [vision] holdout_every), writes calib/handeye.json
  (--out, --force). A URSim dataset (meta "truth") also prints the error against the ground truth.
verify: new look poses (seed + 1000), board measured in the base frame with the calibrated T_flange_cam; reports each
  pose's deviation from the calibrated board pose, the spread, and handeye.holdout_check (incl. reprojection).

Real robot: --host is required ([ur].host is an empty PLACEHOLDER), a typed confirmation precedes the first motion
(--yes skips it), speeds come from [ur] (v_joint/a_joint, ASSUMPTION first-run values), and the tool refuses to move
while [ur] payload_tool_kg is 0 (unknown) unless --sim (URSim/simulation only). Ctrl-C aborts the running block.
Run the calibration with the deck magazine EMPTY: the look poses pass ~250-450 mm above the deck around ARES x = 0,
next to the magazine rows (README), and URSim checks no collisions (the clearance column is only a rough DH check).

Poses JSON (written by plan --write, read by measure_target.py poses):
  {"source": str, "frame": "UR base", "units": "mm, rad",
   "poses": [{"name": "p00", "T_base_flange": 4x4 [mm], "qnear_rad": [6] | null}, ...]}
  measure_target.py also accepts {"name", "xyz": [mm], "rpy_deg": [deg]} (flange in base) or {"name", "q_rad": [6]}.
"""
from __future__ import annotations

import argparse
import json
import logging
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

REPO = Path(__file__).resolve().parent.parent
for _p in (REPO, REPO / "tools"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import numpy as np  # noqa: E402

from mauer import config  # noqa: E402
from mauer import geometry as g  # noqa: E402
from mauer.capture import CaptureError, boards_in_base, capture_shot, goto_look, payload  # noqa: E402
from mauer.ur import script  # noqa: E402
from mauer.vision import handeye, intrinsics  # noqa: E402
from mauer.vision.dataset import Dataset  # noqa: E402
from mauer.vision.detect import detect_boards  # noqa: E402
from mauer.vision.targets import BoardSpec, board_specs  # noqa: E402

log = logging.getLogger("calib_handeye")

# Simulator speeds (= tests/test_ur_ursim.py FAST) - URSim only; the real robot always uses [ur].
URSIM_SPEEDS = script.Speeds(v_joint=1.0, a_joint=1.4, v_lin=0.25, a_lin=1.0, v_contact=0.05)
URSIM_HOST = "127.0.0.1"
VERIFY_SEED_OFFSET = 1000           # verify poses: plan seed + this (new poses, not the calibration ones)
MIN_CLEARANCE_MM = 50.0             # ASSUMPTION fallback for [vision.handeye_plan] min_clearance_mm: warn when an
                                    # arm point / camera / TCP comes closer to the deck plane
UNREACHABLE = script.ERR_IK_UNREACHABLE
MAX_START_JUMP_DEG = 60.0           # ASSUMPTION: largest base/shoulder/elbow change from the current joints to the
                                    # first look pose that capture/verify accept on the real robot (arm reconfiguration
                                    # guard); beyond it pass --q-ref-deg with the current joints

# UR5 nominal DH (UR article "DH parameters for calculations of kinematics and dynamics", as in
# tests/test_ur_ursim.py dh_fk_ur5, which matches URSim to < 0.01 mm): d [mm], a [mm], alpha [rad].
UR5_D = (89.159, 0.0, 0.0, 109.15, 94.65, 82.3)
UR5_A = (0.0, -425.0, -392.25, 0.0, 0.0, 0.0)
UR5_ALPHA = (math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0)
# Reference branch for qnear: HOME of tests/test_ur_ursim.py (upper arm up, elbow up, wrist down, tool down) - the
# arm configuration robodk/motion.py uses (README: "elbow up, wrist down"); q1 is turned towards the target.
Q_REF = (0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0)


# ── UR5 nominal kinematics (qnear seeds and offline reachability only) ───────
def _dh(i: int, th: float) -> np.ndarray:
    return g.rotz(th) @ g.transl(0.0, 0.0, UR5_D[i]) @ g.transl(UR5_A[i], 0.0, 0.0) @ g.rotx(UR5_ALPHA[i])


def ur5_fk(q_rad: Sequence[float]) -> list[np.ndarray]:
    """Nominal DH frames base->frame i, i = 0..6 (frame 6 = flange) [mm]."""
    T = np.eye(4)
    out = [T]
    for i, th in enumerate(np.asarray(q_rad, float).reshape(6)):
        T = T @ _dh(i, th)
        out.append(T)
    return out


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def ur5_ik(T_base_flange: np.ndarray, tol_mm: float = 1e-6) -> list[np.ndarray]:
    """All (up to 8) nominal UR5 IK solutions for a flange pose [mm], joints wrapped to [-pi, pi), each verified by FK.
    Closed form after Hawkins, "Analytic Inverse Kinematics for the Universal Robots UR-5/UR-10 Arms" (2013)."""
    T = np.asarray(T_base_flange, float)
    d1, d4, d6 = UR5_D[0], UR5_D[3], UR5_D[5]
    a2, a3 = UR5_A[1], UR5_A[2]
    p05 = T @ np.array([0.0, 0.0, -d6, 1.0])
    r = math.hypot(p05[0], p05[1])
    if r < d4:
        return []
    psi, phi = math.atan2(p05[1], p05[0]), math.acos(d4 / r)
    T60 = g.inv(T)
    sols = []
    for th1 in (psi + phi + math.pi / 2, psi - phi + math.pi / 2):
        s1, c1 = math.sin(th1), math.cos(th1)
        c5 = (T[0, 3] * s1 - T[1, 3] * c1 - d4) / d6
        if abs(c5) > 1.0 + 1e-12:
            continue
        c5 = max(-1.0, min(1.0, c5))
        for th5 in (math.acos(c5), -math.acos(c5)):
            s5 = math.sin(th5)
            th6 = 0.0 if abs(s5) < 1e-9 else math.atan2((-T60[1, 0] * s1 + T60[1, 1] * c1) / s5,
                                                       (T60[0, 0] * s1 - T60[0, 1] * c1) / s5)
            T14 = g.inv(_dh(0, th1)) @ T @ g.inv(_dh(4, th5) @ _dh(5, th6))
            p13 = T14[:3, :3] @ np.array([0.0, -d4, 0.0]) + T14[:3, 3]
            n13 = float(np.linalg.norm(p13))
            c3 = (n13 ** 2 - a2 ** 2 - a3 ** 2) / (2.0 * a2 * a3)
            if abs(c3) > 1.0 + 1e-12 or n13 < 1e-9:
                continue
            c3 = max(-1.0, min(1.0, c3))
            for th3 in (math.acos(c3), -math.acos(c3)):
                th2 = -math.atan2(p13[1], -p13[0]) + math.asin(max(-1.0, min(1.0, a3 * math.sin(th3) / n13)))
                T34 = g.inv(_dh(1, th2) @ _dh(2, th3)) @ T14
                th4 = math.atan2(T34[1, 0], T34[0, 0])
                q = np.array([_wrap(x) for x in (th1, th2, th3, th4, th5, th6)])
                if g.pose_delta(ur5_fk(q)[-1], T)[0] < tol_mm:
                    sols.append(q)
    return sols


def q_reference(target_xy_mm: Sequence[float]) -> np.ndarray:
    """Q_REF with the base joint turned towards a point (x, y) in the base frame (at q1 = 0 the UR5 reaches
    towards base -x)."""
    q = np.array(Q_REF)
    q[0] = math.atan2(-float(target_xy_mm[1]), -float(target_xy_mm[0]))
    return q


def choose_branch(sols: Sequence[np.ndarray], q_ref: Sequence[float]) -> np.ndarray | None:
    """IK solution closest to q_ref in joints 1-5 (wrapped differences; wrist 3 is free)."""
    if not sols:
        return None
    ref = np.asarray(q_ref, float)
    return min(sols, key=lambda q: sum(abs(_wrap(q[i] - ref[i])) for i in range(5)))


# ── look-pose plan ────────────────────────────────────────────────────────────
@dataclass
class LookPose:
    """A planned look pose with the nominal-kinematics checks."""
    index: int
    name: str
    T_base_flange: np.ndarray            # [mm]
    qnear_rad: np.ndarray | None         # nominal IK solution of the reference branch (None: no nominal solution)
    n_ik: int                            # nominal IK solutions found
    clearance_mm: float                  # lowest elbow/wrist/flange/camera/TCP point above the deck plane [mm]
    dist_mm: float = float("nan")        # camera origin -> board centre
    tilt_deg: float = float("nan")       # optical axis vs board normal

    def to_json(self) -> dict:
        return {"name": self.name, "T_base_flange": self.T_base_flange.tolist(),
                "qnear_rad": None if self.qnear_rad is None else self.qnear_rad.tolist()}


def apply_board_pose(cfg: dict, path: str | Path | None) -> dict:
    """--board-pose: the calib board pose measured by `locate` (UR base frame) replaces the [boards.calib] deck pose
    in cfg (in place) for calib_board_in_base / deck_z_in_base; no-op without a path."""
    if path:
        doc = json.loads(config.repo_path(path).read_text(encoding="utf-8"))
        cfg.setdefault("vision", {}).setdefault("handeye_plan", {})["board_pose"] = {
            "T_base_board": [[float(v) for v in row] for row in doc["T_base_board"]], "file": str(path)}
    return cfg


def _board_pose(cfg: dict) -> dict | None:
    return cfg.get("vision", {}).get("handeye_plan", {}).get("board_pose")


def calib_board_in_base(cfg: dict) -> np.ndarray:
    """T_base_board of the calib board: the measured pose of --board-pose (apply_board_pose), else its [boards.calib]
    deck pose (ARES frame, PLACEHOLDER) via T_ares_base."""
    bp = _board_pose(cfg)
    if bp is not None:
        return np.asarray(bp["T_base_board"], float)
    return g.inv(config.T_ares_base(cfg)) @ config.pose(cfg["boards"]["calib"])


def deck_z_in_base(cfg: dict) -> float:
    """Clearance plane in the UR base frame [mm]: the deck top ([ares] deck_top_z - [ur5] mount height; 0 with no
    adapter), or with --board-pose the plane the board lies on (its printed face, the table + plate)."""
    bp = _board_pose(cfg)
    if bp is not None:
        return float(np.asarray(bp["T_base_board"], float)[2, 3])
    return float(cfg["ares"]["deck_top_z"]) - float(config.T_ares_base(cfg)[2, 3])


def annotate(cfg: dict, T_base_flange: np.ndarray, index: int, name: str | None = None,
             T_flange_cam: np.ndarray | None = None, aim_base_mm: Sequence[float] | None = None,
             T_base_board: np.ndarray | None = None, q_ref: Sequence[float] | None = None) -> LookPose:
    """LookPose with the nominal IK branch (qnear), the deck clearance and - when aim_base_mm/T_base_board are
    given - the camera distance to the aim point and the tilt against the board normal. q_ref [rad]: take the branch
    closest to these joints (unwrapped towards them, so the controller's IK near qnear stays there) instead of the
    deck convention q_reference."""
    T = np.asarray(T_base_flange, float)
    X = config.T_flange_cam_nominal(cfg) if T_flange_cam is None else np.asarray(T_flange_cam, float)
    T_ft = config.T_flange_tcp(cfg)
    cam = T @ X
    aim = cam[:3, 3] + 300.0 * cam[:3, 2] if aim_base_mm is None else np.asarray(aim_base_mm, float)
    sols = ur5_ik(T)
    if q_ref is None:
        q = choose_branch(sols, q_reference(aim[:2]))
    else:
        ref = np.asarray(q_ref, float)
        q = choose_branch(sols, ref)
        q = None if q is None else np.array([ref[i] + _wrap(q[i] - ref[i]) for i in range(6)])
    z0 = deck_z_in_base(cfg)
    pts = [cam[2, 3], (T @ T_ft)[2, 3], T[2, 3]]
    if q is not None:
        pts += [F[2, 3] for F in ur5_fk(q)[2:]]
    dist = float(np.linalg.norm(aim - cam[:3, 3]))
    tilt = float("nan")
    if T_base_board is not None:
        tilt = math.degrees(math.acos(float(np.clip(cam[:3, 2] @ T_base_board[:3, 2], -1.0, 1.0))))
    return LookPose(index, name or f"p{index:02d}", T, q, len(sols), float(min(pts) - z0), dist, tilt)


def parse_q_ref(text: str | None) -> np.ndarray | None:
    """--q-ref-deg "q1,...,q6" [deg] -> joints [rad] (None without)."""
    if not text:
        return None
    v = [float(x) for x in text.split(",")]
    if len(v) != 6:
        raise SystemExit(f"--q-ref-deg needs 6 joint angles in degrees, got {len(v)}")
    return np.radians(v)


def plan_settings(cfg: dict, n: int | None = None, seed: int = 0, q_ref: Sequence[float] | None = None) -> dict:
    p = cfg.get("vision", {}).get("handeye_plan", {})
    return {"n": int(p.get("n_poses", 25) if n is None else n), "seed": int(seed),
            "q_ref_deg": None if q_ref is None else [round(float(v), 3) for v in np.degrees(q_ref)],
            "dist_mm": [float(v) for v in p.get("dist_mm", (290.0, 355.0))],
            "tilt_deg": [float(v) for v in p.get("tilt_deg", (15.0, 30.0))],
            "roll_deg": [float(v) for v in p.get("roll_deg", (-90.0, 90.0))],
            "T_base_board": calib_board_in_base(cfg).tolist(),
            "T_flange_cam_nominal": config.T_flange_cam_nominal(cfg).tolist()}


def make_plan(cfg: dict, n: int | None = None, seed: int = 0, prefix: str = "p",
              q_ref: Sequence[float] | None = None) -> list[LookPose]:
    """handeye.plan_poses around the calib board centre with the [vision.handeye_plan] ranges and the nominal
    T_flange_cam (the plan only needs to be roughly right), annotated with the nominal IK and deck clearance."""
    st = plan_settings(cfg, n, seed)
    spec = board_specs(cfg)["calib"]
    T_bb = calib_board_in_base(cfg)
    X = config.T_flange_cam_nominal(cfg)
    roll = tuple(st["roll_deg"])
    if q_ref is not None:       # roll range around the camera's roll at q_ref: wrist 3 stays within ~it of q_ref
        R_bc = T_bb[:3, :3].T @ (ur5_fk(q_ref)[-1] @ X)[:3, :3]
        r0 = math.degrees(math.atan2(R_bc[1, 0], R_bc[0, 0]))
        roll = (r0 + roll[0], r0 + roll[1])
    poses = handeye.plan_poses(T_bb, X, st["n"], tuple(st["dist_mm"]), tuple(st["tilt_deg"]),
                               roll, seed=seed, spec=spec)
    aim = g.apply(T_bb, [[*spec.centre_mm, 0.0]])[0]
    return [annotate(cfg, T, i, f"{prefix}{i:02d}", X, aim, T_bb, q_ref) for i, T in enumerate(poses)]


def start_jump_refusal(q_now: Sequence[float], poses: Sequence[LookPose], limit_deg: float = MAX_START_JUMP_DEG
                       ) -> str | None:
    """None if the first pose with a nominal IK branch is within limit_deg of q_now in base / shoulder / elbow, else
    the refusal message (an arm reconfiguration sweeps across the table/deck)."""
    first = next((p for p in poses if p.qnear_rad is not None), None)
    if first is None:
        return None
    d = [abs(math.degrees(_wrap(float(first.qnear_rad[i]) - float(q_now[i])))) for i in range(3)]
    if max(d) <= limit_deg:
        return None
    now = ",".join(f"{v:.1f}" for v in np.degrees(np.asarray(q_now, float)))
    return (f"refusing to move: the first look pose {first.name} needs base/shoulder/elbow changes of "
            f"{', '.join(f'{v:.0f}' for v in d)} deg (> {limit_deg:g}) - another arm configuration. Plan in the "
            f"current one: --q-ref-deg={now}")


def write_poses_json(path: str | Path, poses: Sequence[LookPose], source: str) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    doc = {"source": source, "frame": "UR base", "units": "mm, rad", "poses": [p.to_json() for p in poses]}
    path.write_text(json.dumps(doc, indent=1) + "\n", encoding="utf-8")
    return path


def load_poses_json(path: str | Path, cfg: dict) -> list[dict]:
    """[{"name", "T_base_flange" (4x4 mm) | None, "q_rad" (6,) | None, "qnear_rad" (6,) | None}, ...] from the poses
    JSON (see the module docstring). Cartesian poses without qnear get the nominal IK branch (None if none)."""
    doc = json.loads(Path(path).read_text(encoding="utf-8"))
    items = doc["poses"] if isinstance(doc, dict) else doc
    out = []
    for i, it in enumerate(items):
        name = str(it.get("name", f"p{i:02d}"))
        T = q = None
        if it.get("T_base_flange") is not None:
            T = np.asarray(it["T_base_flange"], float).reshape(4, 4)
        elif it.get("xyz") is not None:
            T = g.pose_xyz_rpy(it["xyz"], it.get("rpy_deg", (0.0, 0.0, 0.0)))
        elif it.get("q_rad") is not None:
            q = np.asarray(it["q_rad"], float).reshape(6)
        else:
            raise ValueError(f"{path}: pose {name} has neither T_base_flange, xyz nor q_rad")
        qn = it.get("qnear_rad")
        qn = None if qn is None else np.asarray(qn, float).reshape(6)
        if T is not None and qn is None:
            qn = annotate(cfg, T, i, name).qnear_rad
        out.append({"name": name, "T_base_flange": T, "q_rad": q, "qnear_rad": qn})
    return out


# ── formatting ────────────────────────────────────────────────────────────────
def fmt_T(T: np.ndarray) -> str:
    xyz, rpy = g.xyz_rpy(T)
    return ("xyz (" + ", ".join(f"{v:8.2f}" for v in xyz) + ") mm  rpy (" + ", ".join(f"{v:8.3f}" for v in rpy)
            + ") deg")


def fmt_q(q: np.ndarray | None) -> str:
    return "-" if q is None else "[" + ", ".join(f"{v:6.1f}" for v in np.degrees(q)) + "] deg"


def parse_inject(text: str | None) -> np.ndarray:
    """'dx,dy,dz,rx,ry,rz' (mm, deg) -> 4x4 (pose_xyz_rpy); None/'' -> identity."""
    if not text:
        return np.eye(4)
    v = [float(x) for x in text.replace(" ", "").split(",")]
    if len(v) != 6:
        raise argparse.ArgumentTypeError("--inject needs 6 comma-separated values: dx,dy,dz [mm],rx,ry,rz [deg]")
    return g.pose_xyz_rpy(v[:3], v[3:])


# ── robot + camera session ────────────────────────────────────────────────────
def add_robot_args(ap: argparse.ArgumentParser,
                   settle_help: str = "wait before each image [s] (default [camera] settle_s)") -> None:
    """Options shared by the tools that move the robot (also used by tools/measure_target.py)."""
    ap.add_argument("--host", default=None, help="UR controller IP (required for the real robot; [ur].host is an "
                                                 "empty PLACEHOLDER)")
    ap.add_argument("--ursim", action="store_true", help=f"URSim at {URSIM_HOST} (or --host) with the synthetic "
                                                         "camera; simulator speeds, payload fallback")
    ap.add_argument("--start-ursim", action="store_true", help="start the URSim container first (tests/ursim.py; "
                                                               "never pulls; left running)")
    ap.add_argument("--sim", action="store_true", help="allow the simulation payload fallback while [ur] "
                                                       "payload_tool_kg is unknown (NEVER on the real robot)")
    ap.add_argument("--yes", action="store_true", help="skip the confirmation before the first motion")
    ap.add_argument("--camera", choices=["ids", "files", "synth"], default=None,
                    help="camera (default: synth with --ursim, else ids)")
    ap.add_argument("--files", default=None, help="image folder for --camera files")
    ap.add_argument("--inject", default=None, help="synth camera only: true T_flange_cam = nominal @ "
                                                   "pose_xyz_rpy(dx,dy,dz [mm], rx,ry,rz [deg])")
    ap.add_argument("--settle-s", default=None, help=settle_help)
    ap.add_argument("--max-qd", type=float, default=None, help="max |joint speed| during the exposure [rad/s] "
                                                               "(default [vision] max_qd_rad_s)")
    ap.add_argument("--retries", type=int, default=2, help="grabs retried while the arm moves")
    ap.add_argument("--timeout-s", type=float, default=60.0, help="timeout of one look move [s]")


@dataclass
class Rig:
    """Connected robot link + camera (+ ground truth in URSim mode)."""
    cfg: dict
    link: Any
    camera: Any
    sim: bool                              # payload fallback allowed
    ursim: bool
    speeds: script.Speeds
    truth: dict | None = None              # URSim: {"T_flange_cam", "T_base_board", "intrinsics"}
    notes: list[str] = field(default_factory=list)

    def close(self) -> None:
        for fn in (getattr(self.camera, "close", None), getattr(self.link, "stop", None)):
            try:
                if fn is not None:
                    fn()
            except Exception as e:  # closing must not hide the real error
                log.warning("close failed: %s", e)

    def close_popup(self) -> None:
        close_popup(self.link)


def close_popup(link) -> None:
    """Close the PolyScope popup opened by the IK guard (Dashboard 'close popup'; a link object may provide its own
    close_popup(), e.g. a test fake)."""
    from mauer.ur.dashboard import Dashboard, DashboardError
    if hasattr(link, "close_popup"):
        link.close_popup()
        return
    try:
        with Dashboard(link.host, getattr(link, "dashboard_port", 29999), timeout_s=3.0) as d:
            d.close_popup()
    except (OSError, DashboardError) as e:
        print(f"  (could not close the popup: {e})", flush=True)


def settle_value(args, cfg: dict) -> float:
    if args.settle_s is not None:
        return float(str(args.settle_s).split(",")[0])
    return float(cfg.get("camera", {}).get("settle_s", 0.0))


def max_qd_value(args, cfg: dict) -> float:
    return float(cfg.get("vision", {}).get("max_qd_rad_s", 0.01) if args.max_qd is None else args.max_qd)


def resolve_host(args, cfg: dict) -> str:
    host = args.host or (URSIM_HOST if args.ursim else cfg.get("ur", {}).get("host", ""))
    if not host:
        raise SystemExit("no robot host: pass --host <UR IP> ([ur].host in config/station.toml is an empty "
                         "PLACEHOLDER) or --ursim")
    return host


def connect_link(args, cfg: dict):
    """Started URLink to --host / URSim (--start-ursim first starts the container via tests/ursim.py)."""
    from mauer.ur.link import URLink
    host = resolve_host(args, cfg)
    if getattr(args, "start_ursim", False):
        if str(REPO / "tests") not in sys.path:
            sys.path.insert(0, str(REPO / "tests"))
        import ursim as ursim_helper            # tests/ursim.py: never pulls the image
        ok, why = ursim_helper.available()
        if not ok:
            raise SystemExit(f"URSim not available: {why}")
        ursim_helper.start()
    return URLink.from_config(cfg, host=host).start()


def open_rig(args, cfg: dict, boards_true: Sequence[tuple[BoardSpec, np.ndarray]] | None = None) -> Rig:
    """Start the URLink (host from --host / --ursim) and open the camera. URSim: SynthCamera rendering
    `boards_true` (default: the calib board at its nominal deck pose) from the RTDE flange pose with the true
    T_flange_cam = nominal @ --inject and the nominal intrinsics."""
    kind = args.camera or ("synth" if args.ursim else "ids")
    if kind == "synth" and not args.ursim:
        raise SystemExit("--camera synth only with --ursim (it renders from the robot's RTDE pose)")
    link = connect_link(args, cfg)
    truth = None
    try:
        if kind == "synth":
            from mauer.simcam import SynthCamera
            X_true = config.T_flange_cam_nominal(cfg) @ parse_inject(args.inject)
            spec = board_specs(cfg)["calib"]
            boards = list(boards_true) if boards_true is not None else [(spec, calib_board_in_base(cfg))]
            intr = intrinsics.nominal(cfg)
            cam = SynthCamera(lambda: link.flange_T(), X_true, boards, intr)
            cam.open()
            truth = {"T_flange_cam": X_true, "T_base_board": {s.name: T for s, T in boards},
                     "intrinsics": "nominal", "inject": args.inject or ""}
        else:
            from mauer.camera import open_camera
            cam = open_camera(cfg, kind, **({"folder": args.files} if kind == "files" else {}))
    except BaseException:
        link.stop()
        raise
    sim = bool(args.sim or args.ursim)
    speeds = URSIM_SPEEDS if args.ursim else script.Speeds.from_config(cfg)
    return Rig(cfg, link, cam, sim, bool(args.ursim), speeds, truth)


def confirm_motion(args, what: str) -> bool:
    """Typed confirmation before the first motion of the real robot (skipped for --ursim / --yes)."""
    if args.ursim or args.yes:
        return True
    print(f"\nABOUT TO MOVE THE ROBOT: {what}\nCheck: deck magazine empty, nobody in the work space, e-stop in reach, "
          "speed slider on the pendant low.", flush=True)
    try:
        return input("type 'yes' to start: ").strip().lower() == "yes"
    except EOFError:
        return False


def check_payload(rig_or_cfg, sim: bool) -> str | None:
    """None if moving is allowed, else the refusal message."""
    cfg = rig_or_cfg.cfg if isinstance(rig_or_cfg, Rig) else rig_or_cfg
    try:
        payload(cfg, sim=sim)
    except CaptureError as e:
        return str(e)
    return None


def load_intrinsics(args, cfg: dict, ursim: bool = False) -> tuple[intrinsics.Intrinsics, str]:
    """--nominal-intrinsics / --intrinsics / [vision] intrinsics_file; URSim (synthetic images) defaults to nominal."""
    if getattr(args, "nominal_intrinsics", False) or (ursim and not getattr(args, "intrinsics", None)):
        return intrinsics.nominal(cfg), "nominal ([camera] focal / pixel_um)"
    p = config.repo_path(getattr(args, "intrinsics", None) or cfg.get("vision", {}).get(
        "intrinsics_file", "calib/camera_intrinsics.json"))
    if not p.exists():
        raise SystemExit(f"intrinsics file {p} missing - run tools/calib_intrinsics.py solve first "
                         "(or --nominal-intrinsics for synthetic images)")
    return intrinsics.load(p), str(p)


def goto(rig: Rig, T_base_flange=None, q_rad=None, qnear_rad=None, timeout_s: float = 60.0, name: str = "look"):
    """goto_look with the rig's speeds and payload mode."""
    return goto_look(rig.link, rig.cfg, T_base_flange=T_base_flange, q_rad=q_rad, qnear_rad=qnear_rad,
                     timeout_s=timeout_s, sim=rig.sim, speeds=rig.speeds, name=name)


# ── plan ──────────────────────────────────────────────────────────────────────
def print_plan(cfg: dict, poses: Sequence[LookPose]) -> None:
    spec = board_specs(cfg)["calib"]
    T_bb = calib_board_in_base(cfg)
    print(f"calib board {spec.describe()}", flush=True)
    src = (f"measured, --board-pose {_board_pose(cfg)['file']}" if _board_pose(cfg) is not None else
           "[boards.calib] PLACEHOLDER deck pose via [ur5] mount")
    print(f"  T_base_board ({src}): {fmt_T(T_bb)}", flush=True)
    print(f"  board centre in base: ({', '.join(f'{v:.1f}' for v in g.apply(T_bb, [[*spec.centre_mm, 0]])[0])}) mm",
          flush=True)
    print(f"T_flange_cam nominal ([camera.mount] PLACEHOLDER): {fmt_T(config.T_flange_cam_nominal(cfg))}", flush=True)
    lim = float(cfg.get("vision", {}).get("handeye_plan", {}).get("min_clearance_mm", MIN_CLEARANCE_MM))
    print("pose  flange in base                                                      cam dist  tilt  IK  qnear "
          "(nominal UR5, elbow up / wrist down)          deck clearance", flush=True)
    for p in poses:
        warn = "  < MIN" if p.clearance_mm < lim else ""
        print(f"{p.name}  {fmt_T(p.T_base_flange)}  {p.dist_mm:6.1f} {p.tilt_deg:5.1f}  {p.n_ik}  "
              f"{fmt_q(p.qnear_rad)}  {p.clearance_mm:6.0f} mm{warn}", flush=True)
    n_ok = sum(p.qnear_rad is not None for p in poses)
    n_low = sum(p.clearance_mm < lim for p in poses)
    print(f"{n_ok}/{len(poses)} poses with a nominal IK solution; {n_low} closer than {lim:g} mm to the "
          "deck plane (rough DH check, no collision model)", flush=True)


def ik_check_block(cfg: dict, T_base_flange: np.ndarray, qnear_rad: Sequence[float], reg_error: int) -> str:
    """Block body that does not move: set_tcp, the TCP target and the IK guard (halts with error code 1)."""
    T_ft = config.T_flange_tcp(cfg)
    return "\n".join([script.set_tcp(T_ft), f"look = {script.pose(np.asarray(T_base_flange) @ T_ft)}",
                      script.ik_guard("look", qnear_rad, reg_error)])


def cmd_plan(args) -> int:
    cfg = apply_board_pose(config.load(args.config), args.board_pose)
    poses = make_plan(cfg, args.n, args.seed, q_ref=parse_q_ref(args.q_ref_deg))
    print_plan(cfg, poses)
    if args.write:
        p = write_poses_json(args.write, poses, f"calib_handeye plan n={len(poses)} seed={args.seed}")
        print(f"written {p}", flush=True)
    if not args.check:
        return 0
    link = connect_link(args, cfg)
    n_ok = 0
    try:
        reg = script.REG_ERROR if link.reg_error is None else link.reg_error
        for p in poses:
            q = p.qnear_rad if p.qnear_rad is not None else q_reference(p.T_base_flange[:2, 3])
            r = link.run_block(ik_check_block(cfg, p.T_base_flange, q, reg), name="ik_check", timeout_s=10.0)
            if r.ok:
                n_ok += 1
                print(f"{p.name}: controller IK ok", flush=True)
            elif r.error_code == UNREACHABLE:
                print(f"{p.name}: controller says NOT reachable", flush=True)
                close_popup(link)
            else:
                print(f"{p.name}: check failed: {r.error}", flush=True)
                return 1
    finally:
        link.stop()
    print(f"controller IK: {n_ok}/{len(poses)} poses reachable", flush=True)
    return 0


# ── locate ────────────────────────────────────────────────────────────────────
def cmd_locate(args) -> int:
    """The calib board in the UR base frame from the current pose (no motion) -> JSON for --board-pose."""
    from mauer.simworld import ur5_fk as fk_flange
    from mauer.vision.detect import measure
    cfg = config.load(args.config)
    spec = board_specs(cfg)["calib"]
    intr, intr_label = load_intrinsics(args, cfg, args.ursim)
    X = config.T_flange_cam_nominal(cfg)
    rig = open_rig(args, cfg)
    try:
        st = rig.link.state()
        try:
            T_bf, src = rig.link.flange_T(st), f"controller TCP pose ({rig.link.tcp_source})"
        except ValueError:                  # PolyScope 3.3: no tcp_offset and no set_tcp sent by this link yet
            T_bf, src = fk_flange(st.actual_q), "actual_q with the nominal UR5 DH (~1 mm)"
        bp = measure(rig.camera.grab().image, {"calib": spec}, intr, cfg.get("vision", {}))["calib"]
    finally:
        rig.close()
    if not bp.ok:
        print(f"calib board not measured ({bp.n_corners} corners, {bp.reason}) - put it in view", flush=True)
        return 1
    T_bb = T_bf @ X @ bp.T_cam_board
    centre = g.apply(T_bb, [[*spec.centre_mm, 0.0]])[0]
    tilt = math.degrees(math.acos(min(1.0, abs(float(T_bb[2, 2])))))
    print(f"calib board: {bp.n_corners} corners, RMS {bp.rms_px:.2f} px; flange from {src}; nominal [camera.mount], "
          f"intrinsics {intr_label}", flush=True)
    print(f"  T_base_board {fmt_T(T_bb)}", flush=True)
    print(f"  centre ({', '.join(f'{v:.1f}' for v in centre)}) mm, normal {tilt:.1f} deg from vertical", flush=True)
    if args.write:
        out = config.repo_path(args.write)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps({"source": "calib_handeye locate", "frame": "UR base", "units": "mm",
                                   "T_base_board": T_bb.tolist(), "flange_source": src, "intrinsics": intr_label,
                                   "T_flange_cam": "nominal [camera.mount]", "n_corners": bp.n_corners,
                                   "rms_px": bp.rms_px}, indent=1) + "\n", encoding="utf-8")
        print(f"written {out} (use: --board-pose {args.write})", flush=True)
    return 0


# ── capture ───────────────────────────────────────────────────────────────────
def capture_poses(rig: Rig, poses: Sequence[LookPose], ds: Dataset, spec: BoardSpec, settle_s: float,
                  max_qd: float, retries: int, timeout_s: float, done: set[int] | None = None) -> dict:
    """Move to every pose not in `done`, capture, store into ds. Returns {"captured", "unreachable", "failed",
    "skipped_done"} (lists of pose names). Stops at a robot failure other than an unreachable pose."""
    border = float(rig.cfg.get("vision", {}).get("border_px", 10))
    out = {"captured": [], "unreachable": [], "failed": [], "skipped_done": []}
    done = done or set()
    for p in poses:
        if p.index in done:
            out["skipped_done"].append(p.name)
            continue
        res = goto(rig, p.T_base_flange, qnear_rad=p.qnear_rad if p.qnear_rad is not None else
                   q_reference(p.T_base_flange[:2, 3]), timeout_s=timeout_s, name="he_look")
        if not res.ok:
            if res.error_code == UNREACHABLE:
                print(f"{p.name}: unreachable (controller IK guard) - skipped", flush=True)
                out["unreachable"].append(p.name)
                rig.close_popup()
                continue
            print(f"{p.name}: look move FAILED: {res.error} - stopping", flush=True)
            out["failed"].append(p.name)
            out["stopped"] = res.error
            break
        try:
            shot = capture_shot(rig.link, rig.camera, settle_s=settle_s, max_qd=max_qd, retries=retries)
        except CaptureError as e:
            print(f"{p.name}: no usable image: {e}", flush=True)
            out["failed"].append(p.name)
            continue
        det = detect_boards(shot.frame.image, [spec], border).get(spec.name)
        n = 0 if det is None else det.n
        d_mm, d_deg = g.pose_delta(p.T_base_flange, shot.T_base_flange)
        ds.add(shot.frame.image, **shot.dataset_fields(), pose_index=p.index, pose_name=p.name,
               T_base_flange_planned=p.T_base_flange, corners=n)
        out["captured"].append(p.name)
        print(f"{p.name}: {n:3d} corners | flange vs plan {d_mm:.3f} mm / {d_deg:.4f} deg | {shot.n_samples} RTDE "
              f"samples ({shot.match}), max |qd| {shot.max_qd:.2e} rad/s, attempts {shot.attempts}", flush=True)
    return out


def cmd_capture(args) -> int:
    cfg = apply_board_pose(config.load(args.config), args.board_pose)
    q_ref = parse_q_ref(args.q_ref_deg)
    settings = plan_settings(cfg, args.n, args.seed, q_ref)
    poses = make_plan(cfg, args.n, args.seed, q_ref=q_ref)
    spec = board_specs(cfg)["calib"]
    path = config.repo_path(args.dataset)
    done: set[int] = set()
    if (path / "meta.json").exists():
        if not args.resume:
            print(f"dataset {path} exists - use --resume to continue it or another --dataset", flush=True)
            return 2
        old_ds = Dataset.load(path)
        old = old_ds.meta.get("plan")
        if old_ds.kind != "handeye" or (old is not None and old != json.loads(json.dumps(settings))):
            print(f"dataset {path} was captured with another plan or kind ({old_ds.kind}, "
                  f"{ {k: (old or {}).get(k) for k in ('n', 'seed')} }) - refusing to mix poses; use the same "
                  "--n/--seed/config or a new --dataset", flush=True)
            return 2
        done = {int(s["pose_index"]) for s in old_ds.samples if s.get("pose_index") is not None}
    todo = [p for p in poses if p.index not in done]
    print(f"dataset {path}: {len(done)} poses done, {len(todo)} of {len(poses)} planned poses to go", flush=True)
    if not todo:
        print("nothing to do (all poses captured)", flush=True)
        return 0
    refusal = check_payload(cfg, args.sim or args.ursim)
    if refusal:
        print(refusal, flush=True)
        return 2
    rig = open_rig(args, cfg)
    try:
        ds = Dataset.create(path, "handeye", camera=rig.camera.info(), board="calib", exist_ok=args.resume,
                            notes="tools/calib_handeye.py capture" + (" (URSim + SynthCamera)" if args.ursim else ""))
        ds.update_meta(plan=settings, camera=rig.camera.info(), robot=rig.link.info())
        if rig.truth is not None:
            old_truth = ds.meta.get("truth")
            if old_truth is not None and not np.allclose(old_truth["T_flange_cam"], rig.truth["T_flange_cam"]):
                print("resume with another --inject than the dataset's ground truth - refusing", flush=True)
                return 2
            ds.update_meta(truth=rig.truth)
        jump = None if args.ursim else start_jump_refusal(rig.link.state().actual_q, todo)
        if jump:
            print(jump, flush=True)
            return 2
        if not confirm_motion(args, f"{len(todo)} look poses around the calib board at "
                                    f"v_joint {rig.speeds.v_joint} rad/s"):
            print("not confirmed - nothing moved", flush=True)
            return 1
        try:
            out = capture_poses(rig, poses, ds, spec, settle_value(args, cfg), max_qd_value(args, cfg),
                                args.retries, args.timeout_s, done)
        except KeyboardInterrupt:
            print(f"interrupted - abort: {rig.link.abort()}", flush=True)
            return 130
        unreachable = sorted((set(ds.meta.get("unreachable", [])) - set(out["captured"])) | set(out["unreachable"]))
        ds.update_meta(unreachable=unreachable)
    finally:
        rig.close()
    n_reach = len(poses) - len(unreachable)
    print(f"captured {len(out['captured'])} now, {len(ds)} images in the dataset; reachable {n_reach}/{len(poses)} "
          f"(unreachable: {', '.join(unreachable) or 'none'}); failed {len(out['failed'])}", flush=True)
    if len(ds) < 15:
        print("WARNING: fewer than 15 images (research: 20-25 poses) - add poses (--n, another --seed and a new "
              "dataset) or move the board where more poses are reachable", flush=True)
    return 1 if out.get("stopped") else 0


# ── solve ─────────────────────────────────────────────────────────────────────
def cmd_solve(args) -> int:
    import handeye_solve
    cfg = config.load(args.config)
    ds = Dataset.load(config.repo_path(args.dataset))
    truth = ds.meta.get("truth")
    argv = [str(ds.path), "--method", args.method]
    for opt, val in (("--config", args.config), ("--intrinsics", args.intrinsics), ("--out", args.out),
                     ("--holdout-every", args.holdout_every), ("--max-qd", args.max_qd)):
        if val is not None:
            argv += [opt, str(val)]
    nominal = args.nominal_intrinsics or (truth is not None and truth.get("intrinsics") == "nominal"
                                          and not args.intrinsics)
    if nominal:
        argv.append("--nominal-intrinsics")
        if not args.nominal_intrinsics:
            print("synthetic dataset rendered with the nominal intrinsics -> --nominal-intrinsics", flush=True)
    argv += [f for f, on in (("--force", args.force), ("--dry-run", args.dry_run)) if on]
    rc = handeye_solve.main(argv)
    if rc != 0 or truth is None or args.dry_run:
        return rc
    out = config.repo_path(args.out or cfg.get("vision", {}).get("handeye_file", "calib/handeye.json"))
    res = handeye.load(out)
    X_true = np.asarray(truth["T_flange_cam"], float)
    dp, da = g.pose_delta(X_true, res.T_flange_cam)
    print(f"vs ground truth (URSim, inject {truth.get('inject') or 'none'}): {dp:.4f} mm / {da:.5f} deg", flush=True)
    T_bb_true = (truth.get("T_base_board") or {}).get("calib")
    if T_bb_true is not None:
        bp, ba = g.pose_delta(np.asarray(T_bb_true, float), res.T_base_board)
        print(f"board in base vs ground truth: {bp:.4f} mm / {ba:.5f} deg", flush=True)
    return 0


# ── verify ────────────────────────────────────────────────────────────────────
def measure_poses(rig: Rig, poses: Sequence[dict], specs: dict, intr, T_flange_cam: np.ndarray, settle_s: float,
                  max_qd: float, retries: int, timeout_s: float, ds: Dataset | None = None) -> list[dict]:
    """goto_look + capture_shot + boards_in_base per pose dict ({"name", "T_base_flange" | "q_rad", "qnear_rad"}).
    Returns [{"name", "ok", "reason", "shot", "boards": {name: (BoardPose, T_base_board | None)}}]."""
    vcfg = rig.cfg.get("vision", {})
    out = []
    for p in poses:
        res = goto(rig, p.get("T_base_flange"), p.get("q_rad"), p.get("qnear_rad"), timeout_s, name="meas_look")
        if not res.ok:
            if res.error_code == UNREACHABLE:
                print(f"{p['name']}: unreachable (controller IK guard) - skipped", flush=True)
                rig.close_popup()
                out.append({"name": p["name"], "ok": False, "reason": "unreachable"})
                continue
            print(f"{p['name']}: look move FAILED: {res.error} - stopping", flush=True)
            out.append({"name": p["name"], "ok": False, "reason": f"move failed: {res.error}", "stop": True})
            break
        try:
            shot = capture_shot(rig.link, rig.camera, settle_s=settle_s, max_qd=max_qd, retries=retries)
        except CaptureError as e:
            print(f"{p['name']}: no usable image: {e}", flush=True)
            out.append({"name": p["name"], "ok": False, "reason": str(e)})
            continue
        seen = boards_in_base(shot, specs, intr, vcfg, T_flange_cam)
        if ds is not None:
            ds.add(shot.frame.image, **shot.dataset_fields(), pose_name=p["name"],
                   boards={k: {"ok": bp.ok, "n_corners": bp.n_corners, "rms_px": bp.rms_px,
                               "T_cam_board": bp.T_cam_board, "T_base_board": T} for k, (bp, T) in seen.items()})
        out.append({"name": p["name"], "ok": True, "reason": "", "shot": shot, "boards": seen})
    return out


def cmd_verify(args) -> int:
    cfg = apply_board_pose(config.load(args.config), args.board_pose)
    he_path = config.repo_path(args.handeye or cfg.get("vision", {}).get("handeye_file", "calib/handeye.json"))
    if not he_path.exists():
        print(f"{he_path} missing - run 'solve' first", flush=True)
        return 1
    res = handeye.load(he_path)
    intr, intr_label = load_intrinsics(args, cfg, args.ursim)
    spec = board_specs(cfg)["calib"]
    seed = args.seed if args.seed is not None else VERIFY_SEED_OFFSET
    poses = make_plan(cfg, args.n, seed, prefix="v", q_ref=parse_q_ref(args.q_ref_deg))
    print(f"verify {he_path} ({res.method}, {res.n_poses} poses) on {len(poses)} new poses (seed {seed}), "
          f"intrinsics {intr_label}", flush=True)
    refusal = check_payload(cfg, args.sim or args.ursim)
    if refusal:
        print(refusal, flush=True)
        return 2
    rig = open_rig(args, cfg)
    try:
        jump = None if args.ursim else start_jump_refusal(rig.link.state().actual_q, poses)
        if jump:
            print(jump, flush=True)
            return 2
        if not confirm_motion(args, f"{len(poses)} verification poses around the calib board"):
            print("not confirmed - nothing moved", flush=True)
            return 1
        ds = None
        if args.dataset:
            try:
                ds = Dataset.create(config.repo_path(args.dataset), "measure", camera=rig.camera.info(),
                                    board="calib", notes=f"calib_handeye verify of {he_path}")
            except FileExistsError as e:
                print(f"{e} - choose another --dataset", flush=True)
                return 2
        try:
            items = [{"name": p.name, "T_base_flange": p.T_base_flange, "qnear_rad": p.qnear_rad} for p in poses]
            meas = measure_poses(rig, items, {"calib": spec}, intr, res.T_flange_cam, settle_value(args, cfg),
                                 max_qd_value(args, cfg), args.retries, args.timeout_s, ds)
        except KeyboardInterrupt:
            print(f"interrupted - abort: {rig.link.abort()}", flush=True)
            return 130
        truth = rig.truth
    finally:
        rig.close()
    A, B, names = [], [], []
    for m in meas:
        if not m["ok"]:
            continue
        bp, T = m["boards"]["calib"]
        if T is None:
            print(f"{m['name']}: board not measured ({bp.reason})", flush=True)
            continue
        A.append(m["shot"].T_base_flange)
        B.append(bp.T_cam_board)
        names.append(m["name"])
        dp, da = g.pose_delta(res.T_base_board, T)
        print(f"{m['name']}: {bp.n_corners} corners, RMS {bp.rms_px:.3f} px | board vs calibrated pose "
              f"{dp:.3f} mm / {da:.4f} deg", flush=True)
    if not A:
        print("no valid measurement", flush=True)
        return 1
    ho = handeye.holdout_check(res, A, B, spec, intr)
    print(f"{ho['n']} verification poses: board deviation from the calibrated pose rms {ho['pos_rms_mm']:.3f} / max "
          f"{ho['pos_max_mm']:.3f} mm, ang rms {ho['ang_rms_deg']:.4f} / max {ho['ang_max_deg']:.4f} deg; "
          f"reprojection rms {ho['reproj_rms_px']:.2f} / max {ho['reproj_max_px']:.2f} px", flush=True)
    Ts = handeye.board_in_base(A, res.T_flange_cam, B)
    if len(Ts) > 1:
        sp = g.spread(Ts)
        print(f"spread of the new measurements: pos rms {sp['pos_rms_mm']:.3f} / max {sp['pos_max_mm']:.3f} mm, ang "
              f"rms {sp['ang_rms_deg']:.4f} / max {sp['ang_max_deg']:.4f} deg", flush=True)
    if truth is not None:
        T_true = truth["T_base_board"]["calib"]
        d = np.array([g.pose_delta(T_true, T) for T in Ts])
        print(f"vs ground truth board pose: max {d[:, 0].max():.3f} mm / {d[:, 1].max():.4f} deg", flush=True)
    ok = ho["pos_max_mm"] <= args.tol_mm and ho["ang_max_deg"] <= args.tol_deg
    print(f"{'PASS' if ok else 'FAIL'}: max deviation vs --tol-mm {args.tol_mm} / --tol-deg {args.tol_deg} "
          "(ASSUMPTION thresholds)", flush=True)
    return 0 if ok else 3


# ── main ──────────────────────────────────────────────────────────────────────
Q_REF_HELP = ("IK branch closest to these joints 'q1,...,q6' [deg] (the robot's current ones) instead of the deck "
              "convention elbow up / wrist down")
BOARD_POSE_HELP = "calib board pose JSON from 'locate' (board on a table) instead of the [boards.calib] deck pose"


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None, help="station.toml (default config/station.toml)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("locate", help="measure the calib board in the base frame from the current pose (no motion)")
    p.add_argument("--intrinsics", default=None)
    p.add_argument("--nominal-intrinsics", action="store_true")
    p.add_argument("--write", default="data/board_table.json", help="output JSON for --board-pose")
    add_robot_args(p)

    p = sub.add_parser("plan", help="look poses around the deck calib board (optionally checked on the controller)")
    p.add_argument("--n", type=int, default=None, help="number of poses (default [vision.handeye_plan] n_poses)")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--write", default=None, help="write the poses as JSON (for measure_target.py poses)")
    p.add_argument("--check", action="store_true", help="ask the controller (get_inverse_kin_has_solution, no "
                                                        "motion) - needs --host or --ursim")
    p.add_argument("--board-pose", default=None, help=BOARD_POSE_HELP)
    p.add_argument("--q-ref-deg", default=None, help=Q_REF_HELP)
    add_robot_args(p)

    p = sub.add_parser("capture", help="move through the plan and record a 'handeye' dataset")
    p.add_argument("--dataset", required=True, help="dataset folder, e.g. data/he_2026-10-06")
    p.add_argument("--resume", action="store_true", help="continue an existing dataset (same plan)")
    p.add_argument("--n", type=int, default=None, help="number of poses (default [vision.handeye_plan] n_poses)")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--board-pose", default=None, help=BOARD_POSE_HELP)
    p.add_argument("--q-ref-deg", default=None, help=Q_REF_HELP)
    add_robot_args(p)

    p = sub.add_parser("solve", help="calibrate T_flange_cam from a dataset (tools/handeye_solve.py)")
    p.add_argument("dataset")
    p.add_argument("--intrinsics", default=None)
    p.add_argument("--nominal-intrinsics", action="store_true")
    p.add_argument("--method", default=handeye.DEFAULT_METHOD, choices=handeye.METHODS)
    p.add_argument("--holdout-every", type=int, default=None)
    p.add_argument("--max-qd", type=float, default=None)
    p.add_argument("--out", default=None, help="output JSON (default [vision] handeye_file)")
    p.add_argument("--force", action="store_true")
    p.add_argument("--dry-run", action="store_true")

    p = sub.add_parser("verify", help="new poses: board in the base frame with the calibrated T_flange_cam")
    p.add_argument("--handeye", default=None, help="calibration JSON (default [vision] handeye_file)")
    p.add_argument("--n", type=int, default=5, help="number of verification poses")
    p.add_argument("--seed", type=int, default=None, help=f"plan seed (default {VERIFY_SEED_OFFSET})")
    p.add_argument("--intrinsics", default=None)
    p.add_argument("--nominal-intrinsics", action="store_true")
    p.add_argument("--dataset", default=None, help="also store the images as a 'measure' dataset")
    p.add_argument("--tol-mm", type=float, default=0.5, help="ASSUMPTION pass threshold, max deviation [mm]")
    p.add_argument("--tol-deg", type=float, default=0.1, help="ASSUMPTION pass threshold [deg]")
    p.add_argument("--board-pose", default=None, help=BOARD_POSE_HELP)
    p.add_argument("--q-ref-deg", default=None, help=Q_REF_HELP)
    add_robot_args(p)
    return ap


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s")
    logging.getLogger("rtde").setLevel(logging.ERROR)
    return {"locate": cmd_locate, "plan": cmd_plan, "capture": cmd_capture, "solve": cmd_solve,
            "verify": cmd_verify}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())

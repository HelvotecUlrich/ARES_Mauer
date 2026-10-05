"""Show the camera setup: station with ARES at a stop, the flange camera on its adapter, the reference boards, and
what the camera sees from the first look pose of the job.

Runs in its OWN RoboDK instance (the user's RoboDK is not touched), recomputes the reach table if the tool/config
changed (simulate.reach_table cache), builds the nominal job (tools/make_job.py), poses ARES at the stop and the arm
at the first look pose, and writes:
    results/setup_overview.png      ARES at the stop, wall, boards, pick-up station
    results/setup_top.png           top view of the stop: ARES front edge, boards in the gap, wall
    results/setup_flange.png        flange close-up: adapter plate, camera, gripper
    results/setup_camera_view.png   the flange camera's image (RoboDK pinhole) with the detected ChArUco corners
and saves the station to robodk/ARES_UR5_Mauer.rdk (with [[wall.legs]]: robodk/ARES_UR5_Mauer_L.rdk - the L never
overwrites the straight-wall station).

    py.exe robodk/show_setup.py [--stop 0] [--look 0] [--keep-open]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parent / "tools"))

import build_station  # noqa: E402
import simulate  # noqa: E402
from rdk_common import REPO, STATION_NAME, camera_params, close_instance, connect, load_config, snapshot  # noqa: E402

from mauer import geometry as g  # noqa: E402
from mauer.vision import detect, targets  # noqa: E402


def main(argv: list | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--stop", type=int, default=0, help="stop index of the nominal job")
    ap.add_argument("--look", type=int, default=0, help="look index within the stop")
    ap.add_argument("--keep-open", action="store_true", help="leave the RoboDK instance open (close it yourself)")
    args = ap.parse_args(argv)

    import cv2
    from make_job import build_nominal

    cfg = load_config()
    res = REPO / "results"
    RDK = connect(new_instance=True, minimized=False)
    try:
        it = build_station.build(RDK, cfg)
        dist = cfg["wall"]["dist_nominal"]
        simulate.reach_table(RDK, cfg, dist)          # recomputes only when the cache key changed
        job = build_nominal(cfg)
        stop = job.stops[args.stop]
        look = stop.looks[args.look]
        print(f"stop {stop.index}: ARES at wall u = {stop.a_mm:.0f} mm; look {look.name} -> boards {look.boards}",
              flush=True)

        # ARES at the stop: wall frame fixed in the world, ARES frame placed from its nominal pose in the wall frame
        f_wall, f_ares, robot = it["f_wall"], it["f_ares"], it["robot"]
        f_ares.setPose(f_wall.Pose() * g.to_robodk(stop.ares.T))
        q = look.q_rad if look.q_rad is not None else look.qnear_rad
        robot.setJoints(np.degrees(q).tolist())

        T_world_base = robot.Parent().PoseAbs()
        T_world_flange = T_world_base * robot.SolveFK(robot.Joints())
        T_world_ares = f_ares.PoseAbs()
        p = lambda T, x, y, z: [float(c) for c in (T * [x, y, z])]   # noqa: E731

        target = p(T_world_ares, 700.0, 0.0, 150.0)
        snapshot(RDK, res / "setup_overview.png", p(T_world_ares, -1300.0, 1500.0, 2000.0), target)
        # top view of the stop: ARES front, gap with the boards, wall
        snapshot(RDK, res / "setup_top.png", p(T_world_ares, 450.0, 0.0, 3200.0), p(T_world_ares, 700.0, 0.0, 0.0))
        # flange close-up in a neutral arm pose (tool down, away from the wall), seen from the camera side
        q_look = robot.Joints()
        robot.setJoints([90.0, -110.0, 110.0, -90.0, -90.0, 0.0])
        T_wf = T_world_base * robot.SolveFK(robot.Joints())
        snapshot(RDK, res / "setup_flange.png", p(T_wf, -330.0, -330.0, 120.0), p(T_wf, -70.0, 0.0, 50.0),
                 size="1200x900")
        robot.setJoints(q_look)

        # the flange camera's own image (camera body and adapter hidden: the camera frame lies inside the lens model)
        hidden = [it[k] for k in ("cam_body", "cam_adapter") if k in it]
        for h in hidden:
            h.setVisible(False)
        cam = RDK.Cam2D_Add(it["cam_tool"], camera_params(cfg, color=False))
        RDK.Render(True)
        img = cv2.imdecode(np.frombuffer(RDK.Cam2D_Snapshot("", cam), np.uint8), cv2.IMREAD_GRAYSCALE)
        img = cv2.imdecode(np.frombuffer(RDK.Cam2D_Snapshot("", cam), np.uint8), cv2.IMREAD_GRAYSCALE)  # 2nd = warm
        RDK.Cam2D_Close(cam)
        cam.Delete()
        for h in hidden:
            h.setVisible(True)
        specs = targets.board_specs(cfg)
        dets = detect.detect_boards(img, specs, cfg["vision"]["border_px"])
        seen = {n: d.n for n, d in dets.items() if d.n}
        print(f"camera image {img.shape[1]}x{img.shape[0]}: ChArUco corners per board {seen}", flush=True)
        cv2.imwrite(str(res / "setup_camera_view.png"), detect.draw_detections(img, dets, scale=0.5))

        out = REPO / "robodk" / f"{STATION_NAME}{'_L' if cfg['wall'].get('legs') else ''}.rdk"
        RDK.Save(str(out), it["station"])
        print("Saved", out, flush=True)
    finally:
        if not args.keep_open:
            close_instance(RDK)
    return 0


def stop_board_T(cfg: dict, name: str) -> np.ndarray:
    """T_wall_board of a [[targets]] wall board (leg boards of the L composed with their leg frame)."""
    from mauer.reference import placements
    for p in placements(cfg):
        if p.name == name:
            return p.T_parent_board
    raise KeyError(name)


if __name__ == "__main__":
    sys.exit(main())

"""The UR at the vehicle REAR ([ares] frame_x_points_to = "rear", Samuel 2026-10-08): this repo's ARES frame is
base_link turned by 180 deg about z. ARES moves cross into the PLC's base_link in mauer.backends.AdsAres (body-frame
translation negated, rotation the same) and the PLC odometry comes back as the pose of this repo's frame
(mauer.ares.ads.odom_in_ares_frame) - for the sequencer and for the HMI's live pose / Apply odometry."""
from __future__ import annotations

import copy
import math
from dataclasses import replace

import pytest

from hmi.core.sources import ares_live, compose_odometry
from mauer import config
from mauer.ares.ads import MoveOutcome, OdomPose, odom_in_ares_frame, vehicle_words
from mauer.backends import AdsAres
from mauer.reference import Pose2D


def _outcome(kind: str, dx: float, dy: float, dth: float, before: OdomPose, after: OdomPose, ok: bool = True):
    return MoveOutcome(ok=ok, cmd_id=1, cmd_result=0, state=4, result=1, final_err_mm=0.0, text="", elapsed_s=1.0,
                       limited=False, odom_before=before, odom_after=after, kind=kind, dx_mm=dx, dy_mm=dy,
                       dtheta_deg=dth)


class _Ads:
    """AresAds stand-in: records the commands and moves its odometry like the PLC (base_link body frame)."""

    def __init__(self, odom: OdomPose):
        self.odom, self.sent = odom, []

    def translate(self, dx_mm, dy_mm):
        self.sent.append(("translate", dx_mm, dy_mm))
        b, t = self.odom, math.radians(self.odom.theta_deg)
        self.odom = OdomPose(b.x_mm + math.cos(t) * dx_mm - math.sin(t) * dy_mm,
                             b.y_mm + math.sin(t) * dx_mm + math.cos(t) * dy_mm, b.theta_deg)
        return _outcome("translate", dx_mm, dy_mm, 0.0, b, self.odom)

    def rotate(self, dtheta_deg):
        self.sent.append(("rotate", dtheta_deg))
        b = self.odom
        self.odom = OdomPose(b.x_mm, b.y_mm, (b.theta_deg + dtheta_deg + 180.0) % 360.0 - 180.0)
        return _outcome("rotate", 0.0, 0.0, dtheta_deg, b, self.odom)


def test_ares_rear_reads_the_config():
    cfg = config.load()
    assert cfg["ares"]["frame_x_points_to"] == "rear" and config.ares_rear(cfg)          # station.toml 2026-10-08
    for value, rear in (("front", False), ("rear", True)):
        c = copy.deepcopy(cfg)
        c["ares"]["frame_x_points_to"] = value
        assert config.ares_rear(c) is rear
    c = copy.deepcopy(cfg)
    del c["ares"]["frame_x_points_to"]
    assert config.ares_rear(c) is False                                                  # the plan until 2026-10-08
    c["ares"]["frame_x_points_to"] = "left"
    with pytest.raises(ValueError, match="frame_x_points_to"):
        config.ares_rear(c)


@pytest.mark.parametrize("b, a", [(OdomPose(0.0, 0.0, 0.0), OdomPose(-500.0, -100.0, 0.0)),
                                  (OdomPose(1200.0, -300.0, 37.0), OdomPose(800.0, 50.0, 49.5)),
                                  (OdomPose(-40.0, 900.0, 170.0), OdomPose(10.0, 950.0, -175.0))])
def test_odometry_in_the_ares_frame_negates_the_body_translation(b, a):
    dx, dy, dth = b.delta_to(a)
    rb, ra = odom_in_ares_frame(b, True), odom_in_ares_frame(a, True)
    assert rb.delta_to(ra) == pytest.approx((-dx, -dy, dth))
    assert odom_in_ares_frame(b, False) is b and odom_in_ares_frame(None, True) is None


def test_ads_ares_rear_sends_base_link_moves_and_returns_the_ares_frame():
    ads = _Ads(OdomPose(1000.0, 2000.0, 30.0))
    ares = AdsAres(ads, rear=True)
    out = ares.translate(500.0, 120.0)                      # repo frame: towards the UR end = vehicle BACKWARD
    assert ads.sent == [("translate", -500.0, -120.0)]
    assert (out.dx_mm, out.dy_mm) == (500.0, 120.0) and out.ok
    assert out.odom_delta() == pytest.approx((500.0, 120.0, 0.0))
    out = ares.rotate(-12.5)                                # a turn about z is the same in both frames
    assert ads.sent[-1] == ("rotate", -12.5) and out.dtheta_deg == -12.5
    assert out.odom_delta() == pytest.approx((0.0, 0.0, -12.5))
    front = _Ads(OdomPose(0.0, 0.0, 0.0))
    out = AdsAres(front).translate(500.0, 120.0)
    assert front.sent == [("translate", 500.0, 120.0)] and out.odom_delta() == pytest.approx((500.0, 120.0, 0.0))


def test_vehicle_words():
    assert vehicle_words(-500.0, 0.0) == "500 mm BACKWARD"
    assert vehicle_words(300.0, -120.0) == "300 mm FORWARD, 120 mm RIGHT"
    assert vehicle_words(0.4, 80.0) == "80 mm LEFT"


def test_sequencer_follows_a_failed_rear_move_by_the_odometry():
    """A move that ends not ok: Sequencer._from_odometry composes the outcome's odometry - in the ARES frame."""
    from mauer.sequencer import Sequencer
    seq = Sequencer.__new__(Sequencer)
    seq.pose_est, seq.dr = Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 0.0, 0.0).T
    seq.log = type("L", (), {"write": lambda *a, **k: None})()
    ads = _Ads(OdomPose(0.0, 0.0, 90.0))
    out = AdsAres(ads, rear=True).translate(300.0, 0.0)
    seq._from_odometry(replace(out, ok=False), "translate")
    assert (seq.pose_est.x_mm, seq.pose_est.y_mm) == pytest.approx((300.0, 0.0))


def test_hmi_live_pose_and_apply_odometry_in_the_ares_frame():
    p0 = Pose2D(1000.0, 840.0, -math.pi / 2)              # ARES frame +x towards wall frame -y
    o0 = OdomPose(5000.0, 2000.0, 0.0)
    o1 = OdomPose(4700.0, 2000.0, 0.0)                    # base_link 300 mm backward = ARES frame +x 300 mm
    assert (compose_odometry(p0, o0, o1, rear=True).x_mm, compose_odometry(p0, o0, o1, rear=True).y_mm) == \
        pytest.approx((1000.0, 540.0))
    assert compose_odometry(p0, o0, o1).y_mm == pytest.approx(1140.0)       # front: the other way
    from hmi.core.snapshot import RunSnapshot
    from hmi_fakes import short_sim_session
    snap = replace(RunSnapshot.initial(short_sim_session()), pose_est=p0, pose_src="predicted",
                   ares_cmd={"pose": p0, "odom": o0})
    st = {"fPosX_m": 4.7, "fPosY_m": 2.0, "fPosTheta_deg": 0.0}
    live = ares_live(snap, st, rear=True)
    assert (live.pose.x_mm, live.pose.y_mm) == pytest.approx((1000.0, 540.0))

"""Peg-hole fit test (2026-10-08): a small 4 mm MDF strip with one row of peg holes of increasing diameter, to find
the press fit of the printed locating cone's peg ([guides] peg_d) before the floor guides are cut.

    py.exe tools/make_peg_test.py                      # 7.5 .. 8.4 mm in 0.1 mm steps -> targets/guides/peg_fit_test.dxf
    py.exe tools/make_peg_test.py --from 7.7 --to 8.2 --step 0.05

The engraved number under a hole is its FINISHED diameter: the cut circle is drawn kerf ([guides] kerf_mm, ASSUMPTION)
smaller, as tools/make_guides.py cuts the peg holes. Push a printed cone (targets/guides/locating_cone.scad) into each
hole: the smallest hole the peg goes into by hand without play is the press fit - enter that value as [guides]
peg_hole_d (and correct kerf_mm if the holes come out visibly larger or smaller than labelled, e.g. measured with a
drill bit / pin gauge). Layers CUT red / ENGRAVE blue as tools/make_plates.py (DXF text engraves on the Trotec).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
import make_plates as mp  # noqa: E402

from mauer import config  # noqa: E402

PITCH_MM = 16.0             # hole centre spacing: >= 7 mm MDF web between 8.5 mm holes
MARGIN_MM = 8.0             # strip edge to the first / last hole centre minus half a pitch
HEIGHT_MM = 36.0            # strip height: holes at y 22, labels at y 8, title at the top
TEXT_MM = 4.0               # label height


def diameters(d_from: float, d_to: float, step: float) -> list[float]:
    n = int(round((d_to - d_from) / step))
    if n < 0 or step <= 0:
        raise ValueError("need --from <= --to and --step > 0")
    return [round(d_from + i * step, 3) for i in range(n + 1)]


def build(ds: list[float], kerf: float, peg_d: float, decimals: int = 1) -> mp.Dxf:
    d = mp.Dxf()
    w = 2 * MARGIN_MM + len(ds) * PITCH_MM
    k = kerf / 2.0
    d.poly(mp.CUT, [(-k, -k), (w + k, -k), (w + k, HEIGHT_MM + k), (-k, HEIGHT_MM + k)])   # outline kerf/2 outside
    for i, dia in enumerate(ds):
        x = MARGIN_MM + (i + 0.5) * PITCH_MM
        d.circle(mp.CUT, (x, 22.0), (dia - kerf) / 2.0)                                  # hole kerf/2 inside
        s = f"{dia:.{decimals}f}"
        d.text(mp.ENGRAVE, (x - 0.33 * TEXT_MM * len(s), 8.0), TEXT_MM, s)
    d.text(mp.ENGRAVE, (MARGIN_MM, HEIGHT_MM - 6.0), 3.0,
           f"peg {peg_d:.1f} - hole d (finished, kerf {kerf:g} comp.)")
    return d


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--from", dest="d_from", type=float, default=7.5, help="smallest finished hole diameter [mm]")
    ap.add_argument("--to", dest="d_to", type=float, default=8.4, help="largest finished hole diameter [mm]")
    ap.add_argument("--step", type=float, default=0.1, help="diameter step [mm]")
    ap.add_argument("--config", default=None)
    ap.add_argument("--out", type=Path, default=ROOT / "targets" / "guides" / "peg_fit_test.dxf")
    a = ap.parse_args()
    gd = config.load(a.config)["guides"]
    ds = diameters(a.d_from, a.d_to, a.step)
    decimals = max(1, len(f"{a.step:g}".partition(".")[2]))
    d = build(ds, float(gd.get("kerf_mm", 0.0)), float(gd["peg_d"]), decimals)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    d.write(a.out)
    w = 2 * MARGIN_MM + len(ds) * PITCH_MM
    print(f"written {a.out}: {len(ds)} holes {ds[0]:g}..{ds[-1]:g} mm, strip {w:g} x {HEIGHT_MM:g} mm, "
          f"kerf {gd.get('kerf_mm', 0.0)} mm, peg {gd['peg_d']} mm (config peg_hole_d {gd['peg_hole_d']})")
    return 0


if __name__ == "__main__":
    sys.exit(main())

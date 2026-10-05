# Reference boards: print, laser-cut, glue, place

Generated from `config/station.toml` (`[boards.*]`, `[[targets]]`, `[plates]`) by `tools/print_targets.py` (PDF/PNG)
and `tools/make_plates.py` (DXF). Regenerate after any config change; do not edit these files by hand.

| Board | Use | Markers | Size (board / paper with quiet zone) | MDF plate |
|---|---|---|---|---|
| W0–W7 | wall: L layout 2026-10-05, see "4. Place (wall)" (was one every 800 mm on the straight wall) | AprilTag 36h11, ids 30–39 … 100–109 | 80 × 64 / 112 × 96 mm | 220 × 110 mm, notches on the block joints |
| S0, S1 | pick-up station (layout still open) | ids 200–209, 210–219 | 80 × 64 / 112 × 96 mm | 140 × 132 mm, plain |
| calib | camera + hand-eye calibration on the ARES deck | DICT_5X5_100, ids 0–43 | 165 × 120 / 195 × 150 mm | 223 × 186 mm, plain |

## 1. Print
- `<name>.pdf` at **100 %** (no "fit to page"), laser printer, matt paper.
- Check the 100 mm scale bar with a ruler, then measure the board over all squares with a caliper and enter the real
  square size (and marker size × the same factor) in `[boards.ref]` / `[boards.calib]`.
- Cut along the dashed line (crop marks at the corners; 16 mm white margin, calib 8.7 mm); keep the origin crosshair.

## 2. Laser-cut the MDF (4 mm)
- **Trotec Speedy / raster-engraving software: use `plates_all.svg`** (or `<name>_plate.svg`): cuts are red
  hairlines (#FF0000, 0.01 mm), engrave marks are blue **filled** 0.3 mm strips (#0000FF) and blue text. A raster
  engrave pass only fills areas, so the zero-width blue lines of the DXF were skipped (2026-10-05).
- `plates_all.dxf` / `<name>_plate.dxf` (DXF R12, mm): layer and entity colour CUT = ACI 1 red, ENGRAVE = ACI 5 blue,
  all hairlines - only if blue is set to a vector process (low-power cut = scoring).
- Plates already cut: put them back into the holes of the sheet (unmoved in the machine) and run only blue.

## 3. Glue
- Glue the print onto the engraved rectangle: the board's outer squares on the inner engraved rectangle, the printed
  origin crosshair and x/y arrows exactly on the engraved ones (on the wall plates the print sits upside down
  relative to the text - that is correct). Spray glue or thin double-sided tape, no bubbles; keep it flat.

## 4. Place (wall)
- Face up on the floor on the ARES side of its leg (the OUTSIDE of the L), **notched edge ("WALL ^") pressed against
  the side faces of the female-pin base blocks**, both V-notches on the joints of the block in the table below. Weigh
  it down or tape it; it must not move during a stop.
- L layout of 2026-10-05 (`config/station.toml` `[[targets]]` with `leg`, `tools/plan_layout.py`,
  `results/l_wall_layout.png`): the 8 existing plates, no new markers. **The engraved `u = ...` on the plates cut before
  2026-10-05 is wrong now - go by the board name and this table** (regenerated DXF/SVG files carry `A: u = ...` /
  `B: u = ...`; the PDFs are unchanged). Block k = 0 is the first block at the leg start; leg A starts at its free end,
  leg B at the inside face of leg A (corner): **set B's first base block 2.69 mm off A's blocks** (A's ribs 1.69 mm +
  1 mm gap, `[brick] rib_mm`, `[wall] corner_gap_mm` ASSUMPTION) - a 2.5-3 mm spacer between the block rows.

| Board | ids | Leg | u (plate centre) | Block (k-th from the leg start) | Spare block |
|---|---|---|---|---|---|
| W0 | 30–39 | A | 100 mm | k = 0 (1st) | no |
| W1 | 40–49 | A | 500 mm | k = 2 (3rd) | no |
| W2 | 50–59 | A | 900 mm | k = 4 (5th) | no |
| W3 | 60–69 | A | 1500 mm | k = 7 (8th) | no |
| W4 | 70–79 | A | 2100 mm | k = 10 (11th) | no |
| W5 | 80–89 | B | 100 mm | k = 0 (1st, in the outer corner: the plate edge also touches the end face of leg A, flush with B) | no |
| W6 | 90–99 | B | 500 mm | k = 2 (3rd) | no |
| W7 | 100–109 | B | 1100 mm | k = 5 (6th, the last block of leg B) | no |

- A stop measures only boards of its own leg, with look poses clear of the wall built by the end of that stop
  (`tools/make_job.py`, `mauer/armcheck.py`; ±50 mm ARES margin and the 40 mm arrival standoff): stop 0 can use
  W0-W3, stop 1 W2-W4, stop 2 W5-W7; the job uses the pair with the longest baseline (stop 0: W0 + W3, stop 1: W2 +
  W4, stop 2: W5 + W7). W1 and W6 are not used by the current job; they stay in place for a re-plan.
- The board positions follow from the ASSUMED block size 120 × 200 mm (`[plates]`, PLACEHOLDER) and the leg lengths
  (`[[wall.legs]]`, ASSUMPTION): enter the real block size and regenerate (`tools/plan_layout.py --evaluate`) if it
  differs.

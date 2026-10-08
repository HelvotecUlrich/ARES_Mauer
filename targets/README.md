# Reference boards: print, laser-cut, glue, place

Generated from `config/station.toml` (`[boards.*]`, `[[targets]]`, `[plates]`) by `tools/print_targets.py` (PDF/PNG)
and `tools/make_plates.py` (DXF). Regenerate after any config change; do not edit these files by hand.

| Board | Use | Markers | Size (board / paper with quiet zone) | MDF plate |
|---|---|---|---|---|
| W0–W7 | wall: C layout 2026-10-07 (W7 spare), see "4. Place (wall)" (was one every 800 mm on the straight wall) | AprilTag 36h11, ids 30–39 … 100–109 | 80 × 64 / 112 × 96 mm | 220 × 110 mm, notches on the block joints |
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
- Current layout (C, A 5 1/2 -> B -> C, 2026-10-07; `config/station.toml` `[[targets]]` with `leg`): the plates sit on
  the V-tabs of the laser-cut floor guides, the plate name is engraved next to its tab - lay them as
  `targets/guides/README.md` says (map `targets/guides/floor_map.pdf`). The engraved `u = ...` on the plates cut before
  2026-10-05 is wrong - go by the board name. The L layout of 2026-10-05 that this section described is history.
- Boards per leg and the stops that measure them (`[[targets]]`, job of `tools/make_job.py`, 2026-10-08):

| Board | ids | Leg | Measured at |
|---|---|---|---|
| W0 | 30–39 | A | stop 0 |
| W1 | 40–49 | A | stop 0 |
| W2 | 50–59 | B | stops 1, 2 |
| W3 | 60–69 | B | not used by the job (stays in place for a re-plan) |
| W4 | 70–79 | B | stops 1, 2 |
| W5 | 80–89 | C | stop 3 |
| W6 | 90–99 | C | stop 3 |
| W7 | 100–109 | - | spare (not placed) |
| S0, S1 | 200–209, 210–219 | station | every dock (station windows) |

- The variant `c_acb` (`config/variants/c_acb.toml`) has its own guides: `targets/guides_c_acb/`.
- The board positions follow from the ASSUMED block size 120 × 200 mm (`[plates]`, PLACEHOLDER) and the leg lengths
  (`[[wall.legs]]`, ASSUMPTION): enter the real block size and regenerate (`tools/make_guides.py`) if it differs.

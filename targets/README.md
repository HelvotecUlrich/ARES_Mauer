# Reference boards: print, laser-cut, glue, place

Generated from `config/station.toml` (`[boards.*]`, `[[targets]]`, `[plates]`) by `tools/print_targets.py` (PDF/PNG)
and `tools/make_plates.py` (DXF). Regenerate after any config change; do not edit these files by hand.

| Board | Use | Markers | Size (board / paper with quiet zone) | MDF plate |
|---|---|---|---|---|
| W0–W7 | wall, one every 800 mm (u = 100, 900, …, 5700) | AprilTag 36h11, ids 30–39 … 100–109 | 80 × 64 / 112 × 96 mm | 220 × 110 mm, notches on the block joints |
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
- Face up on the floor in the gap between the ARES front and the wall, **notched edge ("WALL ^") pressed against the
  side faces of the female-pin base blocks**, both V-notches on the joints of the block whose centre is at the board's
  `u` (engraved on the plate). Weigh it down or tape it; it must not move during a stop.
- W6 (u 4900) and W7 (u 5700) lie beyond the wall end (u 4800): put one spare base block under each.
- The board positions follow from the ASSUMED block size 120 × 200 mm (`[plates]`, PLACEHOLDER): enter the real block
  size and regenerate before cutting if it differs.

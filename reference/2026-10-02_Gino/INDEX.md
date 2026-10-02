# Reference archive "Gino" (handed over by Samuel, 2026-10-02)

Copied unchanged from the MA repo `inputs/Gino` (129 MB, 128 files; not versioned except this index). Read-only
reference – work copies of the files used go to `cad/`. Extracted facts with page/line sources:
`_facts_extracted_2026-10-02.md` (subagent, 2026-10-02). **`UR5/UR_Battery_Supply.pdf` is marked confidential by
Universal Robots – do not pass it on.**

| Path | Content |
|---|---|
| `CAD/Ganzer Stein/Kompletter_Stein*` | **Full stone (current, 2026-10-01)**: 120 × 200 × 120 mm, ribbed long faces (rib height 1.69 mm, pitch 8 mm), 4 conical pins below (23.5 mm long, r 15 → 7.9 mm) and 4 conical sockets on top (22.5 mm deep, rim r 18 mm) at x 33.5/86.5, y −49.75/−150.25; volume 2270 cm³. Pin 1 mm longer than socket depth → check. `Kompletter_Stein_Test_Druck`, `Pyramide` = variants |
| `CAD/Halber Stein/*` | Half stone + half base plate – **older size** (144 wide, 183 high), does not match the current full stone |
| `CAD/Ganzer Stein/Ganze_Bodenplatte*`, `CAD/Bodenplatte.FCStd` | Base plate 144 × 240 × 20 mm – older stone size |
| `CAD/Negativ/**`, `CAD/Test_Druck_Negativ.FCStd`, `CAD/Stone.FCStd` | Casting moulds (floor, lid, pin walls, ribs) and earlier stone versions, incl. a test cube |
| `CAD/Greifer/` | Gripper: Festo EHPS-20-A (8070831) with ZBH-7 centring sleeves (FCStd only), custom ribbed jaw `Greifer-Body.step` (26.7 × 60 × 68 mm), adapter `Greifer_uebergang` (54 × 101 × 41 mm), assembly `Backengreifer.FCStd` |
| `CAD/Mauer.FCStd` | Wall assembly (FreeCAD Assembly, 5 bodies) |
| `CAD/Boden/*` | Lifting platform plate 1000 × 600 × 20, robot platform 1000 × 600 × 600, test table |
| `UR5/RoboDK/*.py` | Gino Staub's RoboDK scripts: `init.py` "Brick Laying setup programm" (04.09.2026, 12 stones 200 × 120 × 112, magazines), `main.py` (tower script from HSLU IROB course 2025, Stäubli TX2-40), `test.py`/`Init_test.py` (placement-error "Falltest"), `Stein_Manager.py`, generic planners |
| `UR5/RoboDK/UR5Control/`, `UR5/UR5Control/` | UR5 driver: URScript as text over socket port 30002, `set_digital_out` for the gripper (DO1 pulse close, DO0 open); two slightly different copies |
| `UR5/RoboDK/*.rdk`, `*.robot`, `*.step` | RoboDK stations (`UR5_sim_v3.rdk`: AMR + lifting platform + UR5 + U-shaped 12-stone wall), AMR/lifting-platform mechanisms, Bota SensONE STEP |
| `UR5/RoboDK/test.urp` | RoboDK-generated pick-and-place, `set_tcp` z = 155 mm |
| `UR5/BAT-E-FS25-Bericht-Heini-L1236629.pdf` | HSLU bachelor thesis G. Heini, FS25 (6 June 2025): UR5 mounts terminal blocks with Bota SensONE + Beckhoff CX9020 + camera IDS GV-5040CP-M-GL (1456 × 1088, global shutter) with Fujinon HF8XA-5M 8 mm, HALCON, hand-eye calibration ≥ 15 poses, calibration plate by Samuel Ulrich. Not about bricks |
| `UR5/99202_UR5_User_Manual_en_Global.pdf`, `UR5/scriptManual_3.15.4.pdf` | UR5 CB3 user manual and URScript manual, SW 3.15 (arm 18.4 kg, payload 5 kg, reach 850 mm, repeatability ±0.1 mm, control box 100–240 VAC) |
| `UR5/UR_Battery_Supply.pdf` | UR installation guide "UR Robots with Battery Supply" CB3, Rev. 2.0.1 (2015), **confidential**; alternative recommended by UR: 48 VDC → 230 VAC inverter before the unmodified controller |
| `UR5/UR5_Battery.docx` | **empty (0 bytes)** |
| `UR5/IMU/*` | Bota SensONE F/T sensor datasheet Rev B and user manual Rev 1.8 (Ø 70 × 35 mm, 235 g, ISO 9409-1-50-4-M6, EtherCAT, built-in IMU) |
| `UR5/Technische_Zeichnung_UR5.PDF` | UR5 link dimension drawing |

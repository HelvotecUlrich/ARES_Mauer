# CAD and library files (not versioned)

| File | Source | SHA-256 |
|---|---|---|
| `2026-09-23_030718_A_1-AMR_SIM.stp` | NX2512 export by Samuel, header time stamp 2026-09-23T11:25 (original name `030718_A_1-AMR_SIM_1.stp`), handed over via MA `inputs/` on 2026-10-02 | `e24c0adadf8305a97170b32ebf4d7ca2b5349ee7628d244790aab9da78d3fc56` |
| `stone_full_2026-10-01.step` | Full stone "Kompletter_Stein-Body" (Gino, FreeCAD export 2026-10-01), from `reference/2026-10-02_Gino/CAD/Ganzer Stein/` | `00ae70f1b225194c71a8ef54ec81c16e672c69a1edab50ae1c2ccace667f7b46` |
| `stone_full_2026-10-01.stl` | Mesh of the stone, exported by RoboDK from the STEP (2026-10-02); used by the scripts | `0f007c67129f0e689b27cf0ade272aa86ed884bed897d272e00a8b09e93e55eb` |
| `gripper_jaw_2026-09-25.step` | Ribbed gripper jaw "Greifer-Body" (26.7 × 60 × 68 mm), from `reference/.../CAD/Greifer/` | `41654f103ef2ecf7d032e618233646988d9b5b3710ab6f9da82597a3f9fe8736` |
| `sensone_bota.step` | Bota SensONE F/T sensor (manufacturer STEP 2025-02-03), from `reference/.../UR5/RoboDK/` | `77831946a8343160ca3fb8d8768f9629c617d4d2f610d9c080ad16107dd74042` |
| `gripper_backengreifer_gino.tool` | RoboDK tool "Backengreifer" (Festo EHPS-20-A + ribbed jaws, geometry + TCP 135 mm) exported from `reference/2026-10-02_Gino/UR5/RoboDK/UR5_sim_v3.rdk` (2026-10-02); jaws close along the flange x axis | `cb7b727431ce9a4ae7f14a57697ea3753ba3992e5c73b0aff51902c628209f28` |
| `camera_adapter_018660_A_1.stp` | Camera adapter plate 018660_A_1 (NX2512 export by Samuel, 2026-10-05, from `input/`), between UR flange and gripper adapter (ISO 9409-1-50-4-M6), camera arm to x = -150 mm; coordinates = UR tool-flange frame; design notes `input/kamerahalter_handoff.md` | `a09a158fb455f602822d86882943458ad1caf3a40390e6e68f68991049eeef43` |
| `../robodk/library/UR5.robot` | RoboDK library, https://cdn.robodk.com/downloads-library/library-robots/UR5.robot (downloaded 2026-10-02) | `30e0b6347535a8f6242b96652b3c13e27478d48639af52f7380f039056eb7f46` |

The STEP coordinate system equals `base_link` (floor, centre between steering axes, x forward, y left, z up).
Assembly contents: drive modules, Deckplatte (top z = 333.6 mm), 2× nanoScan3 diagonal, IMU, casters, item frame.

Full stone (CAD, measured from the mesh 2026-10-02): body 120 (width, ribbed long faces) × 200 (length) × 120 mm
(height); ribs 1.69 mm high, 8 mm pitch; 4 conical pins below (23.5 mm long, radius ≈ 15 → 7.9 mm) and 4 conical
sockets on top (22.5 mm deep, rim radius 18 mm), centres at x = 33.5 / 86.5 mm, y = −49.75 / −150.25 mm (pin spacing
100.5 mm along the stone); volume 2270 cm³. Note: pins are 1 mm longer than the sockets are deep → stacked stones
may rest on the pin tips (to check on real stones).

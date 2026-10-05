# Handoff: UR5 camera bracket (IDS GV-51F0CP + 12 mm lens, Festo EHPS-20-A gripper)

Context for a new Claude session. It covers the design that was built, every
dimension, which values are verified and which are assumed, and what is still
open. The full CadQuery script is in the appendix, so the model can be
regenerated without any other files.

---

## 1. Task (user requirements)

A camera bracket for a **Universal Robots UR5, CB3 series**, with a **Festo EHPS-20-A**
electric parallel gripper. Camera **IDS GV-51F0CP-M-GL** (GigE uEye CP Rev. 2.2,
mono) with lens **IDS-12M23-C1228** (12 mm, F2.8, 2/3", C-mount).

The camera adapter sits **between the gripper (adapter) and the robot flange**.

User's mounting spec:

- **Where:** beside the gripper, **~150 mm from the flange centre on the flange −x side**.
  That is the side the jaws close along. Not on ±y: ±y is the stone's long
  direction, and the held stone would fill the view.
- **Direction:** the camera looks straight along the tool axis, the same way as the gripper.
- **Height:** the lens front must stay above the jaw tips so it never touches the wall
  or the stones. The model assumes **60 mm below the flange**.
- **Fixing:** clamp between flange and gripper on **ISO 9409-1-50-4-M6** (4 × M6 on a
  Ø50 circle), or bolt to the gripper adapter. It must be stiff with no play,
  because any wobble shows up as measurement error.
- **Access:** lens focus and iris screws, Hirose plug and RJ45 must stay reachable,
  with strain relief for the cable.
- **Accuracy:** a few mm either way doesn't matter, because hand-eye calibration
  measures the real pose. Once calibrated, the camera must never move.
- 150 mm offset and 60 mm height are the values in the user's config. The RoboDK
  study tested only this one position, and it worked. Nothing else was tested.

Later user decisions:

- The camera's bottom hole pattern is **24 × 24 mm** (4 × M3). The user stated this.
- The part will be **3D printed**.
- For a **short test** the user wants **no steel sleeves**: plain clamping with the
  existing M6 screws, made longer. A sleeve variant also exists for long-term use.

---

## 2. Coordinate system (used everywhere below)

UR tool-flange frame:

- **Origin:** centre of the robot's flange face.
- **+z:** out of the flange, toward the gripper and workpiece. "60 mm below the
  flange" means z = +60.
- **+y:** points **away from the tool connector**, so the M8 tool connector is on
  **−y**. Source: UR forum/manual, "Y is away from the Tool connector, Z is normal
  to the tool flange".
- **−x:** camera side, the direction the jaws close along. Right-handed:
  x = y × z.
- **Robot side:** z < 0.

---

## 3. Component data and confidence

| Item | Value | Confidence / source |
|---|---|---|
| UR5 CB3 tool flange | ISO 9409-1-50-4-M6: 4 × M6 on PCD 50 at 45° to the axes, Ø6 H7 pin hole on the PCD, centring recess Ø31.5 H7 | standard. Recess depth assumed ~6 mm (**verify ≥ 5**). Flange OD Ø63 |
| UR M6 torque | 9 Nm | UR manual |
| UR wrist-3 housing | modelled as Ø75 behind the flange | **rough assumption** |
| Pin-hole angular position on the UR flange | unknown | the design sidesteps this: pin holes in all 4 positions (0/90/180/270°) |
| IDS GV-51F0CP-M-GL | housing 29 × 29 × 29 mm; ~29 × 29 × 49.1 incl. lens mount and connectors; 53 g; 1/1.8" sensor, 5.1 MP; GigE (RJ45, screw lock) + Hirose I/O on the back | IDS / Edmund / 1stVision via search |
| Camera mounting | **4 × M3 on the bottom face only, pattern 24 × 24 mm, centred** | count from search; **pattern from user**. Thread depth unknown (M3 × 10 screws assumed) |
| C-mount ring protrusion in front of the housing | **5.0 mm** | **ASSUMPTION** (`MOUNT_PROT`). Shifts the lens front 1:1 |
| Connector protrusion at the back | 15.1 mm (= 49.1 − 29 − 5) | **ASSUMPTION** (`BACK_PROT`) |
| IDS-12M23-C1228 lens | Ø29 × 47.61 mm, front Ø23.2, back Ø12.5, 90 g, 12 MP, 2/3" | IDS shop / Inosaki via search |
| Festo EHPS-20-A | B1 = 32, L1 = 65, H1 = 127.5 (incl. jaws), H2 = 115 (body); 532 g; 13 mm stroke per jaw; 218 N total grip force; M12 5-pin cable; mounts with thread + centring sleeve or through-hole + centring sleeve | Festo datasheet via search. Modelled as a box: L1 along x (jaw direction), B1 along y |
| Gripper adapter (user's existing part) | unknown. Modelled as a placeholder Ø63 × 12 with a Ø31.5 spigot | **unknown: does it have a Ø31.5 spigot and a Ø6 pin?** |
| Fingers | placeholder, 40 mm long | unknown |

The network egress in the cloud session **blocked** ids-imaging.com, festo.com,
1stvision.com and inosaki.uk, so no drawings could be read; data came from search
snippets. PyPI and npm worked.

---

## 4. Design concept

**One monolithic part** that sits between the UR flange and the gripper adapter:

1. **Flange disc** with the ISO 9409 interface reproduced on both faces:
   - robot side: spigot into the UR recess
   - gripper side: recess for the adapter's spigot
   - through holes for the adapter's M6 screws, which pass through the plate into
     the UR flange and clamp the plate
2. **Arm** in the flange plane (z 0…12) running out to the −x side.
3. **Camera wall** at the arm end, on the **+y side of the camera**. The camera's
   bottom face (4 × M3) is screwed flat against the wall's −y face. The screws enter
   from +y, so they are fully accessible even when mounted. The wall continues
   inward along the arm's +y edge as a **rib** on the robot side (z < 0) for
   stiffness.
4. **Lens** hangs below the plate (z 12.4…60) and is free on all sides, so focus and
   iris screws are reachable. RJ45 and Hirose on the camera back (z < −21.6) are free.
5. **Strain relief:** one M4 (heat-set insert) on the wall's top face for a P-clip.

Why this layout:

- The camera housing straddles the flange plane (z −21.6…+7.4) because the lens
  front must be at z = 60.
- The camera has threads only on its bottom face.
- A wall on the +y side is the only placement where all 4 screws stay reachable
  without the arm in the way.

The alternatives were rejected for these reasons:

- **Wall on +x, between camera and gripper:** the arm blocks the lower screw row.
- **Wall on −x:** the arm would have to pass through the camera.

---

## 5. Derived positions (tool frame, mm)

| Feature | Value |
|---|---|
| Optical axis | x = −150, y = 0, parallel to +z |
| Lens front | z = 60.0 |
| Lens body | Ø29, z 12.39 … 60.0 |
| C-mount ring (model) | Ø27, z 7.39 … 12.39 |
| Camera housing | x −164.5 … −135.5, y −14.5 … 14.5, z −21.61 … 7.39 (centre z −7.11) |
| Camera connectors (model) | 18 × 14 box, z −36.7 … −21.6 |
| Camera bottom face (screwed to wall) | y = +14.5 |
| M3 hole centres (camera and wall) | x ∈ {−162, −138}, z ∈ {−19.11, +4.89} |
| Gripper body (model, with plate) | z 24 … 139; jaws to z 151.5; fingers (placeholder) to z 191.5 |

Clearance: the lens front at z 60 is about 90 mm above the jaw tips. The gripper
body is at |x| ≤ 32.5, so the camera at x −150 is about 117 mm away and outside
the field of view.

---

## 6. Bracket dimensions (all variants)

Plate bounding box with sleeves: x −168 … 36, y −36 … 36, z −24 … 12.
Without sleeves: x −168 … 31.5, y −31.5 … 31.5.

### Flange disc

| Feature | Dimension |
|---|---|
| Disc | z 0 … 12 (**T = 12 → gripper and TCP move +12 mm in z**) |
| Disc diameter | **Ø72** (sleeve variant) / **Ø63** (no-sleeve variant) |
| Spigot (robot side) | Ø31.5 (h7 intent), height 5, z −5 … 0 |
| Recess (gripper side) | Ø31.5 (H7 intent), depth 6, z 6 … 12 |
| Screw holes | 4 × on PCD 50 at 45/135/225/315°: **Ø10** for press-in steel sleeves (sleeve variant) or **Ø6.6** M6 clearance (no-sleeve variant) |
| Pin holes | 4 × on PCD 50 at 0/90/180/270°, through: **printed Ø5.8, ream to Ø6 H7**. Ø6.0 in the aluminium variant |

### Arm (z 0 … 12)

Plan-view polygon (x, y):

```
(−20, −22) → (−100, −22) → (−134.5, −16) → (−134.5, 26.5) → (−20, 26.5)
```

It is unioned with the disc. The arm end at x = −134.5 leaves 1 mm of air to the
camera's +x face (−135.5).

### Wall + rib (y 14.5 … 26.5, thickness 12; aluminium variant 10)

Side profile in XZ, extruded over y 14.5 … 26.5:

```
(−168, 12) → (−168, −24) → (−126.5, −24) → (−110.5, −16) → (−54.6, −16) → (−45, 0) → (−45, 12)
```

- The wall top is z = −24, about 2.4 mm above the camera back.
- From x −126.5 the profile slopes 1:2 down to rib height z −16 at x −110.5.
- The rib runs at z −16 to x −54.6, then slopes to the flange plane at x −45.
- The rib's inner end is at r ≈ 47–57 mm. It was chosen to clear the wrist
  housing (assumed Ø75) as wrist 3 rotates.

### Camera holes and other features

| Feature | Dimension |
|---|---|
| Camera screw holes | 4 × Ø3.4 through the wall along y, at the M3 centres above (x −162/−138, z −19.11/+4.89). Counterbore Ø6 × 3.3 from the +y face (y = 26.5) |
| Strain relief | 1 × Ø5.6 × 9 deep (M4 heat-set insert) from the wall top (z −24) at x −150, y 20.5. Aluminium variant: Ø3.3 × 10 tapped M4. There was originally 2 at x −158/−142, but they intersected the M3 counterbores, so it was reduced to 1 centred |
| Fillet | 2 mm on the vertical edges at min-x (outer end of the wall) |

### Volumes and masses

| Variant | Volume | Mass |
|---|---|---|
| Sleeve variant | 132.2 cm³ | ≈ 168 g in PETG at 100 % infill; ≈ 357 g in aluminium |
| No-sleeve variant | 125.8 cm³ | ≈ 160 g in PETG |

### Checks done

- The plate's intersection volume with camera, lens, adapter placeholder and UR
  flange model is **0 mm³**.
- The STEP re-import is a single valid solid.

---

## 7. Variants

The `PRINT` flag and the `sleeves` argument select the variant.

| Variant | Use | Clamp path |
|---|---|---|
| **Print, no sleeves** (`kamerahalter_platte_ohne_huelsen.stl`) | **short test only**; this is what the user is printing now | M6 clamps the plastic directly. Creep relaxes the preload. See §8 |
| **Print, with sleeves** (`kamerahalter_platte.stl`) | long-term printed version | 4 steel spacer sleeves Ø10 × Ø6.4 × 12, pressed or epoxied into the Ø10 holes. Metal-to-metal clamping (flange → sleeve → adapter). The plate is held by form fit (sleeves, spigot, pin) |
| **Aluminium** (`PRINT = False`) | best for "never move" | EN AW-6082 T6; Ø63 disc, Ø6.6 holes, wall 10 mm, no rib, tapped M4. Two setups + side M3 holes. Faces flat/parallel ≤ 0.02. Mask the H7/h7 fits if anodising |

Fit-test coupons are also exported: the flange disc only, Ø72 or Ø63 × 12 with
spigot, recess and holes. Print one first to check the fits on the robot.

---

## 8. Engineering notes and rationale

### Stiffness

Cantilever estimate: L = 114 mm from the disc edge, 0.2 kg at the tip (camera +
lens + cable share). Section at x = −90 gives I ≈ 39 400 mm⁴ (bending about y).

| Material | k | Deflection at 1 g | f₁ |
|---|---|---|---|
| PETG printed (E ≈ 1.8 GPa) | ≈ 144 N/mm | 14 µm | ≈ 135 Hz |
| PETG-CF / PA-CF (E ≈ 4 GPa) | ≈ 320 N/mm | 6 µm | ≈ 200 Hz |
| Aluminium | ≈ 5 600 N/mm | < 1 µm | ≈ 840 Hz |

Elastic deflection is negligible. **Creep is the real risk for a printed part.**

### Why sleeves matter

- An M6 screw at 9 Nm (preload ~7.5 kN) stretches elastically only about
  **0.05 mm**. That stretch is the whole preload.
- 4 screws on 12 mm of plastic is about 30 kN, or ≈ 13 MPa sustained.
- Plastic creep of just 0.5 % = 0.06 mm removes essentially **all preload**. The
  ~0.53 kg gripper would then hang on loose screws.
- Threadlocker does not prevent preload loss, it only stops the screw rotating
  loose.
- The plastic layer also adds tilt compliance under gripper moments.

### Rules for the no-sleeve test variant

- Use PA-CF / PETG-CF with 100 % infill in the flange zone. **Never PLA**: it
  creeps at ~50 °C, and the camera gets warm.
- Retighten after **1 h and after 24 h**. Mark the screws with a paint pen.
- Check by hand that the gripper doesn't wobble before every run.
- Switching to the sleeve or aluminium version later requires **re-calibration**.

### Print settings

- **Orientation:** the gripper-side face (with the Ø31.5 recess) on the bed. Spigot,
  wall and rib grow upward. Only the Ø31.5 recess roof is a bridge; no other
  supports.
- In this orientation the layers lie parallel to the arm's bending stress.
- ≥ 6 walls, 60–100 % infill.
- The camera screw holes are horizontal holes. Heat-set the M4 insert.

### Camera creep

- Mount with washers and medium threadlocker.
- **Retighten after 24–48 h, then calibrate.**
- Re-verify the calibration occasionally in the first weeks.

### Collision risk

- The camera back, connectors (to z ≈ −37) and cable sit at r ≈ 135–165 mm behind
  the flange plane.
- Depending on the wrist-3 angle, the wrist-2 / wrist-1 housings can come close.
  In the UR5 DH parameters, wrist 2 crosses the tool axis 82.3 mm behind the flange,
  and wrist 1 is offset 94.65 mm along the wrist-2 axis.
- The rib reaches z −16 at r ≥ ~50.
- **Check the used wrist-3 range in RoboDK with camera AND cable.** A 90° angled
  GigE cable reduces the protrusion.

### TCP and payload

After mounting:

- **TCP z +12 mm**.
- Payload ≈ gripper 0.53 + plate 0.16 (print) / 0.36 (alu) + camera 0.053 + lens
  0.09 kg + adapter + fingers.
- The CoG shifts noticeably to −x.

---

## 9. Bill of materials

| Qty | Part |
|---|---|
| 4 | M6 cap screws, **12 mm longer** than the current adapter screws (same thread engagement in the UR flange); 9 Nm |
| 1–2 | Dowel pin ISO 8734 Ø6 m6: one long pin (flange depth + 12 + adapter depth) or two short pins in the same hole |
| 4 | ISO 4762 M3 × 10 + washers (camera). **Verify the camera thread depth.** Grip ≈ 12 − 3.3 = 8.7 mm |
| 1 | M4 heat-set insert (hole Ø5.6 × 9) + P-clip (e.g. DIN 72571) |
| 4 | *(sleeve variant only)* steel spacer sleeve Ø10 × Ø6.4 × 12 |

---

## 10. Assembly order

1. **Bench.**
   - Remove the bridge support from the Ø31.5 recess and deburr.
   - Ream the 4 pin holes to Ø6 H7.
   - *(sleeve variant)* press in the sleeves flush; epoxy them if loose.
   - Heat-set the M4 insert.
   - Screw the lens onto the camera.
2. **Mark the jaw-closing direction (−x)** before disassembly.
3. Power down the gripper and unplug the M12.
4. Remove the gripper from the adapter. This exposes the adapter's M6 screws.
5. Remove the adapter from the flange.
6. Put the Ø6 pin into the UR flange.
7. Fit the plate: spigot into the recess, pin into the matching hole, **arm toward
   the marked −x side**. With 4 pin holes, any 90° orientation fits.
8. Fit the adapter (pin through the plate into the adapter). Fit the **+12 mm M6
   screws** and tighten crosswise to 9 Nm.
9. Refit the gripper and reconnect the M12.
10. Fix the camera to the wall: bottom face flat, 4 × M3 + washers + medium
    threadlocker from the +y side. Align the camera edges parallel to the wall
    before the final tightening.
11. Plug in and lock RJ45 and Hirose. Fit the P-clip on the M4. Leave a cable loop
    for wrist-3 rotation and route the cable along the robot.
12. Installation: TCP +12 mm z, payload and CoG.
13. Jog the used wrist-3 range slowly and check for collisions or a pulled cable.
14. Set **focus and iris** at the working distance and lock them. This must happen
    before calibration, because changing focus invalidates the intrinsics.
15. **Wait 24–48 h** (camera powered/warm). Retighten the M3s and check the M6s.
    For the no-sleeve test variant, also retighten at 1 h.
16. **Hand-eye calibration.** Afterwards never loosen or adjust the camera, lens or
    plate. If the gripper must come off, the pin and spigot make re-mounting
    repeatable, but re-verify the calibration.

---

## 11. Open items / to verify

1. **`MOUNT_PROT` = 5 mm** (C-mount ring protrusion): assumed. It shifts the lens
   front 1:1.
2. **UR flange recess depth ≥ 5 mm**, otherwise reduce `SPIGOT_H`.
3. **Gripper adapter geometry:** does it have a Ø31.5 spigot and a Ø6 pin? If not,
   the plate's bottom recess does nothing and the adapter isn't form-located on the
   plate. Then add a pin between plate and adapter.
4. **Camera M3 thread depth** → screw length.
5. **FDM fits:** spigot Ø31.5 and sleeve holes Ø10. Tune with the test coupon
   (`SPIGOT_D`, `SLEEVE_D`).
6. **Wrist collision** with camera and cable over the used wrist-3 range (RoboDK).
7. The EHPS-20 model is a box from catalogue dimensions. The actual cable exit and
   finger geometry are not modelled.

---

## 12. Files and how to rebuild

Files (`out/` from the script):

- `kamerahalter_platte_ohne_huelsen.stl/.step`: test variant, no sleeves (printing now)
- `kamerahalter_platte.stl/.step`: printed variant with sleeves
- `passungstest_flansch.stl`, `passungstest_flansch_ohne_huelsen.stl`: fit-test coupons
- `kamerahalter_baugruppe.step`: assembly with the simplified environment (UR
  flange + wrist, adapter placeholder, EHPS-20 box, jaws/fingers, camera, lens)
- `_*.stl`: environment parts, used for previews

Rebuild:

```bash
python3 -m venv cadenv && ./cadenv/bin/pip install cadquery   # CadQuery 2.8.0 used
./cadenv/bin/python kamerahalter.py
```

The script prints a bounding box, volume and mass, a stiffness table, the
camera/wall positions and the intersection checks.

Previews were rendered with three.js 0.160 (installed via npm, because the CDN was
blocked) in headless Chromium through Playwright. The page loaded the STLs with
STLLoader, used an orthographic camera, and mapped the tool frame
(x, y, z) → scene (x, −z, y) so the robot is on top.

Names and comments in the script and the deliverables are German, because the user
writes German.

---

## Appendix: full CadQuery script (`kamerahalter.py`)

```python
"""
Kamerahalter UR5 (CB3) + Festo EHPS-20-A + IDS GV-51F0CP-M-GL / IDS-12M23-C1228

Zwischenplatte zwischen Roboterflansch und Greiferadapter (ISO 9409-1-50-4-M6),
mit Arm zur Kamera bei x = -150 mm, Blick entlang +z (Werkzeugrichtung).

Koordinaten = UR-Werkzeugkoordinatensystem (Tool flange):
  Ursprung   Mitte Flanschfläche des Roboters
  +z         aus dem Flansch heraus (Richtung Greifer / Werkstück)
  +y         weg vom Tool-Connector (Connector liegt auf -y)
  -x         Kameraseite = Richtung, in der die Backen schliessen
Alle Masse in mm.

Ausgabe: STEP/STL der Halterplatte, STEP der Baugruppe, STL der Einzelteile
(für die Vorschau).
"""
import math
import os
import cadquery as cq

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
os.makedirs(OUT, exist_ok=True)

# ---------------------------------------------------------------------------
# Parameter
# ---------------------------------------------------------------------------
# Fertigung: True = 3D-Druck (Stahlhülsen im Klemmbereich, Rippe, Gewindeeinsätze)
#            False = Aluminium gefräst
PRINT = True

# ISO 9409-1-50-4-M6 (UR5 CB3)
UR_FLANGE_D = 63.0       # Aussendurchmesser UR-Flansch
# Druck mit Hülsen: Scheibe Ø72 (Platz für Ø10-Bohrungen); ohne Hülsen Ø63
def flange_d(sleeves):
    return 72.0 if (PRINT and sleeves) else UR_FLANGE_D
PCD = 50.0               # Lochkreis
M6_CLEAR = 6.6           # Durchgang M6 (mittel)
# Druck: Klemmkraft der M6 läuft über Stahlhülsen Ø10/Ø6,4 x T, nicht über Kunststoff
SLEEVE_D = 10.0          # Bohrung für Einpresshülse (gedruckt Ø10, Hülse einpressen/kleben)
PIN_D = 5.8 if PRINT else 6.0   # Ø6 H7 – beim Druck 5,8 drucken und auf Ø6 H7 reiben
SPIGOT_D = 31.5          # Zentrierbund Ø31.5 h7 -> greift in UR-Flansch
SPIGOT_H = 5.0           # < Tiefe der UR-Einsenkung (ca. 6 mm) – prüfen
RECESS_D = 31.5          # Einsenkung Ø31.5 H7 -> nimmt Bund des Greiferadapters auf
RECESS_DEPTH = 6.0

T = 12.0                 # Plattendicke (= Versatz des TCP in +z!)

# Kamera-Lage (aus der Konfiguration)
CAM_X = -150.0           # Abstand optische Achse – Flanschmitte
LENS_FRONT_Z = 60.0      # Objektivfront unter der Roboter-Flanschfläche

# IDS GV-51F0CP-M-GL (GigE uEye CP Rev. 2.2)
CAM_W = 29.0             # Gehäuse 29 x 29 x 29
CAM_L = 29.0
MOUNT_PROT = 5.0         # Überstand C-Mount-Ring vor Gehäuse  (ANNAHME)
BACK_PROT = 15.1         # Überstand RJ45/Hirose hinten (49.1 - 29 - 5) (ANNAHME)
# 4x M3 auf der Kamera-Unterseite, Lochabstand 24 x 24 mm (Angabe Nutzer).
# Angabe relativ zur Mitte der Unterseite: (entlang opt. Achse, quer)
CAM_HOLES = [(-12.0, -12.0), (-12.0, 12.0), (12.0, -12.0), (12.0, 12.0)]
M3_CLEAR = 3.4
M3_CBORE_D, M3_CBORE_H = 6.0, 3.3

# IDS-12M23-C1228: Ø29 x 47.61
LENS_D = 29.0
LENS_L = 47.61

# Wand, an die die Kamera mit der Unterseite geschraubt wird (Seite +y)
WALL_T = 12.0 if PRINT else 10.0
WALL_X0 = CAM_X - 18.0
ARM_GAP = 1.0                              # Luft Arm-Ende <-> Kamera
ARM_HALF_W = 22.0                          # halbe Armbreite auf der -y-Seite
# Rippe auf der Flanschseite (z < 0) entlang der +y-Kante des Arms.
# Beginnt erst bei r ≈ 50 mm, damit sie das Handgelenk (Ø75) frei umläuft.
RIB_H = 16.0 if PRINT else 0.0
RIB_X_START = -45.0

# Kabel-Zugentlastung: 1x M4 mittig in der Wand-Oberseite (für Kabelschelle)
# Druck: Gewindeeinsatz M4 (Einschmelzmutter, Bohrung 5,6 x 8)
M4_HOLE, M4_DEPTH = (5.6, 9.0) if PRINT else (3.3, 10.0)

# ---------------------------------------------------------------------------
# abgeleitete Lagen
# ---------------------------------------------------------------------------
cam_front_z = LENS_FRONT_Z - LENS_L - MOUNT_PROT      # Gehäusefront
cam_back_z = cam_front_z - CAM_L                      # Gehäuserückseite
cam_mid_z = 0.5 * (cam_front_z + cam_back_z)
cam_x0, cam_x1 = CAM_X - CAM_W / 2, CAM_X + CAM_W / 2
wall_y0 = CAM_W / 2                                   # Wand-Innenfläche = Kamera-Unterseite
wall_y1 = wall_y0 + WALL_T
wall_top_z = math.floor(cam_back_z) - 2.0             # etwas über Gehäuserückseite
arm_end_x = cam_x1 + ARM_GAP


def box(x0, x1, y0, y1, z0, z1):
    return (cq.Workplane("XY")
            .box(x1 - x0, y1 - y0, z1 - z0, centered=False)
            .translate((x0, y0, z0)))


def cyl_z(d, z0, z1, x=0.0, y=0.0):
    return (cq.Workplane("XY").workplane(offset=z0)
            .center(x, y).circle(d / 2).extrude(z1 - z0))


def polar(r, deg):
    a = math.radians(deg)
    return (r * math.cos(a), r * math.sin(a))


# ---------------------------------------------------------------------------
# Halterplatte (ein Teil; Druck oder Aluminium, siehe PRINT)
# ---------------------------------------------------------------------------
def flange_features(body, sleeves=True):
    """Bund oben, Einsenkung unten, Lochbild ISO 9409-1-50-4-M6."""
    body = body.union(cyl_z(SPIGOT_D, -SPIGOT_H, 0))            # Zentrierbund oben
    body = body.cut(cyl_z(RECESS_D, T - RECESS_DEPTH, T + 1))   # Einsenkung unten
    # 4x M6 (bzw. Hülse) auf 45°, 4x Ø6 Stift auf 0/90/180/270°
    for k in range(4):
        x, y = polar(PCD / 2, 45 + 90 * k)
        body = body.cut(cyl_z(SLEEVE_D if (PRINT and sleeves) else M6_CLEAR, -SPIGOT_H - 1, T + 1, x, y))
        x, y = polar(PCD / 2, 90 * k)
        body = body.cut(cyl_z(PIN_D, -SPIGOT_H - 1, T + 1, x, y))
    return body


def build_test_coupon(sleeves=True):
    """Nur die Flanschscheibe – zum Testen der Passungen vor dem grossen Druck."""
    return flange_features(cyl_z(flange_d(sleeves), 0, T), sleeves)


def build_bracket(sleeves=True):
    disc = cyl_z(flange_d(sleeves), 0, T)

    # Arm (Draufsicht-Kontur), Dicke T; +y-Kante gerade (trägt Rippe und Wand)
    arm_pts = [
        (-20.0, -ARM_HALF_W),
        (-100.0, -ARM_HALF_W),
        (arm_end_x, -16.0),
        (arm_end_x, wall_y1),
        (-20.0, wall_y1),
    ]
    arm = cq.Workplane("XY").polyline(arm_pts).close().extrude(T)

    # Wand + Rippe: Profil in XZ, extrudiert über y = wall_y0..wall_y1
    rib_start = arm_end_x + 8.0          # ab hier fällt die Wand auf Rippenhöhe ab
    if RIB_H > 0:
        prof = [
            (WALL_X0, T),
            (WALL_X0, wall_top_z),
            (rib_start, wall_top_z),
            (rib_start + 2.0 * (-wall_top_z - RIB_H), -RIB_H),   # Schräge 1:2
            (RIB_X_START - RIB_H * 0.6, -RIB_H),
            (RIB_X_START, 0.0),
            (RIB_X_START, T),
        ]
    else:
        prof = [
            (WALL_X0, T),
            (WALL_X0, wall_top_z),
            (rib_start, wall_top_z),
            (-100.0, 0.0),
            (-100.0, T),
        ]
    wall = (cq.Workplane("XZ", origin=(0, wall_y0, 0))
            .polyline(prof).close().extrude(-WALL_T))   # XZ-Normale = -y

    body = flange_features(disc.union(arm).union(wall), sleeves)

    # Kameraschrauben: Durchgang in y, Senkung von +y
    for (dz, dx) in CAM_HOLES:
        hx, hz = CAM_X + dx, cam_mid_z + dz
        thru = (cq.Workplane("XZ", origin=(0, wall_y0 - 1, 0)).center(hx, hz)
                .circle(M3_CLEAR / 2).extrude(-(WALL_T + 2)))
        cb = (cq.Workplane("XZ", origin=(0, wall_y1 - M3_CBORE_H, 0)).center(hx, hz)
              .circle(M3_CBORE_D / 2).extrude(-(M3_CBORE_H + 1)))
        body = body.cut(thru).cut(cb)

    # Zugentlastung: 1x M4 (Gewindeeinsatz) von oben in die Wand
    for hx in (CAM_X,):   # mittig, zwischen den M3-Senkungen
        body = body.cut(cyl_z(M4_HOLE, wall_top_z - 1, wall_top_z + M4_DEPTH,
                              hx, 0.5 * (wall_y0 + wall_y1)))

    # Kanten brechen (aussen, vertikal am Arm-Ende)
    try:
        body = body.edges("|Z and <X").fillet(2.0)
    except Exception:
        pass
    return body


# ---------------------------------------------------------------------------
# Umgebung (vereinfacht, nur zur Kontrolle)
# ---------------------------------------------------------------------------
def build_env():
    parts = {}
    # UR5 CB3 Werkzeugflansch + Handgelenk 3 (vereinfacht)
    robot = cyl_z(UR_FLANGE_D, -10, 0).union(cyl_z(75, -95, -10))
    robot = robot.cut(cyl_z(SPIGOT_D, -6, 0.1))
    # Tool-Connector (M8) auf -y
    conn = (cq.Workplane("XZ", origin=(0, -37, 0)).center(0, -25)
            .circle(6).extrude(15))
    parts["ur5_flansch"] = robot.union(conn)

    # Greiferadapter – Platzhalter
    parts["greiferadapter_platzhalter"] = (cyl_z(UR_FLANGE_D, T, T + 12)
                                           .union(cyl_z(SPIGOT_D - 0.1, T - 5, T)))
    z0 = T + 12
    # Festo EHPS-20-A: L1 65 (Backenrichtung x), B1 32, H2 115, H1 127.5
    parts["festo_ehps20"] = box(-32.5, 32.5, -16, 16, z0, z0 + 115)
    jaws = (box(-14, -4, -12, 12, z0 + 115, z0 + 127.5)
            .union(box(4, 14, -12, 12, z0 + 115, z0 + 127.5)))
    # Finger: Platzhalter 40 mm
    jaws = jaws.union(box(-12, -6, -10, 10, z0 + 127.5, z0 + 167.5))
    jaws = jaws.union(box(6, 12, -10, 10, z0 + 127.5, z0 + 167.5))
    parts["backen_finger_platzhalter"] = jaws

    # Kamera
    cam = box(cam_x0, cam_x1, -CAM_W / 2, CAM_W / 2, cam_back_z, cam_front_z)
    cam = cam.union(cyl_z(27, cam_front_z, cam_front_z + MOUNT_PROT, CAM_X, 0))
    cam = cam.union(box(CAM_X - 9, CAM_X + 9, -7, 7, cam_back_z - BACK_PROT, cam_back_z))
    parts["kamera_gv51f0cp"] = cam
    lens = cyl_z(LENS_D, LENS_FRONT_Z - LENS_L, LENS_FRONT_Z, CAM_X, 0)
    parts["objektiv_12m23_c1228"] = lens
    return parts


if __name__ == "__main__":
    bracket = build_bracket()
    env = build_env()

    cq.exporters.export(bracket, os.path.join(OUT, "kamerahalter_platte.step"))
    cq.exporters.export(bracket, os.path.join(OUT, "kamerahalter_platte.stl"),
                        tolerance=0.02, angularTolerance=0.1)
    if PRINT:   # Kurztest-Variante: direkt geklemmt, ohne Hülsen
        b2 = build_bracket(sleeves=False)
        cq.exporters.export(b2, os.path.join(OUT, "kamerahalter_platte_ohne_huelsen.stl"),
                            tolerance=0.02, angularTolerance=0.1)
        cq.exporters.export(b2, os.path.join(OUT, "kamerahalter_platte_ohne_huelsen.step"))
        cq.exporters.export(build_test_coupon(sleeves=False),
                            os.path.join(OUT, "passungstest_flansch_ohne_huelsen.stl"),
                            tolerance=0.02, angularTolerance=0.1)
        print(f"ohne Hülsen: valid={b2.val().isValid()}, "
              f"Volumen {b2.val().Volume()/1000:.1f} cm³, "
              f"Überschneidung Adapter {b2.intersect(build_env()['greiferadapter_platzhalter']).val().Volume():.3f} mm³")
    coupon = build_test_coupon()
    cq.exporters.export(coupon, os.path.join(OUT, "passungstest_flansch.stl"),
                        tolerance=0.02, angularTolerance=0.1)

    assy = cq.Assembly(name="kamerahalter_baugruppe")
    assy.add(bracket, name="kamerahalter_platte", color=cq.Color(0.75, 0.78, 0.82))
    colors = {
        "ur5_flansch": (0.55, 0.62, 0.70),
        "greiferadapter_platzhalter": (0.45, 0.45, 0.45),
        "festo_ehps20": (0.20, 0.45, 0.75),
        "backen_finger_platzhalter": (0.30, 0.30, 0.30),
        "kamera_gv51f0cp": (0.85, 0.85, 0.85),
        "objektiv_12m23_c1228": (0.12, 0.12, 0.12),
    }
    for name, shape in env.items():
        assy.add(shape, name=name, color=cq.Color(*colors[name]))
        cq.exporters.export(shape, os.path.join(OUT, f"_{name}.stl"),
                            tolerance=0.05, angularTolerance=0.2)
    assy.save(os.path.join(OUT, "kamerahalter_baugruppe.step"))

    # Kontrollwerte
    bb = bracket.val().BoundingBox()
    vol = bracket.val().Volume()
    print(f"Platte BBox x[{bb.xmin:.1f},{bb.xmax:.1f}] y[{bb.ymin:.1f},{bb.ymax:.1f}] "
          f"z[{bb.zmin:.1f},{bb.zmax:.1f}]")
    print(f"Volumen {vol/1000:.1f} cm³ -> Masse Al {vol*2.70e-3:.0f} g, "
          f"PETG 100 % {vol*1.27e-3:.0f} g")

    # Steifigkeit Arm (Kragbalken, Querschnitt bei x = -90, Länge Scheibe -> Kamera)
    # dünne Scheibe (t = 0.1 mm) bei x = -90 ausschneiden; ∫(x²+z²)dV / t ≈ ∫z² dA
    import OCP.GProp, OCP.BRepGProp
    t = 0.1
    sl = bracket.intersect(box(-90 - t / 2, -90 + t / 2, -50, 50, -50, 50))
    props = OCP.GProp.GProp_GProps()
    OCP.BRepGProp.BRepGProp.VolumeProperties_s(sl.val().wrapped, props)
    I_bend = props.MatrixOfInertia().Value(2, 2) / t   # um y durch Schwerpunkt
    L = abs(CAM_X) - flange_d(True) / 2
    m = 0.20  # kg Kamera + Objektiv + Kabelanteil
    for name, E in (("Aluminium", 70000.0), ("PETG (gedruckt)", 1800.0),
                    ("PA-CF / PETG-CF (gedruckt)", 4000.0)):
        k = 3 * E * I_bend / L**3                     # N/mm
        f = (k * 1000 / m) ** 0.5 / (2 * math.pi)
        print(f"  {name:28s} k={k:7.0f} N/mm  Durchbiegung 1g={m*9.81/k*1000:6.1f} µm  f≈{f:4.0f} Hz")
    print(f"  (I_biegung={I_bend:.0f} mm⁴, L={L:.0f} mm)")
    print(f"Kamera: Gehäuse z[{cam_back_z:.1f},{cam_front_z:.1f}], "
          f"Stecker bis z={cam_back_z-BACK_PROT:.1f}, Objektivfront z={LENS_FRONT_Z}")
    print(f"Wand oben z={wall_top_z}, Arm-Ende x={arm_end_x}")
    # Kollisionskontrolle Platte <-> Kamera/Objektiv
    for n in ("kamera_gv51f0cp", "objektiv_12m23_c1228", "greiferadapter_platzhalter",
              "ur5_flansch"):
        inter = bracket.intersect(env[n]).val().Volume()
        print(f"Überschneidung Platte/{n}: {inter:.3f} mm³")
```

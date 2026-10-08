# Camera + robot setup and calibration runbook

Step by step from an unpacked camera to the first closed-loop run. Every command runs from the repo root with Windows
Python (`py.exe`, from WSL or cmd). Parameters live in `config/station.toml` with their status tags; this runbook says
which PLACEHOLDERs each step replaces. Architecture and conventions: `docs/ARCHITECTURE.md`.

Free software only: IDS peak (free runtime + `ids_peak` wheels), OpenCV 4.14.0.94, the official UR RTDE client
(vendored), pyads. No HALCON.

## 0. Status (2026-10-05)

| Item | State |
|---|---|
| Python packages | installed (`requirements.txt`) |
| IDS peak | 26.06.2 installed by Samuel, GigE Vision producer `ids_gevgentlk.cti` (GEVK) |
| Camera | found by IDS peak: model string "GV-51FxCP-M", serial 4110073444, MAC 00:1B:A2:21:21:13, **IP 192.168.0.1 → not openable** (outside every laptop NIC subnet) → step 2 |
| UR5 | IP unknown (`[ur].host` = "" PLACEHOLDER); link tested only on URSim CB3 3.15.8 |
| ARES | not reachable from the laptop during the build (192.168.1.10:48898 timed out); client tested against the fake PLC and the MA reference model |
| Calibration | none yet (`calib/` empty) – real runs refuse to start until both files exist |

## 1. Laptop software

```
py.exe -m pip install -r requirements.txt
py.exe tools/cam_check.py list          # IDS peak version, GenTL producers, cameras, firmware (>= 3.31 needed)
```

IDS peak: install from IDS (Windows 64-bit) with the **GigE Vision transport layer "Kernel (GEVK)"**; if the kernel
filter conflicts with the Hikrobot (MVS) or TwinCAT filters already bound to the NICs, reinstall with "Socket (GEV)".
Do not install the IDS Software Suite / uEye transport layer (only for the older UI models). After installing, a new
login (or WSL restart) makes `GENICAM_GENTL64_PATH` visible; until then the code finds the producer in
`C:\Program Files\IDS\ids_peak\ids_gevgentl\64\` itself.

## 2. Camera network

The camera answers at 192.168.0.1 (as delivered: DHCP + link-local, persistent IP off). The laptop NIC "Ethernet"
carries 192.168.1.20/24 (ARES subnet) + 169.254/16, so the camera cannot be opened. ARES already uses 192.168.1/24,
192.168.200/24 and 169.254/16 (MA `10_robot/ipc_c6030/IP_CONFIG_erwan.md`), so the camera gets its own subnet.

Recommended: a **dedicated NIC for the camera** ("Ethernet 2", Realtek USB GbE, jumbo frames available), static
address 192.168.50.1/24 (PLACEHOLDER subnet, any unused one works), camera persistent IP 192.168.50.10
(`[camera] ip`). In an administrator PowerShell (check the property names first with
`Get-NetAdapterAdvancedProperty -Name "Ethernet 2"`):

```
New-NetIPAddress -InterfaceAlias "Ethernet 2" -IPAddress 192.168.50.1 -PrefixLength 24
Set-NetAdapterAdvancedProperty -Name "Ethernet 2" -RegistryKeyword "*JumboPacket" -RegistryValue 9014
# receive buffers to the maximum the driver offers (Realtek: currently 32)
```

Camera IP, persistent (check the flags with `ids_ipconfig.exe --help` first):

```
"C:\Program Files\IDS\ids_peak\program\ids_ipconfig.exe" -p -s 4110073444 -i 192.168.50.10 -n 255.255.255.0 --enable-persistent-ip
py.exe tools/cam_check.py list          # must now say "openable", firmware >= 3.31
py.exe tools/cam_check.py grab -n 5     # timing, grey statistics, incomplete frames?
```

Alternative if the camera has to share a cable/switch with ARES: add 192.168.50.1/24 as a second address on that NIC.
**Then the camera needs the gateway 192.168.50.1** (2026-10-06, UR5 table test): the IDS GigE producer knows only one
address per NIC (`ids_ipconfig --cti "...\ids_gevgentlk.cti" -l` showed 192.168.1.20) and sets it as the stream
destination; a camera in 192.168.50/24 without a gateway cannot send there - it opens, but no frame arrives ("no
frame within 3000 ms"). With `-g 192.168.50.1` the stream goes to the laptop NIC, which owns 192.168.1.20 too:
`ids_ipconfig.exe --cti "C:\Program Files\IDS\ids_peak\ids_gevgentl\64\ids_gevgentlk.cti" -p -s 4110073444 -i
192.168.50.10 -n 255.255.255.0 -g 192.168.50.1 --enable-persistent-ip --disable-dhcp -R`. A dedicated NIC avoids it.
If frames arrive incomplete: jumbo frames on, receive buffers up, and as a last resort unbind the Hikrobot filter from
the camera NIC (`Disable-NetAdapterBinding -Name "<NIC>" -ComponentID HKR_neuGEVFilter` – system change, decide first).

## 3. Power and wiring

**Since 2026-10-08 (Samuel): PoE from the switch** (camera, UR and ARES on one switch to the laptop NIC "Ethernet");
the Hirose cable to the UR tool connector is unplugged at the camera - never both. The UR tool-connector supply below
was the setup of the 2026-10-06 table test.

- Camera: **12–24 V on the Hirose HR25 8-pin, pin 8 = VCC, pin 1 = GND, max 4.2 W – or PoE, never both** (IDS: both at
  once can destroy the camera). Planned: UR tool connector 24 V (CB3: max 600 mA, UR5 manual 1.9.4; camera ≈ 0.18 A).
  Lumberg RKMV 8-354 cable: grey = power → pin 8, red = 0 V → pin 1. The tool flange is GND.
- PolyScope: Installation → I/O → Tool Output Voltage **24 V** (controlled by the user). Check from the laptop:
  `py.exe tools/ur_check.py --host <UR-IP> tool-voltage 24` (sets it in a block and reads it back via RTDE).
- Gripper Festo EHPS-20-A: up to 2 A at 24 V while moving → not on the tool connector and at the 2 A limit of the
  control box's internal 24 V → supply it from ARES 24 V, only the open/close signals from the UR (`[ur]
  do_grip_close/open`, PLACEHOLDER: Gino's code pulses DO1/DO0).
- GigE cable: high-flex, torsion-rated Cat6 with screw-lock RJ45 at the camera, clipped along the arm with slack loops
  at the wrist joints; keep the wrist-3 rotation range limited in the programs.

## 4. Mount, focus, iris, exposure

1. Bracket on the flange side of the gripper, the camera looking along the tool axis. `[camera.mount]` is a PLACEHOLDER
   (-150, 0, 60) mm; the RoboDK occlusion survey says ≥ ~150 mm off the flange axis on the flange −x side (jaw closing
   direction), never on ±y (the held stone fills the view). Board visibility per stop: `results/look_study.md`.
   Mass of camera + lens (90 g) + bracket + cable counts towards the payload (`[ur] payload_tool_kg`, PLACEHOLDER 0 →
   real runs refuse to start).
2. Focus and iris at the working distance (`[camera] working_dist`, ASSUMPTION 320 mm): put a board 320 mm in front of
   the lens and run `py.exe tools/cam_check.py live` – maximise the focus measure (variance of the Laplacian in the
   centre ROI), iris **F5.6** (F8 already costs ~4 px diffraction blur), then **lock both screws**. Any later change of
   focus, iris, binning or ROI = repeat steps 6 and 7.
3. Exposure: in `live` with `+`/`-` until the white squares are bright but < 1 % saturated; enter `[camera]
   exposure_us` and `gain` (PLACEHOLDER).

## 5. Boards

```
py.exe tools/print_targets.py --out targets     # calib.pdf + one PDF/PNG per [[targets]] board
```

- Print the PDFs at **100 %** (no "fit to page") on a laser printer, glue them flat onto a rigid plate (Alu-Dibond or
  glass). Each sheet shows the board name, marker id range, the origin cross and x/y arrows (OpenCV board frame: origin
  top-left outer corner, x right, y down, z into the board) and a 100 mm scale bar.
- **Measure the printed square size** with a caliper over as many squares as possible and enter `square_mm` (and
  `marker_mm` scaled by the same factor) in `[boards.calib]` / `[boards.ref]` – a 0.1 % scale error is 0.32 mm depth at
  320 mm.
- Every reference board has its own id range (`[[targets]] first_id`); do not print a board twice.
- Wall boards: glued on laser-cut 4 mm MDF plates (`py.exe tools/make_plates.py` -> `targets/*_plate.dxf`) that lie on
  the floor in the gap ARES front – wall, pressed against the female-pin base blocks with V-notches on the block
  joints, one every 800 mm (W0–W7, u = 100 … 5700) -> positions fixed by the blocks. Step by step: `targets/README.md`.
  The block size 120 × 200 mm is a PLACEHOLDER (`[plates]`) – enter the real one and regenerate before cutting.

## 6. Intrinsic calibration

```
py.exe tools/calib_intrinsics.py capture --dataset data/intr_<date>     # SPACE saves a view, q quits
py.exe tools/calib_intrinsics.py solve data/intr_<date>                 # -> calib/camera_intrinsics.json
```

20–30 views of the calib board at 290–355 mm, tilted 10–30° in different directions, corners reaching every image
region (the preview shows a coverage map). Check the report: RMS, standard deviations of fx/fy/cx/cy, per-view errors
(a view far above the median → delete it and solve again). Synthetic reference (research 2026-10-05): RMS ≈ 0.03 px,
fx within 0.01 %; real images will be worse – note the real numbers in the README.

## 7. Hand-eye calibration (eye-in-hand)

Board fixed rigidly on the ARES deck at `[boards.calib] xyz/rpy` (PLACEHOLDER: centre 504 mm behind the UR axis, on
magazine row 1 – **magazine empty**), so ARES sway moves board and robot together.

```
py.exe tools/calib_handeye.py plan --check --host <UR-IP>          # which of the 25 look poses the controller reaches
py.exe tools/calib_handeye.py capture --host <UR-IP> --dataset data/he_<date>
py.exe tools/calib_handeye.py solve data/he_<date>                 # -> calib/handeye.json (PARK, hold-out every 5th)
py.exe tools/calib_handeye.py verify --host <UR-IP>                # new poses: spread of the board in the base frame
```

Each image is taken after `[camera] settle_s` with the arm still (`[vision] max_qd_rad_s`); the flange pose is the mean
of the RTDE samples during the exposure. `solve` prints all five OpenCV methods – if they disagree by more than ~1 mm /
0.1° the data or a convention is wrong. Acceptance (ASSUMPTION thresholds, set them from the first real run): hold-out
and verify spread ≤ 0.5 mm / 0.1°.

## 8. Validation and settle time

```
py.exe tools/measure_target.py repeat --host <UR-IP> --n 10 --settle-s 0,0.5,1,2 --move-mm 20   # -> [camera] settle_s
py.exe tools/calib_handeye.py plan --write data/poses.json
py.exe tools/measure_target.py poses --host <UR-IP> --poses data/poses.json --report data/poses_report.json
```

`repeat` measures how long ARES keeps swaying on its springs after the arm stops (replace the 2 s PLACEHOLDER).
`poses` measures the same board from many poses – its spread is the real accuracy of intrinsics + hand-eye.
Physical check: place one stone at a camera-measured pose and measure the offset by hand.

## 9. Robot and platform day-1 checks

UR5 (CB3):

```
py.exe tools/ur_check.py --host <UR-IP> info            # PolyScope version, RTDE fields (tcp_offset!), modes, q, TCP
py.exe tools/ur_check.py --host <UR-IP> ready           # power on + brake release
py.exe tools/ur_check.py --host <UR-IP> pose
```

Enter the IP in `[ur] host`. Check: tool voltage stays on after an e-stop / protective stop? (camera reconnect), block
start latency, RoboDK FK vs RTDE TCP at a few poses (kinematic difference).

ARES (amr_hmi v2 open, connected, MANUAL; C6030 runtime stopped, otherwise `bExtActive` blocks every move; TwinCAT
route laptop 192.168.1.20.1.1 ↔ CX9240 192.168.1.10.1.1):

```
py.exe tools/ares_check.py status                       # read-only, fails fast if the PLC is unreachable
py.exe tools/ares_check.py move --dy 100 --yes          # one sideways move (+y = left), prints the PLC outcome
```

The sideways (+y) move has never been measured externally (E003 covered x only, unloaded) – measure it (TS16) before
trusting stop distances; the camera loop corrects the error anyway.

## 10. Safety (open, decide before the first run at the wall)

- The front safety scanner is in the E-stop chain; whether its active field trips with the wall ~120 mm ahead is
  unknown (MA runbook W3.4).
- There is no interlock between UR5 and ARES in the PLC; the sequencer only moves ARES when the arm is parked and idle.
  Whether the UR5 and ARES E-stops are linked is unknown.
- Payload: tool 1.68 kg (`[ur] payload_tool_kg`, CONFIRMED 2026-10-06) + stone 3.0 kg (`[brick] mass_kg`, ASSUMPTION
  "about 3 kg", weigh it) = 4.68 kg of the UR5's 5 kg; the half stone is not weighed yet (`[half_brick] mass_kg`).
- First runs: `--step` (confirm every motion), the conservative `[ur]` speeds, one stop at a time.

## 11. Running a job

```
py.exe tools/make_job.py --length 24 --out data/jobs/nominal_L24.json
py.exe tools/run_job.py data/jobs/nominal_L24.json --dry-run
py.exe tools/run_job.py --nominal 24 --sim --scenario realistic --compare      # pure-Python world, with vs without camera
py.exe tools/run_job.py data/jobs/nominal_L24.json --real --step --stops 0     # refuses while PLACEHOLDERs are open
```

The nominal job checks reachability only kinematically; the RoboDK planner (collision-checked looks and transfers)
exports the same format. Run logs: `data/runs/<timestamp>/run.jsonl`.

## 12. Tests and simulators

```
py.exe -m pytest                                    # unit tests (no hardware, no RoboDK, no Docker)
py.exe -m pytest -m ursim                           # URSim CB3 3.15.8 in Docker (image on disk, ~2-3 min)
MAUER_SLOW=1 WSLENV=MAUER_SLOW py.exe -m pytest -m slow   # full-wall world simulations (cmd: set MAUER_SLOW=1)
py.exe -m pytest -m robodk                          # RoboDK simulated camera (opens its own RoboDK instance)
```

## 13. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `IDS peak runtime not installed` | step 1 |
| camera found but `cannot be opened for control` | IP outside the NIC subnet → step 2 |
| incomplete frames / timeouts | jumbo frames, receive buffers, cable; `cam_check grab -n 20` |
| `[ur].host is an empty PLACEHOLDER` | enter the UR IP or pass `--host` |
| block `did not start within 3.0 s` | URScript compile error → PolyScope Log tab (the program text is printed) |
| ARES `not reachable: TCP 192.168.1.10:48898` | laptop not on the ARES network, CX9240 off, wrong `[ares_ads]` |
| ARES preflight: not MANUAL / ext active / heartbeat | amr_hmi open and in MANUAL? C6030 runtime stopped? |
| run_job `--real` refuses | it lists every PLACEHOLDER still open (host, payload, stone mass, calibration files, job) |

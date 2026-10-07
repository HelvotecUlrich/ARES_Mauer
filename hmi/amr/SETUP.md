> Copied 2026-10-07 from the MA repo `10_robot/hmi/amr_hmi/SETUP.md` (commit 5935c5b, unchanged below this note).
> In ARES_Mauer the config is `config/station.toml` [hmi], [hmi.frame], [hmi.move] and [ares_ads] (no config.yaml); start with `py.exe -m hmi`.
> Not copied: config.yaml, list_symbols.py, ads_diag.py, ads_portscan.py, requirements*.txt (see hmi/README.md).

# AMR HMI — New Machine Setup Guide

> Updated 25.09.2026 for HMI v2, rev. 2 (see `README.md`). Relative move and odometry need the PLC build v2
> (`10_robot/twincat/PLC_CX9240_v2`); an older PLC (no `nIfVersion`) is detected and runs in legacy mode. A v2 PLC
> whose symbols do not match this HMI (e.g. without `eMoveCmdResult` / `bMoveLimited`) is not connected; the
> status bar names the missing fields. First drive after installing PLC v2: checklist in `README.md` and
> `40_experiments/RUNBOOK_Robotertag.md` block R.

## Prerequisites

| Requirement | Details |
|---|---|
| Operating System | Windows 10/11 or Linux (Ubuntu 22.04+) |
| Python | ≥ 3.10 (3.11 or 3.12 recommended — see known issues) |
| TwinCAT ADS | Must be reachable from this machine over TCP/IP |
| Network | HMI PC and TwinCAT PLC on the same subnet (e.g. `192.168.1.x`) |

The TwinCAT PLC must be in **RUN** mode and **port 851** (PLC Runtime 1) must be active before the HMI can connect.

---

## 1. Copy the Project

Copy the `amr_hmi` folder to the new machine. The folder should contain:

```
amr_hmi/
  main.py
  ads_worker.py        # ADS thread (connection, poll, heartbeat, commands)
  plc_vars.py          # symbol lists interface v1 / v2
  frame.py             # physical direction -> PLC sign mapping
  logic.py             # move commands, enable rules
  ads_diag.py
  ads_portscan.py
  list_symbols.py
  config.yaml
  requirements.txt
  requirements-dev.txt # pytest (tests only)
  ui/
    main_window.py
    control_widget.py
    jog_panel.py
    move_panel.py
    odom_map.py
    widgets.py
    constants.py
    dashboard_widget.py
    battery_widget.py
    diagnostics_widget.py
  tests/
```

---

## 2. Create a Virtual Environment

Open a terminal in the `amr_hmi` folder:

```bash
# Windows (PowerShell)
python -m venv .venv
.venv\Scripts\activate

# Linux / WSL
python3 -m venv .venv
source .venv/bin/activate
```

---

## 3. Install Dependencies

With the virtual environment active:

```bash
pip install -r requirements.txt
```

This installs:
- `pyads` — TwinCAT ADS communication
- `PySide6` — GUI framework
- `PyYAML` — config file parser

---

## 4. Configure the ADS Route

ADS requires a bidirectional route between the HMI PC and the TwinCAT PLC.
Both sides must know about each other.

### 4a. On the TwinCAT PC (Windows side)

1. Open **TwinCAT XAE** (or the TwinCAT System Manager).
2. Go to **System → Routes → Add**.
3. Add a static route for the HMI PC:
   - **Route name**: `AMR_HMI_Route`  
   - **AMS Net ID**: `<HMI_PC_IP>.1.1`  
     (e.g. if the HMI PC is `192.168.1.20`, use `192.168.1.20.1.1`)
   - **Transport**: TCP/IP
   - **IP Address**: the HMI PC's IP

4. Note down the TwinCAT PLC's own **AMS Net ID** (shown in the TwinCAT tray icon or under *System → Routes*). You will need this for `config.yaml`.

### 4b. On the HMI PC (Windows)

The HMI does not create ADS routes itself. With TwinCAT (or the TwinCAT ADS Setup) installed, add the route to
the CX9240 once in the TwinCAT router (*Edit Routes* / *Add Route* dialog, or from XAE); the HMI then opens the
connection through the local router.

If you do not have TwinCAT installed on the HMI PC, install the **TwinCAT ADS Setup** (free, available from Beckhoff) which installs only the ADS router service without the full TwinCAT environment.

### 4b. On the HMI PC (Linux / WSL)

```bash
# Install the ADS daemon
sudo apt install ads-daemon      # or compile from source (github.com/Beckhoff/ADS)

# Add a route to the TwinCAT PC
sudo adsroutectl add <TC_AMS_NET_ID> <TC_IP> AMR_PLC
# Example:
sudo adsroutectl add 192.168.1.10.1.1 192.168.1.10 AMR_PLC
```

Start the daemon if it is not running:

```bash
sudo systemctl enable --now adsd
```

---

## 5. Edit config.yaml

Open `config.yaml` and update the `ads` section to match your network:

```yaml
ads:
  ams_net_id: "192.168.1.10.1.1"  # TwinCAT PLC AMS Net ID (from Step 4a)
  ads_port: 851                   # TwinCAT 3 PLC Runtime 1 — do not change
  host_ip: "192.168.1.10"         # TwinCAT PC IP address
  timeout_ms: 1000               # applied with Connection.set_timeout()
```

The `plc` and `hmi` sections generally do not need to be changed:

```yaml
plc:
  from_plc_struct: "GVL_HMI.stFromPlc"
  to_plc_struct:   "GVL_HMI.stToPlc"

hmi:
  poll_interval_ms: 100
  heartbeat_interval_ms: 100    # PLC: 2 s watchdog (jog/modes), 500 ms for the relative move
  reconnect_interval_s: 5
  watchdog_timeout_s: 2.0
```

The `frame` section and the `move` section (relative-move presets and limits) are described in `README.md`.
Do not change the `frame` defaults before the direction tests:

```yaml
frame:
  plus_y_is_left: false     # = previous HMI ("Left" -> bCmdJogLeft -> PLC -vy); Y test move decides
  plus_omega_is_ccw: true   # = previous HMI ("CCW" -> bCmdJogRotLeft -> PLC +omega); rotation test decides
  verified: false           # true only after both tests (runbook block R)
```

> **Save `config.yaml` as UTF-8** (not UTF-8 with BOM). Most editors default to this.
> If you use Notepad on Windows, choose *File → Save As → Encoding: UTF-8*.

---

## 6. Start the HMI

With the virtual environment active:

```bash
cd amr_hmi
python main.py
```

To use a config file at a different path:

```bash
python main.py --config /path/to/config.yaml
```

The HMI will attempt to connect to the PLC immediately and retry every 5 seconds if the connection fails. The connection status is shown in the top bar.

---

## 7. Verify Connection

In the HMI:
- **Top bar** shows `Connected` (green) or `Disconnected` (red).
- The **Diagnostics tab** shows live values for all PLC variables and the PLC heartbeat counter — it increments every PLC cycle while the PLC runs (status bar shows "FROZEN" otherwise).
- The status bar shows the PLC build and the interface version (v2 = relative move available, v1 = legacy).
- `bPlcRunning` should be `TRUE` and `bHmiWatchdogOK` should be `TRUE` within 2 seconds of connecting.

---

## 8. Troubleshooting

### Run the ADS diagnostic script

Before starting the HMI, you can run the standalone diagnostic tool:

```bash
python ads_diag.py
```

This probes the ADS connection and reports which step fails (TCP reachability, ADS router, PLC runtime port).

### Scan ADS ports

```bash
python ads_portscan.py
```

Scans common ADS ports on the target and reports which ones are responding.

### List PLC symbols

Once connected, verify that the expected variable names exist on the PLC:

```bash
python list_symbols.py
```

### Common errors

| Error | Cause | Fix |
|---|---|---|
| `ADS error 6` (target port not found) | PLC runtime not running or not in RUN mode | Start the TwinCAT PLC runtime; check port 851 is active |
| `ADS error 1808` (symbol not found) | Variable name mismatch between HMI and PLC | Run `list_symbols.py` to confirm actual symbol paths; check `from_plc_struct` / `to_plc_struct` in `config.yaml` |
| `Connection timeout` | Wrong IP or AMS Net ID in `config.yaml` | Ping the PLC IP; check AMS Net ID in TwinCAT System Manager |
| `UnicodeDecodeError` on startup | `config.yaml` saved with wrong encoding | Re-save as UTF-8 (no BOM) |
| HMI connects but watchdog stays `FALSE` | Heartbeat not reaching PLC | Check that port 851 traffic is not firewalled; verify `GVL_HMI.stToPlc.nHeartbeat` increments in TwinCAT watch |
| Jog buttons have no effect | Not in MANUAL, external control (C6030) active, or a relative move running | Follow the startup sequence to **Manual**; stop the C6030 / ROS bridge |
| "Legacy PLC (interface v1)" banner | PLC build v2 not loaded | Relative move unavailable until PLC build v2 is active |
| "PLC build does not match this HMI: interface v2 but missing ..." | active v2 build older/newer than this HMI | Load the matching PLC build v2; the HMI reconnects by itself (retry every `reconnect_interval_s`) |

---

## HMI Tabs Overview

| Tab | Contents |
|---|---|
| **Dashboard** | AMR state, lamp color, body velocity, wheel speeds/angles, safety flags, fault/warning text, battery summary |
| **Control** | Status header, guided startup sequence (Safety Run, Reset, Start, Manual, Standby), drive status, SOC, sub-tabs **Jog** and **Relative move** (with odometry map). HALT button above all tabs |
| **Diagnostics** | Raw live values for all PLC variables with types and descriptions; ADS statistics |
| **Battery** | SOC gauge (green/yellow/red), voltage, current, power, charge status |

---

## Watchdog Behaviour

The HMI writes a heartbeat counter (`nHeartbeat`) to the PLC every 100 ms from its ADS worker thread.
The PLC monitors this with a 2-second timeout (`FB_HMI_Interface`); a running relative move is aborted after
500 ms without heartbeat (`GVL_Move.tHbTimeout`, PLC build v2):

- **HMI alive**: level commands (jog, safety run, mode) are forwarded normally.
- **HMI disconnected**: after 2 s the watchdog fires, all level outputs are cleared, jogging stops. Drive power (`bCmdSafetyRun`) is intentionally **kept active** so a lost HMI connection does not cut motor power mid-move.

---

## AMR State Reference

| Index | State | Lamp |
|---|---|---|
| 0 | INIT | Off |
| 1 | WAIT_SAFETY | Yellow |
| 2 | SAFETY_STOP | Red |
| 3 | RESET_REQUIRED | Magenta |
| 4 | STANDBY | Blue |
| 5 | DRIVES_ENABLE | Cyan |
| 6 | READY | Green |
| 7 | MANUAL_MODE | White |
| 8 | AUTO_MODE | Green |
| 9 | NAVIGATING | Green (blink) |
| 10 | OBSTACLE_STOP | Yellow (blink) |
| 11 | DOCKING | Cyan |
| 12 | CHARGING | Blue |
| 13 | ERROR | Red (blink) |
| 14 | ERROR_ACK | Red |
| 15 | SHUTDOWN | Off |
| 16 | PRECHARGE | - (transitional, before STANDBY) |

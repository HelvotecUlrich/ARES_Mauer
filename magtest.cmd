@echo off
REM Magazine dry run on ARES (docs\MAGTEST_DE.md): two full stones through every magazine slot (layers 1 + 2), each
REM move goes forward and "places" 50 mm above the front wall WITHOUT opening the jaws. ARES stands still, no camera.
REM Usage:  magtest              build data\jobs\magtest.json from config\station.toml [magtest], open it in the HMI
REM         magtest --rounds 1   one round there and back (20 moves) instead of [magtest] rounds
REM In the HMI: Mauer tab -> SIM first (Prepare, Start), then REAL (step mode on) - REAL needs no --ares.
cd /d "%~dp0"
py tools\make_magtest.py %*
if errorlevel 1 goto end
py -m hmi --job data\jobs\magtest.json
:end
echo.
pause

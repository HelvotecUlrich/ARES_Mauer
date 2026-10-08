@echo off
REM Mauer HMI (hmi/README.md): job, SIM / REAL run, camera, UR, ARES, run log, RoboDK twin.
REM Usage:  hmi                                   SIM only - no ADS connection ("ADS off"), REAL disabled
REM         hmi --job data\jobs\nominal_C.json    load a job at start (with its own config variant)
REM         hmi --job data\jobs\nominal_C.json --twin   ... and mirror the run in an own RoboDK (port 20630+)
REM         hmi --ares                            the HMI's ADS worker connects to the ARES PLC (REAL runs)
cd /d "%~dp0"
py -m hmi %*
echo.
pause

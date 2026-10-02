@echo off
REM Rebuild the RoboDK station (ARES + UR5 + gripper + wall)
cd /d "%~dp0"
py robodk\build_station.py %*

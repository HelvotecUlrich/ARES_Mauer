@echo off
REM Brick-laying simulation in RoboDK (own instance, window minimised - restore it from the taskbar to watch).
REM Usage:  sim --animate      the L of the config: wall + station trips (~20 min), ARES moves animated
REM         sim --no-trips     the wall only, the magazine refilled without driving or picking
REM         sim --length 24    straight wall of 24 stones
REM At the end the station is saved as robodk\ARES_UR5_Mauer_L.rdk (open it in RoboDK), report results\l_wall_sim.md.
cd /d "%~dp0"
py robodk\simulate.py %*
echo.
pause

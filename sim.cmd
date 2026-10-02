@echo off
REM Brick-laying simulation in RoboDK.  Usage:  sim            (16-stone wall)
REM                                          sim --length 24 (longer wall)
REM                                          sim --speed 5   (faster)
cd /d "%~dp0"
py robodk\simulate.py %*
echo.
pause

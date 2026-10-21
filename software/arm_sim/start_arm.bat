@echo off
call "%USERPROFILE%\miniforge3\Scripts\activate.bat" armsim
cd /d "%USERPROFILE%\arm_sim"
start "" http://localhost:8080
python ui_app.py
pause
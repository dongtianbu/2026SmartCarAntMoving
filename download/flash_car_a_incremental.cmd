@echo off
powershell -ExecutionPolicy Bypass -File "%~dp0flash_car_a.ps1" -Mode incremental %*
set "CODE=%ERRORLEVEL%"
echo.
if "%CODE%"=="0" (
    echo CarA incremental burn finished successfully.
) else (
    echo CarA incremental burn failed. Exit code: %CODE%
)
pause
exit /b %CODE%

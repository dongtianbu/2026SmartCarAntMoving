@echo off
powershell -ExecutionPolicy Bypass -File "%~dp0flash_car_a.ps1" %*
set "CODE=%ERRORLEVEL%"
echo.
if "%CODE%"=="0" (
    echo CarA burn finished successfully.
) else (
    echo CarA burn failed. Exit code: %CODE%
)
pause
exit /b %CODE%

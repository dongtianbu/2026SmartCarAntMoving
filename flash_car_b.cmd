@echo off
powershell -ExecutionPolicy Bypass -File "%~dp0flash_car_b.ps1" %*
set "CODE=%ERRORLEVEL%"
echo.
if "%CODE%"=="0" (
    echo CarB burn finished successfully.
) else (
    echo CarB burn failed. Exit code: %CODE%
)
pause
exit /b %CODE%

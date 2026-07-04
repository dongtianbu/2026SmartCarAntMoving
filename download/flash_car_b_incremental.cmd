@echo off
powershell -ExecutionPolicy Bypass -File "%~dp0flash_car_b.ps1" -Mode incremental %*
set "CODE=%ERRORLEVEL%"
echo.
if "%CODE%"=="0" (
    echo CarB incremental burn finished successfully.
) else (
    echo CarB incremental burn failed. Exit code: %CODE%
)
pause
exit /b %CODE%

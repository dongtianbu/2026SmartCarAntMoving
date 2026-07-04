@echo off
chcp 65001 >nul
title 一键推送到 GitHub 仓库
cd /d D:\2026_SmartCar\PersonalCode

echo ============================================
echo   一键推送到 GitHub 仓库
echo   仓库: 2026SmartCarAntMoving (main 分支)
echo ============================================
echo.

echo [1/4] 检查仓库状态...
git status --short
echo.

echo [2/4] 添加所有文件到暂存区...
git add -A
echo.

echo [3/4] 创建提交...
for /f "tokens=2 delims==" %%a in ('wmic OS Get localdatetime /value') do set "dt=%%a"
set "timestamp=%dt:~0,4%-%dt:~4,2%-%dt:~6,2% %dt:~8,2%:%dt:~10,2%:%dt:~12,2%"
git commit -m "Auto push at %timestamp%"
echo.

echo [4/4] 推送到远程 main 分支...
git push origin main
echo.

if %errorlevel%==0 (
    echo ============================================
    echo   推送成功！
    echo ============================================
) else (
    echo ============================================
    echo   推送失败，请检查上方错误信息
    echo ============================================
)

echo.
pause

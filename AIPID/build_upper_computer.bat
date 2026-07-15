@echo off
setlocal

cd /d "%~dp0"

echo [1/1] 开始打包 AIPID 上位机...
python -m PyInstaller ^
  --noconfirm ^
  --clean ^
  --windowed ^
  --onefile ^
  --name AIPID_UpperComputer ^
  aipid_upper_computer.py

if errorlevel 1 (
  echo.
  echo 打包失败，请先检查上面的报错信息。
  pause
  exit /b 1
)

echo.
echo 打包完成，exe 位于 dist\AIPID_UpperComputer.exe
pause

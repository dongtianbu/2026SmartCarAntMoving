param(
    [string]$Port = "",
    [ValidateSet("full", "incremental")]
    [string]$Mode = "full"
)

$ErrorActionPreference = "Stop"

$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$personalCodeRoot = Split-Path -Parent $scriptRoot
$workspaceRoot = Split-Path -Parent $personalCodeRoot
$projectRoot = Join-Path $personalCodeRoot "CarB"
$cliRoot = Join-Path $workspaceRoot "mpy-cli"
$venvPython = Join-Path $cliRoot ".venv\Scripts\python.exe"

if (Test-Path $venvPython) {
    $pythonExe = $venvPython
    $env:PATH = "$(Join-Path $cliRoot '.venv\Scripts');$env:PATH"
} else {
    $pythonExe = "python"
}

$env:PYTHONPATH = $cliRoot

if ([string]::IsNullOrWhiteSpace($Port)) {
    $ports = [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
    if ($ports.Count -gt 0) {
        Write-Host "Available serial ports:" -ForegroundColor Cyan
        foreach ($item in $ports) {
            Write-Host "  $item"
        }
        Write-Host ""
    } else {
        Write-Host "No serial port detected automatically. Enter one manually." -ForegroundColor Yellow
    }
    $Port = Read-Host "Enter CarB serial port (example: COM8)"
}

if ([string]::IsNullOrWhiteSpace($Port)) {
    throw "Serial port is required."
}

if (-not (Test-Path $projectRoot)) {
    throw "CarB project directory not found: $projectRoot"
}

if (-not (Test-Path $cliRoot)) {
    throw "mpy-cli directory not found: $cliRoot"
}

Push-Location $projectRoot
try {
    Write-Host "== CarB deploy start =="
    Write-Host "Project: $projectRoot"
    Write-Host "Port: $Port"
    Write-Host "Mode: $Mode"
    Write-Host ""

    $stdoutPath = Join-Path $env:TEMP "flash_car_b_stdout.txt"
    $stderrPath = Join-Path $env:TEMP "flash_car_b_stderr.txt"
    Remove-Item $stdoutPath, $stderrPath -ErrorAction SilentlyContinue

    $proc = Start-Process `
        -FilePath $pythonExe `
        -ArgumentList @("-m", "mpy_cli", "deploy", "--mode", $Mode, "--port", $Port, "--no-interactive", "--yes") `
        -WorkingDirectory $projectRoot `
        -NoNewWindow `
        -Wait `
        -PassThru `
        -RedirectStandardOutput $stdoutPath `
        -RedirectStandardError $stderrPath

    $deployOutput = @()
    if (Test-Path $stdoutPath) {
        $deployOutput += Get-Content $stdoutPath
    }
    if (Test-Path $stderrPath) {
        $deployOutput += Get-Content $stderrPath
    }
    $deployOutput | ForEach-Object { $_ }
    $exitCode = $proc.ExitCode

    if ($exitCode -ne 0) {
        $deployText = ($deployOutput | Out-String)
        Write-Host ""

        if ($deployText -match "failed to access" -or $deployText -match "in use by another program") {
            Write-Host "COM port is busy." -ForegroundColor Yellow
            Write-Host "Close Thonny, serial assistants, mpremote terminals, or any program using $Port, then try again." -ForegroundColor Yellow
        } elseif ($Mode -eq "incremental" -and $deployText -match "not a git repository") {
            Write-Host "Incremental deploy requires CarB to be inside a valid Git repository." -ForegroundColor Yellow
            Write-Host "Current expected repository root: $personalCodeRoot" -ForegroundColor Yellow
        } elseif ($deployText -match "No module named" -or $deployText -match "not found") {
            Write-Host "If mpremote is missing, install it with:" -ForegroundColor Yellow
            Write-Host "  python -m pip install mpremote" -ForegroundColor Yellow
            Write-Host "or, if you use the mpy-cli venv:" -ForegroundColor Yellow
            Write-Host "  D:\2026_SmartCar\mpy-cli\.venv\Scripts\python.exe -m pip install mpremote" -ForegroundColor Yellow
        }

        throw "mpy-cli deploy failed with exit code $exitCode"
    }

    Write-Host ""
    Write-Host "== CarB deploy success =="
} finally {
    Remove-Item $stdoutPath, $stderrPath -ErrorAction SilentlyContinue
    Pop-Location
}

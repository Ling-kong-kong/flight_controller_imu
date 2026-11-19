param([string]$Python = "py")
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot
if (!(Test-Path ".venv/Scripts/python.exe")) {
    if ($Python -eq "py") { & py -3.11 -m venv .venv } else { & $Python -m venv .venv }
    if ($LASTEXITCODE -ne 0) { throw "Python 3.11+ is required" }
}
& .venv/Scripts/python.exe -m pip install -e .
if ($LASTEXITCODE -ne 0) { throw "Dependency installation failed" }
Write-Host "Ready: run .\run.ps1"

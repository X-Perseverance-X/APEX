param([string]$Python = "")
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
if (-not $Python) {
    $Python = Join-Path $Root ".venv\Scripts\python.exe"
}
if (-not (Test-Path $Python)) {
    throw "Python venv bulunamadı: $Python"
}
& $Python (Join-Path $PSScriptRoot "dummy_stack.py") start
exit $LASTEXITCODE

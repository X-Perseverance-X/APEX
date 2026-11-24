$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$Python = Join-Path $Root ".venv\Scripts\python.exe"
if (-not (Test-Path $Python)) {
    throw "Python venv bulunamadı: $Python"
}
& $Python (Join-Path $PSScriptRoot "dummy_stack.py") stop
exit $LASTEXITCODE

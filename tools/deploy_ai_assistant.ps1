# Deploys the assistant (apex_ai_assistant.py, apex_scene.py, index.html, models/) to the Pi, restarts ONLY
# apex-ai-assistant.service and runs the API test before and after.
# Motor/servo/ESP services and the camera service are not touched.
#
#   powershell -ExecutionPolicy Bypass -File tools\deploy_ai_assistant.ps1
#
# ssh/scp ask for the Pi password (twice; an SSH key avoids that).
param(
    [string]$PiHost = "apex.local",
    [string]$User = "perseverance",
    [switch]$SkipTests,
    [switch]$InstallKey   # one-time: put this PC's SSH key on the Pi so later deploys ask no password
)
$ErrorActionPreference = "Stop"
$root = Split-Path $PSScriptRoot -Parent
$py = Join-Path $root ".venv\Scripts\python.exe"
if (-not (Test-Path $py)) { $py = "python" }
New-Item -ItemType Directory -Force (Join-Path $root "output") | Out-Null

if ($InstallKey) {
    $key = Join-Path $env:USERPROFILE ".ssh\id_ed25519"
    if (-not (Test-Path $key)) { ssh-keygen -t ed25519 -N '""' -f $key | Out-Null }
    Get-Content "$key.pub" | ssh -o StrictHostKeyChecking=accept-new "$User@$PiHost" "mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
    if ($LASTEXITCODE -ne 0) { throw "SSH anahtari kurulamadi" }
    Write-Host "SSH anahtari kuruldu; sonraki calistirmalarda sifre sorulmaz." -ForegroundColor Green
}

if (-not $SkipTests) {
    Write-Host "== API testi (ONCE) ==" -ForegroundColor Cyan
    & $py (Join-Path $root "tools\ai_assistant_api_test.py") --host $PiHost --out (Join-Path $root "output\ai_api_before.json")
}

Write-Host "== Dosyalar gonderiliyor ==" -ForegroundColor Cyan
$stage = Join-Path $env:TEMP "apex-ai-deploy"
Remove-Item -Recurse -Force $stage -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Force $stage | Out-Null
Copy-Item (Join-Path $root "assistant\apex_ai_assistant.py"), (Join-Path $root "assistant\apex_scene.py"), (Join-Path $root "assistant\index.html") $stage
Copy-Item -Recurse (Join-Path $root "assistant\models") $stage   # YOLOX person/object detector (20 MB)
$tag = Get-Date -Format yyyyMMddHHmmss
scp -o StrictHostKeyChecking=accept-new -r $stage "$User@${PiHost}:/tmp/apex-ai-deploy-$tag"
if ($LASTEXITCODE -ne 0) { throw "scp basarisiz" }

$remote = "set -e; d=/opt/apex/assistant; t=/tmp/apex-ai-deploy-$tag; " +
    '/opt/apex/venvs/assistant/bin/python -m py_compile $t/apex_ai_assistant.py $t/apex_scene.py; rm -rf $t/__pycache__; ' +
    'if [ -w $d ] && [ -w $d/apex_ai_assistant.py ]; then S=env; else S=sudo; fi; ' +
    'ts=$(date +%s); ' +
    '$S cp -p $d/apex_ai_assistant.py $d/apex_ai_assistant.py.bak-$ts; $S cp -p $d/index.html $d/index.html.bak-$ts; ' +
    '$S cp -r $t/. $d/; rm -rf $t; ' +
    'sudo -n systemctl restart apex-ai-assistant.service; echo DEPLOY_OK yedek=$ts'
ssh "$User@$PiHost" $remote
if ($LASTEXITCODE -ne 0) { throw "Deploy basarisiz (ssh cikis kodu $LASTEXITCODE)" }

if (-not $SkipTests) {
    Write-Host "== API testi (SONRA) - servis ~30 sn icinde hazir olur ==" -ForegroundColor Cyan
    Start-Sleep -Seconds 8
    & $py (Join-Path $root "tools\ai_assistant_api_test.py") --host $PiHost --out (Join-Path $root "output\ai_api_after.json")
}

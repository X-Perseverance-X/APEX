param(
    [int]$Seconds = 0,
    [double]$Rate = 4.0,
    [int]$LocalPort = 18090
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$capturePython = Join-Path $root ".venv\Scripts\python.exe"
$fusionPython = Join-Path $root ".runtime\dense-depth-env\Scripts\python.exe"
$statusUrl = "http://127.0.0.1:$LocalPort/status"

try {
    $status = Invoke-RestMethod -Uri $statusUrl -TimeoutSec 3
} catch {
    throw "Sensor tuneli hazir degil. Once tools\start_pi4_mapping_tunnel.ps1 calistir ve pencereyi acik birak."
}
if (-not ($status.camera -and $status.lidar -and $status.imu_online)) {
    throw "Sensorler hazir degil: camera=$($status.camera) lidar=$($status.lidar) imu=$($status.imu_online)"
}

$stamp = Get-Date -Format "yyyyMMdd-HHmmss"
$relativeOutput = ".runtime\wireless-map-$stamp"
$output = Join-Path $root $relativeOutput
Write-Host "Sensor kafasini yavasca gezdir: salt donus yerine 10-30 cm konum degistir." -ForegroundColor Yellow
if ($Seconds -le 0) { Write-Host "Kayit suresizdir. Bitirmek icin Ctrl+C kullan; eldeki kareler korunur." -ForegroundColor Cyan }
Read-Host "Hazirsan Enter"

& $capturePython (Join-Path $root "tools\rgbd_imu_capture.py") --base-url "http://127.0.0.1:$LocalPort" --seconds $Seconds --rate $Rate --output $output
if ($LASTEXITCODE -ne 0) { throw "Sensor kaydi basarisiz." }
& $fusionPython (Join-Path $root "tools\dense_rgbd_fusion.py") $output --input-size 420 --stride 4
if ($LASTEXITCODE -ne 0) { throw "RGB-D fuzyonu basarisiz." }

$port = 8877
$listening = Get-NetTCPConnection -LocalAddress 127.0.0.1 -LocalPort $port -State Listen -ErrorAction SilentlyContinue
if (-not $listening) {
    Start-Process -FilePath $capturePython -ArgumentList @("-m", "http.server", "$port", "--bind", "127.0.0.1", "--directory", (Join-Path $root ".runtime")) -WindowStyle Hidden
    Start-Sleep -Milliseconds 800
}
$viewer = "http://127.0.0.1:$port/wireless-map-$stamp/dense-result/dense-rgb-viewer.html"
Write-Host "Harita: $viewer" -ForegroundColor Green
Start-Process $viewer

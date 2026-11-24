param(
    [string]$PiHost = "nirmata.local",
    [int]$SensorPort = 8090,
    [int]$HudPort = 8878
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$python = Join-Path $root ".venv\Scripts\python.exe"
$densePython = Join-Path $root ".runtime\dense-depth-env\Scripts\python.exe"
$denseWorker = Join-Path $root "tools\live_dense_rgbd_worker.py"
$denseOutput = Join-Path $root ".runtime\live-dense-cloud.json"
$resolvedPiHost = $PiHost
if ($PiHost -match '\.local$') {
    $ipv4 = Resolve-DnsName $PiHost -Type A -ErrorAction SilentlyContinue |
        Where-Object { $_.IPAddress -match '^\d+\.\d+\.\d+\.\d+$' } |
        Select-Object -First 1 -ExpandProperty IPAddress
    if ($ipv4) { $resolvedPiHost = $ipv4 }
}
$sensorUrl = "http://${resolvedPiHost}:${SensorPort}"
$hudUrl = "http://127.0.0.1:${HudPort}"

if (-not (Test-Path $python)) {
    throw "APEX Python ortami bulunamadi: $python"
}

try {
    $sensor = Invoke-RestMethod -Uri "$sensorUrl/status" -TimeoutSec 4
} catch {
    throw "Pi4 sensor koprusune ulasilamadi ($sensorUrl). Pi ve laptop ayni agda olmali; apex-sensor-bridge servisini kontrol et."
}
if (-not ($sensor.camera -and $sensor.lidar -and $sensor.imu_online)) {
    throw "Sensorler hazir degil: camera=$($sensor.camera) lidar=$($sensor.lidar) imu=$($sensor.imu_online)"
}

$alreadyRunning = $false
try {
    $health = Invoke-RestMethod -Uri "$hudUrl/healthz" -TimeoutSec 1
    $alreadyRunning = $true
} catch {}

if (-not $alreadyRunning) {
    $env:APEX_SENSOR_BRIDGE_URL = $sensorUrl
    # apex_server camera proxy appends /raw-snapshot; the Pi bridge exposes it
    # below /camera, while sensor status stays at the bridge root.
    $env:CAMERA_STREAM_URL = "$sensorUrl/camera"
    $env:APEX_AUTH_ENABLED = "0"
    $env:APEX_SIM_MODE = "0"
    $env:APEX_MAP_DIR = Join-Path $root ".runtime\maps"
    $env:APEX_DENSE_CLOUD_PATH = $denseOutput
    Start-Process -FilePath $python `
        -ArgumentList @("-m", "uvicorn", "apex_server:app", "--host", "127.0.0.1", "--port", "$HudPort") `
        -WorkingDirectory $root -WindowStyle Hidden `
        -RedirectStandardOutput (Join-Path $root ".runtime\live-mapping-server.out.log") `
        -RedirectStandardError (Join-Path $root ".runtime\live-mapping-server.err.log")
    $deadline = (Get-Date).AddSeconds(20)
    do {
        Start-Sleep -Milliseconds 350
        try {
            $health = Invoke-RestMethod -Uri "$hudUrl/healthz" -TimeoutSec 1
            $alreadyRunning = $true
        } catch {}
    } until ($alreadyRunning -or (Get-Date) -ge $deadline)
    if (-not $alreadyRunning) { throw "Canli haritalama sunucusu 20 saniyede hazir olmadi." }
}

if ((Test-Path $densePython) -and (Test-Path $denseWorker)) {
    $denseProcesses = Get-CimInstance Win32_Process -Filter "Name = 'python.exe'" -ErrorAction SilentlyContinue |
        Where-Object { $_.CommandLine -like "*live_dense_rgbd_worker.py*" }
    if (-not $denseProcesses) {
        Start-Process -FilePath $densePython `
            -ArgumentList @($denseWorker, "--sensor-url", $sensorUrl, "--navigation-url", $hudUrl, "--output", $denseOutput) `
            -WorkingDirectory $root -WindowStyle Hidden `
            -RedirectStandardOutput (Join-Path $root ".runtime\live-dense-worker.out.log") `
            -RedirectStandardError (Join-Path $root ".runtime\live-dense-worker.err.log")
    }
} else {
    Write-Warning "Yoğun RGB-D ortamı bulunamadı; 2D SLAM çalışır fakat renkli oda bulutu üretilemez."
}

Start-Process "$hudUrl/navigation-console"
Write-Host "APEX canli haritalama hazir: $hudUrl/navigation-console" -ForegroundColor Green

param(
    [string]$PiHost = "nirmata.local",
    [string]$PiUser = "betul_enis",
    [int]$LocalPort = 18090
)

$ErrorActionPreference = "Stop"
Write-Host "APEX Pi4 mapping tunnel" -ForegroundColor Cyan
Write-Host "SSH ve sudo parola isterse Pi parolasini gir. Bu pencere tarama boyunca acik kalmali." -ForegroundColor Yellow
Write-Host "Yerel sensor adresi: http://127.0.0.1:$LocalPort/status"

$remote = "sudo systemctl start apex-sensor-bridge && systemctl is-active apex-sensor-bridge && echo SENSOR_BRIDGE_READY; trap 'sudo systemctl stop apex-sensor-bridge' EXIT HUP INT TERM; bash -l"
& ssh -tt -L "127.0.0.1:${LocalPort}:127.0.0.1:8090" "${PiUser}@${PiHost}" $remote
exit $LASTEXITCODE

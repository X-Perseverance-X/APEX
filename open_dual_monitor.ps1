param(
  [string]$MainUrl = "http://localhost:8080",
  [string]$LabUrl  = "http://localhost:8090"
)
Add-Type -AssemblyName System.Windows.Forms
$screens = [System.Windows.Forms.Screen]::AllScreens
$primary = $screens | Where-Object { $_.Primary } | Select-Object -First 1
$secondary = $screens | Where-Object { -not $_.Primary } | Select-Object -First 1
if (-not $secondary) { $secondary = $primary }

$candidates = @(
  "$env:ProgramFiles(x86)\Microsoft\Edge\Application\msedge.exe",
  "$env:ProgramFiles\Microsoft\Edge\Application\msedge.exe",
  "$env:LOCALAPPDATA\Microsoft\Edge\Application\msedge.exe",
  "$env:ProgramFiles\Google\Chrome\Application\chrome.exe",
  "$env:ProgramFiles(x86)\Google\Chrome\Application\chrome.exe",
  "$env:LOCALAPPDATA\Google\Chrome\Application\chrome.exe"
)
$browser = $candidates | Where-Object { Test-Path $_ } | Select-Object -First 1

function Open-OnScreen([string]$Url, $Screen) {
  $b = $Screen.Bounds
  if ($browser) {
    $args = @(
      "--new-window",
      "--window-position=$($b.X),$($b.Y)",
      "--window-size=$($b.Width),$($b.Height)",
      $Url
    )
    Start-Process -FilePath $browser -ArgumentList $args | Out-Null
  } else {
    Start-Process $Url | Out-Null
  }
}

Open-OnScreen -Url $MainUrl -Screen $primary
Start-Sleep -Milliseconds 700
Open-OnScreen -Url $LabUrl -Screen $secondary

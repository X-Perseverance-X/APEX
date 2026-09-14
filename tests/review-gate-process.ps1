$ErrorActionPreference = 'Stop'
$sourceGate = (Resolve-Path (Join-Path $PSScriptRoot '..\tools\claude-diff-review.ps1')).Path
$testRoot = Join-Path ([System.IO.Path]::GetTempPath()) ("apex-review-gate-" + [Guid]::NewGuid().ToString('N'))

try {
    [void](New-Item -ItemType Directory -Path $testRoot)
    $fakeClaude = Join-Path $testRoot 'fake-claude.ps1'
    @'
param([Parameter(ValueFromRemainingArguments=$true)][string[]]$Ignored)
[void][Console]::In.ReadToEnd()
if ($env:FAKE_CLAUDE_MODE -eq 'hang') {
    Start-Sleep -Seconds 30
    exit 0
}
if ($Ignored -notcontains '--tools=') {
    Write-Output '{"type":"system","subtype":"init","tools":["Read"]}'
    Write-Output '{"type":"result","result":"APPROVE"}'
    exit 0
}
Write-Output '{"type":"system","subtype":"init","tools":[]}'
Write-Output '{"type":"result","subtype":"success","is_error":false,"result":"APPROVE\nprocess smoke"}'
'@ | Set-Content -LiteralPath $fakeClaude

    $repo = Join-Path $testRoot 'repo'
    [void](New-Item -ItemType Directory -Path $repo)
    Copy-Item -LiteralPath $sourceGate -Destination (Join-Path $repo 'review.ps1')
    Push-Location $repo
    try {
        & git init --quiet
        & git config user.name 'APEX Review Test'
        & git config user.email 'review-test@example.invalid'
        Set-Content -LiteralPath sample.txt -Value 'base'
        & git add sample.txt
        & git commit --quiet -m base
        Set-Content -LiteralPath sample.txt -Value 'candidate'
        & git add sample.txt
        [void](New-Item -ItemType Directory -Path nested)
        Push-Location nested

        $pwshPath = (Get-Command pwsh).Source
        $prefixJson = @('-NoProfile', '-File', $fakeClaude) | ConvertTo-Json -Compress
        try {
            $env:APEX_REVIEW_TEST_MODE = '1'
            & pwsh -NoProfile -File ..\review.ps1 -Scope staged `
                -ClaudeCommand $pwshPath -ClaudePrefixArgsJson '[1]' *> $null
            if ($LASTEXITCODE -ne 1) {
                throw "Non-string prefix validation returned $LASTEXITCODE, expected 1."
            }

            $conflictingPrefix = @('--tools=Read') | ConvertTo-Json -Compress
            & pwsh -NoProfile -File ..\review.ps1 -Scope staged `
                -ClaudeCommand $pwshPath -ClaudePrefixArgsJson $conflictingPrefix *> $null
            if ($LASTEXITCODE -ne 1) {
                throw "Reserved prefix validation returned $LASTEXITCODE, expected 1."
            }

            & pwsh -NoProfile -File ..\review.ps1 -Scope staged -TimeoutSeconds 10 `
                -ClaudeCommand $pwshPath -ClaudePrefixArgsJson $prefixJson
            if ($LASTEXITCODE -ne 0) { throw "Approve process smoke returned $LASTEXITCODE." }

            Pop-Location
            $baseCommit = (& git rev-parse HEAD).Trim()
            & git commit --quiet -m candidate
            Push-Location nested
            & pwsh -NoProfile -File ..\review.ps1 -Scope branch -Base $baseCommit -TimeoutSeconds 10 `
                -ClaudeCommand $pwshPath -ClaudePrefixArgsJson $prefixJson
            if ($LASTEXITCODE -ne 0) { throw "Branch process smoke returned $LASTEXITCODE." }

            $env:FAKE_CLAUDE_MODE = 'hang'
            $clock = [System.Diagnostics.Stopwatch]::StartNew()
            & pwsh -NoProfile -File ..\review.ps1 -Scope branch -Base $baseCommit -TimeoutSeconds 5 `
                -ClaudeCommand $pwshPath -ClaudePrefixArgsJson $prefixJson *> $null
            $timeoutExit = $LASTEXITCODE
            $clock.Stop()
            if ($timeoutExit -ne 1) { throw "Timeout process smoke returned $timeoutExit, expected 1." }
            if ($clock.Elapsed.TotalSeconds -gt 12) { throw 'Timeout process smoke did not terminate promptly.' }
        } finally {
            Remove-Item Env:FAKE_CLAUDE_MODE -ErrorAction SilentlyContinue
            Remove-Item Env:APEX_REVIEW_TEST_MODE -ErrorAction SilentlyContinue
            Pop-Location
        }
    } finally {
        Pop-Location
    }
    Write-Host 'Claude diff review process smoke test passed.'
} finally {
    if (Test-Path -LiteralPath $testRoot) {
        Remove-Item -LiteralPath $testRoot -Recurse -Force
    }
}

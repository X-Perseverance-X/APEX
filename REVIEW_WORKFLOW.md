# Codex to Claude Diff Review Gate

META-1 development uses one implementation agent and one narrow reviewer:

```text
Owner → Codex → implement/test/debug → git diff → Claude → APPROVE | ISSUES
```

## Contract

1. Codex owns implementation, tests, hardware-safety checks, and the candidate diff.
2. Claude receives only the selected diff plus the compact review rubric.
3. Claude runs with low effort, no tools, no file writes, and no persisted session.
4. `APPROVE` means no actionable correctness, regression, safety, or test-coverage issue was found.
5. `ISSUES` must contain actionable findings in priority order, with file and line context when available.
6. Codex resolves valid findings, reruns tests, and requests a fresh review of the new diff.
7. Human approval remains mandatory immediately before physical servo, gait, route, or autonomy tests.

## Usage

From the repository root:

```powershell
# Review unstaged working-tree changes
.\tools\claude-diff-review.ps1

# Review the exact staged candidate
.\tools\claude-diff-review.ps1 -Scope staged

# Review the PR-style merge-base diff against origin/main
.\tools\claude-diff-review.ps1 -Scope branch -Base origin/main
```

The command exits with code `0` only for `APPROVE`, `2` for `ISSUES`, `3` for an empty diff, and `1` for an invalid or failed review. Empty diffs are rejected instead of spending a model call.
The Claude process is killed if it exceeds the configurable timeout (90 seconds by default).
`-SelfTest` validates the fail-closed stream parser without invoking Claude. A second smoke test uses a local fake executable to exercise process transport and timeout termination. GitHub Actions runs both whenever the gate changes.
The gate requires PowerShell 7+ and, in production, resolves only the trusted per-user `.local/bin/claude.exe`. `APEX_REVIEW_TEST_MODE` is created and removed inside the isolated smoke test and must not be set for normal reviews.

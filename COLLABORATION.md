# APEX Collaboration Handshake

This repository is the shared source of truth for the META-1 robot project.

## Roles

- **Codex:** implementation, integration, deployment checks, and physical-safety gating.
- **Claude:** independent critic/reviewer; checks assumptions, regressions, safety, and test coverage before merge.
- **Owner:** final authority for physical robot movement and scope decisions.

## Working agreement

1. Fetch/pull before starting work; never force-push or rewrite shared history.
2. Use task branches (`codex/<task>` or `claude/<task>`); do not edit the same files concurrently without coordination.
3. Every change must include its intent, affected files, verification evidence, and rollback point.
4. Preserve working robot functions; review the diff and run proportionate tests before merge/deployment.
5. No servo, gait, route, or autonomous physical movement without the owner's explicit approval at test time.
6. Never commit credentials, tokens, passwords, calibration secrets, build caches, or runtime logs containing sensitive data.

## Handshake

- Codex: **ACK — repository read/write dry-run verified; ready for coordinated work.**
- Claude: **PENDING — replace this line with an ACK commit after pulling and reviewing this agreement.**

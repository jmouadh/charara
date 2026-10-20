# Contributing to Charara

This repository holds the whole robot: STM32 firmware, the Raspberry Pi
planning/console stack, hardware sources and the design documentation. One
concern per change, documented and measured.

## Ground rules

1. **The robot must stay safe.** Anything that can move the robot needs a bench
   validation line in the pull request (what was measured, in which mode).
2. **MOCK first.** The console and the planner run without hardware; prove the
   behaviour there before touching the board.
3. **The documentation is part of the change.** If behaviour described in
   `Docs/` or `README.md` changes, update it in the same pull request.

## Branches and commits

Branch from `main`:

```
feat/42-streaming-pursuit      # feature, references the issue
fix/57-i2c-timeout
docs/roadmap-track-b
```

Commits follow [Conventional Commits](https://www.conventionalcommits.org/):

```
feat(planner): blend quarter arcs into the previous segment
fix(firmware): clamp odometry counter before the int32 wrap
docs(protocol): document the streaming command set
chore(ci): cache pip downloads
```

Scopes in use: `firmware`, `software`, `planner`, `vision`, `console`,
`deploy`, `docs`, `ci`.

## Pull requests

- Keep them small; the template asks for the test evidence and the
  hardware/safety impact.
- `main` is protected: CI must be green, the PR is squash-merged.
- Link the issue (`Closes #12`) so the project board updates itself.

## Tests

```bash
python -m pytest -q                # whole suite, no hardware needed
python -m pytest tests/test_api.py -q
```

The suite covers the planner, the protocol codec, the mission engine and the
console API against the MOCK board.

## Project board

Work is tracked on the [Charara project](https://github.com/users/jmouadh/projects/2):
`Area`, `Type`, `Priority`, `Size` and `Iteration` are set on every issue. The
roadmap in [`Docs/10-roadmap.md`](Docs/10-roadmap.md) is the source of truth for
the milestones.

## Security

Never commit credentials. Wi-Fi and VPN credentials live in
`/etc/charara/network.env` on the robot and in `docker/.env` locally; both are
git-ignored, only the `*.example` templates are tracked.

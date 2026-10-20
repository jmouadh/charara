## What changed

<!-- One or two sentences. Link the issue: "Closes #12". -->

## How it was tested

<!-- MOCK console, pytest, bench test, measured run… Delete what does not apply. -->

- [ ] `python -m pytest` passes
- [ ] Behaviour verified in MOCK
- [ ] Behaviour verified on the real board (state what was measured)

## Hardware / safety impact

<!-- Does it change motion limits, I2C protocol, calibration registers or the
     deadman? What should be re-validated on the bench? Write "none" if not. -->

## Documentation

- [ ] `README.md` / `Docs/` updated where behaviour is described
- [ ] Protocol or register changes mirrored in `software/robot/protocol.py` and the firmware headers

## Checklist

- [ ] One concern per PR; no unrelated reformatting
- [ ] `CHANGELOG.md` updated for user-visible changes
- [ ] No secrets, credentials or machine-specific paths added

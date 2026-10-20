# Security policy

## Supported versions

The `main` branch is the supported line. The console and the firmware are
versioned together: the SW/FW pills in the console header show what is running.

## Reporting a vulnerability

Please **do not open a public issue** for a security problem.

Use GitHub's private reporting: **Security → Advisories → Report a
vulnerability** on https://github.com/jmouadh/charara. If that is unavailable,
contact the maintainer directly (see the commit author address).

Include what you can: the affected component (firmware, console service, UI),
the version pills, and the steps to reproduce. A reply should arrive within a
week.

## Scope notes for this project

- The robot is not internet-facing by design: the console listens on the
  hotspot/LAN, and the optional Tailscale sidecar is the only remote path.
  Reports about the console being reachable from the tailnet are expected
  behaviour, not a vulnerability.
- **Never commit credentials.** Wi-Fi and Tailscale credentials live in
  `/etc/charara/network.env` on the robot and in `docker/.env` locally; both are
  git-ignored, and only the `*.example` templates are tracked. If you find a
  credential in the repository or its history, treat it as leaked, revoke it at
  the provider, and report it privately.
- Firmware safety limits (watchdog, deadman, current limits) are documented in
  `Docs/STM_docs/04-motion-odometry-and-safety.md`. A report showing a way to
  exceed them is in scope.

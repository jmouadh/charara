# Host provisioning (Raspberry Pi OS level)

OS-level files that `deploy.ps1` installs on the robot (the repository is the
source of truth, so a re-flashed Pi comes back with the same network
behaviour). Nothing here is part of the docker stack.

| File | Installed to | Role |
|---|---|---|
| `configure_wifi.sh` | `/usr/local/bin/configure_wifi.sh` (root, 0755) | Network bootstrap: connect to the home Wi-Fi when available, otherwise raise the CHARARA hotspot |
| `configure_hotspot.service` | `/etc/systemd/system/` | Runs the bootstrap once at boot |
| `configure_wifi-watch.service` | `/etc/systemd/system/` | Same script in `--reeval` mode |
| `configure_wifi-watch.timer` | `/etc/systemd/system/` | Runs the watch every 2 min (migrates hotspot -> Wi-Fi when it appears) |

## How the choice is made

* The **Wi-Fi profile** (the Imager/netplan profile for the SSID, or one the
  script creates) has autoconnect priority `0`.
* The **hotspot profile** (`CHARARA`, 192.168.0.30/24) has autoconnect
  priority `-5`, so NetworkManager prefers the Wi-Fi whenever it is in range;
  the hotspot exists only as the fallback.
* At boot the script waits for `wlan0`, tries the Wi-Fi for up to 60 s
  (verifying that an address was obtained, ignoring APIPA), and only then
  brings the hotspot up. The timer re-checks every 2 minutes: if the robot is
  on the hotspot and the Wi-Fi is now in range, it switches; if the Wi-Fi is
  not in range it leaves the hotspot alone (no scanning noise).

Run it by hand to migrate immediately:

```bash
sudo /usr/local/bin/configure_wifi.sh          # normal (Wi-Fi first)
sudo /usr/local/bin/configure_wifi.sh --dry-run  # log decisions, change nothing
sudo /usr/local/bin/configure_wifi.sh --reeval   # what the timer runs
journalctl -t configure_wifi -n 50 --no-pager    # log (also in the journal)
```

Credentials are **not** in the script. `configure_wifi.sh` sources
`/etc/charara/network.env` (override with `CHARARA_NET_ENV`) and requires `SSID`
and `PASSWORD` to be set there or exported in the environment; the hotspot name
and password stay in the script because they are the public fallback. Create the
file once, root-owned and unreadable by others:

```bash
sudo install -d -m 0750 -o root -g root /etc/charara
sudo install -m 0600 -o root -g root network.env.example /etc/charara/network.env
sudoedit /etc/charara/network.env     # fill in SSID / PASSWORD (and TS_AUTHKEY)
```

`network.env` is git-ignored; only `network.env.example` is committed.

# Docker - everything runs from here

One folder for all container files: the compose files, the three Dockerfiles
and the environment template. All images build from the REPOSITORY ROOT
(compose sets ``context: ..``) - every COPY path is relative to that root,
and the sources land in the image at ``/srv`` next to ``pyproject.toml``, so
pip installs the ``charara`` package exactly as tested.

## Services

| Service | What it is | Started by |
|---|---|---|
| `api` | console service (REST/WS + plan/calibrate/test), port 8000 | `docker compose up -d --build` |
| `ui` | nginx web console (static app), port 8080 | included above |
| `vpn` | Tailscale node: remote access to the console from any of your devices | included above (set `TS_AUTHKEY`) |
| `brain` | the RPi's main job: plays a saved strategy, no web layer | `docker compose --profile brain up -d` |

Both `api` (8000) and `ui` (8080) publish on **every host interface**, LAN
and Tailscale alike; the console page talks to `/api` and `/ws` on the same
origin it was opened from, so it behaves identically on the hotspot and over
the VPN.

Before using the brain set the strategy it should play (docker/.env):

```
STRATEGY=main          # a name in the shared strategies volume, or a path
```

> Both `api` and `brain` open the I2C buses: ``/dev/i2c-1`` (the STM32 board,
> ``I2C_BUS=1``) and ``/dev/i2c-6`` (the MPU6050 IMU, ``IMU_BUS=6``). Both must
> exist on the host: ``deploy.ps1`` writes the ``config.txt`` lines
> (``dtparam=i2c_arm=on`` + ``dtoverlay=i2c6,pins_22_23``) and reboots the Pi
> once when they are missing. Run the brain while the console is NOT driving
> the robot (stop the api service first).

## Remote access over Tailscale

The stack ships a Tailscale sidecar (the `vpn` service), so the Pi is a node
of your tailnet and the whole console is available remotely. Put the auth
key in `docker/.env` (copy from `.env.example`):

```
TS_AUTHKEY=tskey-auth-xxxxxxxxxxxxxxxx
TS_HOSTNAME=charara
```

```bash
cd docker
docker compose up -d --build      # console + ui + vpn
```

From any device of yours on the tailnet:

| What | URL |
|---|---|
| Web console (nginx) | `http://<TS_HOSTNAME>:8080` (MagicDNS, e.g. `http://charara:8080`) |
| REST + WebSocket API | `http://<TS_HOSTNAME>:8000` (or `:8080/api` through nginx) |

Everything works remotely: draw/plan/execute, calibration and settings,
board errors, the I2C monitor, the camera/vision WebSocket - nginx serves the
same page and proxies to the `api` container, exactly like on the LAN.

Notes:

* Generate the key at <https://login.tailscale.com/admin/settings/keys>.
  The `vpn` container runs in the host network namespace (kernel mode), so
  the published ports `8080`/`8000` are exactly what the tailnet sees. The
  node identity lives in `/var/lib/tailscale` and survives rebuilds
  (`TS_AUTH_ONCE`); it requires `/dev/net/tun` on the host.
* Whoever is on the tailnet can drive the robot - restrict the tailnet with
  ACLs if it is shared.
* Without a `TS_AUTHKEY` the `vpn` container starts but stays offline; the
  console still works on the LAN/hotspot.

## Development / demo (no robot)

```
cd docker
docker compose -f compose.dev.yml up --build
open http://localhost:8080
```

The dev stack runs the console in ``ROBOT_TRANSPORT=mock`` (no `/dev/i2c`,
no Tailscale).

## Environment

Copy ``.env.example`` to ``.env`` in this folder (compose reads it
automatically) to override any port, the field size, the transport, the
strategy name or the Tailscale settings. The optional sensor-fusion gates
(`CAMERA_POSES`, `IMU`, `GYRO_STREAM`, `STREAMING_EXEC`, `REANCHOR`, `IMU_BUS`=6, ...) all
default to ON and degrade to wheel-only when the hardware is absent (a
missing IMU bus or device is a normal operating state, never a startup
failure) - see the ``.env.example`` block and Docs/09. The calibrated gyro
values the console prints for ``docker/.env`` (``GYRO_SCALE`` from the
hand-spin test, ``GYRO_SIGMA_DEG_S`` from the rest-noise measurement) are
passed into the ``api`` and ``brain`` containers too; after editing them run
``docker compose up -d`` once so the containers are recreated with the new
values.

## Direct (no docker)

```
# console, from the repository root
ROBOT_TRANSPORT=mock python -m uvicorn software.console.main:app --port 8090

# brain
python -m software.brain --strategy main --transport mock
```


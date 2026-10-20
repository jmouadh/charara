#!/bin/bash

# ==============================================================================
# Configuration Parameters
# ==============================================================================
# The auth key comes from the environment and is never stored in this file:
#   export TS_AUTHKEY=tskey-auth-...      (or: set -a; . /etc/charara/network.env; set +a)
TS_AUTHKEY="${TS_AUTHKEY:?TS_AUTHKEY is not set: export it before running this script}"
TS_HOSTNAME="CHARARA"                             # Device name in Tailscale dashboard
TS_EXTRA_ARGS="--advertise-tags=tag:charara"      # Tags assigned to the device
CONTAINER_NAME="vpn"                              # Docker container name
STATE_DIR="${HOME}/.tailscale-state"              # Directory to persist Tailscale state

# ==============================================================================
# Execution
# ==============================================================================

# Ensure state directory exists for persistent identity across restarts
mkdir -p "$STATE_DIR"

# Stop and remove existing container with the same name if present
docker rm -f "$CONTAINER_NAME" 2>/dev/null || true

# Run the Tailscale VPN container
docker run -d \
  --name="$CONTAINER_NAME" \
  --restart=always \
  --network=host \
  --cap-add=NET_ADMIN \
  --cap-add=NET_RAW \
  --device=/dev/net/tun:/dev/net/tun \
  -v "$STATE_DIR":/var/lib/tailscale \
  -e TS_AUTHKEY="$TS_AUTHKEY" \
  -e TS_HOSTNAME="$TS_HOSTNAME" \
  -e TS_EXTRA_ARGS="$TS_EXTRA_ARGS" \
  -e TS_STATE_DIR=/var/lib/tailscale \
  tailscale/tailscale:latest

# Verify container status
if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null)" = "true" ]; then
    echo "Tailscale container '$CONTAINER_NAME' started successfully."
else
    echo "Failed to start Tailscale container '$CONTAINER_NAME'."
    exit 1
fi
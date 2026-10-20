#!/bin/bash
# Charara network bootstrap (Raspberry Pi 4 / NetworkManager).
#
# Purpose: connect to the home Wi-Fi when it is available, otherwise bring up
# the CHARARA hotspot - the robot must always be reachable (LAN or hotspot).
#
# Behaviour:
#   * already on the Wi-Fi with a usable address -> does nothing;
#   * Wi-Fi reachable -> connects (bounded retries) before giving up;
#   * otherwise -> makes sure the hotspot is up. The hotspot never wins over a
#     working Wi-Fi: its autoconnect priority is negative, so NetworkManager
#     prefers the Wi-Fi profile whenever it is in range.
#   * idempotent: safe at boot (configure_hotspot.service), from the periodic
#     watch timer (configure_wifi-watch.timer -> --reeval) and by hand
#     (`sudo configure_wifi.sh`, e.g. to migrate from the hotspot to the Wi-Fi).
#
# Usage: configure_wifi.sh [--reeval] [--dry-run]
#   --reeval    periodic mode: keeps the hotspot when the Wi-Fi is NOT in
#               range (short attempt; no repeated scanning/association);
#   --dry-run   only logs what would be done (never touches the network).
#
# NOTE: the Wi-Fi credentials are NOT in this file. They are read from
# ${CHARARA_NET_ENV:-/etc/charara/network.env} (root-owned, 0600) or from the
# environment - see network.env.example. Install this script root-owned (0755).
set -u

# ---------------------------------------------------------------- credentials
NET_ENV="${CHARARA_NET_ENV:-/etc/charara/network.env}"
if [ -r "$NET_ENV" ]; then
    set -a
    . "$NET_ENV"
    set +a
fi
SSID="${SSID:-}"
PASSWORD="${PASSWORD:-}"
HAVE_WIFI_CREDS=0
if [ -n "$SSID" ] && [ -n "$PASSWORD" ]; then
    HAVE_WIFI_CREDS=1
fi

CON_NAME="CHARARA"
CON_PWD="charara01"
HOTSPOT_IP="192.168.0.30"
IF_NAME="wlan0"

WIFI_WAIT_S=75          # boot: total budget for the Wi-Fi attempt
REEVAL_WAIT_S=25        # timer: single short attempt
ATTEMPT_S=20            # per-attempt nmcli activation timeout
HOTSPOT_PRIORITY=-5     # below the Wi-Fi profile (NetworkManager default 0)

REEVAL=0
DRY_RUN=0
for arg in "$@"; do
    case "$arg" in
        --reeval)  REEVAL=1 ;;
        --dry-run) DRY_RUN=1 ;;
    esac
done

log() { echo "[configure_wifi] $*"; logger -t configure_wifi "$*" 2>/dev/null || true; }

# nmcli wrapper honouring --dry-run (everything that changes state goes
# through it); plain `nmcli` reads are used for the state queries below.
mn() {
    if [ "$DRY_RUN" = "1" ]; then
        log "DRY-RUN: nmcli $*" >&2
        return 0
    fi
    nmcli "$@"
}

# ---------------------------------------------------------------- helpers
device_state() { nmcli -t -f DEVICE,STATE device status 2>/dev/null | awk -F: -v d="$IF_NAME" '$1==d {print $2; exit}'; }
active_conn()  { nmcli -t -f GENERAL.CONNECTION device show "$IF_NAME" 2>/dev/null | head -n1 | cut -d: -f2-; }
wlan_ip()      { nmcli -t -f IP4.ADDRESS device show "$IF_NAME" 2>/dev/null | head -n1 | cut -d: -f2 | cut -d/ -f1; }

have_ip() {   # a usable address (no placeholder, no APIPA 169.254.x.x)
    local ip
    ip="$(wlan_ip)"
    [ -n "$ip" ] && [ "$ip" != "--" ] || return 1
    case "$ip" in 169.254.*) return 1 ;; esac
    return 0
}

scan_ssids() {   # SSIDs in the scan list (may trigger a scan; empty = no results)
    nmcli -t -f SSID device wifi list ifname "$IF_NAME" 2>/dev/null | tr -d '\\'
}

ssid_visible() {   # is the home SSID in the scan list?
    scan_ssids | grep -Fxq "$SSID"
}

wait_for_device() {   # NetworkManager needs a moment for wlan0 after boot
    local i
    for i in $(seq 1 90); do
        case "$(device_state)" in
            connected|connecting|disconnected) return 0 ;;
        esac
        sleep 1
    done
    return 1
}

profile_for_ssid() {   # name of an existing client profile for $SSID, if any
    local name ssid
    while IFS= read -r name; do
        [ -n "$name" ] || continue
        ssid="$(nmcli -t -f 802-11-wireless.ssid connection show "$name" 2>/dev/null | head -n1 | cut -d: -f2-)"
        if [ "$ssid" = "$SSID" ]; then
            printf '%s\n' "$name"
            return 0
        fi
    done < <(nmcli -t -f NAME,TYPE connection show 2>/dev/null | awk -F: '$2=="802-11-wireless" {print $1}')
    return 1
}

ensure_wifi_profile() {   # reuse the profile for the SSID (netplan, ...) or create one
    local prof cur_psk
    if prof="$(profile_for_ssid)"; then
        mn connection modify "$prof" \
            connection.autoconnect yes \
            connection.autoconnect-priority 0 >/dev/null
        # Only touch the stored password when it is not already the right one:
        # blindly rewriting it would clobber a working key (e.g. one typed by
        # hand) with a typo from this file.
        cur_psk="$(nmcli -s -g 802-11-wireless-security.psk connection show "$prof" 2>/dev/null)"
        if [ "$cur_psk" != "$PASSWORD" ]; then
            log "updating the stored Wi-Fi password of '$prof'"
            mn connection modify "$prof" wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PASSWORD" >/dev/null
        fi
    else
        prof="$SSID"
        mn connection add type wifi ifname "$IF_NAME" con-name "$prof" ssid "$SSID" \
            connection.autoconnect yes \
            connection.autoconnect-priority 0 \
            wifi-sec.key-mgmt wpa-psk \
            wifi-sec.psk "$PASSWORD" >/dev/null
    fi
    printf '%s\n' "$prof"
}

ensure_hotspot_profile() {
    if nmcli -t -f NAME connection show 2>/dev/null | grep -Fxq "$CON_NAME"; then
        # always ready as a fallback, never preferred over the Wi-Fi
        mn connection modify "$CON_NAME" \
            connection.autoconnect yes \
            connection.autoconnect-priority "$HOTSPOT_PRIORITY" >/dev/null
    else
        mn connection add type wifi ifname "$IF_NAME" con-name "$CON_NAME" autoconnect yes ssid "$CON_NAME" >/dev/null
        mn connection modify "$CON_NAME" \
            802-11-wireless.mode ap \
            802-11-wireless.band bg \
            ipv4.method shared \
            ipv4.addresses "$HOTSPOT_IP/24" \
            connection.autoconnect-priority "$HOTSPOT_PRIORITY" \
            wifi-sec.key-mgmt wpa-psk \
            wifi-sec.psk "$CON_PWD" >/dev/null
    fi
}

try_wifi() {   # $1 = profile, $2 = total budget in seconds
    local prof="$1" budget="$2" start now
    start="$(date +%s)"
    while :; do
        now="$(date +%s)"
        [ $((now - start)) -ge "$budget" ] && break
        if [ "$(active_conn)" != "$prof" ]; then
            act_out="$(mn --wait "$ATTEMPT_S" connection up "$prof" 2>&1)"
            if [ $? -ne 0 ] && [ -n "$act_out" ]; then
                log "activation did not complete: $act_out"
            else
                log "connecting to '$SSID' via '$prof'..."
            fi
        fi
        sleep 3
        if [ "$(active_conn)" = "$prof" ] && have_ip; then
            log "connected to '$SSID' - $IF_NAME $(wlan_ip)"
            return 0
        fi
        sleep 3
    done
    return 1
}

# ------------------------------------------------------------------- main
mn radio wifi on >/dev/null 2>&1

if ! wait_for_device; then
    # Best effort: still try the Wi-Fi - a slow radio right after boot must not
    # leave the robot on the hotspot for the rest of the session.
    log "wlan0 still '$(device_state)' after 90 s - trying the Wi-Fi anyway"
fi

log "device $(device_state), active '$(active_conn)', ip '$(wlan_ip)'"
if [ "$HAVE_WIFI_CREDS" = "1" ]; then
    wifi_prof="$(ensure_wifi_profile)"
else
    wifi_prof=""
    log "no Wi-Fi credentials in $NET_ENV - hotspot only"
fi
ensure_hotspot_profile

if [ "$HAVE_WIFI_CREDS" = "1" ] && [ "$(active_conn)" = "$wifi_prof" ] && have_ip; then
    log "already on '$SSID' ($(wlan_ip)) - nothing to do"
    exit 0
fi

# Periodic mode: on the hotspot, keep it ONLY when a successful scan clearly
# shows the Wi-Fi is absent. An empty scan list (scanning while the radio is in
# AP mode is unreliable) is not a reason to skip the attempt - the robot must
# not stay stuck on the hotspot while the Wi-Fi is reachable.
if [ "$HAVE_WIFI_CREDS" = "1" ] && [ "$REEVAL" = "1" ] && [ "$(active_conn)" = "$CON_NAME" ]; then
    scan_list="$(scan_ssids)"
    if [ -n "$scan_list" ] && ! printf '%s\n' "$scan_list" | grep -Fxq "$SSID"; then
        log "on the hotspot and '$SSID' is not in the scan list - keeping the hotspot"
        exit 0
    fi
fi

budget="$WIFI_WAIT_S"
[ "$REEVAL" = "1" ] && budget="$REEVAL_WAIT_S"

# Wi-Fi first (bounded), hotspot only if it does not come up.
if [ "$HAVE_WIFI_CREDS" = "1" ] && try_wifi "$wifi_prof" "$budget"; then
    exit 0
fi

if [ "$(active_conn)" != "$CON_NAME" ]; then
    if [ "$HAVE_WIFI_CREDS" = "1" ]; then
        log "'$SSID' unavailable - starting the hotspot '$CON_NAME'"
    else
        log "no Wi-Fi credentials configured - starting the hotspot '$CON_NAME'"
    fi
    if ! mn --wait 30 connection up "$CON_NAME" >/dev/null 2>&1; then
        log "ERROR: could not start the hotspot"
        exit 1
    fi
fi
log "hotspot '$CON_NAME' active - console on http://$HOTSPOT_IP:8080"
exit 0

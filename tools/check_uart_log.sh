#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 <uart_log_file>"
    exit 1
fi

log_file="$1"
if [[ ! -f "$log_file" ]]; then
    echo "Log file not found: $log_file"
    exit 1
fi

echo "[INFO] Checking UART log: $log_file"

declare -A checks=(
    ["APP started"]="BLE->Dual PS/2 router started"
    ["PS2 initialized"]="Initialized 4 PS/2 ports on shared timer"
    ["HID scan started"]="SCAN..."
    ["Keyboard connected"]="on_kbd_conn|Known KBD|OPEN try KEYBOARD"
    ["Mouse connected"]="on_mouse_conn|Known MOUSE|OPEN try MOUSE"
    ["HID input seen"]=" INPUT: "
    ["No PS/2 send errors"]="PS/2 keyboard .* failed|PS/2 mouse send failed"
    ["Passkey shown"]="PASSKEY_NOTIF|NC_REQ"
    ["Passkey hidden"]="AUTH SUCCESS|AUTH ERROR|CLOSE"
    ["Switch to PC1"]="Switched active PC to 1"
    ["Switch to PC2"]="Switched active PC to 2"
)

for name in "${!checks[@]}"; do
    pattern="${checks[$name]}"
    if [[ "$name" == "No PS/2 send errors" ]]; then
        if rg -q -e "$pattern" "$log_file"; then
            echo "[WARN] $name check failed (found: $pattern)"
        else
            echo "[PASS] $name"
        fi
    else
        if rg -q -e "$pattern" "$log_file"; then
            echo "[PASS] $name"
        else
            echo "[WARN] $name not found (pattern: $pattern)"
        fi
    fi
done

echo "[INFO] Done"

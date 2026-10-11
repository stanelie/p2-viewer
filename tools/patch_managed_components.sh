#!/usr/bin/env bash
# Idempotent patches to fetched components. managed_components/ is gitignored and is recreated by
# the component manager, so anything we must change there lives here instead of in a diff.
#
# Run after a fresh clone or whenever the component manager re-downloads dependencies:
#     ./tools/patch_managed_components.sh
set -euo pipefail
cd "$(dirname "$0")/.."

F=managed_components/espressif__esp_video/src/device/esp_video_isp_device.c

# esp_video <= 0.9.1: isp_start_awb() and isp_reconfigure_awb() declare
#     esp_isp_awb_config_t awb_config;
# uninitialised, and isp_init_awb_param() fills only .window - never .subwindow. The ISP driver
# validates the subwindow against the window ONLY on chip rev >= 3.0 (esp_driver_isp/src/isp_awb.c,
# "subwindow exceeds window range"), so on an ESP32-P4 rev v3.2 like ours the check runs against
# stack garbage and AWB fails to start, which takes STREAMON down with it. On an older P4 it only
# warns, which is presumably why upstream has not noticed.
if [ -f "$F" ]; then
    if grep -q 'esp_isp_awb_config_t awb_config;' "$F"; then
        sed -i 's/esp_isp_awb_config_t awb_config;/esp_isp_awb_config_t awb_config = {0};  \/* p2-viewer: subwindow must be zeroed, see tools\/patch_managed_components.sh *\//' "$F"
        echo "patched: esp_video AWB subwindow init ($F)"
    else
        echo "already patched (or upstream fixed): $F"
    fi
else
    echo "skipped, not fetched yet: $F"
fi

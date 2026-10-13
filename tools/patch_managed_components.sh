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

G=managed_components/espressif__usb_host_uvc/uvc_host.c

# usb_host_uvc 2.6.0 allocates the endpoint-0 transfer at 64 bytes ("Worst case HS MPS") and then
# validates every uvc_host_usb_ctrl() against it as
#     data_buffer_size >= wLength + sizeof(usb_setup_packet_t)
# so the largest vendor control payload it can carry is 64 - 8 = 56 bytes. That is fine for the
# InfiRay P2 Pro, whose commands are tiny, and too small for the HIKMICRO extension unit on the
# GOYOJO GW192A: every one of its ten control selectors reports a length of 512, and a GET_CUR at
# that size fails in the component before a packet is ever sent. 576 covers 512 plus the setup
# packet with room to spare, and costs one allocation of internal DMA memory per stream.
if [ -f "$G" ]; then
    if grep -q 'usb_host_transfer_alloc(64, 0, &ctrl_xfer)' "$G"; then
        sed -i 's/usb_host_transfer_alloc(64, 0, &ctrl_xfer)/usb_host_transfer_alloc(576, 0, \&ctrl_xfer)/' "$G"
        echo "patched: uvc_host EP0 transfer 64 -> 576 bytes ($G)"
    else
        echo "already patched (or upstream changed): $G"
    fi
else
    echo "skipped, not fetched yet: $G"
fi

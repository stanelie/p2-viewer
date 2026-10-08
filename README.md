# p2-viewer

Handheld thermal imager firmware: an **InfiRay P2 Pro** USB-C thermal camera driven over UVC by a
**Waveshare ESP32-P4-WIFI6-Touch-LCD-3.5**, with a touch OSD, palette mapping and live temperature
readouts. No LVGL — the display is driven straight through `esp_lcd`.

Priorities, in order: **boot time, framerate, latency.**

## Status

Streams 256x384 YUY2 at a stable **25 fps** (the sensor's ceiling), **~36.9 ms** arrival-to-on-screen,
**~2.16 s** from power-on to first image and **~6.7 s** to a fully calibrated one (down from ~11.8 s).

## Build

Needs **ESP-IDF v5.5+**. The board's ESP32-P4 is silicon revision **v3.2**, which IDF v5.3.x
refuses to flash ("requires chip revision in range [v0.1 - v1.99]").

```bash
idf.py set-target esp32p4
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

Flash via the **UART** Type-C port so the OTG port stays free for the camera.

## How the camera works

The P2 Pro is a plain UVC device — no vendor driver needed to stream. It advertises YUY2 256x192
and 256x384; the 384-row format is **preview on top, raw radiometric in the bottom half**. Raw
counts are **Kelvin x 64**:

```
celsius = raw / 64.0 - 273.15
```

### Cold-start behaviour (measured)

Plugging in a cold camera goes through three phases, timed from stream open:

| phase | window | signature |
|---|---|---|
| flat sentinel | 0 – 2.6 s | every radiometric pixel is `0x8000` |
| uncorrected | 2.9 – 8.75 s | real data, ~53x spatially noisier, ~54 °C spread |
| clean | 8.75 s on | flat-field applied, ~11 °C spread |

Phase 2 is uncorrected bolometer output (per-pixel fixed-pattern noise, no flat-field yet). It is
full-length and inside a plausible temperature band, so neither a frame-length check nor the
`0x8000` check catches it — it just looks garbled. The firmware shows **CALIBRATING** during phase 1.

A useful metric for telling these apart is the mean absolute difference between horizontally
adjacent raw pixels: ~0 flat, **~320 uncorrected, ~6 clean**.

### Cutting the cold start (the auto-shutter delays)

The uncorrected phase is **not** a missing FFC command - it is configuration. The camera schedules
its own shutter events, and `get/set_prop_auto_shutter_params` (`0x8214` / `0xc214`) exposes the
schedule. Factory defaults as read from the camera:

| id | name | default |
|---|---|---|
| 0 | `PROP_SWITCH` | 1 (auto-shutter on) |
| 1 | `MIN_INTERVAL` | 5 |
| 2 | `MAX_INTERVAL` | 60 |
| **9** | **`PREVIEW_START_1ST_DELAY`** | **5** |
| **10** | **`PREVIEW_START_2ND_DELAY`** | **4** |
| 11 | `CHANGE_GAIN_1ST_DELAY` | 5 |
| 12 | `CHANGE_GAIN_2ND_DELAY` | 4 |

Units are seconds, and 5 + 4 = 9 matches the measured 8.75 s exactly. The firmware writes both to 1
as soon as the command channel opens (~2.7 s), which brings a usable image forward from
**8753 ms to 4535 ms**. The two early shutter clicks the official phone app produces are these same
two events with shorter delays configured - there is no manual-FFC opcode, which is why other
projects looking for one did not find it.

**These values do not survive the camera losing power** - a cold camera reads back 5/4 - so the
firmware reapplies them on every connect. Nothing is permanently written to the camera.

The remaining ~4.5 s is close to a floor: the command channel is not reachable until ~2.7 s (the
same camera-readiness gate that ends the flat phase at ~2.6 s), plus ~1.8 s for the shutter to
run and the NUC to be applied.

Opcodes and parameter ids were extracted from `libircmd.so` in the official Android APK:
`readelf --dyn-syms` lists ~155 JNI exports whose names are the SDK API, disassembly gives the
opcode and packing, and `baksmali` on `classes.dex` gives the enum semantics
(`CommonParams$PropAutoShutterParameter`).

### Vendor command channel

Beyond UVC, the camera carries an InfiRay command interface behind two vendor control requests.
Transport reverse-engineered by [LeoDJ/P2Pro-Viewer](https://github.com/LeoDJ/P2Pro-Viewer); a copy
of the reference implementation is in [`docs/P2Pro_cmd_reference.py`](docs/P2Pro_cmd_reference.py).

- write `bmRequestType 0x41, bRequest 0x45, wValue 0x78`; read `0xC1, 0x44, 0x78`
- **ready check** at `wIndex 0x200` — must be polled to idle after *every* command
- 8-byte header at `wIndex 0x1d00`: cmd LE(2) + param LE(4) + length **big-endian**(2)

Reads work and are safe (`get_device_info`, `cur_vtemp`, `prop_tpd_params`). The channel opens
~2.93 s after stream start on a cold camera, and instantly on a warm one.

> **Do not send start/stop mode commands speculatively.** `y16_preview_start` (`0x010a`) and
> `preview_start` (`0xc10f`) are state-changing. Sending them left the camera emitting raw clustered
> just above `0x8000` at 27.5 fps — a compressed bottom half and an orange top half — and the mode
> **survived a replug**. `y16_preview_stop` does not undo it (returns a status error); only
> physically unplugging the camera recovered it.
>
> Never send `spi_transfer` (`0x8201`, writes camera flash) or `sys_reset_to_rom` (`0x0805`).

Note `preview_start 0xc10f` already contains the `0x4000` SET bit — don't blindly OR it. Invalid
commands do **not** USB-stall; they return a camera status error in the ready byte.

## Known limitations

- **Screen tearing is not fixable on this board.** Confirmed from the schematic: there is no TE net
  (so no vsync sync) and only MOSI (so no quad-mode speedup). 258 KB cannot be pushed below the
  panel's ~16.7 ms refresh, so scan-out always crosses the write. Host-side double buffering does
  not help — the tear forms in the panel's own GRAM. The tear-free path would be the board's
  unpopulated MIPI-DSI pads.
- **The camera cannot be powered from the internal battery unmodified.** Both Type-C VBUS pins share
  one net feeding the AXP2101's charger *input*; the PMIC is an NVDC charger with no boost at all.
  Powering the camera on battery needs an added 3.7→5 V boost module.
- The image is ~8% vertically stretched at 416x312 (PPA scale factors quantize to 1/16 steps).

## Layout

```
main/p2pro_uvc.c    all firmware
sdkconfig.defaults  config that matters (CONFIG_SPIRAM_MEMTEST=n saves ~890ms of boot)
docs/               P2 Pro protocol reference
```

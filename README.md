# p2-viewer

Handheld thermal imager firmware: an **InfiRay P2 Pro** USB-C thermal camera driven over UVC by a
**Waveshare ESP32-P4-WIFI6-Touch-LCD-3.5**, with a touch OSD, palette mapping and live temperature
readouts. No LVGL — the display is driven straight through `esp_lcd`.

Priorities, in order: **boot time, framerate, latency.**

## Status

Streams 256x384 YUY2 at a stable **25 fps** (the sensor's ceiling), **~36.9 ms** arrival-to-on-screen,
**~1.71 s** from power-on to first image and **~6.4 s** to a fully calibrated one (down from ~11.8 s).
Time-to-calibrated is reported on the console every stream open.

## Flashing a prebuilt binary

The [latest release](https://github.com/stanelie/p2-viewer/releases/latest) carries a single merged
image. Flash it at `0x0` via the **UART** Type-C port, so the OTG port stays free for the camera:

```bash
esptool.py --chip esp32p4 -b 460800 write_flash 0x0 p2-viewer-v1.0.0-esp32p4.bin
```

That writes the bootloader, partition table and app, and leaves a stored flat-field table intact.
For a clean device, `esptool.py --chip esp32p4 erase_flash` first.

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
`0x8000` check catches it — it just looks garbled.

The firmware labels these honestly, which took a correction: it originally showed **CALIBRATING**
through phase 1, when the camera is merely booting and the shutter has not fired, and then let the
genuinely-uncalibrated phase 2 show through as a garbled picture. Now phase 1 shows **STARTING**
and phase 2 shows **CALIBRATING**, which is the phase the flat-field actually lands at the end of.
The phase-2 test is bounded to the startup window and stops once a clean frame is seen, so a
high-contrast scene can never trigger a spurious overlay later.

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

Units are seconds, and 5 + 4 = 9 matches the measurement exactly. The firmware writes both to 1 as
soon as the command channel opens, which brings a usable image forward from **~9.35 s to ~5.4 s
after device connect** (9349/9366 ms vs 5350/5425 ms over repeated cold plugs). The two early
shutter clicks the official phone app produces are these same two events with shorter delays
configured - there is no manual-FFC opcode, which is why other projects looking for one did not
find it.

Measure this from **device connect**, not from stream open: the channel-up time varies by ~900 ms
run to run, so the "after stream open" figure moves around while the total stays within ~75 ms.

The schedule must be rewritten **on every stream open**. The camera forgets it whenever it loses
power, so latching "already configured" after the first success silently restores the stock 5s+4s
on every subsequent replug.

Verified cold-plug timing with the schedule rewritten: **5216 / 5218 ms** from device connect to a
calibrated image, against **9349 / 9366 ms** stock.

Three things that do **not** work, all measured:
- **Configuring before preview starts.** The delays count from preview start, so writing earlier
  should move the first shutter - but the command channel does not answer until preview is
  running, so there is no window in which to do it.
- **Lowering `MIN_INTERVAL`.** The camera rejects the write with a status error and it reads back
  5 unchanged.
- **Forcing the shutter** with `shutter_manual_switch` (`0x420c`, 0=OPEN / 1=CLOSE) the moment the
  channel answers, instead of waiting for the auto logic. 5337/5281 ms vs 5350/5425 ms baseline:
  no gain. The ~1.5 s after the trigger is the flat-field operation itself, not scheduling
  latency, so there is nothing to bring forward — and it blanks the image for ~800 ms.

  The actuator does work (close gives a uniform field, spread 132; open restores it), so it is
  available if a manual FFC is ever wanted. **A closed shutter reads as a smooth, low-roughness
  frame** — roughness ~7, indistinguishable from a calibrated image on roughness alone — so any
  "is it calibrated yet" test must also require a realistic min/max spread. Without that, the
  detector fires mid-shutter and reports a calibration that has not happened.

**These values do not survive the camera losing power** - a cold camera reads back 5/4 - so the
firmware reapplies them on every connect. Nothing is permanently written to the camera.

### Where the remaining time goes

Measured on a true cold boot (board and camera both unpowered), timestamps are ms since power-on:

| segment | cost | whose |
|---|---|---|
| power-on -> camera enumerated | 1009 ms | ours |
| -> first image on screen | 700 ms | camera (SET_INTERFACE) |
| -> command channel answers | 2832 ms | **camera** |
| -> schedule written | 140 ms | ours |
| -> flat-field complete | 1520 ms | **camera** |
| **total** | **6209 ms** | |

The command channel answers **~4.5 s after the camera receives power** - 4549 ms from power-on here,
and ~4.2 s after plug-in across the replug tests, despite very different host timings in the two
cases. So it is gated by the camera's own boot, not by when the host starts preview.

**Further boot optimisation will not improve time-to-calibrated.** The first image is already on
screen at ~1.7 s, 2.8 s before the camera will talk to us at all. Cutting host boot further makes
the first frame appear sooner but leaves the usable image at ~6.2 s. About 4.35 s of that 6.2 s is
the camera's own boot plus its flat-field.

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

## Flat-field correction button

Aim at something thermally uniform and press FFC. It averages 32 frames (~1.3 s) and applies each
pixel's deviation from the frame mean.

**The table is persisted** to a dedicated 64 KB `ffc` partition (48 KB of `int8` offsets plus magic,
version, pixel count and CRC32) and restored at boot. It is rejected if the CRC fails or it came
from a different geometry, so a corrupt table cannot silently degrade the image, and `CLR` erases
it. The side-bar button reads `CLR` whenever a correction is active - including a restored one - so
it is never applied invisibly.

This is worth persisting because a wall-based correction fixes something the camera structurally
cannot. The shutter sits *behind* the lens, so the camera's own FFC is blind to anything in the
optical path - a speck on the window, vignetting - which therefore survives every shutter cycle. A
wall-based capture sees the whole path. That component is a physical property of the camera and is
identical on every power-up.

**The camera's own shutter is NOT used as the reference, although it was tried.** Mechanically it
is the obvious choice - `shutter_manual_switch` (`0x420c`) closes it and it presents a field with
only ~110-130 counts of spread. But the shutter has a thermal gradient of its own, so a table
captured from it encodes that gradient and corrects visibly worse than a plain uniform surface at a
distance. Reverted after testing on hardware.

**Cost, and why it is built the way it is.** Applying the offset is per-pixel work inside a 40 ms
frame budget that already has only ~3 ms spare:

| implementation | convert | e2e |
|---|---|---|
| separate corrected-copy pass (PSRAM) | 13.9 ms | 90-110 ms, runaway backlog |
| folded into the existing passes, `int16` | 12.0 ms | 39.4 ms, stable |
| folded, `int8` offsets (current) | 11.3 ms | 38.3 ms, stable |

Materialising a corrected copy costs read-raw + read-offset + write-copy + re-read-copy, all PSRAM.
Folding the offset into the loops that already read each pixel removes three of those four streams.
The table is `int8` because 1 count is 1/64 K, so +/-127 covers +/-2 C - ample for FPN residual -
and halving the table halves what the per-pixel loop reads.

**Do not move these buffers to internal SRAM.** It makes the apply pass cheaper but starves the USB
stack, which needs internal DMA memory for its ISOC transfers: the camera drops out with a
continuous `Frame buffer underflow`. Measured, not theorised.

**The 5x7 font only carries the characters the UI happens to need.** A missing glyph renders as a
hollow box (deliberately - it used to render blank, which made `CALIBRATING` silently appear as
`CALI RATING` and `FFC` as two squares). Adding a label means checking its characters exist.

## Auto-shutter toggle (AS1 / AS0)

The camera runs its own flat-field every `MAX_INTERVAL` (60 s), closing the shutter and briefly
freezing the image. The **AS1 / AS0** button suppresses that: `PROP_SWITCH` (auto-shutter param 0)
written to 0. Verified by read-back - unlike `MIN_INTERVAL`, which the camera refuses, this write
is accepted.

**It is only ever applied after the camera's first calibration completes.** That startup
calibration is itself an auto-shutter event (the `PREVIEW_START` delays above), so disabling it any
earlier means the camera never runs a flat-field and the image stays permanently uncorrected. The
clean-frame detector is what marks the right moment.

Defaults to suppressed, and is not persisted, so every boot lets the camera do its one startup
calibration and then goes quiet.

**The trade-off is real.** Bolometer fixed-pattern noise drifts as the sensor warms, which is
exactly why the camera re-corrects periodically. With AS0 the image will slowly degrade over
minutes, and temperature readings with it. The FFC button is the manual compensation, and AS1 puts
the camera back in charge.

## Power button

Holding the power button for **2 s** powers the device off. The AXP2101's own `PWROFF` long-press
(REG `0x27` bits 3:2) only offers 4/6/8/10 s and shipped set to 6 s, so 2 s is not reachable by
configuration. Instead the chip's long-press *interrupt* (bits 5:4, options 1/1.5/2/2.5 s) is set to
2 s and the firmware commands the shutdown itself via `0x10` bit 0.

`OFFLEVEL` is also dropped to its 4 s minimum. That is deliberately longer than the 2 s software
path, so it acts as a backstop: if the firmware wedges, holding the button still cuts power in
hardware.

Observed on hardware, status register `0x49`: **bit 1 = press, bit 2 = long press at IRQLEVEL,
bit 0 = release**. Those bits latch even with the interrupt masked in `0x41`, so polling is
sufficient and the interrupt-enable registers are left untouched. The task requires a *fresh press*
before honouring a long press - otherwise holding the button to switch the device **on** leaves a
long-press bit set and it would shut straight back down.

The PMIC shares the touch I2C bus (port 1, SDA 7, SCL 8). A scan of that bus finds `0x18`,
`0x34` (AXP2101), `0x36` and `0x38` (FT5x06 touch).

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
sdkconfig.defaults  config that matters (see Boot time below)
docs/               P2 Pro protocol reference
```

## Boot time

Measured breakdown to first image, 3049 ms originally, **1709 ms** now:

| phase | cost | notes |
|---|---|---|
| ROM + bootloader + PSRAM + system init | ~410 ms | `CONFIG_SPIRAM_MEMTEST=n` removed ~890 ms of this |
| USB host install | 40 ms | moved to the *front* of `app_main` |
| display init | 310 ms | now runs **inside** the enumeration wait, so it is free |
| camera USB enumeration | ~550 ms | the camera's own; overlapped with display init |
| `uvc_host_stream_start` | ~650 ms | the camera processing SET_INTERFACE; not reducible from the host |

The structural win was ordering: USB enumeration takes ~550 ms and proceeds on its own task, so
installing the host first and doing display init and buffer allocation inside that window hides
~310 ms. A semaphore gates the streaming task so it cannot touch buffers before they exist.

Things tried that did **not** help: `CONFIG_ESPTOOLPY_FLASHMODE_QIO` (the esp32p4 target forces
`dio` regardless), silencing the bootloader log (~3 ms once it was already at WARN), and
`CONFIG_ESP_CONSOLE_UART_BAUDRATE` (only settable with `ESP_CONSOLE_UART_CUSTOM`, otherwise pinned
at 115200).

Set `P2_DIAG 1` in `main/p2pro_uvc.c` to re-enable the investigation instrumentation (auto-shutter
dump, per-frame content diagnostic, UVC component DEBUG logs). The one-line
`camera calibrated: clean image N ms after stream open` report is always on.

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
esptool.py --chip esp32p4 -b 460800 write_flash 0x0 p2-viewer-v1.0.3-esp32p4.bin
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

Defaults to **ON** (the camera's own behaviour) and is not persisted, so every boot returns to the
camera keeping fixed-pattern noise corrected. Either state is applied only after that first
calibration.

**The trade-off is real.** Bolometer fixed-pattern noise drifts as the sensor warms, which is
exactly why the camera re-corrects periodically. With AS0 the image will slowly degrade over
minutes, and temperature readings with it. The FFC button is the manual compensation, and AS1 puts
the camera back in charge.

## Power button

Holding the power button for **1 s** powers the device off. The AXP2101's own `PWROFF` long-press
(REG `0x27` bits 3:2) only offers 4/6/8/10 s and shipped set to 6 s, so 2 s is not reachable by
configuration. Instead the chip's long-press *interrupt* (bits 5:4, options 1/1.5/2/2.5 s) is set to
1 s and the firmware commands the shutdown itself via `0x10` bit 0. The IRQLEVEL mapping was
confirmed from two measured points: `01` produced a 1.48 s gap between the press and long-press
interrupts, and `10` produced a verified 2 s power-off, so `00` is 1 s.

`OFFLEVEL` is also dropped to its 4 s minimum. That is deliberately longer than the 1 s software
path, so it acts as a backstop: if the firmware wedges, holding the button still cuts power in
hardware.

Observed on hardware, status register `0x49`: **bit 1 = press, bit 2 = long press at IRQLEVEL,
bit 0 = release**. Those bits latch even with the interrupt masked in `0x41`, so polling is
sufficient and the interrupt-enable registers are left untouched. The task requires a *fresh press*
before honouring a long press - otherwise holding the button to switch the device **on** leaves a
long-press bit set and it would shut straight back down.

The PMIC shares the touch I2C bus (port 1, SDA 7, SCL 8). A scan of that bus finds `0x18`,
`0x34` (AXP2101), `0x36` and `0x38` (FT5x06 touch).

## Other USB cameras (generic preview mode)

Any UVC camera that is not a P2 Pro gets a plain full-screen preview instead of the thermal
interface. Detection is on **capability, not VID:PID**: the P2's signature is that it advertises
**256x384** YUY2 - preview stacked on raw radiometric, which no ordinary webcam offers. Anything
else is treated as a generic camera, which also means a P2 left in image-only mode degrades to a
picture rather than a garbled thermal parse.

The generic path costs almost no CPU: one PPA pass converts YUY2 to RGB565 *and* scales
(`PPA_SRM_COLOR_MODE_YUV422_YUYV` is a supported input mode), and a second does the byte swap the
ST7796 needs. The swap cannot be folded into the first pass because PPA's `byte_swap` affects how
it *reads*.

Resolution is chosen as the largest YUY2 mode that fits the panel. The panel is only 480x320, so a
larger capture buys no visible detail and costs isochronous bandwidth. Both axes share one 1/16
scale factor so the aspect ratio is exact - picking each axis's own best fit would stretch the
picture, since PPA quantizes them independently.

Worked example, the camera tested here (Sunplus `1bcf:2085`, an auricular scope):

| | |
|---|---|
| advertises | MJPEG and YUY2, each at 640x480 / 640x360 / 320x240 / 1280x720 |
| speed | high speed (480 Mbps), largest isoc endpoint MPS 1020 -> **~8.16 MB/s** |
| chosen | **320x240 YUY2 @ 25 fps** = 3.84 MB/s |
| rejected | 640x360 needs 11.5 MB/s, 640x480 needs 15.4 MB/s, 1280x720 needs 46 MB/s |
| scaled to | 420x315 at (30,2), aspect 1.3333 = exactly 4:3 |

Check the **negotiated speed** before doing this arithmetic. At full speed the packet rate is
1000/s rather than 8000/s, giving ~1 MB/s, and no YUY2 mode would be viable at 25 fps - the design
would have to go through MJPEG and the P4's hardware JPEG decoder instead.

**Verified working** against an iMac webcam: `352x288 -> 374x306`, **5.3 ms** convert and
**29.0 ms** end-to-end, no PPA errors and no underflows. The conversion is cheaper than the
thermal path (5.3 ms vs 11.3 ms) because the PPA does the colour conversion in hardware.

Two things had to be fixed before any frame appeared, and both are easy to misdiagnose:

**1. `CONFIG_UVC_CHECK_PAYLOAD_HEADER_EOH=n`.** Some cameras do not set the End-of-Header bit in
the UVC payload header, and the component rejects every such payload (`uvc-frame: EOH bit not
set`), so no frame ever completes even though the stream opened and data is arriving. It is a
Kconfig option, so this is not a patch to `managed_components` and survives a component upgrade.
The P2 sets the bit correctly and is unaffected.

**2. `out.buffer_size` must be cache-line aligned, not just the buffer address.** PPA validates
the size too, and declaring the *used extent* fails whenever `w*h*2` is not a multiple of 64 -
374x306 gives 228888, which is 24 over, and every pass failed with `ESP_ERR_INVALID_ARG` giving a
**black screen**. The thermal path only ever worked because 416x312x2 = 259584 happens to divide
by 64. `ppa_out_bytes()` rounds the extent up, which leaves the thermal sizes byte-identical.

The driver names the cause exactly - `E ppa_core: out.buffer addr or out.buffer_size not aligned
to cache line size` - so read the PPA tag's own errors before theorising about alignment of the
input.

**P2 vendor commands must be thermal-only.** `p2_camera_init()` ran after every stream open, so it
fired InfiRay vendor control requests at the webcam, which correctly stalled every one
(`USBH: Dev 1 EP 0 STALL`) - and its channel-availability probe retries for 15 s at 25 ms
intervals, a control-transfer storm that drowns the stream it is meant to precede.

### That camera does not enumerate on this board

Measured, and it appears to be physical rather than a software problem:

- It enumerates and streams fine on a Linux host.
- The **P2 Pro works reliably on the same port with the same firmware**, so the port and host stack
  are fine.
- It is plugged **directly** into the OTG connector, no adapter.
- Failures land at varying enumeration stages - `CHECK_SHORT_DEV_DESC`, `CHECK_SHORT_LANGID_TABLE`,
  `CHECK_FULL_CONFIG_DESC` - with `Dev N EP 0 Error` and ~120 `HUB: Root port reset failed` per
  minute, and the device address incrementing as it re-enumerates. A size or parsing fault would
  fail at the *same* stage every time; `CHECK_SHORT_DEV_DESC` is an 8-byte control read.
- `stream_open` then fails `ESP_ERR_INVALID_STATE`, which in `usbh_ep_alloc` means
  `dev_obj->constant.config_desc` is NULL - i.e. it is racing a device instance that already died,
  not a fault in the streaming code.
- It is **not** power: `bMaxPower` requests 500 mA, but the camera runs cooler than the P2, its
  illumination LED stays steady and it never visibly drops out.

What it actually does, measured with a second diagnostic USB host client (`P2_USB_PROBE`):

```
probe: addr 12 speed=HIGH (480 Mbit/s) bMaxPacketSize0=64 cfg=1
Device connected, addr=12
probe: device gone                                  <- ~10ms later
```

It negotiates **high speed** and enumerates **completely** - device descriptor and configuration
both readable - and then detaches within ~10 ms. The address climbs on every cycle. On other
rounds it dies earlier, at `Dev 0 EP 0 Error` (address 0, i.e. the first 8-byte descriptor read).

Eight hypotheses eliminated by measurement, so none of them get retried:

| ruled out | how |
|---|---|
| power / brownout | 150 mA measured on an inline meter; LED steady, runs cooler than the P2 |
| full-speed fallback hitting the known P4 FS enumeration bug | probe reports **HIGH** every time |
| config descriptor too large | `wTotalLength` 702 bytes vs our 4096 limit |
| IDF 5.5.5 NULL `enum_filter_cb` regression | the block is inside `#if ENABLE_ENUM_FILTER_CALLBACK`, and that option is unset here |
| our `usb_host_device_free_all()` on `NO_CLIENTS` | instrumented - it never fires during the loop |
| our `stream_open` destabilising it | with `P2_NO_OPEN_GENERIC` it still cycled 13 times in 40 s |
| raising the USB enumeration timings | `DEBOUNCE 500 / HOLD 50 / RECOVERY 200 / SET_ADDR 50` made it **worse** - the root port reset then failed outright and enumeration never started |
| a powered hub regenerating the signal | tested through a powered dock: no change |

What remains is an electrical / PHY-level interaction specific to this device on this port: EP0
control transfers error at random stages and the device detaches, while the **P2 Pro works on the
same port** and this camera works on a PC. The untested variables are the OTG connector's modified
wiring, the scope's ~1 m of thin cable, and the P4 OTG PHY's tolerance of the two combined.

**Answered by testing an iMac webcam on the same port: it enumerates once, cleanly, with zero HUB
errors and streams fine.** So the port, the wiring and the host stack are all sound, and this
scope is specifically the device that does not work.

### What it actually does (enum stage trace)

With `P2_USB_ENUM_TRACE` the enumerator names every stage, and the answer is not what the error
messages suggest:

```
ENUM: GET_SHORT_DEV_DESC OK
ENUM: CHECK_SHORT_DEV_DESC OK
ENUM: SECOND_RESET OK
ENUM: SECOND_RESET_COMPLETE OK
ENUM: SET_ADDR OK          (dev_addr=1)
ENUM: CHECK_ADDR OK
ENUM: SET_ADDR_RECOVERY OK
ENUM: GET_FULL_DEV_DESC OK
HUB:  Device tree node (port 0, uid=1): device gone     <-- a DISCONNECT
ENUM: CANCEL OK
```

**Every stage passes, and then the hub reports a disconnect** - 18 cycles in 22 s. So the
`CHECK_SHORT_DEV_DESC FAILED` / `CHECK_FULL_CONFIG_DESC FAILED` messages are collateral: the device
vanishes mid-sequence and whichever transfer is in flight errors. The enumerator is working
correctly, and `ENUM_STAGE_SECOND_RESET` (the "old devices get confused" workaround) passes fine,
so that is not it either.

A host seeing a device vanish when the device is demonstrably fine points at **false high-speed
disconnect detection**: HS disconnect is sensed electrically off the differential pair, and a
marginal link makes the host conclude the device unplugged. That fits the asymmetry - the webcam's
short lead works, the scope's ~1 m of thin cable does not, and a PC's PHY tolerates it.

**No USB timing knob helps.** Tested one at a time (the earlier four-at-once test was bad
experimental design and its conclusion was worthless), 25 s windows:

| config | HUB errors | ENUM failures | enumerations | stream opened |
|---|---|---|---|---|
| baseline | 51 | 3 | 6 | no |
| `RESET_RECOVERY_MS=100` | 59 | 1 | 0 | no |
| `SET_ADDR_RECOVERY_MS=50` | 46 | 3 | 3 | no |
| `DEBOUNCE_DELAY_MS=500` | 27 | 3 | 3 | no |

**A powered USB-C dock does not help either, but inconclusively:** with the dock between board and
camera the board sees *nothing at all* - no `HUB: Root port reset`, no device tree node, silence
after boot. Enumeration never starts, so this is at connection-detect level, not a descriptor or
protocol problem.

Why the dock does not attach is **unknown**, and two plausible-sounding explanations are already
ruled out:

- Not the VBUS *source*. A hub sees 5 V on the pin and does not care whether an external boost or
  the host cable put it there.
- Not VBUS *collapsing* under the dock's own load. The dock is powered from its PD input by a
  USB-C supply, so its hub controller does not draw from the board at all.

What is established is only that enumeration never starts, so it is at connection-detect level
rather than anything protocol-shaped. No mechanism beyond that is worth writing down until
something is measured.

Either way this cannot say whether a hub would fix the signalling - a **plain bus-powered USB 2.0
hub**, which needs no CC negotiation and no PD, is the cheaper way to get a signal repeater into
the path.

### The workaround: a plain USB 2.0 hub

**Confirmed working.** A plain bus-powered USB 2.0 hub between the board and the borescope gives a
clean, stable image at the full 25 fps. A hub terminates and re-drives the camera's link on its own
downstream port, so the marginal cable stops being the ESP's problem - exactly what false
high-speed disconnect detection predicts. This is the recommended fix: no firmware change, no
framerate cost.

**Full speed is NOT available as an alternative on this board.** Speed is settled by the chirp
handshake during bus reset so it cannot be forced from the host for a given port, but the P4 has
two OTG controllers - `USB_DWC_LL_GET_HW(num)` gives `USB_DWC_FS` for `num == 1` - so
`usb_host_config_t.peripheral_map = BIT1` installs the host on the full-speed one, where 12 Mbit/s
signalling would sidestep HS disconnect detection entirely.

Tested (`P2_FORCE_FULL_SPEED`): `usb_host_install()` succeeds with no error, and then **nothing
enumerates at all** - not one root port reset in 28 s. The FS controller's pins are not routed to
this board's OTG connector. Do not retry on this hardware.

It would have cost framerate anyway: full speed gives ~1.1 MB/s realistically, so 320x240 YUY2
manages ~7 fps against 25 through the hub. Recovering 25 fps at full speed would mean MJPEG plus
the P4's hardware JPEG decoder.

### Why it works on a phone but not here

High-speed disconnect detection is **purely electrical and has no handshake**: the host samples the
differential amplitude during the EOP of each SOF, and an unloaded bus roughly doubles it past
~625 mV. On a long thin cable, attenuation and impedance discontinuities mean reflections in that
EOP window can cross the threshold while the device is still present and perfectly healthy - which
is exactly the observed "every stage OK, then device gone".

Phone and PC SoCs tolerate the same cable because their PHYs have trimmed terminations and
*adjustable* disconnect thresholds and squelch, plus impedance-controlled routing to the connector.

**The P4 DOES expose analogue PHY trims**, and no IDF change is needed to reach them. They are not
in the DWC core register map (which is where I first looked, and wrongly concluded there was
nothing) but in a separate UTMI PHY block - `soc/usb_utmi_struct.h` under
`components/soc/esp32p4/register/hw_ver3/`, already on the include path because soc's CMakeLists
appends the `hw_ver` directory for the chip revision. `usb_phy.c` never writes them (its
`utmi_hal_context` is "unused for now"), so they sit at hardware defaults and can be set straight
from application code via `USB_UTMI`:

| field | default | options | relevance |
|---|---|---|---|
| `adj_res_hs` | `0x4` = 45 Ω | `0x0`=40 Ω, `0x6`=50 Ω | HS termination trim - changes the very amplitude disconnect is judged on |
| `adj_vref_sq` | `0x2` = 124 mV | `0x0`=92 mV, `0x3`=152 mV | squelch detection threshold |
| `adj_vsw_hs` | `0x4` = 400 mV | `0x0`=320 mV, `0x7`=460 mV | TX eye / output swing |
| `adj_pw_hs` | `0xF` = 400 mV | `0x1`=100 mV … | reduced-swing power saving |
| `adj_iref_res`, `adj_pll`, `adj_txclk_phase` | | | other analogue trims |

`P2_UTMI_TRIM` wires these up. **Write them between `usb_host_install()` and powering the root
port** - use `root_port_unpowered = true` and call `usb_host_lib_set_root_port_power(true)`
afterwards. That sequencing is verified harmless on its own (HUB 25-27, enum 4-6, same as
baseline), and it is also what makes the power call valid; it returns `ESP_ERR_INVALID_STATE`
("already powered") otherwise.

**The header's documented defaults are not all correct.** Measured on hardware:

```
fc_00 = 0x00000080  ->  adj_res_hs  = 0x4 (45 ohm)      as documented
fc_01 = 0x000000f8  ->  adj_vref_sq = 0x8               header says 0x2 - IT IS 0x8
                        adj_pw_hs   = 0xF (400 mV)      as documented
fc_02 = 0x00000047  ->  adj_iref_res = 0x7, adj_vsw_hs = 0x4 (400 mV)
```

This matters: writing `adj_vref_sq = 0x2` because the header calls it the default actually *cuts*
the squelch threshold hard and kills enumeration - 42-43 HUB errors and 0 enumerations, against
20-31 and 2-10 at the real `0x8`. An earlier claim here that "the write itself breaks enumeration"
was wrong; it was the value. Writing a register with its true value is harmless (verified
per-register).

Squelch sweep result: `0x9`, `0xA`, `0xC`, `0xF` all make the device **invisible** - no hub
activity at all - so `0x8` already sits at the edge of detection with no headroom upward. **One
run at `0xA` did produce `enum=3` and `streamopen=3`, the only stream opens ever observed on this
camera**, but it was not reproducible and the camera went electrically absent shortly afterwards.
Unconfirmed, and worth retrying on a known-good rig.

### Squelch 0xA is a real improvement, but not a fix

With the camera freshly power-cycled and `adj_vref_sq = 0xA` as the **only** change:

| | default `0x8` | `0xA` |
|---|---|---|
| HUB errors | 20-43 | **2-7** |
| enumerations | 3-10 | 11 |
| **stream opens** | **0, always** | **7-11** |
| frames | 0 | 0 |

Stream opens had *never* happened on this camera before. So raising the squelch threshold genuinely
fixes the link-level instability - the device stops being dropped mid-enumeration. The failure then
moves downstream: the stream opens and isochronous data never completes a frame. Disabling the
remaining payload check (`CONFIG_UVC_CHECK_PAYLOAD_HEADER_ERR=n`) does not change that.

`0xA` is **not** the default, because it is untested against the P2 Pro and the webcam: higher
squelch makes a device harder to see (`0x9` and above made the borescope invisible in other runs),
so it could plausibly break the cameras that currently work. Test those before adopting it.

**Measurement cost warning.** The borescope *wedges* after repeated failed attempts - it keeps its
LED lit and keeps drawing current while presenting nothing on the bus - so every data point needs a
physical replug, and run-to-run variance is large (identical firmware gives 2-10 enumerations, and
one run gave `ESP_ERR_NOT_FOUND` and no stream at all). That variance eventually exceeded the
effect sizes being measured, which is where this investigation stopped.

### A bandwidth table earlier in this file was wrong

The isoc figures I first derived for this camera were too low by 3x. `lsusb -v` prints
`wMaxPacketSize 0x03fc  3x 1020 bytes`, and my parser took the byte count while discarding the
`3x` - the transactions-per-microframe multiplier for high-bandwidth isochronous. The component
gets it right: `max_packet_size *= (USB_EP_DESC_GET_MULT(ep_desc) + 1)`, giving 3060 bytes per
microframe, i.e. **~24.5 MB/s, not 8.16 MB/s**. So 640x480 YUY2 at 25 fps (15.4 MB/s) was always
within budget, and "320x240 is the only mode that fits" was a parsing bug, not a finding.

### Validity rule for any future sweep

A run with **`HUB == 0` and `enum == 0` means nothing was attached** - discard it, do not score it
as "this setting failed". Several of my sweep results were exactly that, and I initially scored
them as negative results for the trims. Check for device presence before attributing an outcome to
a setting. Use *"does a stream open"* as the metric; enumeration counts are noisy (identical runs
give 2-10).

Masking the disconnect interrupt does **not** work (5 enumerations, 0 stream opens, unchanged):
`disconnint` is the OTG-level disconnect, whereas the host port state machine detects removal via
`HPRT.prtconndet` and the port interrupt, and masking *that* would mask connect events too.

A hub remains the reliable answer - it is what the USB specification provides repeaters for, and it
re-drives the marginal segment with a properly terminated PHY at the camera's end.

Anything further needs a scope on D+/D-, not more software.

## Flash layout (16MB)

The board carries 16MB but the image header declared 2MB, so the bootloader logged
`Detected size(16384k) larger than the size in the binary image header(2048k)` and ignored the
rest. With `CONFIG_ESPTOOLPY_FLASHSIZE_16MB` the whole chip is addressable:

| partition | type | offset | size | |
|---|---|---|---|---|
| `nvs` | data/nvs | 0x9000 | 24K | |
| `phy_init` | data/phy | 0xf000 | 4K | |
| `factory` | app | 0x10000 | 1M | app is ~509KB, 49% used |
| `ffc` | data/0x40 | 0x110000 | 64K | flat-field table, 48KB + header |
| `storage` | data/fat | 0x120000 | **14.875M** | declared, not mounted |

Every offset below `0x120000` is deliberately unchanged from the 2MB layout, so a stored
flat-field table survives the change and previously released merged binaries stay
layout-compatible. The app's 1M is left alone at 49% used.

`storage` is **declared but not mounted** — nothing consumes it yet, and mounting a filesystem at
boot would cost time against a ~1.7s startup. Mount it on demand when something needs it.

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
